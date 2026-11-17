#include <libusb-1.0/libusb.h>
#include <algorithm>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }

struct Settings {
    uint16_t vid = 0x2345;
    uint16_t pid = 0xe053;
    int iface = 0;
    unsigned char endpoint = 0x82;
    bool raw = true;
};

void usage(const char* program) {
    std::cout << "Usage: " << program << " [--vid 0x2345] [--pid 0xe053]"
              << " [--interface 0] [--endpoint 0x82] [--no-raw]\n"
              << "Read wireless receiver USB interrupt reports. Press Ctrl+C to exit.\n";
}

bool parseNumber(const char* s, unsigned long max, unsigned long& out) {
    if (!s || !*s || *s == '-') return false;
    char* end = nullptr;
    out = std::strtoul(s, &end, 0);
    return end != s && *end == '\0' && out <= max;
}

bool parseArgs(int argc, char** argv, Settings& s) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { usage(argv[0]); std::exit(0); }
        if (arg == "--no-raw") { s.raw = false; continue; }
        if (i + 1 >= argc) return false;
        unsigned long v = 0;
        if (!parseNumber(argv[++i], 65535, v)) return false;
        if (arg == "--vid") s.vid = static_cast<uint16_t>(v);
        else if (arg == "--pid") s.pid = static_cast<uint16_t>(v);
        else if (arg == "--interface" && v <= 255) s.iface = static_cast<int>(v);
        else if (arg == "--endpoint" && v <= 255) s.endpoint = static_cast<unsigned char>(v);
        else return false;
    }
    return true;
}

void dumpRaw(const unsigned char* data, int length) {
    std::cout << "[RAW " << length << "] ";
    for (int i = 0; i < length; ++i)
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(data[i]) << ' ';
    std::cout << std::dec << std::setfill(' ') << '\n';
}

struct Button {
    const char* label;
    int index;
    unsigned char mask;
};

const Button buttons[] = {
    {"DPAD_UP", 2, 0x01}, {"DPAD_DOWN", 2, 0x02},
    {"DPAD_LEFT", 2, 0x04}, {"DPAD_RIGHT", 2, 0x08},
    {"START", 2, 0x10}, {"BACK", 2, 0x20},
    {"L3", 2, 0x40}, {"R3", 2, 0x80},
    {"LB", 3, 0x01}, {"RB", 3, 0x02},
    {"GUIDE", 3, 0x04}, {"A", 3, 0x10},
    {"B", 3, 0x20}, {"X", 3, 0x40}, {"Y", 3, 0x80}
};

class InputDecoder {
public:
    void onPacket(const unsigned char* d, int n) {
        // Decode only known wired-style 20-byte reports.
        // Wireless receiver framing can differ; inspect RAW for unknown packets.
        if (n != 20 || d[0] != 0x00 || d[1] != 0x14) {
            if (!warned_) {
                std::cout << "[INFO] Non-standard report: button mapping not assumed. Inspect RAW changes.\n";
                warned_ = true;
            }
            validLast_ = false;
            return;
        }
        if (!validLast_) {
            last2_ = d[2]; last3_ = d[3]; validLast_ = true;
            std::cout << "[INFO] Standard 20-byte input report detected.\n";
            return;
        }
        for (const Button& b : buttons) {
            const unsigned char oldByte = b.index == 2 ? last2_ : last3_;
            const bool before = (oldByte & b.mask) != 0;
            const bool after = (d[b.index] & b.mask) != 0;
            if (before != after) {
                std::cout << "[BUTTON] " << b.label
                          << (after ? " PRESSED" : " RELEASED") << '\n';
            }
        }
        last2_ = d[2]; last3_ = d[3];
    }
private:
    bool warned_ = false;
    bool validLast_ = false;
    unsigned char last2_ = 0;
    unsigned char last3_ = 0;
};

} // namespace

int main(int argc, char** argv) {
    Settings s;
    if (!parseArgs(argc, argv, s)) { usage(argv[0]); return 2; }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    libusb_context* ctx = nullptr;
    int rc = libusb_init(&ctx);
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "libusb_init: " << libusb_error_name(rc) << '\n';
        return 1;
    }
    std::cout << "USB receiver VID:PID=" << std::hex << std::setw(4)
              << std::setfill('0') << s.vid << ':' << std::setw(4) << s.pid
              << " endpoint=0x" << std::setw(2) << static_cast<int>(s.endpoint)
              << std::dec << std::setfill(' ') << " interface=" << s.iface << '\n';

    libusb_device_handle* handle = libusb_open_device_with_vid_pid(ctx, s.vid, s.pid);
    if (!handle) {
        std::cerr << "Cannot open USB device (absent or insufficient permissions).\n";
        libusb_exit(ctx);
        return 1;
    }
    std::cout << "[OK] Device opened\n";
    bool detached = false;
    rc = libusb_kernel_driver_active(handle, s.iface);
    if (rc == 1) {
        rc = libusb_detach_kernel_driver(handle, s.iface);
        if (rc != LIBUSB_SUCCESS) {
            std::cerr << "Detach failed: " << libusb_error_name(rc) << '\n';
            libusb_close(handle); libusb_exit(ctx); return 1;
        }
        detached = true;
    }
    rc = libusb_claim_interface(handle, s.iface);
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "Claim failed: " << libusb_error_name(rc) << '\n';
        if (detached) libusb_attach_kernel_driver(handle, s.iface);
        libusb_close(handle); libusb_exit(ctx); return 1;
    }
    std::cout << "[OK] Interface claimed. Press buttons; Ctrl+C to quit.\n";
    InputDecoder decoder;
    std::vector<unsigned char> previous;
    while (running) {
        unsigned char data[32] = {};
        int size = 0;
        rc = libusb_interrupt_transfer(handle, s.endpoint, data, sizeof(data), &size, 100);
        if (rc == LIBUSB_ERROR_TIMEOUT) continue;
        if (rc != LIBUSB_SUCCESS) {
            std::cerr << "USB read failed: " << libusb_error_name(rc) << " (" << rc << ")\n";
            break;
        }
        if (size <= 0) continue;
        if (s.raw && (previous.size() != static_cast<size_t>(size) ||
                      !std::equal(previous.begin(), previous.end(), data))) {
            dumpRaw(data, size);
            previous.assign(data, data + size);
        }
        decoder.onPacket(data, size);
        std::cout.flush();
    }
    libusb_release_interface(handle, s.iface);
    if (detached) libusb_attach_kernel_driver(handle, s.iface);
    libusb_close(handle);
    libusb_exit(ctx);
    std::cout << "Exited\n";
    return rc == LIBUSB_SUCCESS || rc == LIBUSB_ERROR_TIMEOUT ? 0 : 1;
}
