#include "gamepad.hpp"
#include <iostream>

namespace {
constexpr uint16_t kVid = 0x2345, kPid = 0xe053;
constexpr int kInterface = 0;
constexpr unsigned char kEndpoint = 0x82;
}

Gamepad::Gamepad() {
    const int rc = libusb_init(&ctx_);
    if (rc != LIBUSB_SUCCESS) {
        ctx_ = nullptr;
        std::cerr << "libusb_init: " << libusb_error_name(rc) << '\n';
    }
}
Gamepad::~Gamepad() {
    close();
    if (ctx_) libusb_exit(ctx_);
}
bool Gamepad::open() {
    close();
    if (!ctx_) return false;
    handle_ = libusb_open_device_with_vid_pid(ctx_, kVid, kPid);
    if (!handle_) return false;
    int rc = libusb_kernel_driver_active(handle_, kInterface);
    if (rc < 0 && rc != LIBUSB_ERROR_NOT_SUPPORTED) { close(); return false; }
    if (rc == 1) {
        rc = libusb_detach_kernel_driver(handle_, kInterface);
        if (rc != LIBUSB_SUCCESS) { close(); return false; }
        detached_ = true;
    }
    rc = libusb_claim_interface(handle_, kInterface);
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "claim interface: " << libusb_error_name(rc) << '\n';
        close();
        return false;
    }
    claimed_ = kInterface;
    return true;
}
void Gamepad::close() {
    if (!handle_) return;
    if (claimed_ >= 0) libusb_release_interface(handle_, claimed_);
    if (detached_) libusb_attach_kernel_driver(handle_, kInterface);
    libusb_close(handle_);
    handle_ = nullptr;
    claimed_ = -1;
    detached_ = false;
}
double Gamepad::normalized(const unsigned char* p) {
    const uint16_t bits = static_cast<uint16_t>(p[0]) | (uint16_t(p[1]) << 8);
    const int32_t value = bits < 32768 ? bits : int32_t(bits) - 65536;
    return value < 0 ? double(value) / 32768.0 : double(value) / 32767.0;
}
bool Gamepad::poll(GamepadState& state, bool& fresh) {
    fresh = false;
    if (!handle_) return false;
    std::array<unsigned char, 32> data{{}};
    int len = 0;
    const int rc = libusb_interrupt_transfer(handle_, kEndpoint, data.data(),
                                             static_cast<int>(data.size()), &len, 20);
    if (rc == LIBUSB_ERROR_TIMEOUT) return true;
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "gamepad read: " << libusb_error_name(rc) << '\n';
        return false;
    }
    if (len != 20 || data[0] != 0x00 || data[1] != 0x14) return true;
    state.lx = normalized(data.data() + 6);
    state.ly = normalized(data.data() + 8);
    state.rx = normalized(data.data() + 10);
    state.ry = normalized(data.data() + 12);
    state.rb = (data[3] & 0x02) != 0;
    fresh = true;
    return true;
}
