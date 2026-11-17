#include "plc_udp.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>

namespace {
void u16(std::vector<uint8_t>& b, uint16_t n) {
    b.push_back(uint8_t(n)); b.push_back(uint8_t(n >> 8));
}
void u32(std::vector<uint8_t>& b, uint32_t n) {
    for (int i = 0; i < 4; ++i) b.push_back(uint8_t(n >> (8*i)));
}
void u64(std::vector<uint8_t>& b, uint64_t n) {
    for (int i = 0; i < 8; ++i) b.push_back(uint8_t(n >> (8*i)));
}
uint8_t checksum(const uint8_t* p, size_t n) {
    unsigned int sum = 0;
    for (size_t i = 0; i < n; ++i) sum += p[i];
    return uint8_t(sum & 0xff);
}
uint8_t motion(double vx, double wz, bool enable) {
    if (!enable || (std::abs(vx) < 0.0005 && std::abs(wz) < 0.0005)) return 0;
    if (std::abs(vx) < 0.0005) return 5; // self-spin
    return vx > 0 ? 1 : 2; // forward/backward, including simultaneous steering
}
}

PlcUdp::~PlcUdp() { close(); }

std::vector<uint8_t> PlcUdp::encode(uint32_t seq, uint32_t ip_be,
                                   double vx, double wz, bool enable) {
    std::vector<uint8_t> b;
    b.reserve(49);
    u16(b, 0xFAFF);             // fixed header, little endian: FF FA
    u16(b, 0x0201);             // version 2.1
    u32(b, seq);
    const uint8_t* ip = reinterpret_cast<const uint8_t*>(&ip_be);
    b.insert(b.end(), ip, ip + 4); // sender IP stored octet by octet
    u16(b, 0xF000);             // navigation control
    u16(b, 24);                 // body length
    u64(b, 0);                  // reserved

    const int16_t sx = static_cast<int16_t>(std::lround(vx * 1000.0));
    const int16_t sw = static_cast<int16_t>(std::lround(wz * 1000.0));
    u16(b, uint16_t(enable ? sx : 0)); // Vx, 0.001 m/s
    u16(b, 0);                         // Vy
    u16(b, uint16_t(enable ? sw : 0)); // W, 0.001 rad/s
    b.push_back(motion(vx, wz, enable)); // motion type
    b.push_back(0);                   // voice ID
    b.push_back(0);                   // obstacle zone (site-specific, REVIEW!)
    b.push_back(0);                   // obstacle state: normal
    u16(b, 0);                        // IPC fault bitmap
    b.push_back(enable ? 1 : 0);      // speed enable
    b.push_back(0);                   // charging control: unknown
    b.push_back(0);                   // shield: all safety sensors enabled
    b.push_back(0);                   // compound command: none
    b.push_back(0);                   // HMI manual button: none
    b.push_back(0);                   // reflector count
    b.push_back(0);                   // localization score
    b.push_back(0);                   // reset ID
    u16(b, 0);                        // reset target
    b.push_back(0);                   // QR detected: no (site-specific)
    b.push_back(0);                   // brake enable control: 0 (site-specific)
    // Document states byte-sum checksum but not its width.
    // ASSUMPTION: final byte = lower 8 bits of sum. Verify with PLC capture.
    b.push_back(checksum(b.data(), b.size()));
    return b;
}

bool PlcUdp::verify(const uint8_t* b, size_t n) {
    if (n < 25 || b[0] != 0xFF || b[1] != 0xFA) return false;
    const uint16_t body = uint16_t(b[14]) | (uint16_t(b[15]) << 8);
    if (n != size_t(24) + body + 1) return false;
    if (b[12] != 0x00 || b[13] != 0xF0) return false;
    return b[n-1] == checksum(b, n-1);
}

bool PlcUdp::open(const std::string& ip, uint16_t local_port, uint16_t plc_port) {
    close();
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) return false;
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(local_port);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        std::cerr << "UDP bind port " << local_port << ": " << std::strerror(errno) << '\n';
        close(); return false;
    }
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(plc_port);
    if (inet_pton(AF_INET, ip.c_str(), &peer.sin_addr) != 1 ||
        connect(fd_, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) != 0) {
        std::cerr << "UDP target address invalid/unreachable\n";
        close(); return false;
    }
    sockaddr_in actual{};
    socklen_t len = sizeof(actual);
    if (getsockname(fd_, reinterpret_cast<sockaddr*>(&actual), &len) != 0) {
        close(); return false;
    }
    sender_ipv4_be_ = actual.sin_addr.s_addr;
    char address[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &actual.sin_addr, address, sizeof(address));
    std::cout << "UDP local " << address << ":" << local_port
              << " -> " << ip << ":" << plc_port << '\n';
    return true;
}
void PlcUdp::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}
bool PlcUdp::sendVelocity(double vx, double wz, bool enable) {
    if (fd_ < 0) return false;
    const auto p = encode(++sequence_, sender_ipv4_be_, vx, wz, enable);
    const ssize_t sent = send(fd_, p.data(), p.size(), 0);
    if (sent != static_cast<ssize_t>(p.size())) {
        std::cerr << "UDP send: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}
bool PlcUdp::receiveReply() {
    if (fd_ < 0) return false;
    bool valid = false;
    for (;;) {
        uint8_t b[2048];
        const ssize_t n = recv(fd_, b, sizeof(b), 0);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                std::cerr << "UDP recv: " << std::strerror(errno) << '\n';
            break;
        }
        if (n == 0) break;
        if (verify(b, static_cast<size_t>(n))) valid = true;
    }
    return valid;
}
