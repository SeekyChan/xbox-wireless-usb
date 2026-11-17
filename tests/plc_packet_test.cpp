#include "plc_udp.hpp"
#include <arpa/inet.h>
#include <cassert>
#include <cstdint>
#include <iostream>
int main() {
    const uint32_t ip = inet_addr("192.168.2.110");
    const auto stop = PlcUdp::encode(1, ip, 0.3, -0.6, false);
    assert(stop.size() == 49);
    assert(stop[0] == 0xFF && stop[1] == 0xFA);
    assert(stop[2] == 0x01 && stop[3] == 0x02);
    assert(stop[8] == 192 && stop[9] == 168 && stop[10] == 2 && stop[11] == 110);
    assert(stop[12] == 0x00 && stop[13] == 0xF0);
    assert(stop[14] == 24 && stop[15] == 0);
    assert(stop[24] == 0 && stop[25] == 0);
    assert(stop[26] == 0 && stop[27] == 0);
    assert(stop[28] == 0 && stop[29] == 0);
    assert(stop[30] == 0 && stop[36] == 0);
    assert(PlcUdp::verify(stop.data(), stop.size()));
    const auto forward = PlcUdp::encode(2, ip, 0.3, -0.6, true);
    assert(forward.size() == 49);
    assert(forward[24] == 0x2C && forward[25] == 0x01); // +300 mm/s
    assert(forward[26] == 0 && forward[27] == 0);
    assert(forward[28] == 0xA8 && forward[29] == 0xFD); // -600 mrad/s
    assert(forward[30] == 1); // forward
    assert(forward[36] == 1); // speed enable
    assert(PlcUdp::verify(forward.data(), forward.size()));
    auto corrupted = forward;
    corrupted[24] ^= 1;
    assert(!PlcUdp::verify(corrupted.data(), corrupted.size()));
    const auto spin = PlcUdp::encode(3, ip, 0, 0.2, true);
    assert(spin[30] == 5);
    const auto backward = PlcUdp::encode(4, ip, -0.1, 0, true);
    assert(backward[30] == 2);
    std::cout << "plc_packet_test PASS (format uses documented fields + 1-byte checksum assumption)\n";
}
