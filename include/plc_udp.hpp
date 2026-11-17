#pragma once
#include <cstdint>
#include <string>
#include <vector>

class PlcUdp {
public:
    PlcUdp() = default;
    ~PlcUdp();
    PlcUdp(const PlcUdp&) = delete;
    PlcUdp& operator=(const PlcUdp&) = delete;

    bool open(const std::string& plc_ip, uint16_t local_port = 1101,
              uint16_t plc_port = 1101);
    void close();
    bool sendVelocity(double vx_mps, double wz_radps, bool enable);
    bool receiveReply(); // checks framing and checksum; actual PLC motion status not decoded
    bool connected() const { return fd_ >= 0; }

    static std::vector<uint8_t> encode(uint32_t seq, uint32_t sender_ipv4_be,
                                        double vx_mps, double wz_radps, bool enable);
    static bool verify(const uint8_t* data, size_t length);
private:
    int fd_ = -1;
    uint32_t sender_ipv4_be_ = 0; // IPv4 address in network order
    uint32_t sequence_ = 0;
};
