#pragma once
#include <array>
#include <cstdint>
#include <libusb-1.0/libusb.h>

struct GamepadState {
    double lx = 0, ly = 0, rx = 0, ry = 0;
    bool rb = false;
};

class Gamepad {
public:
    Gamepad();
    ~Gamepad();
    Gamepad(const Gamepad&) = delete;
    Gamepad& operator=(const Gamepad&) = delete;
    bool open();
    bool poll(GamepadState& state, bool& fresh); // true=transport OK, fresh=valid input
    void close();
private:
    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    bool detached_ = false;
    int claimed_ = -1;
    static double normalized(const unsigned char* ptr);
};
