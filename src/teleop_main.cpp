#include "gamepad.hpp"
#include "plc_udp.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
using Clock = std::chrono::steady_clock;
double deadzone(double x) { return std::abs(x) < 0.08 ? 0.0 : x; }
struct Args {
    std::string plc_ip = "192.168.2.69";
    int level = 1; // start slow for physical tests
    bool live = false;
    bool acknowledged = false;
};
void help(const char* name) {
    std::cout << "Usage: " << name
              << " [--plc-ip 192.168.2.69] [--speed-level 1..10]"
              << " [--live --confirm-motion]\n"
              << "Default: dry-run; NO UDP packets sent.\n"
              << "Live motion requires RB held, fresh gamepad input and PLC replies.\n"
              << "WARNING: only test in an isolated safe area, with physical E-stop.\n";
}
bool parse(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help" || key == "-h") { help(argv[0]); std::exit(0); }
        if (key == "--live") a.live = true;
        else if (key == "--confirm-motion") a.acknowledged = true;
        else if ((key == "--plc-ip" || key == "--speed-level") && i + 1 < argc) {
            std::string value = argv[++i];
            if (key == "--plc-ip") a.plc_ip = value;
            else {
                char* end = nullptr;
                long n = std::strtol(value.c_str(), &end, 10);
                if (!end || *end || n < 1 || n > 10) return false;
                a.level = static_cast<int>(n);
            }
        } else return false;
    }
    return !a.live || a.acknowledged;
}
}

int main(int argc, char** argv) {
    Args args;
    if (!parse(argc, argv, args)) { help(argv[0]); return 2; }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    const double max_vx = args.level * 0.1;
    const double max_vy = args.level * 0.1;
    const double max_wz = std::min(1.0, args.level * 0.2);
    std::cout << "Mode: " << (args.live ? "LIVE PLC UDP" : "DRY RUN (no UDP)")
              << ", level=" << args.level << ", max vx=" << max_vx
              << " m/s, max vy=" << max_vy << " m/s, max wz=" << max_wz << " rad/s\n";
    std::cout << "LY up -> +vx, LX left -> +vy, RX left -> +wz; hold RB = deadman.\n";
    std::cout << "Physical E-stop required. This is NOT a safety controller.\n";

    PlcUdp plc;
    if (args.live && !plc.open(args.plc_ip)) {
        std::cerr << "Cannot bind/connect PLC UDP 1101. Check IP and existing control nodes.\n";
        return 1;
    }

    Gamepad pad;
    bool opened = pad.open();
    if (!opened) std::cerr << "Gamepad not open (check USB permissions).\n";
    GamepadState state{};
    auto last_input = Clock::time_point::min();
    auto last_reply = Clock::time_point::min();
    auto next_reopen = Clock::now() + std::chrono::seconds(1);
    auto next_log = Clock::now();
    auto next_tick = Clock::now();
    bool prev_enabled = false;
    bool io_failed = false;

    while (running) {
        next_tick += std::chrono::milliseconds(20); // 50Hz
        if (!opened && Clock::now() >= next_reopen) {
            opened = pad.open();
            if (opened) {
                state = GamepadState{};
                last_input = Clock::time_point::min();
                std::cout << "[USB] reconnected; RB must be held for motion\n";
            }
            next_reopen = Clock::now() + std::chrono::seconds(1);
        }
        if (opened) {
            bool fresh = false;
            if (!pad.poll(state, fresh)) {
                pad.close();
                opened = false;
                state = GamepadState{};
                last_input = Clock::time_point::min();
                next_reopen = Clock::now() + std::chrono::seconds(1);
                std::cerr << "[USB] disconnected: command zero\n";
            } else if (fresh) {
                last_input = Clock::now();
            }
        }
        if (args.live && plc.receiveReply()) last_reply = Clock::now();

        const auto now = Clock::now();
        const bool input_recent =
            opened && last_input != Clock::time_point::min() &&
            now - last_input <= std::chrono::milliseconds(250);
        const bool reply_recent =
            last_reply != Clock::time_point::min() &&
            now - last_reply <= std::chrono::milliseconds(400);
        const bool enabled = input_recent && state.rb &&
                             (!args.live || reply_recent);
        // No deadman, stale USB, missing PLC heartbeat => 0 speed, enable=0.
        const double vx = enabled ? deadzone(state.ly) * max_vx : 0.0;
        const double vy = enabled ? -deadzone(state.lx) * max_vy : 0.0;
        const double wz = enabled ? -deadzone(state.rx) * max_wz : 0.0;

        if (args.live && !plc.sendVelocity(vx, vy, wz, enabled)) {
            std::cerr << "[PLC] UDP send failed; exiting after stop attempts\n";
            io_failed = true;
            break;
        }
        if (prev_enabled && !enabled) std::cerr << "[SAFE] speed disabled / zero command\n";
        prev_enabled = enabled;

        if (now >= next_log) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[" << (args.live ? "LIVE" : "DRY") << "]"
                      << " LX=" << state.lx << " LY=" << state.ly << " RX=" << state.rx
                      << " RB=" << state.rb << " fresh=" << input_recent
                      << " plc_ack=" << (args.live ? reply_recent : false)
                      << " -> cmd vx=" << vx << " vy=" << vy << " wz=" << wz
                      << " enable=" << enabled << '\n';
            next_log = now + std::chrono::milliseconds(200);
        }
        std::this_thread::sleep_until(next_tick);
        if (Clock::now() > next_tick + std::chrono::milliseconds(200))
            next_tick = Clock::now();
    }

    if (args.live) {
        for (int i = 0; i < 10; ++i) {
            plc.sendVelocity(0.0, 0.0, 0.0, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "[SAFE] sent ten disable/zero UDP packets (delivery not guaranteed)\n";
    }
    return io_failed ? 1 : 0;
}
