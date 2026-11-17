#include "gamepad.hpp"
#include "plc_udp.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
using Clock = std::chrono::steady_clock;

double deadzone(double x) { return std::abs(x) < 0.08 ? 0.0 : x; }

struct Config {
    std::string plc_ip;
    int plc_port = 0;
    int local_port = 0;
    bool udp_enabled = false;
    double max_vx_mps = 0;
    double max_vy_mps = 0;
    double max_wz_radps = 0;
};

std::string trim(const std::string& s) {
    const size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const size_t last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}
double parseDouble(const std::string& s) {
    size_t pos = 0;
    double v = std::stod(s, &pos);
    if (pos != s.size() || !std::isfinite(v)) throw std::runtime_error("invalid number: " + s);
    return v;
}
int parsePort(const std::string& s) {
    size_t pos = 0;
    const int v = std::stoi(s, &pos);
    if (pos != s.size() || v < 1 || v > 65535) throw std::runtime_error("invalid port: " + s);
    return v;
}
Config readConfig(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) throw std::runtime_error("cannot read config: " + filename + " (run from project root)");
    Config cfg;
    bool seen_ip=false, seen_plc_port=false, seen_local_port=false, seen_mode=false;
    bool seen_vx=false, seen_vy=false, seen_wz=false;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        const size_t comment = line.find('#');
        if (comment != std::string::npos) line.resize(comment);
        line = trim(line);
        if (line.empty()) continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) throw std::runtime_error("invalid config line " + std::to_string(lineNumber));
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        if (key == "plc_ip") { cfg.plc_ip = value; seen_ip = true; }
        else if (key == "plc_port") { cfg.plc_port = parsePort(value); seen_plc_port = true; }
        else if (key == "local_port") { cfg.local_port = parsePort(value); seen_local_port = true; }
        else if (key == "udp_enabled") {
            if (value != "true" && value != "false") throw std::runtime_error("udp_enabled must be true/false");
            cfg.udp_enabled = (value == "true"); seen_mode = true;
        }
        else if (key == "max_vx_mps") { cfg.max_vx_mps = parseDouble(value); seen_vx = true; }
        else if (key == "max_vy_mps") { cfg.max_vy_mps = parseDouble(value); seen_vy = true; }
        else if (key == "max_wz_radps") { cfg.max_wz_radps = parseDouble(value); seen_wz = true; }
        else throw std::runtime_error("unknown config key: " + key);
    }
    if (!(seen_ip && seen_plc_port && seen_local_port && seen_mode &&
          seen_vx && seen_vy && seen_wz))
        throw std::runtime_error("missing required configuration fields");
    if (cfg.plc_ip.empty() || cfg.max_vx_mps <= 0 || cfg.max_vy_mps <= 0 ||
        cfg.max_wz_radps <= 0 || cfg.max_vx_mps > 1.0 || cfg.max_vy_mps > 1.0 ||
        cfg.max_wz_radps > 1.0)
        throw std::runtime_error("invalid IP or speed limits (must be >0 and <=1.0)");
    return cfg;
}

void usage(const char* app) {
    std::cout << "Usage: " << app << " [--config config/teleop.conf] [--dry-run]\n"
              << "Default: load config/teleop.conf from current directory.\n"
              << "udp_enabled=true sends real UDP! Hold RB to command movement.\n";
}
}

int main(int argc, char** argv) {
    std::string configPath = "config/teleop.conf";
    bool forceDryRun = false;
    for (int i = 1; i < argc; ++i) {
        const std::string opt = argv[i];
        if (opt == "--help" || opt == "-h") { usage(argv[0]); return 0; }
        if (opt == "--dry-run") forceDryRun = true;
        else if (opt == "--config" && i + 1 < argc) configPath = argv[++i];
        else { usage(argv[0]); return 2; }
    }
    Config config;
    try { config = readConfig(configPath); }
    catch (const std::exception& e) {
        std::cerr << "[CONFIG] " << e.what() << '\n';
        return 2;
    }
    const bool live = config.udp_enabled && !forceDryRun;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    std::cout << "Config: " << configPath
              << "\nMode: " << (live ? "LIVE UDP" : "DRY RUN (no UDP)")
              << ", PLC=" << config.plc_ip << ":" << config.plc_port
              << ", max vx=" << config.max_vx_mps
              << " m/s, max vy=" << config.max_vy_mps
              << " m/s, max wz=" << config.max_wz_radps << " rad/s\n"
              << "Scale: 0.2 -> signed integer 200 for the PLC.\n"
              << "LY forward/back, LX strafe, RX rotate; RB held = enable.\n"
              << "CAUTION: physical robot may move; physical E-stop and independent PLC watchdog required.\n";

    PlcUdp plc;
    if (live && !plc.open(config.plc_ip,
                          static_cast<uint16_t>(config.local_port),
                          static_cast<uint16_t>(config.plc_port))) {
        std::cerr << "Cannot bind/connect PLC UDP. Check address, interface, or existing controller.\n";
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
        next_tick += std::chrono::milliseconds(20); // 50 Hz
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
                std::cerr << "[USB] disconnected; output zero\n";
            } else if (fresh) {
                last_input = Clock::now();
            }
        }
        if (live && plc.receiveReply()) last_reply = Clock::now();

        const auto now = Clock::now();
        const bool freshInput = opened && last_input != Clock::time_point::min()
                               && now - last_input <= std::chrono::milliseconds(250);
        const bool freshReply = last_reply != Clock::time_point::min()
                               && now - last_reply <= std::chrono::milliseconds(400);
        const bool enabled = freshInput && state.rb && (!live || freshReply);

        const double vx = enabled ? deadzone(state.ly) * config.max_vx_mps : 0.0;
        const double vy = enabled ? -deadzone(state.lx) * config.max_vy_mps : 0.0;
        const double wz = enabled ? -deadzone(state.rx) * config.max_wz_radps : 0.0;

        if (live && !plc.sendVelocity(vx, vy, wz, enabled)) {
            std::cerr << "[PLC] send failed; attempting stop\n";
            io_failed = true;
            break;
        }
        if (prev_enabled && !enabled) std::cerr << "[SAFE] zero speed / disabled\n";
        prev_enabled = enabled;

        if (now >= next_log) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[" << (live ? "LIVE" : "DRY") << "]"
                      << " LX=" << state.lx << " LY=" << state.ly << " RX=" << state.rx
                      << " RB=" << state.rb << " fresh=" << freshInput
                      << " plc_ack=" << (live ? freshReply : false)
                      << " -> vx=" << vx << " vy=" << vy << " wz=" << wz
                      << " (raw " << std::lround(vx * 1000)
                      << "," << std::lround(vy * 1000)
                      << "," << std::lround(wz * 1000) << ")"
                      << " enable=" << enabled << '\n';
            next_log = now + std::chrono::milliseconds(200);
        }
        std::this_thread::sleep_until(next_tick);
        if (Clock::now() > next_tick + std::chrono::milliseconds(200))
            next_tick = Clock::now();
    }

    if (live) {
        for (int i = 0; i < 10; ++i) {
            plc.sendVelocity(0.0, 0.0, 0.0, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "[SAFE] ten zero/disable packets sent; UDP delivery not guaranteed\n";
    }
    return io_failed ? 1 : 0;
}
