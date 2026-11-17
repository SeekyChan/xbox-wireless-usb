# Xbox Wireless USB — PLC Teleop

A C++11/CMake joystick-to-UDP controller for **x86_64 Ubuntu**. No ROS dependency.

## Features

- USB controller `2345:e053` via libusb (interface 0, IN endpoint 0x82).
- `Gamepad` class for 20-byte joystick/button input and reconnection.
- `PlcUdp` class for UDP packet encoding, response checking and transmission.
- Swerved-wheel chassis input: **LY = forward/reverse (Vx); LX = left/right strafe (Vy); RX = left/right yaw (Wz)**.
- **RB must be held** for nonzero velocity; release RB to stop.
- 50 Hz send loop; stop on USB input freshness timeout and missing PLC responses.
- Default PLC IP **192.168.2.69**, UDP port 1101.
- Default speed level 1: maximum |Vx| and |Vy| = 0.1 m/s, maximum |Wz| = 0.2 rad/s.
- Dry-run by default; live UDP requires an explicit confirmation switch.

## Build on Ubuntu

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libusb-1.0-0-dev
git clone https://github.com/SeekyChan/xbox-wireless-usb.git
cd xbox-wireless-usb
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

## Test input and Vx/Vy/Wz safely (no PLC transmission)

```bash
sudo ./build/xbox_plc_teleop
```

Or `sudo ./build/xbox_plc_teleop --speed-level 3`. This prints `LX`, `LY`, `RX`, `RB`, `vx`, `vy`, `wz` and `enable`. Example:

```text
[DRY] LX=-1.000 LY=0.000 RX=0.000 RB=1 fresh=1 plc_ack=0 -> cmd vx=0.000 vy=0.100 wz=0.000 enable=1
```

This is only an example and not a measured result. `plc_ack=0` is expected in DRY mode.

| Hand control | Command |
| --- | --- |
| Left stick up/down | Vx positive/negative |
| Left stick left/right | Vy positive/negative |
| Right stick left/right | Wz positive/negative for CCW/CW respectively |
| RB released | zero velocity, speed disabled |
| Stale USB input or PLC timeout | zero velocity, speed disabled |

## Live UDP bench test — use with caution

**WARNING: this can move a physical robot.** Verify that only one process controls the PLC UDP port; stop the normal IPC/ROS control stack before bench testing. Keep the vehicle lifted or the area isolated, provide physical E-stop and a supervisor, and confirm independent PLC communication-loss stopping. Do not use on occupied or production equipment.

The UDP implementation has **not been confirmed against a live PLC**. Check received PLC replies, configured motion modes, checksum width and site-specific status/enable fields against firmware before attempting physical motion. The current reply handler validates basic frame/checksum only; it does not decode the PLC fault, mode, or emergency stop status. A failed response or a stopped PC cannot guarantee physical stopping without PLC firmware protections.

First confirm the simulated outputs. When the PLC packet format, status and E-stop behavior are validated at the test site, execute:

```bash
sudo ./build/xbox_plc_teleop --live --confirm-motion --speed-level 1
```

The target is **192.168.2.69:1101** by default. Override it using `--plc-ip ADDRESS`. Nonzero speed also requires a fresh input report, recent valid PLC response and **RB held**. If the PLC does not reply in the expected framing, the program will keep sending disabled/zero commands. Ctrl+C attempts repeated stop packets, but UDP delivery is not guaranteed.

**Special note:** motion-type handling for combined translation/rotation (especially diagonal strafe and yaw) must be verified on the actual PLC before use.

## USB-only diagnostics

```bash
sudo ./build/xbox_usb_test --no-raw
```

This tool reports controller buttons and all stick axes, without sending UDP.

## Permissions

To run without sudo, use a udev group rule:

```bash
sudo groupadd -f usbgamepad
sudo usermod -aG usbgamepad "$USER"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2345", ATTR{idProduct}=="e053", GROUP="usbgamepad", MODE="0660"' | sudo tee /etc/udev/rules.d/99-usbgamepad.rules
sudo udevadm control --reload-rules
```

Replug the receiver and log out/in.

## Code structure

```text
include/gamepad.hpp       Gamepad class
include/plc_udp.hpp       PlcUdp class
src/gamepad.cpp           USB joystick parsing
src/plc_udp.cpp           UDP transport and command packing
src/teleop_main.cpp       Teleoperation loop / safety gates
src/main.cpp              USB-only diagnostic tool
tests/plc_packet_test.cpp Local packet unit tests
```

No original PLC protocol document is included in this repository. Only implementation source code and developer-oriented usage notes are provided.
