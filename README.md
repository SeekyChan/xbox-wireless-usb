# Xbox Wireless USB — PLC Teleoperation

C++11 / CMake project for x86 Ubuntu. USB joystick parsing and PLC UDP transport are independent classes; no ROS runtime required.

## Quick start

```bash
git clone https://github.com/SeekyChan/xbox-wireless-usb.git
cd xbox-wireless-usb
sudo apt update
sudo apt install -y build-essential cmake pkg-config libusb-1.0-0-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
sudo ./build/xbox_plc_teleop
```

**The last command starts real UDP transmission by default.** Run it only when the PLC and physical machine are ready for supervised testing. You do not need to enter IP, port or speed flags.

## Default configuration

Edit [config/teleop.conf](config/teleop.conf):

```ini
plc_ip=192.168.2.69
plc_port=1101
local_port=1101
udp_enabled=true
max_vx_mps=0.2
max_vy_mps=0.2
max_wz_radps=0.2
```

Move velocity channels through `Gamepad` and `PlcUdp`:

| Controller | Output | Max | Raw integer at max |
| --- | --- | --- | --- |
| Left stick vertical (LY) | Vx, forward / backward | ±0.2 m/s | ±200 |
| Left stick horizontal (LX) | Vy, lateral left / right | ±0.2 m/s | ±200 |
| Right stick horizontal (RX) | Wz, counterclockwise / clockwise | ±0.2 rad/s | ±200 |

The receiver is `2345:e053`, USB interface 0, endpoint 0x82. Speed conversion to the UDP payload multiplies physical units by 1000 and rounds to the nearest integer. The program logs values in m/s, rad/s, **and the encoded raw integers**.

**Hold RB** to enable speed commands; releasing RB sends zero/disabled. The output also returns to zero if valid gamepad messages stop arriving for 250 ms, and does not require PLC acknowledgements. The loop sends at 50 Hz.

## Test without UDP traffic

```bash
sudo ./build/xbox_plc_teleop --dry-run
```

To change the configuration path:

```bash
sudo ./build/xbox_plc_teleop --config /path/to/teleop.conf
```

Run from the repository root for default config lookup. Missing/broken configuration fails closed rather than silently switching to other addresses.

You can also run the independent USB diagnostics:

```bash
sudo ./build/xbox_usb_test --no-raw
```

## Safety and known integration limitations

This program can command a **physical mobile robot**. Only test in an isolated space with a working physical emergency stop, verified PLC communications-loss stopping, adequate clearance, and supervision. Stop the normal PLC command writer / ROS controller before using this utility; ensure port 1101 has one responsible controller.

The UDP implementation **has not been confirmed on real hardware**. The command loop sends without waiting for PLC acknowledgements; the program therefore cannot confirm that the PLC received or accepted a command, nor detect PLC fault / E-stop state from responses. Verify PLC firmware protocol compatibility, interlocks, checksum, and supported combined-motion cases before moving a real vehicle. The program does not override physical safety circuits; failed UDP delivery cannot guarantee a stop.

## Files

```text
config/teleop.conf          PLC IP / UDP ports / max velocities
include/gamepad.hpp         USB gamepad class
src/gamepad.cpp             Controller and stick parsing
include/plc_udp.hpp         UDP transport class
src/plc_udp.cpp             PLC packet encoding and reply checks
src/teleop_main.cpp         Configuration, 50 Hz control, safety gates
src/main.cpp                USB-only diagnostic executable
tests/plc_packet_test.cpp   Local protocol serialization tests
```

No internal source protocol document or document excerpts are included in this repository.
