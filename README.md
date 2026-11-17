# Xbox Wireless USB Receiver Test

A standalone C++11/CMake diagnostic program for the USB wireless receiver. **Primary test target: x86_64 Ubuntu**; the same source can later be cross-compiled for EPC3562 (ARM Linux).

- Receiver VID:PID: `2345:e053`
- Vendor-specific interface: `0`
- Interrupt IN endpoint: `0x82` (32-byte maximum)
- No ROS, xpad, or `/dev/input/js0` required
- Prints changing raw USB reports and detects button transitions for **standard** 20-byte `00 14` Xbox 360-style input packets

**Important:** The wireless receiver's proprietary report format has not been verified. If the input packets do not match standard `00 14` format, the program prints their raw bytes but does not guess button mappings.

## Dependencies

Linux usbfs, CMake >= 3.10, C++11 compiler, pkg-config, and libusb-1.0 headers/library.

On Debian/Ubuntu, if needed:

```bash
sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev
```

## Build and run

```bash
git clone https://github.com/SeekyChan/xbox-wireless-usb.git
cd xbox-wireless-usb
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
lsusb -d 2345:e053
./build/xbox_usb_test
```

Press buttons to inspect events, and move the left/right sticks to view decoded LX/LY/RX/RY values (-32768..32767), plus normalized values (-1..+1). Ctrl+C exits.\n\nFor a quieter joystick test, suppress raw hex dumps:\n\n```bash\nsudo ./build/xbox_usb_test --no-raw\n```\n\nOnly one stick axis should change significantly when moving that stick horizontally or vertically. Check signs on your actual hardware; do not assume vehicle steering orientation from USB axis signs.

Optional:

```bash
./build/xbox_usb_test --vid 0x2345 --pid 0xe053 --interface 0 --endpoint 0x82
./build/xbox_usb_test --no-raw
./build/xbox_usb_test --help
```

For x86_64 Ubuntu, use the native build steps above. No cross-toolchain is required.\n\nCross-compiling: pass `-DCMAKE_TOOLCHAIN_FILE=/path/to/toolchain.cmake`, and make sure pkg-config resolves **ARM-target** libusb.

## USB permissions

Check current bus/device using `lsusb`, then inspect `/dev/bus/usb/BBB/DDD`. For development, use a dedicated group:

```bash
sudo groupadd -f usbgamepad
sudo usermod -aG usbgamepad "$USER"
printf '%s\n' 'SUBSYSTEM=="usb", ATTR{idVendor}=="2345", ATTR{idProduct}=="e053", GROUP="usbgamepad", MODE="0660"' | sudo tee /etc/udev/rules.d/99-usbgamepad.rules
sudo udevadm control --reload-rules
```

Replug receiver and log out/in so group membership takes effect.

## Project structure

```text
CMakeLists.txt
README.md
.gitignore
src/main.cpp
```

## Troubleshooting

- Cannot open: check receiver enumeration, permissions, and current bus/device number.
- Claim failed: interface may be claimed by another process or kernel driver.
- No named buttons: inspect RAW changes; the wireless packet framing can differ.
- The program only claims vendor-specific interface 0; interface 1 is keyboard HID.

This is a diagnostic utility, not a safety-certified AMR controller. Before integrating vehicle motion, require deadman, link-loss timeout, and zero-velocity failsafe.

---

## USB joystick to PLC UDP (experimental)

**Primary platform: x86_64 Ubuntu, no ROS required.**

Architecture:

- Gamepad: include/gamepad.hpp, src/gamepad.cpp. USB 2345:e053, interface 0, endpoint 0x82, parses 20-byte Xbox-style input report.
- PlcUdp: include/plc_udp.hpp, src/plc_udp.cpp. Full PLC navigation packet encoding, UDP request/reply, checksum verification.
- xbox_plc_teleop: src/teleop_main.cpp. 50Hz control loop, velocity mapping, timeout watchdog, RB hold-to-run.

Protocol basis: uploaded document "非磁导类_ROS运控+单片机或PLC交互规范_标准版", version V1.2 dated 2026-10-22, sections 4, 5, 6.1, 6.2.1.1.

PLC navigation endpoint is UDP port **1101** (1103 controls actuators; unused here). Document example: PLC 192.168.2.69, IPC 192.168.2.110. Check actual installation before live use.

A navigation packet has 24-byte header, 24-byte body, and a checksum. Header includes FAFF marker, version 2.1, sequence, sender IPv4, function F000, body length, reserved bytes. Body includes Vx, Vy, W, motion type, voice, obstacle area/state, fault flags, speed enable, charge, shield bits, compound command, HMI, localization, QR and brake fields. Speeds are signed 16-bit in units of 0.001 m/s or 0.001 rad/s. Multi-byte fields little-endian; sender IPv4 stored octet by octet.

**Protocol verification blocker:** the document describes checksum as sum of bytes but does not explicitly specify the checksum width. Current code assumes a **single trailing byte** (sum modulo 256). Verify using an actual PLC capture or firmware before enabling physical control. Other site-specific fields (obstacle zone 0, QR 0, brake control 0, localization score 0) also require validation. The implementation is a prototype, NOT verified to work with a real PLC.

### Build and unit test

    sudo apt update
    sudo apt install -y build-essential cmake pkg-config libusb-1.0-0-dev
    git clone https://github.com/SeekyChan/xbox-wireless-usb.git
    cd xbox-wireless-usb
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j$(nproc)
    ctest --test-dir build --output-on-failure

### Dry-run (no UDP packets leave the computer)

    sudo ./build/xbox_plc_teleop

Prints normalized LY/RX, hold-to-run RB, output vx/wz and enable. Default speed level is 1 (0.1m/s max linear, 0.2rad/s max angular). LY up -> forward; right stick RX left -> counter-clockwise; deadzone 0.08. **Hold RB** to enable the nonzero output in dry-run. Release RB, unplug USB or stop input updates for 250ms to force zero output.

    sudo ./build/xbox_plc_teleop --speed-level 3

### Live PLC mode: only after bench verification

A **physical emergency-stop**, separate PLC communications-loss watchdog, safe unoccupied test zone and supervision are required. Stop the existing ROS/IPC control process before testing; do not allow multiple writers to PLC UDP 1101. Verify checksum, full packet, motion types, and all safety-critical body fields with PLC firmware and actual captured responses; don't guess.

The current response decoder validates framing and checksum but **does not yet inspect PLC run-state, manual/auto or fault flags**. This limits live mode to controlled integration testing, not production vehicle operation.

    sudo ./build/xbox_plc_teleop --plc-ip 192.168.2.69 --speed-level 1 --live --confirm-motion

Live mode requires explicit arguments, valid recent PLC replies (<400ms), fresh gamepad input (<250ms), and holding RB. It starts with zero/disabled commands. Ctrl+C sends repeated zero/disabled packets, but UDP does not guarantee delivery.

The original xbox_usb_test remains available for button/raw packet diagnostics.
