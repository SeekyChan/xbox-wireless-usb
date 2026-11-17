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
