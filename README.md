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
max_vy_mps=0.0
max_wz_radps=0.2
```

Move velocity channels through `Gamepad` and `PlcUdp`:

| Controller | Output | Max | Raw integer at max |
| --- | --- | --- | --- |
| Left stick vertical (LY) | Vx, forward / backward | ±0.2 m/s | ±200 |
| Left stick horizontal (LX) | Unused, Vy always zero | 0 m/s | 0 |
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


---

## 第一阶段：普通 USB 摄像头的人体检测与目标跟踪（不控制车辆）

**这个功能只读取摄像头、显示检测框与临时跟踪 ID。它不引用 PlcUdp、不打开 PLC UDP 端口、不会发布速度指令。** 已有手柄和 PLC 程序保持不变。

### Ubuntu 安装和运行

~~~bash
sudo apt update
sudo apt install -y libopencv-dev v4l-utils
cd ~/xbox-wireless-usb
git pull origin main
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
v4l2-ctl --list-devices
./build/person_vision
~~~

如设备不是默认的 /dev/video0，尝试：

~~~bash
./build/person_vision --camera 2
~~~

如果检测较慢，降低检测频率：

~~~bash
./build/person_vision --width 640 --height 480 --detect-every 5
~~~

如果配置时显示 OpenCV not found，请确认 libopencv-dev 已安装，再重新运行 CMake 配置。请使用普通用户运行窗口程序（不用 sudo），并检查 /dev/video* 权限及设备是否被浏览器等进程占用。

### 操作方法

- 尽量让全身完整进入画面。采用 OpenCV 自带的 HOG/SVM 行人检测器检测人物，并画出边框。
- 跨检测帧关联人体框，显示临时编号 ID 1 / ID 2。可同时检测多名行人。
- center_err_x 是人物中心相对画面中心的水平偏差（约 -1 到 +1），仅为后续规划预留，**不是实际距离**。
- 鼠标点击检测框：高亮该目标；键盘 c 清除；q 或 Esc 退出。
- 顶部固定显示 CAMERA-ONLY / NO UDP，确认不会驱动车辆。

### 当前局限

本阶段采用 **HOG/SVM**，优点是 Ubuntu 安装 OpenCV 后无需再下载模型权重。它对完整站立行人的效果通常比近距离上半身或遮挡场景更好，对不同姿态和低光环境可能漏检。跟踪是**检测框关联**，并非人脸识别或可靠的身份重新识别；遮挡或快速运动可能导致 ID 变化。非检测帧仅显示最近有效检测框，并没有做光流预测。

**这一阶段不估计距离，不发送 UDP、也不运行自动跟随控制。** 后续可改用 ONNX 人体检测模型、深度/测距模块和避障机制，再隔离验证运动控制。

新增源码：

~~~text
include/person_tracker.hpp     C++ 关联检测框与分配临时 ID
src/person_vision_main.cpp     OpenCV 摄像头、检测和可视化
tests/person_tracker_test.cpp  无摄像头的跟踪单元测试
~~~


---

## 第二代视觉：双腿关键点识别和目标跟踪（推荐）

之前的 person_vision 使用传统 HOG，近距离/局部身体识别效果有限。新增 person_leg_vision 使用姿态估计 ONNX 模型，识别双腿关键点，特别是左右膝盖 (13,14)，并绘制脚踝 (15,16)、髋部 (11,12) 参考骨架。它使用两个膝盖的中点作为目标位置、绘制骨架，并维持临时目标 ID。

**独立视觉程序，不包含任何 PLC / UDP 控制连接。不会让小车移动。** 原有手柄/PLC 程序不受影响。

### 开源参考

- Ultralytics YOLO Pose: https://github.com/ultralytics/ultralytics
- monocular_person_following: https://github.com/koide3/monocular_person_following
- C++ DeepSORT / ByteTrack: https://github.com/shaoshengsong/DeepSORT

实现基于公开的 COCO 17-keypoint 定义，没有将上述项目源代码或权重复制到本仓库。

**许可注意**：Ultralytics 官方模型与代码默认使用 AGPL-3.0 / 企业授权机制。开发封闭源码商业产品前，应核实许可义务。仓库不包含模型权重。

### 1. Ubuntu 编译

~~~bash
cd ~/xbox-wireless-usb
git pull origin main
sudo apt update
sudo apt install -y libopencv-dev python3-venv
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
~~~

应生成 build/person_leg_vision。OpenCV 必须包含 dnn 模块；旧版 OpenCV DNN 可能不支持部分 ONNX 算子。

### 2. 一次性导出人体姿态模型

使用独立 Python 虚拟环境导出模型，正常程序运行仍然是 C++：

~~~bash
cd ~/xbox-wireless-usb
python3 -m venv .venv-pose
source .venv-pose/bin/activate
python -m pip install -U pip
python -m pip install ultralytics onnx
mkdir -p models
yolo export model=yolo11n-pose.pt format=onnx imgsz=640 nms=False opset=12
mv yolo11n-pose.onnx models/
deactivate
~~~

模型应在 models/yolo11n-pose.onnx。不要提交权重到 Git；.gitignore 已忽略 models/*.onnx。

### 3. USB 摄像头运行

~~~bash
./build/person_leg_vision
~~~

如摄像头不是 /dev/video0，尝试：

~~~bash
./build/person_leg_vision --camera 2
~~~

CPU 慢时减少检测频率：

~~~bash
./build/person_leg_vision --every 3
~~~

窗口会显示绿色人体框、两腿骨架、临时 ID、LEGS OK 或 LEGS NOT VISIBLE。左右腿各至少有一个高置信度的膝/踝点才标注两腿中心；不足时不会假装双腿已检出。

- 鼠标单击人物框：高亮选中目标（黄色）。
- 键盘 c：清除选择；q / Esc：退出。
- 选中目标后终端打印 normalized_x（腿部中心的水平归一化位置）。这不是实际米制距离，不会发运动 UDP。

### 4. 注意事项

这种模型仍然以人体姿态为目标，**不是只看鞋子就能识别对应人的模型**。人离得很近、只拍到小腿、被遮挡或多人交错时仍可能漏检或交换 ID；本程序没有身份 ReID，没有运动预测，更不具备碰撞保护。建议先在多个距离和不同光照下观察两膝、两脚踝的实际输出，再考虑升级跟踪器。

当前只支持输出形状为 [1,56,N] 或 [1,N,56] 的 YOLO11 Pose ONNX，须以 nms=False 导出。若 OpenCV DNN 报 ONNX import 错误，可根据报错选择更新 OpenCV 或改用 ONNX Runtime。

新增文件：

~~~text
src/person_leg_vision_main.cpp   ONNX Pose + 双腿关键点显示
include/person_tracker.hpp       复用临时 ID 关联
models/                          本地模型，不入 Git
~~~


### 更新：仅使用膝盖 13/14 的中点进行转向预览

YOLO Pose 关键点 13 与 14 分别对应左右膝盖。只有两点均达到置信度阈值时才计算中点与水平误差。选定目标后，摄像头程序只预览 `Wz = clamp(-0.2 * error_x, -0.2, +0.2)`，中心死区为 0.08；`Vx_preview=0`、`Vy_preview=0`。**不会发送 UDP**。这是因为单目图像中膝盖中点只能决定目标方位，不能测量安全的车人间距；缺少可靠测距与避障前，不让视觉控制自动前进。已有手柄控制只保留 `Vx` 和 `Wz`，`Vy=0`，左摇杆横向操作不再下发。
