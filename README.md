# RealMan WHJ Joint Motor Driver

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20Windows-lightgrey.svg)](.)

RealMan 睿尔曼 WHJ 系列关节电机驱动库，支持 CAN FD 通信，提供 C++ 和 Python API。

## 功能特性

- **跨平台**: Linux (SocketCAN) / Windows (ZLG CAN FD)
- **CAN FD 高速通信**: 支持 5 Mbps 数据速率
- **现代 C++17**: 类型安全的 API
- **Python 支持**: 纯 Python 实现，无需编译
- **ROS2 集成**: 原生 ROS2 节点支持

## 支持的硬件

| 型号 | 关节类型 | 电流分辨率 |
|------|----------|------------|
| J3   | Joint 03 | 1 mA/LSB   |
| J14  | Joint 10 | 1 mA/LSB   |
| J17  | Joint 30 | 1 mA/LSB   |
| J20  | Joint 60 | 2 mA/LSB   |
| J25  | Joint 120| 2 mA/LSB   |

**支持的 CAN 设备**: ZLG USBCANFD-100U-mini / 100U / 200U / 800U

---

## 快速开始 (Python)

### 1. 环境要求

- Python 3.8+
- ZLG CAN FD 设备 (如 USBCANFD-100U-mini)
- ZLG 设备驱动已安装

### 2. 测试电机通信

```bash
cd examples/python

# 测试 CAN 设备连接
python zlgcan_driver.py

# 测试电机通信 (默认 ID=1)
python test_whj_motor.py

# 指定电机 ID
python test_whj_motor.py 2
```

### 3. 在代码中使用

```python
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, WorkMode

# 初始化 CAN 设备
driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# 发送读位置命令
cmd = WHJProtocol.build_read_frame(motor_id=1, reg=Register.CUR_POSITION_L, count=2)
driver.send(can_id=1, data=cmd)

# 接收响应
response = driver.receive(timeout_ms=100)
if response:
    print(f"RX: ID=0x{response.can_id:03X}, Data={response.data.hex()}")

driver.close()
```

---

## 快速开始 (C++)

### 编译

```bash
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=ON
cmake --build . --config Release
```

### 运行示例

```bash
# Windows (ZLG CAN)
./Release/example_basic.exe

# Linux (SocketCAN)
./example_basic can0

# 仿真模式 (无需硬件)
./example_basic sim
```

---

## 项目结构

```
RealMan_Motor_Joint/
├── examples/
│   ├── cpp/                    # C++ 示例
│   └── python/                 # Python 驱动和示例
│       ├── zlgcan_driver.py    # ZLG CAN FD 驱动
│       ├── whj_protocol.py     # WHJ 协议实现
│       └── test_whj_motor.py   # 电机测试脚本
├── include/realman_whj/        # C++ 头文件
├── src/                        # C++ 源代码
├── third_party/
│   └── zlgcan/                 # ZLG CAN 驱动库
│       ├── x64/zlgcan.dll      # 64位 DLL
│       ├── x86/zlgcan.dll      # 32位 DLL
│       └── include/*.h         # C/C++ 头文件
├── CMakeLists.txt
└── README.md
```

---

## CAN FD 参数配置

WHJ 电机默认参数：

| 参数 | 值 |
|------|-----|
| 仲裁段波特率 | 1 Mbps |
| 数据段波特率 | 5 Mbps |
| 帧格式 | ISO CAN FD |
| 终端电阻 | 120Ω (需使能) |

---

## Python API 参考

### ZlgCanDriver (底层驱动)

```python
driver = ZlgCanDriver(dll_path=None)  # 自动检测 DLL
driver.open(device_type, device_index=0, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
driver.send(can_id=0x01, data=b'\x01\x10\x00\x01')
frame = driver.receive(timeout_ms=100)
driver.close()
```

### WHJProtocol (协议层)

```python
# 读寄存器
cmd = WHJProtocol.build_read_frame(motor_id=1, reg=Register.CUR_POSITION_L)

# 写寄存器
cmd = WHJProtocol.build_write_frame(motor_id=1, reg=Register.SYS_ENABLE_DRIVER, value=1)

# 设置位置
cmds = WHJProtocol.build_set_target_position(motor_id=1, position_deg=90.0)

# 解析响应
state = WHJProtocol.parse_state_response(motor_id=1, data=response.data)
```

### WHJMotorController (高层控制)

```python
from test_whj_motor import WHJMotorController

controller = WHJMotorController(driver)
controller.enable_motor(1, enable=True)
controller.set_work_mode(1, WorkMode.POSITION_MODE)
controller.set_target_position(1, 90.0)
state = controller.get_joint_state(1)
```

---

## 通信协议

### CAN ID

- **请求 ID**: `motor_id` (1-30)
- **响应 ID**: `motor_id + 0x100`

### 命令格式

**读寄存器** (3 bytes):
```
Byte 0: 0x01 (CMD_READ)
Byte 1: 寄存器地址
Byte 2: 读取数量
```

**写寄存器** (4 bytes):
```
Byte 0: 0x02 (CMD_WRITE)
Byte 1: 寄存器地址
Byte 2: 数据低字节
Byte 3: 数据高字节
```

### 重要寄存器

| 寄存器 | 地址 | 说明 |
|--------|------|------|
| SYS_ENABLE_DRIVER | 0x0A | 使能驱动 (0/1) |
| SYS_ERROR | 0x04 | 错误代码 |
| SYS_VOLTAGE | 0x05 | 电压 (0.01V/LSB) |
| SYS_TEMP | 0x06 | 温度 (0.1°C/LSB) |
| CUR_POSITION_L/H | 0x14/0x15 | 当前位置 |
| TAG_POSITION_L/H | 0x36/0x37 | 目标位置 |
| TAG_WORK_MODE | 0x30 | 工作模式 |

### 数据转换

| 参数 | 分辨率 | 转换公式 |
|------|--------|----------|
| 位置 | 0.0001° | `degrees = raw * 0.0001` |
| 目标速度 | 0.002 RPM | `rpm = raw * 0.002` |
| 实际速度 | 0.02 RPM | `rpm = raw * 0.02` |
| 电压 | 0.01 V | `volts = raw * 0.01` |
| 温度 | 0.1 °C | `celsius = raw * 0.1` |

---

## Linux SocketCAN 配置

```bash
# 加载模块
sudo modprobe can can_raw

# 配置 CAN FD 接口
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# 查看状态
ip -details link show can0

# 监听报文
candump can0
```

---

## 故障排除

### CAN 灯不亮

1. 检查 USB 连接
2. 确认 ZCANPro 能识别设备
3. 检查 DLL 架构与 Python 匹配 (64位 Python 用 x64 DLL)

### 电机无响应

1. 确认终端电阻已使能
2. 检查电机 ID 是否正确
3. 确认波特率设置 (1M/5M)
4. 检查电机是否上电

### DLL 加载失败

```python
# 手动指定 DLL 路径
driver = ZlgCanDriver(dll_path="D:/path/to/zlgcan.dll")
```

---

## 参考文档

- [RealMan WHJ 开发文档](https://develop.realman-robotics.com/joints/CANFD/explanation/)
- ZLG CAN 二次开发文档 (见 `third_party/zlgcan/`)

---

## 许可

MIT License - 详见 [LICENSE](LICENSE)
