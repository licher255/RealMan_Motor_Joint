# RealMan Motor Joint - Python Driver

Python 驱动库，用于控制 RealMan WHJ 系列关节电机和 Kinco 伺服电机（通过 CAN 总线）。

## 目录结构

```
examples/python/
├── README.md                    # 本文档
├── core/                        # 核心模块
│   ├── __init__.py
│   ├── zlgcan_driver.py        # ZLG CAN FD 设备驱动
│   └── protocol/               # 通信协议
│       ├── __init__.py
│       ├── whj_protocol.py     # RealMan WHJ 电机协议 (CAN FD)
│       ├── kinco_protocol.py   # Kinco 基础协议 (RPDO/TPDO方式)
│       └── kinco_canopen.py    # Kinco CANopen SDO 协议
├── drivers/                     # 电机驱动模块
│   ├── __init__.py
│   ├── base_driver.py          # 基础驱动抽象类
│   ├── whj_motor_control.py    # WHJ 基础电机控制
│   ├── whj_motion_controller.py # WHJ 轨迹规划控制器 (推荐)
│   ├── whj_driver.py           # WHJ 专用驱动
│   ├── kinco_driver.py         # Kinco 驱动 (RPDO/TPDO方式)
│   └── kinco_pdo_driver.py     # Kinco SDO 驱动 (推荐)
├── test_kinco_simple.py        # Kinco 基础测试
└── test_kinco_pdo.py           # Kinco SDO 功能测试
```

## 快速开始

### WHJ 电机控制（带轨迹规划）

```bash
python -m drivers.whj_motion_controller <motor_id>
```

示例：
```bash
python -m drivers.whj_motion_controller 7
```

交互命令：
| 命令 | 说明 |
|------|------|
| `m <pos>` | 移动到指定位置（度） |
| `e` | 使能电机 |
| `d` | 禁用电机 |
| `c` | 清除错误 |
| `r` | 读取当前位置 |
| `s` | 显示状态 |
| `q` | 退出 |

### Python API 使用

```python
from core import ZlgCanDriver, ZCANDeviceType
from drivers import SmoothMotorController, MotionProfile

# 初始化 CAN 设备
driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# 创建运动控制器（带 CAN FD 过滤，自动忽略 Kinco 标准 CAN 帧）
profile = MotionProfile(
    max_velocity=1800.0,      # 最大速度 °/s
    max_acceleration=720.0,   # 最大加速度 °/s²
    max_deceleration=720.0    # 最大减速度 °/s²
)
motor = SmoothMotorController(driver, motor_id=7, profile=profile)

# 初始化并控制
motor.initialize()
motor.enable(True)
motor.move_to_position(90.0)  # 平滑移动到 90°
```

### Kinco 电机控制 (PDO 方式 - 推荐)

```python
from core import ZlgCanDriver, ZCANDeviceType
from drivers import KincoPDODriver

driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# 创建 PDO 驱动 (按照操作指南)
motor = KincoPDODriver(driver, node_id=1)

# 初始化
motor.initialize()  # NMT启动 + 使能 + 绝对位置模式

# 移动到 90 度 @ 50 rpm
motor.move_to_degree(90.0, velocity_rpm=50.0)

# 读取状态
motor.poll_state(duration=0.5)
motor.print_state()

# 设置原点
motor.set_origin()

# 禁用
motor.disable()
```

### Kinco 电机控制 (SDO 方式)

```python
from core import ZlgCanDriver, ZCANDeviceType
from drivers import KincoSDODriver

driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# 创建 SDO 驱动 (基于 Kinco FD1X5 手册第7章)
motor = KincoSDODriver(driver, node_id=1)
motor.initialize()
motor.move_to_degree(90.0, wait_complete=True)
motor.print_status()
```

## 硬件接线

### 终端电阻配置

| 设备 | 终端电阻 | 说明 |
|------|----------|------|
| ZLG USBCANFD-100U-mini | 内置 120Ω | 软件启用，无需外接 |
| RealMan WHJ 电机 | 无需额外电阻 | 驱动内部处理 |
| Kinco 电机 | SW4 = OFF | 禁用内部 120Ω 终端电阻 |

**重要**：CAN 总线两端必须有且仅有 2 个 120Ω 终端电阻。

### 接线图

```
ZLG CAN FD 100U-mini      WHJ Motor              Kinco Motor
       |                      |                        |
    [CAN_H]----------------[CAN_H]------------------[CAN_H]
       |                      |                        |
    [CAN_L]----------------[CAN_L]------------------[CAN_L]
       |                      |                        |
    [GND]------------------[GND]--------------------[GND]
       
   内置 120Ω                无电阻              SW4=OFF (无电阻)
   (软件启用)
```

## CAN FD 过滤（混合模式关键）

当 WHJ (CAN FD) 和 Kinco (标准 CAN) 共用总线时：

**问题**：Kinco 频繁发送标准 CAN 帧，会淹没接收缓冲区，导致 WHJ 命令超时。

**解决方案**：使用 CAN FD 帧过滤，只接收 CAN FD 帧，自动忽略标准 CAN 帧。

```python
# 默认启用 CAN FD 过滤
motor = SmoothMotorController(driver, motor_id=7, filter_canfd_only=True)
```

| 帧类型 | WHJ 电机 | Kinco 电机 |
|--------|----------|------------|
| 标准 CAN | ✗ 不使用 | ✓ 大量使用 |
| CAN FD | ✓ 专用 | ✗ 不支持 |

## 通信协议差异

| 特性 | WHJ 电机 | Kinco 电机 |
|------|----------|------------|
| 协议 | 自定义 CAN FD | CANopen (标准 CAN) |
| 命令 ID | 0x01 - 0x7F | 0x600 + node_id |
| 响应 ID | 命令 ID + 0x100 | 0x580 + node_id |
| 帧类型 | CAN FD (64 字节) | 标准 CAN (8 字节) |
| 数据波特率 | 5 Mbps | 1 Mbps |

## 模块说明

### core.zlgcan_driver
ZLG CAN 设备驱动，支持：
- CAN FD 模式（WHJ）
- 标准 CAN 模式（Kinco）
- 混合模式（同时支持 CAN FD + 标准 CAN）
- 帧类型过滤

### core.protocol.whj_protocol
WHJ 电机通信协议实现：
- 寄存器定义
- 命令构建/解析
- 状态转换

### core.protocol.kinco_canopen
Kinco CANopen SDO 协议实现（基于 FD1X5 手册第7章）：
- SDO 读写操作 (0x600/0x580)
- 控制字/状态字 (0x6040/0x6041)
- 错误读取和清除 (0x2601)
- 限位状态读取 (0x60FD)

### core.protocol.kinco_protocol
Kinco 基础协议实现（RPDO/TPDO 方式）

### drivers.whj_motion_controller
主运动控制器，功能：
- 梯形速度轨迹规划
- 平滑运动控制
- CAN FD 帧过滤
- 自动超时计算

### drivers.kinco_pdo_driver
**Kinco PDO 驱动（推荐 - 符合操作指南）**

| 功能 | 方法 | CAN ID | 数据格式 |
|------|------|--------|----------|
| NMT 启动 | `start_node()` | 0x000 | [01, node_id] |
| 使能+绝对模式 | `enable_absolute_mode()` | 0x201 | [01 3F 10 ...] |
| 使能+相对模式 | `enable_relative_mode()` | 0x201 | [01 3F 0F ...] |
| 禁用 | `disable()` | 0x201 | [01 06 10 ...] |
| 清除错误 | `clear_fault()` | 0x201 | [01 86 10 ...] |
| 设置原点 | `set_origin()` | 0x201 | [06 0F/1F ...] |
| 移动位置 | `move_to_degree(deg, rpm)` | 0x301 | [pos(4) + vel(4)] |
| 读取状态 | `poll_state()` | 0x181+id | TPDO1 |

### drivers.kinco_sdo_driver
**Kinco SDO 驱动（完整功能）**

| 功能 | 方法 | SDO 对象 |
|------|------|----------|
| 读实际位置 | `read_actual_position()` | 0x60630020 |
| 读实际速度 | `read_actual_velocity()` | 0x606C0020 |
| 读错误状态 | `read_error_status()` | 0x26010010 |
| 读限位状态 | `read_input_status()` | 0x60FD0020 |
| 清除错误 | `clear_fault()` | 0x60400010 |
| 使能/禁用 | `enable()` / `disable()` | 0x60400010 |
| 设置工作模式 | `set_work_mode(1)` | 0x60600008 |

### drivers.kinco_driver
Kinco 基础驱动（旧版 RPDO 方式）

## 依赖

- Python 3.8+
- ZLG CAN 设备 (USBCANFD-100U-mini 或兼容设备)
- ZLG CAN 驱动 DLL (zlgcan.dll)

## 电机 ID 参考

| 电机 | 命令 ID | 响应 ID |
|------|---------|---------|
| WHJ Joint 1 | 0x01 | 0x101 |
| WHJ Joint 2 | 0x02 | 0x102 |
| ... | ... | ... |
| WHJ Joint 7 | 0x07 | 0x107 |
| Kinco (node 1) | 0x601 | 0x581 |
| Kinco (node 2) | 0x602 | 0x582 |

## 更多信息

- 项目 README: ../../README.md
- Agent 指南: ../../AGENTS.md
