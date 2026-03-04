# RealMan WHJ Motor - Python 运动控制驱动

用于 ZLG USBCANFD-100U-mini (或其他 ZLG CAN FD 设备) 控制 RealMan WHJ 关节电机的 Python 驱动。

---

## 📁 交付文件（共 6 个）

| 文件 | 说明 | 用途 |
|------|------|------|
| `zlgcan_driver.py` | ZLG CAN FD 设备驱动 | 底层 CAN 通信 |
| `whj_protocol.py` | WHJ 通信协议 | 寄存器定义、帧构造 |
| `motor_control.py` | 基础电机控制类 | 寄存器读写、状态获取 |
| `motion_controller.py` | **主要使用：运动控制器** | 带轨迹规划的位置控制 |
| `encoder_recovery.py` | **故障恢复工具** | 编码器错误修复、零点标定 |
| `query_motor_state.py` | 状态查询工具 | 查看电机状态、诊断问题 |

---

## 🚀 快速开始

### 1. 硬件连接

```
PC (USB) <---> ZLG USBCANFD-100U-mini <---> WHJ Motor (CAN FD)
                                              |
                                         120Ω 终端电阻
```

### 2. 安装依赖

```bash
pip install pyzlgcan  # 如果使用 ZLG 官方库
# 或直接使用内置的 zlgcan_driver.py（需 ZCANPro 驱动已安装）
```

### 3. 使用方法

#### 查询电机状态

```bash
python query_motor_state.py <电机ID>

# 示例
python query_motor_state.py 1
```

输出：
```
============================================================
RealMan WHJ Motor State Query
============================================================
Motor ID: 1

[1] System Information:
  Model:           J20 (Joint 60)
  Firmware:        v1.2
  Voltage:         24.50 V
  Temperature:     35.2 °C

[2] Current State:
  Position:        45.23°
  Speed:           0.00 RPM
  Current:         50 mA
  Error Code:      0x0000
  Status:          OK (No errors)
```

#### 运动控制（主要使用）

```bash
python motion_controller.py <电机ID>

# 示例
python motion_controller.py 1
```

交互命令：
| 命令 | 说明 | 示例 |
|------|------|------|
| `m <位置>` | 移动到指定角度（带平滑轨迹） | `m 90` |
| `r` | 读取当前位置 | - |
| `e` | 使能电机 | - |
| `d` | 禁用电机 | - |
| `q` | 退出 | - |

**特点**：
- 自动梯形速度规划，防止电机过冲
- 自动使能和模式切换
- 实时位置反馈

#### 在自己的代码中使用

```python
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from motion_controller import SmoothMotorController, MotionProfile

# 初始化 CAN
 driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# 创建运动控制器
profile = MotionProfile(
    max_velocity=180.0,      # 最大速度 °/s
    max_acceleration=360.0,  # 最大加速度 °/s²
    max_deceleration=360.0
)
motor = SmoothMotorController(driver, motor_id=1, profile=profile)

# 使用
motor.initialize()                    # 初始化连接
motor.enable(True)                    # 使能电机
motor.move_to_position(90.0)          # 移动到 90°

# 关闭
driver.close()
```

---

## 🔧 编码器故障处理

### 问题
- 断电后手动转动过电机
- 电机报错无法使能
- 错误码：`Encoder error` (0x0020) 或 `Multi-turn counter lost` (0x8000)

### 解决

```bash
python encoder_recovery.py <电机ID>
```

操作流程：
1. 脚本自动检查错误
2. 尝试清除错误
3. 如错误持续，提示将电机转到**机械零点位置**
4. 按 Enter 确认后，设置当前位置为 0°
5. **保存到 Flash**（永久生效）
6. 重新启用电机
7. **断电重启电机**验证

⚠️ **重要**：保存 Flash 前必须先禁用电机！脚本已自动处理。

---

## 📋 故障速查

| 现象 | 可能原因 | 解决 |
|------|----------|------|
| 电机无法使能，报错编码器 | 断电后转动过电机 | `encoder_recovery.py` |
| 位置偏移 | 多圈计数器丢失 | 重新设零点 + 保存 Flash + 断电重启 |
| 通信无响应 | CAN 线/供电问题 | 检查接线和终端电阻 |
| 运动不平稳 | 速度/加速度过大 | 降低 MotionProfile 参数 |

---

## ⚙️ 技术参数

### CAN FD 配置
- 仲裁段波特率：1 Mbps
- 数据段波特率：5 Mbps
- 终端电阻：120Ω（自动使能）

### 数据分辨率
- 位置：0.0001°/LSB
- 速度：0.02 RPM/LSB
- 电流：1 或 2 mA/LSB（取决于电机型号）
- 电压：0.01 V/LSB
- 温度：0.1 °C/LSB

### 关键寄存器
| 寄存器 | 地址 | 功能 |
|--------|------|------|
| SYS_ENABLE_DRIVER | 0x0A | 使能驱动 |
| SYS_ERROR | 0x04 | 错误代码 |
| SYS_CLEAR_ERROR | 0x0F | 清除错误 |
| SYS_SET_ZERO_POS | 0x0E | 设置零点 |
| SYS_SAVE_TO_FLASH | 0x0C | 保存配置 |
| TAG_WORK_MODE | 0x30 | 工作模式 (3=位置) |
| TAG_POSITION_L/H | 0x36/37 | 目标位置 |

---

## 📂 其他文件

`tools/` 文件夹包含调试和测试工具（非必须）：
- `debug_can.py` - CAN 总线调试
- `reset_can_device.py` - 重置 CAN 设备
- `test_whj_motor.py` - 完整通信测试
- `simple_position_control.py` - 简单位置控制
- `position_sine.py` - 正弦波运动示例

---

## 📞 支持

- RealMan WHJ 文档：https://develop.realman-robotics.com/joints/CANFD/explanation/
- 电机型号：J3, J14(J10), J17(J30), J20(J60), J25(J120)
