# RealMan WHJ + Kinco 混合控制速查手册

> 现场调试快速参考 - 解决 CAN 总线干扰问题

---

## 🔥 关键要点 (TL;DR)

| 问题 | 解决方案 |
|------|---------|
| WHJ 通信超时/失败 | 启用 `filter_canfd_only=True` |
| Kinco 干扰 | WHJ 走 CAN FD，Kinco 走标准 CAN，用帧类型隔离 |
| 终端电阻 | ZLG 内部有 120Ω；**Kinco SW4=OFF**；WHJ 不用接 |

---

## 🔌 硬件接线检查清单

```
[ZLG CAN FD 100U-mini] ======== [WHJ Motor] ======== [Kinco Motor]
        |                             |                      |
     CAN_H                       CAN_H                  CAN_H
     CAN_L                       CAN_L                  CAN_L
     GND                         GND                    GND
        |                             |                      |
   内部120Ω(软件开)              不并电阻              SW4=OFF
```

### 拨码开关设置

**Kinco 电机 (关键!)**:
```
SW1-SW3: 设置 CAN ID (根据实际)
SW4: OFF ← 必须关闭内部 120Ω 终端电阻
```

**WHJ 电机**:
- 无需设置终端电阻
- CAN ID 通过软件/固件设置

---

## 💻 软件使用

### 1. 单 WHJ 电机控制 (推荐)

```bash
python motion_controller.py 7
```

> 默认启用 CAN FD 过滤，自动忽略 Kinco 干扰

### 2. 混合模式测试

```bash
python examples/mixed_mode_demo.py
```

### 3. 调试 CAN 总线

```bash
python tools/debug_can.py
```

---

## 🐛 故障排查流程

### 症状：WHJ 命令超时/无响应

```
Step 1: 检查硬件
  └── ZLG CAN 设备是否识别？
  └── 120Ω 终端电阻是否正确？(两端各一个)
  └── Kinco SW4 是否为 OFF？
  └── 线缆是否松动？

Step 2: 单独测试 WHJ
  └── 断开 Kinco，只连 WHJ
  └── python motion_controller.py <id>
  └── 如果正常 → 确认是 Kinco 干扰

Step 3: 启用 CAN FD 过滤 (已默认启用)
  └── 确认使用 motion_controller.py (不是旧版)
  └── 或手动设置 filter_canfd_only=True

Step 4: 验证帧类型隔离
  └── 运行 debug_can.py
  └── 确认能看到 WHJ 的 CAN FD 帧
  └── 确认 Kinco 发送的是标准 CAN 帧
```

### 症状：Kinco 不响应

```
Step 1: 检查 Kinco 拨码开关
  └── SW4 必须为 OFF (禁用内部 120Ω)
  └── SW1-SW3 设置正确的 CAN ID

Step 2: 检查通信参数
  └── 波特率: 1Mbps (标准 CAN)
  └── Node ID 是否匹配命令中的 ID?

Step 3: 单独测试 Kinco
  └── 断开 WHJ，只连 Kinco
  └── python examples/basic_kinco.py
```

---

## 📋 关键代码片段

### 启用 CAN FD 过滤

```python
from drivers.motor_control import MotorController

# 方式1: 使用 motion_controller.py (已默认启用)
# 方式2: 手动启用过滤
motor = MotorController(driver, motor_id=7, filter_canfd_only=True)
```

### 初始化混合模式

```python
from core import ZlgCanDriver, ZCANDeviceType

driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)

# 混合模式: CAN FD 设备用 5Mbps 数据段
#           标准 CAN 设备只用 1Mbps 仲裁段
driver.init_mixed_mode(
    arbitration_bps=1000000,  # 1Mbps - WHJ 和 Kinco 都用这个
    data_bps=5000000,         # 5Mbps - 只有 WHJ 用这个
    internal_resistance=True  # 启用 ZLG 内部 120Ω
)
```

### 发送命令 (区分设备类型)

```python
# 给 WHJ 发 CAN FD 帧
driver.send_canfd(
    can_id=0x07,              # WHJ 电机 ID
    data=whj_command,
    bitrate_switch=True       # 数据段用 5Mbps
)

# 给 Kinco 发标准 CAN 帧
driver.send_can(
    can_id=0x601,             # Kinco SDO RX ID
    data=kinco_command        # 最多 8 字节
)
```

### 接收时过滤

```python
# 只接收 CAN FD 帧 (WHJ 响应)
frame = driver.receive(frame_type="CANFD")

# 只接收标准 CAN 帧 (Kinco 响应)
frame = driver.receive(frame_type="CAN")

# 接收所有帧
frame = driver.receive(frame_type="any")
```

---

## 🎯 常见误区

| 误区 | 真相 |
|------|------|
| "需要复杂的 ID 过滤" | ❌ 不需要，用 CAN FD 帧类型即可隔离 |
| "WHJ 需要并 120Ω 电阻" | ❌ WHJ 不需要额外电阻 |
| "Kinco SW4 要 ON" | ❌ 必须 OFF，总线两端已有终端电阻 |
| "需要两根 CAN 线" | ❌ 可以共用总线，用帧类型区分 |
| "filter_canfd_only 影响发送" | ❌ 只影响接收过滤，发送不受影响 |

---

## 🔧 调试命令

```bash
# 查看当前 CAN 总线状态
cd examples/python
python -c "
from core import ZlgCanDriver, ZCANDeviceType
d = ZlgCanDriver()
d.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
d.init_mixed_mode()
import time
for i in range(10):
    status = d.get_mixed_mode_status()
    print(f'CAN:{status[\"can\"]:3d}  CANFD:{status[\"canfd\"]:3d}')
    time.sleep(0.5)
d.close()
"
```

---

## 📞 快速检查表

启动前确认：

- [ ] ZLG USB 灯正常 (绿色/蓝色)
- [ ] WHJ 电机上电 (有提示音)
- [ ] Kinco SW4 = **OFF**
- [ ] 终端电阻 = **两个** (ZLG 内部 + 总线另一端)
- [ ] 使用 `motion_controller.py` (不是旧版脚本)

通信测试：

- [ ] 单独 WHJ 能通信
- [ ] 单独 Kinco 能通信
- [ ] 混合模式下 WHJ 仍能通信
- [ ] 混合模式下 Kinco 仍能通信

---

*最后更新: 2024*
