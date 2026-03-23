# RealMan Motor SDK

Python SDK for controlling RealMan WHJ series joint motors and Kinco FD1X5 servo motors via CAN FD (mixed mode support).

## Package Structure

This SDK is organized into three independent packages:

| Package | Description | Dependencies |
|---------|-------------|--------------|
| `zlg_can` | ZLG USB-CAN FD device driver | None |
| `realman_whj` | RealMan WHJ joint motor driver | `zlg_can` |
| `kinco_motor` | Kinco FD1X5 servo motor driver | `zlg_can` |

## Installation

### Install All Packages

```bash
cd python_sdk
pip install -e .
```

### Install Individual Packages

```bash
# Install only CAN driver
pip install -e src/zlg_can

# Install only WHJ driver
pip install -e src/realman_whj

# Install only Kinco driver
pip install -e src/kinco_motor
```

## Quick Start

### 1. WHJ Motor Only

```python
from zlg_can import ZlgCanDriver, ZCANDeviceType
from realman_whj import WHJMotor

with ZlgCanDriver() as can:
    can.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
    can.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    
    with WHJMotor(can, motor_id=7) as motor:
        motor.initialize()
        motor.move_to(90.0)
```

### 2. Kinco Motor Only

```python
from zlg_can import ZlgCanDriver, ZCANDeviceType
from kinco_motor import KincoMotor

with ZlgCanDriver() as can:
    can.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
    can.init_canfd(arbitration_bps=1000000)
    
    with KincoMotor(can, node_id=1) as motor:
        motor.initialize()
        motor.move_to_degree(90.0, velocity_rpm=50.0)
```

### 3. Hybrid Control (WHJ + Kinco)

```python
from zlg_can import ZlgCanDriver, ZCANDeviceType
from realman_whj import WHJMotor
from kinco_motor import KincoMotor

with ZlgCanDriver() as can:
    can.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
    can.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    
    # Both motors share the same CAN interface
    whj = WHJMotor(can, motor_id=7)
    kinco = KincoMotor(can, node_id=1)
    
    whj.initialize()
    kinco.initialize()
    
    # Move both motors
    whj.move_to(90.0)
    kinco.move_to_degree(90.0)
    
    whj.close()
    kinco.close()
```

## Directory Structure

```
python_sdk/
├── src/
│   ├── zlg_can/               # ZLG CAN FD driver
│   │   ├── __init__.py
│   │   ├── interface.py       # CANInterface, CANFrame, CANFDFrame
│   │   ├── zlg_driver.py      # ZlgCanDriver implementation
│   │   └── exceptions.py      # CANError, TimeoutError
│   │
│   ├── realman_whj/           # WHJ motor driver
│   │   ├── __init__.py
│   │   ├── core/
│   │   │   └── protocol/
│   │   │       └── whj.py     # WHJ protocol
│   │   ├── motor/
│   │   │   └── whj.py         # WHJMotor class
│   │   ├── motion/            # Trajectory planning
│   │   └── config.py          # WHJMotorConfig
│   │
│   └── kinco_motor/           # Kinco motor driver
│       ├── __init__.py
│       ├── protocol.py        # Kinco protocol
│       ├── motor.py           # KincoMotor class
│       ├── config.py          # KincoMotorConfig
│       └── exceptions.py      # KincoError
│
├── examples/
│   ├── basic_whj.py           # WHJ only example
│   ├── basic_kinco.py         # Kinco only example
│   └── hybrid_control.py      # Mixed mode example
│
├── pyproject.toml
└── README.md
```

## Supported Hardware

### CAN Devices
- ZLG USBCANFD-100U-mini
- ZLG USBCANFD-100U/200U/400U/800U

### Motors
| Motor | Protocol | CAN Type | Package |
|-------|----------|----------|---------|
| RealMan WHJ Series | Custom CAN FD | CAN FD | `realman_whj` |
| Kinco FD1X5 | CANopen PDO | Standard CAN | `kinco_motor` |

## Mixed Mode Wiring

```
ZLG CAN FD 100U-mini      WHJ Motor              Kinco Motor
       |                      |                        |
    [CAN_H]----------------[CAN_H]------------------[CAN_H]
       |                      |                        |
    [CAN_L]----------------[CAN_L]------------------[CAN_L]
       |                      |                        |
    [GND]------------------[GND]--------------------[GND]
       
   Internal 120Ω           No resistor          SW4=OFF (No resistor)
   (Software enabled)
```

**Important:**
- ZLG device: Enable 120Ω termination in software
- WHJ motor: No additional termination needed
- Kinco motor: Set SW4=OFF to disable internal 120Ω termination

## Package Details

### zlg_can

ZLG USB-CAN FD driver providing:
- `CANInterface`: Abstract base class for CAN hardware
- `CANFrame` / `CANFDFrame`: Frame data structures
- `ZlgCanDriver`: ZLG USB-CAN FD driver implementation
- `ZCANDeviceType`: Device type enumeration

### realman_whj

WHJ joint motor driver providing:
- `WHJMotor`: High-level motor control class
- `WHJProtocol`: Protocol frame building/parsing
- `MotionController`: Trajectory planning and execution
- `WHJMotorConfig`: Configuration management

### kinco_motor

Kinco servo motor driver providing:
- `KincoMotor`: High-level motor control class
- `KincoProtocol`: CANopen PDO protocol implementation
- `KincoMotorConfig`: Configuration management
- NMT node management, position control, homing

## License

MIT License - See LICENSE file for details.
