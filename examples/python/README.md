# RealMan WHJ Motor Control - Python Examples

This directory contains Python examples for controlling RealMan WHJ series joint motors via CAN FD.

## Directory Structure

```
examples/python/
├── README.md                           # This file
├── core/                               # Core protocol modules
│   ├── __init__.py
│   ├── zlgcan_driver.py               # ZLG CAN FD driver (mixed mode support)
│   └── protocol/
│       ├── __init__.py
│       ├── whj_protocol.py            # WHJ motor protocol
│       └── kinco_protocol.py          # Kinco motor protocol
├── drivers/                            # Motor driver modules
│   ├── __init__.py
│   ├── motor_control.py               # Base motor controller (with CAN FD filter)
│   ├── motion_controller.py           # Trajectory planning controller (default)
│   ├── kinco_driver.py                # Kinco-specific driver
│   └── whj_driver.py                  # WHJ-specific driver
├── utils/                              # Utility modules
│   ├── __init__.py
│   ├── can_multiplexer.py             # CAN multiplexer
│   └── dual_motor_manager.py          # Dual motor coordination
├── tools/                              # Tool scripts
│   ├── debug_can.py                   # CAN debugging tool
│   ├── position_sine.py               # Sine wave position test
│   ├── reset_can_device.py            # Reset CAN device
│   ├── simple_position_control.py     # Simple position control
│   └── test_whj_motor.py              # Motor test script
├── tests/                              # Test scripts
│   ├── __init__.py
│   ├── test_whj.py
│   ├── test_kinco.py
│   └── test_dual.py
├── examples/                           # Simple examples
│   ├── __init__.py
│   ├── basic_whj.py                   # Basic WHJ example
│   ├── basic_kinco.py                 # Basic Kinco example
│   ├── dual_motor_basic.py            # Dual motor example
│   ├── interactive_control.py         # Interactive control
│   └── mixed_mode_demo.py             # CAN FD + Standard CAN demo
│
# Main motion controller:
├── motion_controller.py                # Trajectory controller with CAN FD filter
│
# Other utility scripts:
├── check_setup.py                      # Environment check
├── query_motor_state.py                # Query motor state
├── encoder_recovery.py                 # Encoder recovery
├── dual_motor_control.py               # Dual motor control
├── dual_motor_switchable.py            # Switchable dual motor control
└── main.py                             # Main entry point

```

## Quick Start

### Single WHJ Motor

```bash
python motion_controller.py <motor_id>
```

Example:
```bash
python motion_controller.py 7
```

### Mixed Mode (WHJ + Kinco on Same Bus)

```bash
python examples/mixed_mode_demo.py
```

## Hardware Setup

### Terminal Resistance (120Ω)

| Device | Terminal Resistance | Note |
|--------|---------------------|------|
| ZLG USBCANFD-100U-mini | **Internal 120Ω** | Enabled via software, no external resistor needed |
| RealMan WHJ Motor | **No resistor needed** | Internal resistance handled by driver |
| Kinco Motor | **SW4 = OFF** | Disable internal 120Ω termination (use bus termination instead) |

**Important**: 
- ZLG USBCANFD-100U-mini already has internal 120Ω termination, enable it in software
- WHJ motors do NOT need parallel resistors
- **Kinco motors**: Set SW4 (last dial switch) to **OFF** to disable internal 120Ω termination
- Ensure exactly two 120Ω terminations at both ends of the CAN bus

### Wiring Diagram

```
ZLG CAN FD 100U-mini          WHJ Motor                   Kinco Motor
       |                          |                            |
    [CAN_H]--------------------[CAN_H]------------------------[CAN_H]
       |                          |                            |
    [CAN_L]--------------------[CAN_L]------------------------[CAN_L]
       |                          |                            |
    [GND]----------------------[GND]--------------------------[GND]
       
    Internal 120Ω              No resistor              SW4=OFF (no resistor)
    (software enabled)                                   
```

## Key Lessons Learned: CAN FD Filter

### The Problem

When WHJ motors and Kinco motors share the same CAN bus:
- Kinco motors send standard CAN frames (SDO/PDO) frequently
- These frames flood the receive buffer
- WHJ commands timeout because the driver processes Kinco frames instead of WHJ responses
- Position queries take 2-3 seconds or fail completely

### The Solution

**Use CAN FD frame filtering** - Only receive CAN FD frames, ignore standard CAN.

This is implemented in `zlgcan_driver.py` and enabled by default in `motion_controller.py`:

```python
from drivers.motion_controller import SmoothMotorController

# filter_canfd_only=True is the default - filters out Kinco CAN frames
motor = SmoothMotorController(driver, motor_id=7, filter_canfd_only=True)
```

Or use the base controller:

```python
from drivers.motor_control import MotorController

motor = MotorController(driver, motor_id=7, filter_canfd_only=True)
```

### Why This Works

| Frame Type | WHJ Motor | Kinco Motor |
|------------|-----------|-------------|
| Standard CAN | ✗ Not used | ✓ Uses extensively |
| CAN FD | ✓ Uses exclusively | ✗ Not supported |

By filtering for only CAN FD frames:
- WHJ responses (CAN FD) are received normally
- Kinco frames (standard CAN) are automatically ignored
- No complex ID filtering needed
- Hardware-level separation via frame type

### Communication Protocol Differences

| Aspect | WHJ Motor | Kinco Motor |
|--------|-----------|-------------|
| Protocol | Custom CAN FD | CANopen (standard CAN) |
| Command ID | `0x01` - `0x7F` | `0x600` + node_id (SDO RX) |
| Response ID | Command ID + `0x100` | `0x580` + node_id (SDO TX) |
| Frame Type | CAN FD (up to 64 bytes) | Standard CAN (8 bytes) |
| Bitrate Switch | Enabled (5Mbps data) | N/A |

## Common Commands

All motion controllers support these commands:

| Command | Description |
|---------|-------------|
| `m <pos>` | Move to position (degrees) with trajectory planning |
| `e` | Enable motor |
| `d` | Disable motor |
| `c` | Clear errors |
| `r` | Read current position |
| `s` | Show status |
| `q` | Quit |

## Troubleshooting

### "Failed to get current position" / Communication Timeout

**Problem**: Kinco CAN frames interfering with WHJ communication.

**Solution**: 
- Use `motion_controller.py` (already has CAN FD filter enabled)
- Or manually enable filter: `MotorController(driver, id, filter_canfd_only=True)`

### "Failed to open CAN device"

**Problem**: CAN device in use by another program.

**Solution**:
```bash
# Reset CAN device
python tools/reset_can_device.py
```

### Position errors / Unstable movement

**Check**:
1. Terminal resistance properly configured (see Hardware Setup)
2. Kinco SW4 is OFF
3. CAN cables shielded and properly grounded
4. No loose connections

### Mixed mode not working

**Problem**: WHJ works alone but fails when Kinco is connected.

**Check**:
1. Both motors use same arbitration bitrate (1Mbps)
2. ZLG driver initialized with `init_mixed_mode()`
3. WHJ using CAN FD frames, Kinco using standard CAN
4. No duplicate CAN IDs

## Code Examples

### Basic WHJ Control (Filtered)

```python
from core import ZlgCanDriver, ZCANDeviceType
from drivers.motor_control import MotorController

driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# Enable CAN FD filtering
motor = MotorController(driver, motor_id=7, filter_canfd_only=True)
motor.initialize()
motor.enable(True)
motor.set_target_position(90.0)
```

### Mixed Mode (WHJ + Kinco)

```python
from core import ZlgCanDriver, ZCANDeviceType

driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)

# Initialize mixed mode
# 1Mbps for arbitration (both CAN and CAN FD)
# 5Mbps for data phase (CAN FD only)
driver.init_mixed_mode(arbitration_bps=1000000, data_bps=5000000)

# Send to WHJ (CAN FD)
driver.send_canfd(can_id=0x07, data=whj_cmd, bitrate_switch=True)

# Send to Kinco (Standard CAN)
driver.send_can(can_id=0x601, data=kinco_cmd)

# Receive WHJ response only (filtered)
frame = driver.receive(frame_type="CANFD")
```

## Dependencies

- Python 3.8+
- ZLG CAN device (USBCANFD-100U-mini or compatible)
- ZLG CAN driver DLL (zlgcan.dll)

## Motor ID Reference

| Motor | Default Command ID | Response ID |
|-------|-------------------|-------------|
| WHJ Joint 1 | 0x01 | 0x101 |
| WHJ Joint 2 | 0x02 | 0x102 |
| ... | ... | ... |
| WHJ Joint 7 | 0x07 | 0x107 |
| Kinco (node 1) | 0x601 | 0x581 |
| Kinco (node 2) | 0x602 | 0x582 |

## Support

For more information, see the project-level README.md and AGENTS.md.
