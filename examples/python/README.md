# RealMan WHJ Motor Control - Python Examples

This directory contains Python examples for controlling RealMan WHJ series joint motors via CAN FD.

## Directory Structure

```
examples/python/
├── README.md                           # This file
├── core/                               # Core protocol modules
│   ├── __init__.py
│   ├── zlgcan_driver.py               # ZLG CAN FD driver
│   └── protocol/
│       ├── __init__.py
│       ├── whj_protocol.py            # WHJ motor protocol
│       └── kinco_protocol.py          # Kinco motor protocol (if applicable)
├── drivers/                            # Motor driver modules
│   ├── __init__.py
│   ├── motor_control.py               # Base motor controller
│   ├── motion_controller.py           # Trajectory planning controller
│   ├── whj_driver.py                  # WHJ-specific driver
│   └── kinco_driver.py                # Kinco-specific driver
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
│   └── interactive_control.py         # Interactive control
│
# Main motion controllers (choose based on your scenario):
├── motion_controller.py                # [原始版本] Standard trajectory controller
├── motion_controller_filter_switching.py  # [方案C] Hardware filter switching
├── motion_controller_software_filter.py   # [备用方案] Software filtering
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

### 1. Standard Version (Original)

Use this for single motor or clean CAN bus environments:

```bash
python motion_controller.py <motor_id>
```

Example:
```bash
python motion_controller.py 7
```

### 2. With Kinco Interference - Hardware Filter (方案C)

Use this when Kinco motors are on the same CAN bus causing interference:

```bash
python motion_controller_filter_switching.py <motor_id>
```

This version uses ZLG CAN hardware filters to isolate WHJ motor communication.

### 3. With Kinco Interference - Software Filter (备用方案)

Use this when hardware filtering is not available or you need to control both Kinco and WHJ:

```bash
python motion_controller_software_filter.py <motor_id>
```

This version filters Kinco frames in software.

## Common Commands

All motion controllers support these commands:

| Command | Description |
|---------|-------------|
| `m <pos>` | Move to position (degrees) |
| `e` | Enable motor |
| `d` | Disable motor |
| `c` | Clear errors |
| `r` | Read current position |
| `s` | Show status |
| `q` | Quit |

## Troubleshooting

### Position query timeout (2-3 seconds)

**Problem**: Kinco heartbeat frames flooding the CAN bus.

**Solutions**:
1. Use `motion_controller_filter_switching.py` (hardware filter)
2. Use `motion_controller_software_filter.py` (software filter)
3. Check Kinco ID range in software filter matches your setup

### "Failed to get current position"

**Problem**: Motor not responding or CAN communication issue.

**Check**:
- Motor is powered on
- CAN cable connected
- Correct motor ID
- No other program using the CAN device

### Import errors

**Problem**: Module not found.

**Solution**: Run from the `examples/python` directory:
```bash
cd examples/python
python motion_controller.py 7
```

## Dependencies

- Python 3.8+
- ZLG CAN device (USBCANFD-100U-mini or compatible)
- ZLG CAN driver DLL (zlgcan.dll)

## Version Differences

| Version | File | Use Case | Pros | Cons |
|---------|------|----------|------|------|
| Original | `motion_controller.py` | Clean CAN bus | Simple, standard | No interference handling |
| Hardware Filter | `motion_controller_filter_switching.py` | Kinco + WHJ | Hardware-level isolation | Requires ZLG filter support |
| Software Filter | `motion_controller_software_filter.py` | Kinco + WHJ | No hardware dependency, flexible | Higher CPU usage |

## Notes

- WHJ motor ID: Default is 7 (command ID 0x007, response ID 0x107)
- Kinco motor ID: Check your specific configuration (commonly 0x601-0x6FF)
- CAN bitrate: 1Mbps arbitration, 5Mbps data (CAN FD)

## Support

For more information, see the project-level README.md and AGENTS.md.
