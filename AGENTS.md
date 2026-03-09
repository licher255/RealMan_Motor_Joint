# RealMan WHJ Joint Motor Driver - Agent Guide

## Project Overview

This is a cross-platform C++ driver library for **RealMan WHJ series joint motors** with ROS2 integration and Python bindings. The project provides hardware abstraction for CAN FD communication with RealMan Robotics joint motors used in robotic arms.

### Supported Hardware

| Model | Joint Type | Current Scale |
|-------|------------|---------------|
| J3    | Joint 03   | 1 mA/LSB      |
| J14   | Joint 10   | 1 mA/LSB      |
| J17   | Joint 30   | 1 mA/LSB      |
| J20   | Joint 60   | 2 mA/LSB      |
| J25   | Joint 120  | 2 mA/LSB      |

**Supported CAN Devices (Windows):**
- ZLG USBCANFD-100U-mini
- ZLG USBCANFD-100U/200U/400U/800U

### Key Features

- **Cross-Platform**: Linux (SocketCAN) and Windows (USB-CAN adapters)
- **CAN FD Support**: High-speed 5 Mbps data rate
- **Modern C++17**: Clean, type-safe API with move semantics
- **ROS2 Integration**: Native ROS2 node with joint state publishing
- **Python Support**: Pure Python implementation using ctypes + C++ pybind11 bindings
- **Simulation Mode**: Test without hardware using "sim" interface

## Technology Stack

| Component | Technology |
|-----------|------------|
| Language | C++17, Python 3.8+ |
| Build System | CMake 3.16+ |
| CAN Communication | SocketCAN (Linux) / ZLG USB-CAN (Windows) |
| ROS2 Integration | rclcpp, sensor_msgs, std_msgs |
| Python Bindings | pybind11 (optional), ctypes (pure Python) |
| Testing | Custom C++ test framework |

## Project Structure

```
RealMan_Motor_Joint/
├── include/realman_whj/         # Public header files
│   ├── core/                    # Protocol definitions
│   │   ├── types.hpp            # Data structures & unit conversions
│   │   ├── command.hpp          # Register definitions & error codes
│   │   └── protocol.hpp         # Frame builder/parser (header-only)
│   ├── platform/                # Platform abstraction
│   │   ├── can_interface.hpp    # Abstract CAN interface
│   │   ├── linux_can.hpp        # Linux SocketCAN implementation
│   │   └── windows_can.hpp      # Windows USB-CAN implementation
│   └── driver.hpp               # Main WHJDriver class
├── src/                         # Source files
│   ├── core/                    # Core implementation
│   │   ├── driver.cpp           # Main driver implementation
│   │   └── types.cpp            # Types & conversion functions
│   ├── platform/                # Platform implementations
│   │   ├── can_interface.cpp    # Interface factory
│   │   ├── linux_can.cpp        # Linux SocketCAN
│   │   └── windows_can.cpp      # Windows USB-CAN + simulation
│   ├── python/                  # Python bindings
│   │   └── bindings.cpp         # pybind11 bindings
│   └── ros2/                    # ROS2 node
│       └── whj_driver_node.cpp  # ROS2 driver node
├── examples/                    # Example programs
│   ├── cpp/                     # C++ examples
│   │   ├── basic_example.cpp    # Basic usage
│   │   ├── basic_read_example.cpp
│   │   ├── position_control.cpp # Position control demo
│   │   └── scan_example.cpp     # Bus scanning
│   └── python/                  # Python implementation
│       ├── zlgcan_driver.py     # ZLG CAN FD driver (ctypes)
│       ├── whj_protocol.py      # WHJ protocol implementation
│       ├── test_whj_motor.py    # Motor test script
│       └── tools/               # Utility scripts
├── tests/                       # Test programs
│   ├── unit/                    # Unit tests (no hardware)
│   │   ├── test_protocol.cpp    # Protocol frame tests
│   │   └── test_types.cpp       # Type conversion tests
│   └── hardware/                # Hardware tests
│       └── test_basic.cpp       # Basic hardware tests
├── config/                      # Configuration files
│   └── driver_params.yaml       # ROS2 parameters
├── launch/                      # ROS2 launch files
│   └── whj_driver.launch.py     # Driver launch file
├── third_party/                 # Third-party libraries
│   └── zlgcan/                  # ZLG CAN driver library
│       ├── x64/                 # 64-bit DLLs
│       ├── x86/                 # 32-bit DLLs
│       └── include/             # C/C++ headers
├── cmake/                       # CMake configuration
│   └── realman_whjConfig.cmake.in
├── CMakeLists.txt               # Main CMake configuration
├── package.xml                  # ROS2 package manifest
├── setup.py                     # Python package setup (pybind11)
├── README.md                    # User documentation (Chinese)
├── BUILD_TEST.md                # Build and test instructions
└── AGENTS.md                    # This file
```

## Build Instructions

### Prerequisites

**Linux:**
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y libsocketcan-dev can-utils

# For ROS2
sudo apt-get install -y ros-humble-rclcpp ros-humble-sensor-msgs

# For Python bindings
sudo apt-get install -y python3-dev python3-pybind11
```

**Windows:**
- Visual Studio 2019 or later with C++ support
- CMake 3.16+
- ZCANPro installed (for zlgcan.dll)

### Build Commands

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake .. \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_ROS2_NODE=OFF

# Build
cmake --build . --parallel $(nproc)

# Install (optional)
sudo cmake --install .
```

### Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_TESTS` | ON | Build unit and hardware tests |
| `BUILD_EXAMPLES` | ON | Build C++ example programs |
| `BUILD_PYTHON_BINDINGS` | OFF | Build Python bindings via pybind11 |
| `BUILD_ROS2_NODE` | OFF | Build ROS2 node (requires ROS2) |
| `BUILD_SHARED_LIBS` | ON | Build shared libraries |

### Windows Build

```powershell
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
cmake --build . --config Release
```

## Testing

### Unit Tests (No Hardware Required)

```bash
cd build
./unit_tests
```

Tests include:
- Protocol frame building/parsing
- Type conversions (position, speed, current)
- Register read/write operations

### Hardware Tests (Requires Motor)

```bash
cd build
./hardware_tests can0 1  # can0 interface, motor ID 1
```

Tests include:
- Connection/ping test
- State reading
- Enable/disable
- Position control
- Bus scanning

### Simulation Mode

Test without hardware:
```bash
./hardware_tests sim 1
./example_basic sim
./example_scan sim
```

### CTest Integration

```bash
cd build
ctest --output-on-failure
```

## Code Style Guidelines

### C++ Style

- **Standard**: C++17
- **Naming**: 
  - Classes: `PascalCase` (e.g., `WHJDriver`)
  - Functions: `camelCase` (e.g., `getJointState`)
  - Private members: `snake_case_with_underscore_` (e.g., `pImpl_`)
  - Constants: `UPPER_SNAKE_CASE` or `kPascalCase`
- **Indentation**: 4 spaces
- **Braces**: Allman style (opening brace on new line)
- **Comments**: Doxygen-style documentation for all public APIs

### Example:
```cpp
/**
 * @brief Enable or disable motor driver
 * @param motor_id Target motor ID
 * @param enable true to enable, false to disable
 * @return true if successful
 */
bool enableMotor(uint8_t motor_id, bool enable);
```

### Python Style

- Follow PEP 8
- Use snake_case for functions and variables
- Module: `realman_whj` for bindings, pure Python modules use descriptive names

## CAN Configuration (Linux)

### Setup SocketCAN Interface

```bash
# Load CAN kernel module
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan  # For virtual CAN testing

# Bring up CAN interface with CAN FD
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# Check interface status
ip -details link show can0

# Monitor CAN traffic
candump can0
```

### Virtual CAN (Testing)

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## ROS2 Usage

### Launch Driver Node

```bash
# Launch with default parameters
ros2 launch realman_whj_driver whj_driver.launch.py

# With custom parameters
ros2 launch realman_whj_driver whj_driver.launch.py \
    can_interface:=can0 \
    motor_ids:=[1,2,3,4,5,6] \
    joint_names:='["joint_1","joint_2",...]' \
    update_rate:=50.0
```

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Published joint states |
| `/position_command` | std_msgs/Float64MultiArray | Position commands (subscribed) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/enable_motors` | std_srvs/SetBool | Enable/disable all motors |
| `/reset_zero_position` | std_srvs/Empty | Reset zero position |

## Python Usage

### Pure Python (Recommended for Windows)

```python
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, WorkMode

# Initialize CAN device
driver = ZlgCanDriver()
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)

# Build and send command
cmd = WHJProtocol.build_read_frame(motor_id=1, reg=Register.CUR_POSITION_L, count=2)
driver.send(can_id=1, data=cmd)

# Receive response
response = driver.receive(timeout_ms=100)
if response:
    state = WHJProtocol.parse_state_response(motor_id=1, data=response.data)
    print(f"Position: {state.position_deg}°")

driver.close()
```

### C++ Bindings (via pybind11)

```bash
pip install .
```

```python
import realman_whj as whj

driver = whj.WHJDriver()
driver.init("can0")

# Enable motor
driver.enable_motor(1, True)
driver.set_work_mode(1, whj.WorkMode.POSITION_MODE)

# Get state
state = driver.get_joint_state(1)
print(f"Position: {state.position_deg} °")

# Move to position
driver.set_target_position(1, 90.0)

driver.deinit()
```

## Architecture Notes

### CAN FD Protocol

- **Command Types**: READ (0x01), WRITE (0x02), ERROR (0xFF)
- **Response ID**: Request ID + 0x100
- **Data Format**: Little-endian, 16-bit registers
- **32-bit Values**: Stored in consecutive registers (low, high)

### Motor Control Modes

1. **OPEN_LOOP** (0): PWM control (NOT RECOMMENDED)
2. **CURRENT_MODE** (1): Current/torque control
3. **SPEED_MODE** (2): Speed control
4. **POSITION_MODE** (3): Position control (default, RECOMMENDED)

### Unit Conversions

| Parameter | Resolution | Range |
|-----------|------------|-------|
| Position | 0.0001° | ±214748° |
| Target Speed | 0.002 RPM | ±4294967 RPM |
| Actual Speed | 0.02 RPM | ±42949672 RPM |
| Current (J10/J30) | 1 mA | ±2147483 mA |
| Current (J60) | 2 mA | ±4294966 mA |
| Voltage | 0.01 V | 0-655.35 V |
| Temperature | 0.1°C | -3276.8-3276.7°C |

### Key Registers

| Register | Address | Description |
|----------|---------|-------------|
| SYS_ENABLE_DRIVER | 0x0A | Enable driver (0/1) |
| SYS_ERROR | 0x04 | Error code bitmap |
| SYS_VOLTAGE | 0x05 | Voltage (0.01V/LSB) |
| SYS_TEMP | 0x06 | Temperature (0.1°C/LSB) |
| CUR_POSITION_L/H | 0x14/0x15 | Current position |
| TAG_POSITION_L/H | 0x36/0x37 | Target position |
| TAG_WORK_MODE | 0x30 | Work mode (0-3) |

### Error Codes (16-bit bitmap)

- Bit 0: FOC frequency too high
- Bit 1: Over-voltage
- Bit 2: Under-voltage
- Bit 3: Over-temperature
- Bit 4: Startup failed
- Bit 5: Encoder error
- Bit 6: Over-current
- Bit 7: Software/hardware mismatch
- Bit 8: Temperature sensor error
- Bit 9: Position out of range
- Bit 10: Invalid motor ID
- Bit 11: Position tracking error
- Bit 12: Current sensor error
- Bit 13: Brake failed
- Bit 14: Position step too large (>10°)
- Bit 15: Multi-turn counter lost

## Security Considerations

1. **CAN Bus Access**: On Linux, CAN interfaces typically require root privileges or membership in the `can` group
2. **Motor Safety**: Always implement emergency stop mechanisms
3. **Position Limits**: Configure position limits to prevent mechanical damage
4. **Current Limits**: Set appropriate current limits based on motor model

## Troubleshooting

### Linux SocketCAN Issues

```bash
# Check interface
ip link show can0

# Bring up interface
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# Add user to can group
sudo usermod -aG can $USER
```

### Windows DLL Issues

```python
# Architecture mismatch - use correct DLL
# 64-bit Python needs x64/zlgcan.dll
# 32-bit Python needs x86/zlgcan.dll

# Manual DLL path
driver = ZlgCanDriver(dll_path="D:/path/to/zlgcan.dll")
```

### Build Issues

```bash
# Clean build
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
cmake --build .
```

### CAN Connection Issues

1. Check USB connection and ZCANPro can detect device
2. Verify terminal resistance is enabled (120Ω)
3. Confirm motor ID matches expected value
4. Check baud rate settings (1M/5M for CAN FD)
5. Ensure motor is powered on

## Reference Documentation

- [RealMan WHJ Development Docs](https://develop.realman-robotics.com/joints/CANFD/explanation/)
- ZLG CAN Secondary Development Documentation (see `third_party/zlgcan/`)

## License

MIT License - See LICENSE file for details.
