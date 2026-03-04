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

### Key Features

- **Cross-Platform**: Linux (SocketCAN) and Windows (USB-CAN adapters)
- **CAN FD Support**: High-speed 5 Mbps data rate
- **Modern C++17**: Clean, type-safe API with move semantics
- **ROS2 Integration**: Native ROS2 node with joint state publishing
- **Python Bindings**: Full Python API via pybind11
- **Simulation Mode**: Test without hardware using "sim" interface

## Technology Stack

| Component | Technology |
|-----------|------------|
| Language | C++17 |
| Build System | CMake 3.16+ |
| CAN Communication | SocketCAN (Linux) / USB-CAN (Windows) |
| ROS2 Integration | rclcpp, sensor_msgs, std_msgs |
| Python Bindings | pybind11 |
| Testing | Custom C++ test framework |

## Project Structure

```
realman_whj_driver/
├── include/realman_whj/         # Public header files
│   ├── core/                    # Protocol definitions
│   │   ├── types.hpp            # Data structures & enums
│   │   ├── command.hpp          # Register definitions & constants
│   │   └── protocol.hpp         # Frame builder/parser
│   ├── platform/                # Platform abstraction
│   │   ├── can_interface.hpp    # Abstract CAN interface
│   │   ├── linux_can.hpp        # Linux SocketCAN implementation
│   │   └── windows_can.hpp      # Windows USB-CAN implementation
│   └── driver.hpp               # Main driver class
├── src/                         # Source files
│   ├── core/                    # Core implementation
│   │   ├── driver.cpp           # Main driver implementation
│   │   └── types.cpp            # Types implementation
│   ├── platform/                # Platform implementations
│   │   ├── can_interface.cpp    # Interface factory
│   │   ├── linux_can.cpp        # Linux SocketCAN
│   │   └── windows_can.cpp      # Windows USB-CAN
│   ├── python/                  # Python bindings
│   │   └── bindings.cpp         # pybind11 bindings
│   └── ros2/                    # ROS2 node
│       └── whj_driver_node.cpp  # ROS2 driver node
├── tests/                       # Test programs
│   ├── unit/                    # Unit tests (no hardware)
│   │   ├── test_protocol.cpp    # Protocol tests
│   │   └── test_types.cpp       # Type conversion tests
│   └── hardware/                # Hardware tests
│       └── test_basic.cpp       # Basic hardware tests
├── examples/                    # Example programs
│   ├── cpp/                     # C++ examples
│   │   ├── basic_example.cpp    # Basic usage
│   │   ├── position_control.cpp # Position control demo
│   │   └── scan_example.cpp     # Bus scanning
│   ├── python/                  # Python examples
│   │   ├── basic_example.py     # Basic usage
│   │   └── position_sine.py     # Sine wave motion
│   └── ros2/                    # ROS2 examples
├── config/                      # Configuration files
│   └── driver_params.yaml       # ROS2 parameters
├── launch/                      # ROS2 launch files
│   └── whj_driver.launch.py     # Driver launch file
├── cmake/                       # CMake configuration
│   └── realman_whjConfig.cmake.in
├── dual_arm-2/                  # Dual arm MoveIt2 integration
│   ├── dual_arm_configure/      # MoveIt configuration
│   ├── dual_arm_description/    # URDF and meshes
│   ├── dual_arm_hardware_interface/  # ros2_control hardware interface
│   └── dual_arm_msgs/           # Custom ROS2 messages
├── CMakeLists.txt               # Main CMake configuration
├── package.xml                  # ROS2 package manifest
├── setup.py                     # Python package setup
├── README.md                    # User documentation
└── BUILD_TEST.md                # Build and test instructions
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
- (Optional) USB-CAN adapter drivers

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

### Platform-Specific Notes

- **Linux**: Uses SocketCAN (`can0`, `vcan0`, etc.)
- **Windows**: Uses USB-CAN adapter (implementation-specific)
- **Simulation**: Use `sim` as interface name for testing without hardware

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
- Module: `realman_whj`

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

### Installation

```bash
pip install .
# or
python setup.py install
```

### Basic Example

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

### Build Issues

```bash
# Clean build
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
cmake --build .
```

## ⚠️ Dual Arm Package (dual_arm-2/) - NOT COMPATIBLE

**⚠️ WARNING: This package is NOT compatible with the standard RealMan WHJ protocol.**

The `dual_arm-2` folder contains reference code downloaded from another source. Its CAN communication protocol differs significantly from the standard RealMan WHJ protocol documented at https://develop.realman-robotics.com/joints/CANFD/explanation/.

### Key Incompatibilities

| Aspect | Standard RealMan WHJ | dual_arm_hardware_interface |
|--------|---------------------|----------------------------|
| Control Method | Memory control table (0x30-0x37) | Special CAN ID offsets (0x200, 0x300, 0x400) |
| Response ID | motor_id + 0x100 | motor_id + 0x500 (servo_resp) |
| Position Command | Write registers 0x36/0x37 | CAN ID = motor_id + 0x200 |
| Data Format | CMD(1B) + INDEX(1B) + DATA(2B) | Raw 4-byte value |

### Recommendation

**DO NOT USE** `dual_arm_hardware_interface` with motors running standard RealMan WHJ firmware. 

Instead, use the driver implementations in this repository:
- `src/ros2/whj_driver_node.cpp` - Standard ROS2 node
- `src/core/driver.cpp` - Core driver library

If your motors require the dual_arm protocol, you may need to:
1. Check your motor firmware version (read register 0x03)
2. Contact RealMan support for firmware compatibility information
3. Consider modifying the driver to support the custom protocol (advanced)

### Original Purpose (for reference only)

This was intended as a ROS2 workspace for dual-arm robot control with MoveIt2:
- `dual_arm_configure/`: MoveIt configuration files
- `dual_arm_description/`: URDF models and STL meshes
- `dual_arm_hardware_interface/`: ros2_control hardware interface (custom protocol)
- `dual_arm_msgs/`: Custom ROS2 messages and services

## License

MIT License - See LICENSE file for details.
