# Build and Test Instructions

## Quick Build

### Linux

```bash
# 1. Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y libsocketcan-dev can-utils

# 2. Create build directory
mkdir -p build && cd build

# 3. Configure with all options
cmake .. \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_ROS2_NODE=OFF

# 4. Build
cmake --build . --parallel $(nproc)

# 5. Run tests
ctest --output-on-failure

# 6. Run unit tests directly
./unit_tests

# 7. Run hardware tests (requires motor connected)
./hardware_tests sim 1  # Use simulation mode
```

### Windows

```powershell
# 1. Create build directory
mkdir build
cd build

# 2. Configure
cmake .. -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON

# 3. Build
cmake --build . --config Release

# 4. Run tests
ctest -C Release --output-on-failure
```

## Testing

### Unit Tests (No Hardware Required)

```bash
cd build
./unit_tests
```

Output should show:
```
Running unit tests...
=====================
Testing position conversion... OK
Testing speed conversion... OK
Testing int32 packing/unpacking... OK
=====================
Results: 3/3 tests passed
```

### Protocol Tests

```bash
cd build
./protocol_tests
```

### Hardware Tests (Requires Motor)

**Simulation Mode (No hardware):**
```bash
cd build
./hardware_tests sim 1
```

**Real Hardware (Linux):**
```bash
# Setup CAN interface first
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# Run tests
cd build
./hardware_tests can0 1
```

### Examples

```bash
cd build

# Basic example
./example_basic sim 1

# Scan for motors
./example_scan sim

# Position control (simulation mode - just prints)
./example_position_control sim 1
```

## Troubleshooting

### Linux SocketCAN Issues

```bash
# Check if CAN interface exists
ip link show can0

# Bring up interface
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# Check interface status
ip -details link show can0

# View CAN traffic
watch -n 0.5 'candump can0 -n 10'
```

### Build Errors

```bash
# Clean build
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
cmake --build .
```

### Permission Issues

```bash
# Add user to can group (requires logout/login)
sudo usermod -aG can $USER

# Or run with sudo for testing
sudo ./hardware_tests can0 1
```

## Project Structure Summary

```
realman_whj_driver/
├── include/realman_whj/     # Header files
│   ├── core/                # Protocol definitions
│   └── platform/            # Platform abstraction
├── src/                     # Source files
│   ├── core/                # Core implementation
│   ├── platform/            # Platform implementations
│   ├── python/              # Python bindings
│   └── ros2/                # ROS2 node
├── tests/                   # Test programs
│   ├── unit/                # Unit tests
│   └── hardware/            # Hardware tests
├── examples/                # Example programs
│   ├── cpp/                 # C++ examples
│   └── python/              # Python examples
├── CMakeLists.txt           # Main CMake configuration
├── package.xml              # ROS2 package manifest
└── setup.py                 # Python package setup
```
