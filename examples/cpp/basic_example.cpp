/**
 * @file basic_example.cpp
 * @brief Basic usage example of RealMan WHJ Driver
 * 
 * Usage: basic_example <can_interface> [motor_id]
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "realman_whj/driver.hpp"

using namespace realman_whj;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <can_interface> [motor_id]" << std::endl;
        std::cout << "  can_interface: 'can0' on Linux, 'sim' for simulation" << std::endl;
        return 1;
    }
    
    const char* can_interface = argv[1];
    uint8_t motor_id = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    
    std::cout << "RealMan WHJ Driver - Basic Example" << std::endl;
    std::cout << "==================================" << std::endl;
    
    // Create and initialize driver
    WHJDriver driver;
    
    std::cout << "Connecting to " << can_interface << "... ";
    if (!driver.init(can_interface)) {
        std::cout << "FAILED" << std::endl;
        std::cout << "Error: " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl;
    
    // Scan for motors
    std::cout << "\nScanning for motors..." << std::endl;
    auto motors = driver.scan();
    std::cout << "Found " << motors.size() << " motor(s)" << std::endl;
    for (auto id : motors) {
        std::cout << "  - Motor " << (int)id << std::endl;
    }
    
    // Get motor info
    std::cout << "\nMotor " << (int)motor_id << " Information:" << std::endl;
    auto info = driver.getMotorInfo(motor_id);
    if (info) {
        std::cout << "  Model: " << info->getModelName() << std::endl;
        std::cout << "  Firmware: v" << info->getFirmwareString() << std::endl;
    }
    
    // Read current state
    std::cout << "\nCurrent State:" << std::endl;
    auto state = driver.getJointState(motor_id);
    if (state) {
        std::cout << "  Position: " << state->position_deg << " °" << std::endl;
        std::cout << "  Speed: " << state->speed_rpm << " RPM" << std::endl;
        std::cout << "  Current: " << state->current_ma << " mA" << std::endl;
        std::cout << "  Voltage: " << state->voltage_v << " V" << std::endl;
        std::cout << "  Temperature: " << state->temperature_c << " °C" << std::endl;
        std::cout << "  Enabled: " << (state->is_enabled ? "Yes" : "No") << std::endl;
    }
    
    // Check for errors
    if (state && state->error_code != 0) {
        std::cout << "\nErrors detected:" << std::endl;
        auto errors = driver.parseErrors(state->error_code);
        for (const auto& err : errors) {
            if (err != ErrorCode::NONE) {
                std::cout << "  - " << getErrorDescription(err) << std::endl;
            }
        }
    }
    
    // Cleanup
    std::cout << "\nDisconnecting... ";
    driver.deinit();
    std::cout << "OK" << std::endl;
    
    return 0;
}
