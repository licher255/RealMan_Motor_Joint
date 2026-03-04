/**
 * @file test_basic.cpp
 * @brief Basic hardware tests for RealMan WHJ motors
 * 
 * These tests require actual motor hardware connected via CAN.
 * Usage: test_basic <can_interface> [motor_id]
 *   - can_interface: "can0" on Linux, "sim" for simulation
 *   - motor_id: Motor ID to test (default: 1)
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include "realman_whj/driver.hpp"

using namespace realman_whj;

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " <can_interface> [motor_id]" << std::endl;
    std::cout << "  can_interface: CAN interface name (e.g., 'can0', 'sim')" << std::endl;
    std::cout << "  motor_id: Motor ID to test (default: 1)" << std::endl;
}

bool testConnection(WHJDriver& driver, uint8_t motor_id) {
    std::cout << "\n[TEST] Connection Test" << std::endl;
    std::cout << "----------------------" << std::endl;
    
    std::cout << "Pinging motor " << (int)motor_id << "... ";
    if (driver.ping(motor_id)) {
        std::cout << "OK (motor online)" << std::endl;
        
        auto info = driver.getMotorInfo(motor_id);
        if (info) {
            std::cout << "  Model: " << info->getModelName() << std::endl;
            std::cout << "  Firmware: " << info->getFirmwareString() << std::endl;
            std::cout << "  Reduction Ratio: " << info->reduction_ratio << std::endl;
        }
        return true;
    } else {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return false;
    }
}

bool testStateReading(WHJDriver& driver, uint8_t motor_id) {
    std::cout << "\n[TEST] State Reading Test" << std::endl;
    std::cout << "-------------------------" << std::endl;
    
    auto state = driver.getJointState(motor_id);
    if (!state) {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return false;
    }
    
    std::cout << "Joint State:" << std::endl;
    std::cout << "  Position: " << state->position_deg << " °" << std::endl;
    std::cout << "  Speed: " << state->speed_rpm << " RPM" << std::endl;
    std::cout << "  Current: " << state->current_ma << " mA" << std::endl;
    std::cout << "  Voltage: " << state->voltage_v << " V" << std::endl;
    std::cout << "  Temperature: " << state->temperature_c << " °C" << std::endl;
    std::cout << "  Enabled: " << (state->is_enabled ? "Yes" : "No") << std::endl;
    std::cout << "  Work Mode: " << workModeToString(static_cast<WorkMode>(state->work_mode)) << std::endl;
    std::cout << "  Error Code: 0x" << std::hex << state->error_code << std::dec << std::endl;
    
    if (state->error_code != 0) {
        auto errors = driver.parseErrors(state->error_code);
        std::cout << "  Errors:" << std::endl;
        for (const auto& err : errors) {
            if (err != ErrorCode::NONE) {
                std::cout << "    - " << getErrorDescription(err) << std::endl;
            }
        }
    }
    
    return true;
}

bool testEnableDisable(WHJDriver& driver, uint8_t motor_id) {
    std::cout << "\n[TEST] Enable/Disable Test" << std::endl;
    std::cout << "--------------------------" << std::endl;
    
    // Get initial state
    auto state = driver.getJointState(motor_id);
    bool was_enabled = state && state->is_enabled;
    
    std::cout << "Initial state: " << (was_enabled ? "Enabled" : "Disabled") << std::endl;
    
    // Disable
    std::cout << "Disabling motor... ";
    if (!driver.enableMotor(motor_id, false)) {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK" << std::endl;
    
    // Wait for disable delay
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Enable
    std::cout << "Enabling motor... ";
    if (!driver.enableMotor(motor_id, true)) {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK" << std::endl;
    
    // Wait for enable
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Verify enabled
    state = driver.getJointState(motor_id);
    if (!state || !state->is_enabled) {
        std::cout << "FAILED - Motor not enabled" << std::endl;
        return false;
    }
    
    // Restore original state if different
    if (!was_enabled) {
        std::cout << "Restoring original disabled state... ";
        driver.enableMotor(motor_id, false);
        std::cout << "OK" << std::endl;
    }
    
    return true;
}

bool testPositionControl(WHJDriver& driver, uint8_t motor_id) {
    std::cout << "\n[TEST] Position Control Test" << std::endl;
    std::cout << "----------------------------" << std::endl;
    
    // Ensure motor is enabled
    auto state = driver.getJointState(motor_id);
    if (!state || !state->is_enabled) {
        std::cout << "Enabling motor first... ";
        if (!driver.enableMotor(motor_id, true)) {
            std::cout << "FAILED" << std::endl;
            return false;
        }
        std::cout << "OK" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Set position mode
    std::cout << "Setting position mode... ";
    if (!driver.setWorkMode(motor_id, WorkMode::POSITION_MODE)) {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK" << std::endl;
    
    // Get current position
    state = driver.getJointState(motor_id);
    if (!state) {
        std::cout << "Failed to get current position" << std::endl;
        return false;
    }
    
    float current_pos = state->position_deg;
    std::cout << "Current position: " << current_pos << " °" << std::endl;
    
    // Move to current + 5 degrees (small test move)
    float target_pos = current_pos + 5.0f;
    std::cout << "Moving to " << target_pos << " °... ";
    
    if (!driver.setTargetPosition(motor_id, target_pos)) {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK" << std::endl;
    
    // Wait for movement
    std::cout << "Waiting for movement (2s)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Check final position
    state = driver.getJointState(motor_id);
    if (!state) {
        std::cout << "Failed to read final position" << std::endl;
        return false;
    }
    
    std::cout << "Final position: " << state->position_deg << " °" << std::endl;
    float error = std::abs(state->position_deg - target_pos);
    std::cout << "Position error: " << error << " °" << std::endl;
    
    if (error > 1.0f) {
        std::cout << "WARNING: Position error > 1 degree" << std::endl;
    }
    
    // Return to original position
    std::cout << "Returning to original position... ";
    driver.setTargetPosition(motor_id, current_pos);
    std::cout << "OK" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    return true;
}

bool testScan(WHJDriver& driver) {
    std::cout << "\n[TEST] Bus Scan Test" << std::endl;
    std::cout << "--------------------" << std::endl;
    
    std::cout << "Scanning for motors... ";
    auto motors = driver.scan(50);
    
    std::cout << "Found " << motors.size() << " motor(s)" << std::endl;
    for (uint8_t id : motors) {
        std::cout << "  - Motor ID: " << (int)id << std::endl;
    }
    
    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "======================================" << std::endl;
    std::cout << "RealMan WHJ Driver - Hardware Tests" << std::endl;
    std::cout << "======================================" << std::endl;
    
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    const char* can_interface = argv[1];
    uint8_t motor_id = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    
    std::cout << "CAN Interface: " << can_interface << std::endl;
    std::cout << "Motor ID: " << (int)motor_id << std::endl;
    
    // Initialize driver
    WHJDriver driver;
    DriverConfig config;
    config.timeout_ms = 200;
    
    std::cout << "\nInitializing driver... ";
    if (!driver.init(can_interface, config)) {
        std::cout << "FAILED - " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl;
    
    // Run tests
    int passed = 0;
    int total = 0;
    
    if (testScan(driver)) passed++;
    total++;
    
    if (testConnection(driver, motor_id)) passed++;
    total++;
    
    if (testStateReading(driver, motor_id)) passed++;
    total++;
    
    if (testEnableDisable(driver, motor_id)) passed++;
    total++;
    
    if (testPositionControl(driver, motor_id)) passed++;
    total++;
    
    // Cleanup
    std::cout << "\nCleaning up... ";
    driver.deinit();
    std::cout << "OK" << std::endl;
    
    // Summary
    std::cout << "\n======================================" << std::endl;
    std::cout << "Test Results: " << passed << "/" << total << " passed" << std::endl;
    std::cout << "======================================" << std::endl;
    
    return (passed == total) ? 0 : 1;
}
