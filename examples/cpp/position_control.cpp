/**
 * @file position_control.cpp
 * @brief Position control example
 * 
 * Demonstrates position control with sine wave motion.
 * Usage: position_control <can_interface> [motor_id]
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include "realman_whj/driver.hpp"

using namespace realman_whj;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <can_interface> [motor_id]" << std::endl;
        return 1;
    }
    
    const char* can_interface = argv[1];
    uint8_t motor_id = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    
    std::cout << "RealMan WHJ Driver - Position Control Example" << std::endl;
    std::cout << "=============================================" << std::endl;
    
    // Create driver
    WHJDriver driver;
    
    std::cout << "Initializing... ";
    if (!driver.init(can_interface)) {
        std::cout << "FAILED" << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl;
    
    // Enable motor
    std::cout << "Enabling motor... ";
    if (!driver.enableMotor(motor_id, true)) {
        std::cout << "FAILED" << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Set position mode
    std::cout << "Setting position mode... ";
    if (!driver.setWorkMode(motor_id, WorkMode::POSITION_MODE)) {
        std::cout << "FAILED" << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl;
    
    // Get current position
    auto state = driver.getJointState(motor_id);
    float center_pos = state ? state->position_deg : 0.0f;
    std::cout << "Center position: " << center_pos << " °" << std::endl;
    
    // Parameters for sine motion
    float amplitude = 10.0f;    // 10 degrees
    float frequency = 0.5f;     // 0.5 Hz
    float duration = 5.0f;      // 5 seconds
    
    std::cout << "\nRunning sine wave motion:" << std::endl;
    std::cout << "  Amplitude: " << amplitude << " °" << std::endl;
    std::cout << "  Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "  Duration: " << duration << " s" << std::endl;
    std::cout << "  (Press Ctrl+C to stop)" << std::endl;
    
    auto start = std::chrono::steady_clock::now();
    int iterations = 0;
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - start).count();
        
        if (elapsed > duration) break;
        
        // Calculate target position (sine wave)
        float target = center_pos + amplitude * std::sin(2.0f * M_PI * frequency * elapsed);
        
        // Send position command
        driver.setTargetPosition(motor_id, target);
        
        // Read and display actual position every 10 iterations
        if (++iterations % 10 == 0) {
            state = driver.getJointState(motor_id);
            if (state) {
                float error = std::abs(state->position_deg - target);
                std::cout << "\rTarget: " << target << " ° | "
                          << "Actual: " << state->position_deg << " ° | "
                          << "Error: " << error << " °        " << std::flush;
            }
        }
        
        // 20Hz control rate
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << std::endl;
    
    // Return to center
    std::cout << "\nReturning to center... ";
    driver.setTargetPosition(motor_id, center_pos);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "OK" << std::endl;
    
    // Disable motor
    std::cout << "Disabling motor... ";
    driver.enableMotor(motor_id, false);
    std::cout << "OK" << std::endl;
    
    driver.deinit();
    std::cout << "\nDone!" << std::endl;
    
    return 0;
}
