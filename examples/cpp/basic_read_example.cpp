/**
 * @file basic_read_example.cpp
 * @brief Basic example - directly specify motor ID to read state
 * 
 * Usage: basic_read_example <can_interface> [motor_id]
 *   can_interface: CAN interface name (e.g., "can0", "vcan0", "sim")
 *   motor_id:      Motor ID (1-30), default is 7
 */

#include <iostream>
#include <cstdlib>
#include "realman_whj/driver.hpp"

using namespace realman_whj;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <can_interface> [motor_id]" << std::endl;
        std::cout << "  can_interface: CAN interface name (e.g., \"can0\", \"vcan0\", \"sim\")" << std::endl;
        std::cout << "  motor_id:      Motor ID (1-30), default is 7" << std::endl;
        return 1;
    }
    
    const char* can_interface = argv[1];
    uint8_t motor_id = 7;  // Default motor ID
    
    if (argc >= 3) {
        motor_id = static_cast<uint8_t>(std::atoi(argv[2]));
        if (motor_id < 1 || motor_id > 30) {
            std::cerr << "Error: Motor ID must be between 1 and 30" << std::endl;
            return 1;
        }
    }
    
    std::cout << "RealMan WHJ Driver - Basic Read Example" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Interface: " << can_interface << std::endl;
    std::cout << "Motor ID:  " << (int)motor_id << std::endl;
    std::cout << std::endl;
    
    WHJDriver driver;
    
    std::cout << "Initializing driver... ";
    if (!driver.init(can_interface)) {
        std::cout << "FAILED" << std::endl;
        std::cerr << "Error: " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl << std::endl;
    
    // Ping the motor first
    std::cout << "Pinging motor " << (int)motor_id << "... ";
    if (!driver.ping(motor_id, 500)) {
        std::cout << "FAILED" << std::endl;
        std::cerr << "Error: " << driver.getLastError() << std::endl;
        std::cerr << "\nPossible reasons:" << std::endl;
        std::cerr << "  - Motor is not powered on" << std::endl;
        std::cerr << "  - CAN cable is not connected properly" << std::endl;
        std::cerr << "  - Wrong motor ID (expected: " << (int)motor_id << ")" << std::endl;
        std::cerr << "  - CAN interface is not configured correctly" << std::endl;
        driver.deinit();
        return 1;
    }
    std::cout << "OK (motor is online)" << std::endl << std::endl;
    
    // Get motor info
    std::cout << "Reading motor info... ";
    auto info = driver.getMotorInfo(motor_id);
    if (info) {
        std::cout << "OK" << std::endl;
        std::cout << "  Model:      " << info->getModelName() << std::endl;
        std::cout << "  Firmware:   v" << info->getFirmwareString() << std::endl;
        std::cout << "  Reduction:  1:" << info->reduction_ratio << std::endl;
    } else {
        std::cout << "FAILED" << std::endl;
        std::cerr << "Error: " << driver.getLastError() << std::endl;
    }
    std::cout << std::endl;
    
    // Read joint state multiple times
    std::cout << "Reading joint state (3 samples)..." << std::endl;
    for (int i = 0; i < 3; ++i) {
        auto state = driver.getJointState(motor_id);
        if (state) {
            std::cout << "  Sample " << (i + 1) << ":" << std::endl;
            std::cout << "    Position: " << state->position_deg << " °" << std::endl;
            std::cout << "    Speed:    " << state->speed_rpm << " RPM" << std::endl;
            std::cout << "    Current:  " << state->current_ma << " mA" << std::endl;
            std::cout << "    Enabled:  " << (state->is_enabled ? "Yes" : "No") << std::endl;
            
            if (state->error_code != 0) {
                std::cout << "    Errors:   0x" << std::hex << state->error_code << std::dec << std::endl;
                auto errors = driver.parseErrors(state->error_code);
                for (const auto& err : errors) {
                    if (err != ErrorCode::NONE) {
                        std::cout << "      - " << getErrorDescription(err) << std::endl;
                    }
                }
            }
        } else {
            std::cerr << "  Sample " << (i + 1) << ": FAILED - " << driver.getLastError() << std::endl;
        }
        
        if (i < 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    driver.deinit();
    std::cout << "\nDone!" << std::endl;
    
    return 0;
}
