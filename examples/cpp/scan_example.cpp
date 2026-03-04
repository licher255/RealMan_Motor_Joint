/**
 * @file scan_example.cpp
 * @brief Motor scanning example with debug support
 * 
 * Usage: scan_example <can_interface> [motor_id]
 *   can_interface: CAN interface name (e.g., "can0", "vcan0", "sim")
 *   motor_id:      Optional - specific motor ID to test (1-30)
 *                  If not provided, scans all IDs
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include "realman_whj/driver.hpp"

using namespace realman_whj;

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " <can_interface> [motor_id]" << std::endl;
    std::cout << "  can_interface: CAN interface name (e.g., \"can0\", \"vcan0\", \"sim\")" << std::endl;
    std::cout << "  motor_id:      Optional - specific motor ID to test (1-30)" << std::endl;
    std::cout << "                 If not provided, scans all IDs 1-30" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program << " can0       # Scan all IDs on can0" << std::endl;
    std::cout << "  " << program << " can0 7     # Test only motor ID 7" << std::endl;
}

void printFrame(const std::string& prefix, const CANFDFrame& frame) {
    std::cout << prefix << " ID=0x" << std::hex << std::setfill('0') << std::setw(3) << frame.id 
              << std::dec << " Len=" << (int)frame.len;
    if (frame.len > 0) {
        std::cout << " Data=";
        for (int i = 0; i < frame.len && i < 8; i++) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[i] << " ";
        }
    }
    std::cout << std::dec << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    const char* can_interface = argv[1];
    int specific_id = -1;  // -1 means scan all
    
    if (argc >= 3) {
        specific_id = std::atoi(argv[2]);
        if (specific_id < 1 || specific_id > 30) {
            std::cerr << "Error: Motor ID must be between 1 and 30" << std::endl;
            return 1;
        }
    }
    
    std::cout << "RealMan WHJ Driver - Motor Scanner" << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "Interface: " << can_interface << std::endl;
    if (specific_id > 0) {
        std::cout << "Mode:      Test specific motor ID " << specific_id << std::endl;
    } else {
        std::cout << "Mode:      Scan all IDs (1-30)" << std::endl;
    }
    std::cout << std::endl;
    
    // Protocol info
    std::cout << "Protocol Info:" << std::endl;
    std::cout << "  Response ID = Request ID + 0x100" << std::endl;
    if (specific_id > 0) {
        std::cout << "  For ID=" << specific_id << ":" << std::endl;
        std::cout << "    Request  ID = 0x" << std::hex << specific_id << std::dec << std::endl;
        std::cout << "    Response ID = 0x" << std::hex << (specific_id + 0x100) << std::dec << std::endl;
    }
    std::cout << std::endl;
    
    WHJDriver driver;
    
    std::cout << "Initializing driver... ";
    if (!driver.init(can_interface)) {
        std::cout << "FAILED" << std::endl;
        std::cerr << "Error: " << driver.getLastError() << std::endl;
        return 1;
    }
    std::cout << "OK" << std::endl << std::endl;
    
    std::vector<uint8_t> found;
    
    if (specific_id > 0) {
        // Test specific motor ID
        std::cout << "Testing motor ID " << specific_id << "..." << std::endl;
        std::cout << "  Sending ping (read SYS_MODEL_TYPE register)..." << std::endl;
        
        if (driver.ping(static_cast<uint8_t>(specific_id), 500)) {
            found.push_back(static_cast<uint8_t>(specific_id));
            std::cout << "  SUCCESS: Motor responded!" << std::endl;
        } else {
            std::cout << "  FAILED: No response (timeout)" << std::endl;
            std::cout << "\nTroubleshooting:" << std::endl;
            std::cout << "  - Check CAN cable connection" << std::endl;
            std::cout << "  - Verify motor is powered on" << std::endl;
            std::cout << "  - Verify motor ID is actually " << specific_id << std::endl;
            std::cout << "  - Check CAN interface is correct: " << can_interface << std::endl;
            std::cout << "  - Use zcanpro to verify the actual motor ID" << std::endl;
        }
    } else {
        // Scan all IDs
        std::cout << "Scanning for motors (timeout 100ms per ID)..." << std::endl;
        found = driver.scan(100);
    }
    
    std::cout << "\n==================================" << std::endl;
    std::cout << "Found " << found.size() << " motor(s):" << std::endl;
    
    for (uint8_t id : found) {
        std::cout << "\n----------------------------------" << std::endl;
        std::cout << "Motor ID: " << (int)id << std::endl;
        std::cout << "----------------------------------" << std::endl;
        
        // Get motor info
        std::cout << "Reading motor info... ";
        auto info = driver.getMotorInfo(id);
        if (info) {
            std::cout << "OK" << std::endl;
            std::cout << "  Model:      " << info->getModelName() << std::endl;
            std::cout << "  Firmware:   v" << info->getFirmwareString() << std::endl;
            std::cout << "  Reduction:  1:" << info->reduction_ratio << std::endl;
        } else {
            std::cout << "FAILED" << std::endl;
            std::cout << "  Error: " << driver.getLastError() << std::endl;
        }
        
        // Get joint state
        std::cout << "Reading joint state... ";
        auto state = driver.getJointState(id);
        if (state) {
            std::cout << "OK" << std::endl;
            std::cout << "  Position:   " << state->position_deg << " °" << std::endl;
            std::cout << "  Speed:      " << state->speed_rpm << " RPM" << std::endl;
            std::cout << "  Current:    " << state->current_ma << " mA" << std::endl;
            std::cout << "  Enabled:    " << (state->is_enabled ? "Yes" : "No") << std::endl;
            
            if (state->error_code != 0) {
                std::cout << "  Errors:     0x" << std::hex << state->error_code << std::dec << std::endl;
                auto errors = driver.parseErrors(state->error_code);
                for (const auto& err : errors) {
                    if (err != ErrorCode::NONE) {
                        std::cout << "    - " << getErrorDescription(err) << std::endl;
                    }
                }
            }
        } else {
            std::cout << "FAILED" << std::endl;
            std::cout << "  Error: " << driver.getLastError() << std::endl;
        }
    }
    
    driver.deinit();
    std::cout << "\n==================================" << std::endl;
    std::cout << "Done!" << std::endl;
    
    return 0;
}
