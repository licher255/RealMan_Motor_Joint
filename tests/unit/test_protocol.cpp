/**
 * @file test_protocol.cpp
 * @brief Unit tests for protocol frame building/parsing
 */

#include <iostream>
#include "realman_whj/core/protocol.hpp"

using namespace realman_whj;

bool test_read_frame_building() {
    std::cout << "Testing read frame building... ";
    
    uint8_t motor_id = 5;
    Register reg = Register::CUR_POSITION_L;
    uint8_t count = 2;
    
    auto frame = Protocol::buildReadFrame(motor_id, reg, count);
    
    // Check frame properties
    if (frame.id != motor_id) {
        std::cout << "FAIL (wrong ID)" << std::endl;
        return false;
    }
    
    if (frame.len != 3) {
        std::cout << "FAIL (wrong length)" << std::endl;
        return false;
    }
    
    if (frame.data[0] != Protocol::CMD_READ) {
        std::cout << "FAIL (wrong command)" << std::endl;
        return false;
    }
    
    if (frame.data[1] != toUnderlying(reg)) {
        std::cout << "FAIL (wrong register)" << std::endl;
        return false;
    }
    
    if (frame.data[2] != count) {
        std::cout << "FAIL (wrong count)" << std::endl;
        return false;
    }
    
    if (!frame.is_fd || !frame.bit_rate_switch) {
        std::cout << "FAIL (wrong CAN FD flags)" << std::endl;
        return false;
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

bool test_write_frame_building() {
    std::cout << "Testing write frame building... ";
    
    uint8_t motor_id = 3;
    Register reg = Register::SYS_ENABLE_DRIVER;
    uint16_t value = 1;
    
    auto frame = Protocol::buildWriteFrame(motor_id, reg, value);
    
    if (frame.id != motor_id) {
        std::cout << "FAIL (wrong ID)" << std::endl;
        return false;
    }
    
    if (frame.len != 4) {
        std::cout << "FAIL (wrong length)" << std::endl;
        return false;
    }
    
    if (frame.data[0] != Protocol::CMD_WRITE) {
        std::cout << "FAIL (wrong command)" << std::endl;
        return false;
    }
    
    if (frame.data[1] != toUnderlying(reg)) {
        std::cout << "FAIL (wrong register)" << std::endl;
        return false;
    }
    
    uint16_t data_value = frame.data[2] | (frame.data[3] << 8);
    if (data_value != value) {
        std::cout << "FAIL (wrong data value)" << std::endl;
        return false;
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

bool test_response_id_calculation() {
    std::cout << "Testing response ID calculation... ";
    
    uint8_t test_ids[] = {1, 5, 10, 30};
    
    for (uint8_t id : test_ids) {
        uint32_t response_id = Protocol::getResponseId(id);
        uint32_t expected = id + Protocol::RESPONSE_ID_OFFSET;
        
        if (response_id != expected) {
            std::cout << "FAIL (id=" << (int)id << ")" << std::endl;
            return false;
        }
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

bool test_enable_motor_frame() {
    std::cout << "Testing enable motor frame... ";
    
    auto enable_frame = Protocol::buildEnableMotor(1, true);
    auto disable_frame = Protocol::buildEnableMotor(1, false);
    
    // Check enable frame
    if (enable_frame.data[0] != Protocol::CMD_WRITE ||
        enable_frame.data[1] != toUnderlying(Register::SYS_ENABLE_DRIVER) ||
        enable_frame.data[2] != 1) {
        std::cout << "FAIL (enable frame)" << std::endl;
        return false;
    }
    
    // Check disable frame
    if (disable_frame.data[0] != Protocol::CMD_WRITE ||
        disable_frame.data[1] != toUnderlying(Register::SYS_ENABLE_DRIVER) ||
        disable_frame.data[2] != 0) {
        std::cout << "FAIL (disable frame)" << std::endl;
        return false;
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

int main() {
    std::cout << "Running protocol tests..." << std::endl;
    std::cout << "=========================" << std::endl;
    
    int passed = 0;
    int total = 0;
    
    if (test_read_frame_building()) passed++;
    total++;
    
    if (test_write_frame_building()) passed++;
    total++;
    
    if (test_response_id_calculation()) passed++;
    total++;
    
    if (test_enable_motor_frame()) passed++;
    total++;
    
    std::cout << "=========================" << std::endl;
    std::cout << "Results: " << passed << "/" << total << " tests passed" << std::endl;
    
    return (passed == total) ? 0 : 1;
}
