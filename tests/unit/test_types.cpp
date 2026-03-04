/**
 * @file test_types.cpp
 * @brief Unit tests for type conversions
 */

#include <iostream>
#include <cmath>
#include "realman_whj/core/types.hpp"

using namespace realman_whj;

bool test_position_conversion() {
    std::cout << "Testing position conversion... ";
    
    float test_degrees[] = {0.0f, 90.0f, -90.0f, 360.0f, 180.0f, -180.0f};
    
    for (float deg : test_degrees) {
        int32_t units = degToPositionUnits(deg);
        float converted = positionUnitsToDeg(units);
        
        // Allow 0.0001 degree tolerance
        if (std::abs(deg - converted) > 0.0001f) {
            std::cout << "FAIL" << std::endl;
            std::cout << "  Input: " << deg << " deg, Units: " << units 
                      << ", Converted: " << converted << " deg" << std::endl;
            return false;
        }
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

bool test_speed_conversion() {
    std::cout << "Testing speed conversion... ";
    
    float test_rpms[] = {0.0f, 10.0f, -10.0f, 100.0f, 1000.0f};
    
    for (float rpm : test_rpms) {
        // Target speed uses 0.002 RPM resolution
        int32_t units = rpmToTargetSpeedUnits(rpm);
        float converted = targetSpeedUnitsToRpm(units);
        
        // Allow 0.002 RPM tolerance
        if (std::abs(rpm - converted) > 0.002f) {
            std::cout << "FAIL" << std::endl;
            std::cout << "  Input: " << rpm << " RPM, Units: " << units 
                      << ", Converted: " << converted << " RPM" << std::endl;
            return false;
        }
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

bool test_int32_packing() {
    std::cout << "Testing int32 packing/unpacking... ";
    
    int32_t test_values[] = {0, 1, -1, 32767, -32768, 2147483647, -2147483648};
    
    for (int32_t value : test_values) {
        uint16_t low, high;
        packInt32(value, low, high);
        int32_t unpacked = unpackInt32(low, high);
        
        if (value != unpacked) {
            std::cout << "FAIL" << std::endl;
            std::cout << "  Input: " << value << ", Low: 0x" << std::hex << low 
                      << ", High: 0x" << high << std::dec 
                      << ", Unpacked: " << unpacked << std::endl;
            return false;
        }
    }
    
    std::cout << "OK" << std::endl;
    return true;
}

int main() {
    std::cout << "Running unit tests..." << std::endl;
    std::cout << "=====================" << std::endl;
    
    int passed = 0;
    int total = 0;
    
    if (test_position_conversion()) passed++;
    total++;
    
    if (test_speed_conversion()) passed++;
    total++;
    
    if (test_int32_packing()) passed++;
    total++;
    
    std::cout << "=====================" << std::endl;
    std::cout << "Results: " << passed << "/" << total << " tests passed" << std::endl;
    
    return (passed == total) ? 0 : 1;
}
