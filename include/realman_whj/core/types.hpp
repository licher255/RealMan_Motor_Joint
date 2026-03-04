/**
 * @file types.hpp
 * @brief RealMan WHJ Joint Motor Driver - Data Types Definition
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 * 
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <cstdint>
#include <chrono>
#include <string>
#include <optional>
#include <vector>
#include <functional>

namespace realman_whj {

// ============================================================================
// CAN FD Frame Structure
// ============================================================================

/**
 * @brief CAN FD Frame structure compatible with 64-byte data
 */
struct CANFDFrame {
    uint32_t id{0};                     ///< Arbitration ID (11-bit or 29-bit)
    bool     is_extended{false};        ///< Extended frame flag (29-bit ID)
    bool     is_fd{true};               ///< CAN FD frame flag
    bool     bit_rate_switch{true};     ///< Bit Rate Switching (BRS)
    uint8_t  data[64]{};                ///< Data field (0-64 bytes)
    uint8_t  len{0};                    ///< Data length code (0-64)
    std::chrono::steady_clock::time_point timestamp;  ///< Reception timestamp
    
    CANFDFrame() = default;
    explicit CANFDFrame(uint32_t id_) : id(id_) {}
};

// ============================================================================
// Driver Configuration
// ============================================================================

/**
 * @brief CAN interface configuration
 */
struct DriverConfig {
    uint32_t can_bitrate{1000000};      ///< CAN arbitration bit rate (1 Mbps)
    uint32_t can_fd_bitrate{5000000};   ///< CAN FD data bit rate (5 Mbps)
    uint32_t timeout_ms{100};           ///< Response timeout in milliseconds
    uint8_t  default_retries{3};        ///< Default retry count
    bool     enable_logging{false};     ///< Enable debug logging
    std::string log_level{"INFO"};      ///< Log level: DEBUG, INFO, WARN, ERROR
};

// ============================================================================
// Joint Status Structures
// ============================================================================

/**
 * @brief Complete joint state information
 */
struct JointState {
    uint8_t  motor_id{0};               ///< Motor ID
    
    // Current feedback
    int32_t  current_ma{0};             ///< Current in mA (1mA for J10/J30, 2mA for J60)
    
    // Speed feedback
    float    speed_rpm{0.0f};           ///< Speed in RPM (output shaft, resolution 0.02 RPM)
    
    // Position feedback  
    float    position_deg{0.0f};        ///< Position in degrees (output shaft, resolution 0.0001°)
    
    // System status
    float    voltage_v{0.0f};           ///< System voltage in volts
    float    temperature_c{0.0f};       ///< System temperature in Celsius
    uint16_t error_code{0};             ///< Error code bitmap
    bool     is_enabled{false};         ///< Driver enabled status
    uint8_t  work_mode{0};              ///< Current work mode
    
    // Metadata
    std::chrono::steady_clock::time_point timestamp;
    bool     valid{false};              ///< Data validity flag
};

/**
 * @brief Joint target setpoint
 */
struct JointTarget {
    int32_t  current_ma{0};             ///< Target current (mA)
    float    speed_rpm{0.0f};           ///< Target speed (RPM)
    float    position_deg{0.0f};        ///< Target position (degrees)
    uint8_t  work_mode{3};              ///< Target work mode (default: position)
};

// ============================================================================
// PID Parameters
// ============================================================================

/**
 * @brief PID controller parameters
 */
struct PIDParams {
    uint16_t kp{0};                     ///< Proportional gain
    uint16_t ki{0};                     ///< Integral gain  
    uint16_t kd{0};                     ///< Derivative gain
    uint16_t dead_zone{0};              ///< Dead zone threshold
    
    PIDParams() = default;
    PIDParams(uint16_t p, uint16_t i, uint16_t d, uint16_t dz = 0)
        : kp(p), ki(i), kd(d), dead_zone(dz) {}
};

/**
 * @brief Three-loop PID configuration
 */
struct ThreeLoopPID {
    PIDParams current;                  ///< Current loop PID
    PIDParams speed;                    ///< Speed loop PID
    PIDParams position;                 ///< Position loop PID
    
    // Additional parameters
    uint16_t pos_smooth_coeff{0};       ///< Position loop smooth coefficient
    uint16_t speed_ff_coeff{0};         ///< Speed feedforward coefficient
};

// ============================================================================
// Limit Parameters
// ============================================================================

/**
 * @brief Joint limit parameters
 */
struct LimitParams {
    uint16_t max_current_ma{0};         ///< Maximum current (mA)
    float    max_speed_rpm{3000.0f};    ///< Maximum speed (RPM)
    uint16_t max_acc{5000};             ///< Maximum acceleration (0.1 rpm/s)
    uint16_t max_dec{5000};             ///< Maximum deceleration (0.1 rpm/s)
    float    min_position_deg{-360.0f}; ///< Minimum position (degrees)
    float    max_position_deg{360.0f};  ///< Maximum position (degrees)
};

// ============================================================================
// Motor Information
// ============================================================================

/**
 * @brief Motor hardware information
 */
struct MotorInfo {
    uint8_t  motor_id{0};               ///< Motor ID
    uint8_t  model_type{0};             ///< Model type code
    uint16_t firmware_version{0};       ///< Firmware version (e.g., 0x0102 = v1.2)
    uint16_t reduction_ratio{80};       ///< Reduction ratio (80 or 100)
    uint64_t unique_id{0};              ///< Global unique ID (48-bit)
    
    std::string getModelName() const;
    std::string getFirmwareString() const;
};

// ============================================================================
// Communication Result
// ============================================================================

/**
 * @brief Result of a communication operation
 */
enum class CommResult {
    SUCCESS = 0,                        ///< Operation successful
    TIMEOUT,                            ///< Response timeout
    INVALID_RESPONSE,                   ///< Invalid response received
    CHECKSUM_ERROR,                     ///< Checksum mismatch
    BUS_ERROR,                          ///< CAN bus error
    NOT_INITIALIZED,                    ///< Driver not initialized
    INVALID_PARAMETER,                  ///< Invalid parameter
    WRITE_FAILED,                       ///< Write operation failed
    READ_FAILED,                        ///< Read operation failed
    BUSY,                               ///< Driver busy
    UNKNOWN_ERROR                       ///< Unknown error
};

/**
 * @brief Convert CommResult to string
 */
const char* commResultToString(CommResult result);

// ============================================================================
// Unit Conversion Constants
// ============================================================================

namespace Units {
    // Voltage: 0.01V per LSB
    constexpr float VOLTAGE_SCALE = 0.01f;
    
    // Temperature: 0.1°C per LSB
    constexpr float TEMP_SCALE = 0.1f;
    
    // Position: 0.0001° per LSB
    constexpr float POSITION_SCALE = 0.0001f;
    constexpr float POSITION_INV_SCALE = 10000.0f;  // 1/0.0001
    
    // Target speed: 0.002 RPM per LSB
    constexpr float TARGET_SPEED_SCALE = 0.002f;
    constexpr float TARGET_SPEED_INV_SCALE = 500.0f;
    
    // Actual speed: 0.02 RPM per LSB  
    constexpr float ACTUAL_SPEED_SCALE = 0.02f;
    constexpr float ACTUAL_SPEED_INV_SCALE = 50.0f;
    
    // Current scales
    constexpr int32_t CURRENT_SCALE_10_30 = 1;   // 1mA per LSB for J10/J30
    constexpr int32_t CURRENT_SCALE_60    = 2;   // 2mA per LSB for J60
    
    // Acceleration: 0.1 rpm/s per LSB
    constexpr float ACC_SCALE = 0.1f;
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Convert degrees to motor units
 */
inline int32_t degToPositionUnits(float degrees) {
    return static_cast<int32_t>(degrees * Units::POSITION_INV_SCALE);
}

/**
 * @brief Convert motor units to degrees
 */
inline float positionUnitsToDeg(int32_t units) {
    return static_cast<float>(units) * Units::POSITION_SCALE;
}

/**
 * @brief Convert RPM to target speed units (0.002 RPM resolution)
 */
inline int32_t rpmToTargetSpeedUnits(float rpm) {
    return static_cast<int32_t>(rpm * Units::TARGET_SPEED_INV_SCALE);
}

/**
 * @brief Convert target speed units to RPM
 */
inline float targetSpeedUnitsToRpm(int32_t units) {
    return static_cast<float>(units) * Units::TARGET_SPEED_SCALE;
}

/**
 * @brief Convert RPM to actual speed units (0.02 RPM resolution)
 */
inline int32_t rpmToActualSpeedUnits(float rpm) {
    return static_cast<int32_t>(rpm * Units::ACTUAL_SPEED_INV_SCALE);
}

/**
 * @brief Convert actual speed units to RPM
 */
inline float actualSpeedUnitsToRpm(int32_t units) {
    return static_cast<float>(units) * Units::ACTUAL_SPEED_SCALE;
}

/**
 * @brief Pack 32-bit integer to two 16-bit values (little-endian)
 */
inline void packInt32(int32_t value, uint16_t& low, uint16_t& high) {
    low = static_cast<uint16_t>(value & 0xFFFF);
    high = static_cast<uint16_t>((value >> 16) & 0xFFFF);
}

/**
 * @brief Unpack two 16-bit values to 32-bit integer (little-endian, sign-extended)
 */
inline int32_t unpackInt32(uint16_t low, uint16_t high) {
    return static_cast<int32_t>((static_cast<uint32_t>(high) << 16) | low);
}

} // namespace realman_whj
