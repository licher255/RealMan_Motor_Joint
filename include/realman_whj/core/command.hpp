/**
 * @file command.hpp
 * @brief RealMan WHJ Joint Motor Driver - Command and Register Definitions
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 */

#pragma once

#include <cstdint>
#include <type_traits>
#include <vector>
#include <string>

namespace realman_whj {

// ============================================================================
// CAN FD Command Types
// ============================================================================

/**
 * @brief Command types for CAN FD communication
 */
enum class CommandType : uint8_t {
    READ  = 0x01,   ///< Read control table data
    WRITE = 0x02,   ///< Write to control table
    ERROR = 0xFF    ///< Error response
};

/**
 * @brief Error codes in error response
 */
enum class ProtocolError : uint8_t {
    NONE            = 0x00,  ///< No error
    FORMAT_ERROR    = 0x01,  ///< Frame format error
    PERMISSION_ERROR= 0x02,  ///< Permission error
    INDEX_ERROR     = 0x03,  ///< Invalid register index
    VALUE_ERROR     = 0x04   ///< Invalid value
};

// ============================================================================
// Memory Control Table Register Addresses
// ============================================================================

/**
 * @brief Register addresses in the memory control table
 * 
 * All registers are 2-byte (16-bit) signed integers unless otherwise noted.
 * Multi-byte values (32-bit) are stored in consecutive registers (low, high).
 */
enum class Register : uint8_t {
    // =========================================================================
    // System Information (0x00-0x0F)
    // =========================================================================
    RESERVED_00         = 0x00,  ///< Reserved
    SYS_ID              = 0x01,  ///< Driver ID (0x00-0x1E, 0x00=broadcast)
    SYS_MODEL_TYPE      = 0x02,  ///< Driver model type (read-only)
    SYS_FW_VERSION      = 0x03,  ///< Firmware version (e.g., 0x0102=v1.2)
    SYS_ERROR           = 0x04,  ///< Error code bitmap (read-only)
    SYS_VOLTAGE         = 0x05,  ///< System voltage (0.01V)
    SYS_TEMP            = 0x06,  ///< System temperature (0.1°C)
    SYS_REDU_RATIO      = 0x07,  ///< Reduction ratio (read-only)
    RESERVED_08         = 0x08,  ///< Reserved
    RESERVED_09         = 0x09,  ///< Reserved
    
    // Control flags
    SYS_ENABLE_DRIVER   = 0x0A,  ///< Driver enable (1=enable, 0=disable)
    SYS_ENABLE_ON_POWER = 0x0B,  ///< Auto-enable on power up
    SYS_SAVE_TO_FLASH   = 0x0C,  ///< Save to flash flag (write 1 to save)
    RESERVED_0D         = 0x0D,  ///< Reserved (auto-calibration, deprecated)
    SYS_SET_ZERO_POS    = 0x0E,  ///< Set current position as zero
    SYS_CLEAR_ERROR     = 0x0F,  ///< Clear error flag (write 1 to clear)
    
    // =========================================================================
    // Current Feedback (0x10-0x11)
    // =========================================================================
    CUR_CURRENT_L       = 0x10,  ///< Current low 16-bit (mA)
    CUR_CURRENT_H       = 0x11,  ///< Current high 16-bit (mA)
    
    // =========================================================================
    // Speed Feedback (0x12-0x13)
    // =========================================================================
    CUR_SPEED_L         = 0x12,  ///< Speed low 16-bit (0.02 RPM)
    CUR_SPEED_H         = 0x13,  ///< Speed high 16-bit (0.02 RPM)
    
    // =========================================================================
    // Position Feedback (0x14-0x15)
    // =========================================================================
    CUR_POSITION_L      = 0x14,  ///< Position low 16-bit (0.0001°)
    CUR_POSITION_H      = 0x15,  ///< Position high 16-bit (0.0001°)
    
    // =========================================================================
    // Reserved (0x16-0x1E)
    // =========================================================================
    RESERVED_16         = 0x16,
    RESERVED_17         = 0x17,
    RESERVED_18         = 0x18,
    RESERVED_19         = 0x19,
    RESERVED_1A         = 0x1A,
    RESERVED_1B         = 0x1B,
    RESERVED_1C         = 0x1C,
    RESERVED_1D         = 0x1D,
    RESERVED_1E         = 0x1E,
    
    // =========================================================================
    // Power-on Delay (0x1F)
    // =========================================================================
    ON_DELAY            = 0x1F,  ///< Power-on delay (0=delay, 1=no delay)
    
    // =========================================================================
    // Reserved (0x20-0x29)
    // =========================================================================
    RESERVED_20         = 0x20,
    RESERVED_21         = 0x21,
    RESERVED_22         = 0x22,
    RESERVED_23         = 0x23,
    RESERVED_24         = 0x24,
    RESERVED_25         = 0x25,
    RESERVED_26         = 0x26,
    RESERVED_27         = 0x27,
    RESERVED_28         = 0x28,
    RESERVED_29         = 0x29,
    
    // =========================================================================
    // Unique ID (0x2A-0x2F, read-only)
    // =========================================================================
    MOT_MODEL_ID0       = 0x2A,  ///< Unique ID bits [15:0]
    MOT_MODEL_ID1       = 0x2B,  ///< Unique ID bits [31:16]
    MOT_MODEL_ID2       = 0x2C,  ///< Unique ID bits [47:32]
    MOT_MODEL_ID3       = 0x2D,  ///< Unique ID bits [63:48]
    MOT_MODEL_ID4       = 0x2E,  ///< Unique ID bits [79:64]
    MOT_MODEL_ID5       = 0x2F,  ///< Unique ID bits [95:80]
    
    // =========================================================================
    // Target Control (0x30-0x37)
    // =========================================================================
    TAG_WORK_MODE       = 0x30,  ///< Work mode (0=open, 1=current, 2=speed, 3=pos)
    TAG_OPEN_PWM        = 0x31,  ///< Open-loop PWM duty (0-100)
    TAG_CURRENT_L       = 0x32,  ///< Target current low 16-bit (mA)
    TAG_CURRENT_H       = 0x33,  ///< Target current high 16-bit (mA)
    TAG_SPEED_L         = 0x34,  ///< Target speed low 16-bit (0.002 RPM)
    TAG_SPEED_H         = 0x35,  ///< Target speed high 16-bit (0.002 RPM)
    TAG_POSITION_L      = 0x36,  ///< Target position low 16-bit (0.0001°)
    TAG_POSITION_H      = 0x37,  ///< Target position high 16-bit (0.0001°)
    
    // =========================================================================
    // Position Following (0x38-0x39)
    // =========================================================================
    RESERVED_38         = 0x38,  ///< Reserved
    POS_FOLLOW_COEFF    = 0x39,  ///< Position following coefficient (0-10)
    
    // =========================================================================
    // Reserved (0x3A-0x3F)
    // =========================================================================
    RESERVED_3A         = 0x3A,
    RESERVED_3B         = 0x3B,
    RESERVED_3C         = 0x3C,
    RESERVED_3D         = 0x3D,
    RESERVED_3E         = 0x3E,
    RESERVED_3F         = 0x3F,
    
    // =========================================================================
    // Limit Parameters (0x40-0x47)
    // =========================================================================
    LIT_MAX_CURRENT     = 0x40,  ///< Maximum current (mA)
    LIT_MAX_SPEED       = 0x41,  ///< Maximum speed (RPM)
    LIT_MAX_ACC         = 0x42,  ///< Maximum acceleration (0.1 rpm/s)
    LIT_MAX_DEC         = 0x43,  ///< Maximum deceleration (0.1 rpm/s)
    LIT_MIN_POSITION_L  = 0x44,  ///< Min position low 16-bit (0.0001°)
    LIT_MIN_POSITION_H  = 0x45,  ///< Min position high 16-bit (0.0001°)
    LIT_MAX_POSITION_L  = 0x46,  ///< Max position low 16-bit (0.0001°)
    LIT_MAX_POSITION_H  = 0x47,  ///< Max position high 16-bit (0.0001°)
    
    // =========================================================================
    // Reserved (0x48)
    // =========================================================================
    RESERVED_48         = 0x48,
    
    // =========================================================================
    // IAP Flag (0x49)
    // =========================================================================
    IAP_FLAG            = 0x49,  ///< IAP update flag (0=no update, 1=update)
    
    // =========================================================================
    // Reserved (0x4A-0x50)
    // =========================================================================
    RESERVED_4A         = 0x4A,
    RESERVED_4B         = 0x4B,
    RESERVED_4C         = 0x4C,
    RESERVED_4D         = 0x4D,
    RESERVED_4E         = 0x4E,
    RESERVED_4F         = 0x4F,
    RESERVED_50         = 0x50,
    
    // =========================================================================
    // PID Parameters (0x51-0x5D)
    // =========================================================================
    // Current loop
    SEV_CURRENT_P       = 0x51,  ///< Current loop P
    SEV_CURRENT_I       = 0x52,  ///< Current loop I
    SEV_CURRENT_D       = 0x53,  ///< Current loop D
    
    // Speed loop
    SEV_SPEED_P         = 0x54,  ///< Speed loop P
    SEV_SPEED_I         = 0x55,  ///< Speed loop I
    SEV_SPEED_D         = 0x56,  ///< Speed loop D
    SEV_SPEED_DS        = 0x57,  ///< Speed P dead zone
    
    // Position loop
    SEV_POSITION_P      = 0x58,  ///< Position loop P
    SEV_POSITION_I      = 0x59,  ///< Position loop I
    SEV_POSITION_D      = 0x5A,  ///< Position loop D
    SEV_POSITION_DS     = 0x5B,  ///< Position P dead zone
    SEV_POS_SMOOTH      = 0x5C,  ///< Position smooth coefficient
    SEV_SPD_FF          = 0x5D,  ///< Speed feedforward coefficient
    
    // =========================================================================
    // Reserved (0x5E-0x68)
    // =========================================================================
    RESERVED_5E         = 0x5E,
    RESERVED_5F         = 0x5F,
    RESERVED_60         = 0x60,
    RESERVED_61         = 0x61,
    RESERVED_62         = 0x62,
    RESERVED_63         = 0x63,
    RESERVED_64         = 0x64,
    RESERVED_65         = 0x65,
    RESERVED_66         = 0x66,
    RESERVED_67         = 0x67,
    RESERVED_68         = 0x68,
    
    // =========================================================================
    // Low Power Mode (0x69)
    // =========================================================================
    LOW_POWER_MODE      = 0x69,  ///< Low power mode
    
    // =========================================================================
    // Reserved (0x6A-0x77)
    // =========================================================================
    RESERVED_6A         = 0x6A,
    RESERVED_6B         = 0x6B,
    RESERVED_6C         = 0x6C,
    RESERVED_6D         = 0x6D,
    RESERVED_6E         = 0x6E,
    RESERVED_6F         = 0x6F,
    RESERVED_70         = 0x70,
    RESERVED_71         = 0x71,
    RESERVED_72         = 0x72,
    RESERVED_73         = 0x73,
    RESERVED_74         = 0x74,
    RESERVED_75         = 0x75,
    RESERVED_76         = 0x76,
    RESERVED_77         = 0x77,
    
    // =========================================================================
    // Error Code (0x78)
    // =========================================================================
    ERROR               = 0x78,  ///< Error code (read-only, same as SYS_ERROR)
    
    // =========================================================================
    // End Board Mode (0x79)
    // =========================================================================
    END_BOARD_MODE      = 0x79,  ///< End board mode
    
    // =========================================================================
    // Reserved (0x7A-0x91)
    // =========================================================================
    RESERVED_7A         = 0x7A,
    RESERVED_7B         = 0x7B,
    RESERVED_7C         = 0x7C,
    RESERVED_7D         = 0x7D,
    RESERVED_7E         = 0x7E,
    RESERVED_7F         = 0x7F,
    RESERVED_80         = 0x80,
    RESERVED_81         = 0x81,
    RESERVED_82         = 0x82,
    RESERVED_83         = 0x83,
    RESERVED_84         = 0x84,
    RESERVED_85         = 0x85,
    RESERVED_86         = 0x86,
    RESERVED_87         = 0x87,
    RESERVED_88         = 0x88,
    RESERVED_89         = 0x89,
    RESERVED_8A         = 0x8A,
    RESERVED_8B         = 0x8B,
    RESERVED_8C         = 0x8C,
    RESERVED_8D         = 0x8D,
    RESERVED_8E         = 0x8E,
    RESERVED_8F         = 0x8F,
    RESERVED_90         = 0x90,
    RESERVED_91         = 0x91,
};

/**
 * @brief Helper to get underlying uint8_t value from Register enum
 */
inline uint8_t toUnderlying(Register reg) {
    return static_cast<std::underlying_type_t<Register>>(reg);
}

// ============================================================================
// Work Modes
// ============================================================================

/**
 * @brief Joint work modes
 */
enum class WorkMode : uint8_t {
    OPEN_LOOP    = 0,  ///< Open-loop mode (PWM control, NOT RECOMMENDED)
    CURRENT_MODE = 1,  ///< Current (torque) mode
    SPEED_MODE   = 2,  ///< Speed mode
    POSITION_MODE= 3   ///< Position mode (default, RECOMMENDED)
};

/**
 * @brief Get string representation of work mode
 */
inline const char* workModeToString(WorkMode mode) {
    switch (mode) {
        case WorkMode::OPEN_LOOP:     return "OPEN_LOOP";
        case WorkMode::CURRENT_MODE:  return "CURRENT_MODE";
        case WorkMode::SPEED_MODE:    return "SPEED_MODE";
        case WorkMode::POSITION_MODE: return "POSITION_MODE";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Joint Models
// ============================================================================

/**
 * @brief Joint model types
 */
enum class JointModel : uint8_t {
    J14     = 0x02,  ///< Joint 10 (J14)
    J17     = 0x03,  ///< Joint 30 (J17)
    J20     = 0x04,  ///< Joint 60 (J20)
    J25     = 0x05,  ///< Joint 120 (J25)
    GRIPPER = 0x06,  ///< Gripper
    J3      = 0x07   ///< Joint 03 (J3)
};

/**
 * @brief Get model name string
 */
inline const char* getJointModelName(JointModel model) {
    switch (model) {
        case JointModel::J14:     return "J14 (Joint 10)";
        case JointModel::J17:     return "J17 (Joint 30)";
        case JointModel::J20:     return "J20 (Joint 60)";
        case JointModel::J25:     return "J25 (Joint 120)";
        case JointModel::GRIPPER: return "GRIPPER";
        case JointModel::J3:      return "J3 (Joint 03)";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Get current scale for joint model
 */
inline int32_t getCurrentScale(JointModel model) {
    switch (model) {
        case JointModel::J14:
        case JointModel::J17:
        case JointModel::J3:
            return 1;  // 1mA
        case JointModel::J20:
        case JointModel::J25:
            return 2;  // 2mA
        default:
            return 1;
    }
}

// ============================================================================
// Error Codes (16-bit bitmap)
// ============================================================================

/**
 * @brief Error code bit definitions
 * 
 * These are bit positions in the 16-bit error code register.
 * Multiple errors can be present simultaneously.
 */
enum class ErrorCode : uint16_t {
    NONE                    = 0x0000,  ///< No error
    FOC_FREQ_HIGH           = 0x0001,  ///< Bit 0: FOC frequency too high
    OVER_VOLTAGE            = 0x0002,  ///< Bit 1: Over-voltage
    UNDER_VOLTAGE           = 0x0004,  ///< Bit 2: Under-voltage
    OVER_TEMPERATURE        = 0x0008,  ///< Bit 3: Over-temperature
    STARTUP_FAILED          = 0x0010,  ///< Bit 4: Startup failed
    ENCODER_ERROR           = 0x0020,  ///< Bit 5: Encoder error
    OVER_CURRENT            = 0x0040,  ///< Bit 6: Over-current
    SOFTWARE_ERROR          = 0x0080,  ///< Bit 7: Software/hardware mismatch
    TEMP_SENSOR_ERROR       = 0x0100,  ///< Bit 8: Temperature sensor error
    POSITION_OUT_OF_RANGE   = 0x0200,  ///< Bit 9: Position out of range
    INVALID_ID              = 0x0400,  ///< Bit 10: Invalid motor ID
    POSITION_TRACK_ERROR    = 0x0800,  ///< Bit 11: Position tracking error
    CURRENT_SENSOR_ERROR    = 0x1000,  ///< Bit 12: Current sensor error
    BRAKE_FAILED            = 0x2000,  ///< Bit 13: Brake failed
    POSITION_STEP_ERROR     = 0x4000,  ///< Bit 14: Position step too large
    MULTI_TURN_LOST         = 0x8000   ///< Bit 15: Multi-turn counter lost
};

// Enable bitwise operations for ErrorCode
inline ErrorCode operator|(ErrorCode a, ErrorCode b) {
    return static_cast<ErrorCode>(
        static_cast<std::underlying_type_t<ErrorCode>>(a) |
        static_cast<std::underlying_type_t<ErrorCode>>(b));
}

inline ErrorCode operator&(ErrorCode a, ErrorCode b) {
    return static_cast<ErrorCode>(
        static_cast<std::underlying_type_t<ErrorCode>>(a) &
        static_cast<std::underlying_type_t<ErrorCode>>(b));
}

inline ErrorCode& operator|=(ErrorCode& a, ErrorCode b) {
    a = a | b;
    return a;
}

inline bool hasError(ErrorCode value, ErrorCode check) {
    return (static_cast<std::underlying_type_t<ErrorCode>>(value) &
            static_cast<std::underlying_type_t<ErrorCode>>(check)) != 0;
}

/**
 * @brief Get error code description
 */
inline const char* getErrorDescription(ErrorCode code) {
    switch (code) {
        case ErrorCode::NONE:                   return "No error";
        case ErrorCode::FOC_FREQ_HIGH:          return "FOC frequency too high";
        case ErrorCode::OVER_VOLTAGE:           return "Over-voltage";
        case ErrorCode::UNDER_VOLTAGE:          return "Under-voltage";
        case ErrorCode::OVER_TEMPERATURE:       return "Over-temperature";
        case ErrorCode::STARTUP_FAILED:         return "Startup failed";
        case ErrorCode::ENCODER_ERROR:          return "Encoder error";
        case ErrorCode::OVER_CURRENT:           return "Over-current";
        case ErrorCode::SOFTWARE_ERROR:         return "Software/hardware mismatch";
        case ErrorCode::TEMP_SENSOR_ERROR:      return "Temperature sensor error";
        case ErrorCode::POSITION_OUT_OF_RANGE:  return "Position out of range";
        case ErrorCode::INVALID_ID:             return "Invalid motor ID";
        case ErrorCode::POSITION_TRACK_ERROR:   return "Position tracking error";
        case ErrorCode::CURRENT_SENSOR_ERROR:   return "Current sensor error";
        case ErrorCode::BRAKE_FAILED:           return "Brake failed";
        case ErrorCode::POSITION_STEP_ERROR:    return "Position step too large (>10°)";
        case ErrorCode::MULTI_TURN_LOST:        return "Multi-turn counter lost";
        default:                                return "Unknown error";
    }
}

/**
 * @brief Convert error bitmap to vector of error codes
 */
inline std::vector<ErrorCode> parseErrorBitmap(uint16_t bitmap) {
    std::vector<ErrorCode> errors;
    if (bitmap == 0) {
        errors.push_back(ErrorCode::NONE);
        return errors;
    }
    
    if (bitmap & 0x0001) errors.push_back(ErrorCode::FOC_FREQ_HIGH);
    if (bitmap & 0x0002) errors.push_back(ErrorCode::OVER_VOLTAGE);
    if (bitmap & 0x0004) errors.push_back(ErrorCode::UNDER_VOLTAGE);
    if (bitmap & 0x0008) errors.push_back(ErrorCode::OVER_TEMPERATURE);
    if (bitmap & 0x0010) errors.push_back(ErrorCode::STARTUP_FAILED);
    if (bitmap & 0x0020) errors.push_back(ErrorCode::ENCODER_ERROR);
    if (bitmap & 0x0040) errors.push_back(ErrorCode::OVER_CURRENT);
    if (bitmap & 0x0080) errors.push_back(ErrorCode::SOFTWARE_ERROR);
    if (bitmap & 0x0100) errors.push_back(ErrorCode::TEMP_SENSOR_ERROR);
    if (bitmap & 0x0200) errors.push_back(ErrorCode::POSITION_OUT_OF_RANGE);
    if (bitmap & 0x0400) errors.push_back(ErrorCode::INVALID_ID);
    if (bitmap & 0x0800) errors.push_back(ErrorCode::POSITION_TRACK_ERROR);
    if (bitmap & 0x1000) errors.push_back(ErrorCode::CURRENT_SENSOR_ERROR);
    if (bitmap & 0x2000) errors.push_back(ErrorCode::BRAKE_FAILED);
    if (bitmap & 0x4000) errors.push_back(ErrorCode::POSITION_STEP_ERROR);
    if (bitmap & 0x8000) errors.push_back(ErrorCode::MULTI_TURN_LOST);
    
    return errors;
}

// ============================================================================
// Protocol Constants
// ============================================================================

namespace ProtocolConstants {
    constexpr uint8_t CMD_READ  = 0x01;     ///< Read command
    constexpr uint8_t CMD_WRITE = 0x02;     ///< Write command
    constexpr uint8_t CMD_ERR   = 0xFF;     ///< Error response
    
    constexpr uint16_t RESPONSE_ID_OFFSET = 0x100;  ///< Response ID = Request ID + 0x100
    constexpr uint8_t BROADCAST_ID = 0x00;          ///< Broadcast ID
    constexpr uint8_t MAX_MOTOR_ID = 0x1E;          ///< Maximum valid motor ID (30)
    
    constexpr uint8_t WRITE_SUCCESS = 0x01; ///< Write success indicator
    constexpr uint8_t WRITE_FAILED  = 0x00; ///< Write failure indicator
    
    // Frame lengths
    constexpr uint8_t FRAME_HEADER_LEN = 2; ///< CMD + INDEX
    constexpr uint8_t READ_DATA_LEN = 1;    ///< Read: number of registers
    constexpr uint8_t WRITE_DATA_LEN = 2;   ///< Write: 2-byte data
    
    // Timing constraints (from documentation)
    constexpr uint32_t DISABLE_DELAY_MS = 5;    ///< 5ms delay after disable
    constexpr uint32_t SAVE_FLASH_DELAY_MS = 50; ///< 50ms delay after save to flash
}

} // namespace realman_whj
