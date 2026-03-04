/**
 * @file protocol.hpp
 * @brief RealMan WHJ Joint Motor Driver - Protocol Frame Builder/Parser
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 */

#pragma once

#include "realman_whj/core/types.hpp"
#include "realman_whj/core/command.hpp"
#include <cstring>
#include <stdexcept>
#include <utility>

namespace realman_whj {

/**
 * @brief CAN FD Protocol frame builder and parser
 * 
 * Handles the low-level construction and parsing of CAN FD frames
 * according to RealMan WHJ communication protocol.
 */
class Protocol {
public:
    /**
     * @brief Build a read command frame
     * 
     * @param motor_id Target motor ID (0x00-0x1E)
     * @param reg Starting register address
     * @param count Number of registers to read (1-N)
     * @return CANFDFrame Constructed frame ready for transmission
     */
    static CANFDFrame buildReadFrame(uint8_t motor_id, Register reg, uint8_t count = 1) {
        CANFDFrame frame;
        frame.id = motor_id & 0x1F;  // 5-bit ID
        frame.is_extended = false;
        frame.is_fd = true;
        frame.bit_rate_switch = true;
        
        frame.data[0] = ProtocolConstants::CMD_READ;
        frame.data[1] = toUnderlying(reg);
        frame.data[2] = count;
        frame.len = 3;
        
        return frame;
    }
    
    /**
     * @brief Build a write command frame (16-bit value)
     * 
     * @param motor_id Target motor ID (0x00-0x1E)
     * @param reg Register address
     * @param value 16-bit value to write (little-endian)
     * @return CANFDFrame Constructed frame ready for transmission
     */
    static CANFDFrame buildWriteFrame(uint8_t motor_id, Register reg, uint16_t value) {
        CANFDFrame frame;
        frame.id = motor_id & 0x1F;
        frame.is_extended = false;
        frame.is_fd = true;
        frame.bit_rate_switch = true;
        
        frame.data[0] = ProtocolConstants::CMD_WRITE;
        frame.data[1] = toUnderlying(reg);
        frame.data[2] = value & 0xFF;        // Low byte
        frame.data[3] = (value >> 8) & 0xFF; // High byte
        frame.len = 4;
        
        return frame;
    }
    
    /**
     * @brief Build a write command frame for 32-bit value (low 16-bit)
     * 
     * @param motor_id Target motor ID
     * @param low_reg Low 16-bit register address
     * @param value 32-bit value (little-endian)
     * @return CANFDFrame Constructed frame
     */
    static CANFDFrame buildWrite32Frame(uint8_t motor_id, Register low_reg, int32_t value) {
        uint16_t low = static_cast<uint16_t>(value & 0xFFFF);
        return buildWriteFrame(motor_id, low_reg, low);
    }
    
    /**
     * @brief Build a write command frame for the high 16-bit of a 32-bit value
     */
    static CANFDFrame buildWrite32HighFrame(uint8_t motor_id, Register low_reg, int32_t value) {
        // High register is always at low_reg + 1
        Register high_reg = static_cast<Register>(toUnderlying(low_reg) + 1);
        uint16_t high = static_cast<uint16_t>((value >> 16) & 0xFFFF);
        return buildWriteFrame(motor_id, high_reg, high);
    }
    
    /**
     * @brief Parse a read response frame
     * 
     * @param frame Received CAN frame
     * @param expected_id Expected motor ID
     * @param expected_reg Expected register address
     * @return std::vector<uint16_t> Vector of register values
     * @throws std::runtime_error if frame is invalid
     */
    static std::vector<uint16_t> parseReadResponse(
        const CANFDFrame& frame,
        uint8_t expected_id,
        Register expected_reg) {
        
        validateResponse(frame, expected_id, ProtocolConstants::CMD_READ);
        
        // Check register address
        if (frame.data[1] != toUnderlying(expected_reg)) {
            throw std::runtime_error("Register address mismatch in response");
        }
        
        // Parse data: each register is 2 bytes, little-endian
        std::vector<uint16_t> values;
        for (size_t i = 2; i < frame.len; i += 2) {
            if (i + 1 < frame.len) {
                uint16_t value = frame.data[i] | (frame.data[i + 1] << 8);
                values.push_back(value);
            }
        }
        
        return values;
    }
    
    /**
     * @brief Parse a write response frame
     * 
     * @param frame Received CAN frame
     * @param expected_id Expected motor ID
     * @return bool true if write successful, false otherwise
     * @throws std::runtime_error if frame is invalid
     */
    static bool parseWriteResponse(const CANFDFrame& frame, uint8_t expected_id) {
        validateResponse(frame, expected_id, ProtocolConstants::CMD_WRITE);
        
        // Write response: data[2] = 0x01 (success) or 0x00 (failed)
        if (frame.len < 3) {
            throw std::runtime_error("Write response too short");
        }
        
        return frame.data[2] == ProtocolConstants::WRITE_SUCCESS;
    }
    
    /**
     * @brief Parse error response frame
     * 
     * @param frame Received CAN frame
     * @return ProtocolError Error type
     */
    static ProtocolError parseErrorResponse(const CANFDFrame& frame) {
        if (frame.data[0] != ProtocolConstants::CMD_ERR) {
            return ProtocolError::NONE;
        }
        
        if (frame.len < 2) {
            return ProtocolError::FORMAT_ERROR;
        }
        
        return static_cast<ProtocolError>(frame.data[1]);
    }
    
    /**
     * @brief Extract 32-bit value from two consecutive 16-bit registers
     * 
     * @param values Vector of register values (from parseReadResponse)
     * @param index Starting index in the vector
     * @return int32_t Sign-extended 32-bit value
     */
    static int32_t extractInt32(const std::vector<uint16_t>& values, size_t index) {
        if (index + 1 >= values.size()) {
            throw std::out_of_range("Not enough data to extract 32-bit value");
        }
        
        uint32_t low = values[index];
        uint32_t high = values[index + 1];
        uint32_t combined = (high << 16) | low;
        
        // Sign extend
        return static_cast<int32_t>(combined);
    }
    
    /**
     * @brief Check if a response frame is valid
     * 
     * @param frame Received frame
     * @param expected_id Expected motor ID
     * @return true if valid response
     */
    static bool isValidResponse(const CANFDFrame& frame, uint8_t expected_id) {
        // Response ID should be request ID + 0x100
        uint32_t expected_response_id = (expected_id & 0x1F) + ProtocolConstants::RESPONSE_ID_OFFSET;
        
        if (frame.id != expected_response_id) {
            return false;
        }
        
        // Minimum frame length for valid response
        if (frame.len < 2) {
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief Get response ID for a given request ID
     */
    static uint32_t getResponseId(uint8_t motor_id) {
        return (motor_id & 0x1F) + ProtocolConstants::RESPONSE_ID_OFFSET;
    }
    
    /**
     * @brief Build enable motor command
     */
    static CANFDFrame buildEnableMotor(uint8_t motor_id, bool enable) {
        return buildWriteFrame(motor_id, Register::SYS_ENABLE_DRIVER, enable ? 1 : 0);
    }
    
    /**
     * @brief Build clear error command
     */
    static CANFDFrame buildClearError(uint8_t motor_id) {
        return buildWriteFrame(motor_id, Register::SYS_CLEAR_ERROR, 1);
    }
    
    /**
     * @brief Build set work mode command
     */
    static CANFDFrame buildSetWorkMode(uint8_t motor_id, WorkMode mode) {
        return buildWriteFrame(motor_id, Register::TAG_WORK_MODE, static_cast<uint16_t>(mode));
    }
    
    /**
     * @brief Build set target current command (32-bit)
     */
    static std::pair<CANFDFrame, CANFDFrame> buildSetTargetCurrent(
        uint8_t motor_id, int32_t current_ma) {
        return {
            buildWrite32Frame(motor_id, Register::TAG_CURRENT_L, current_ma),
            buildWrite32HighFrame(motor_id, Register::TAG_CURRENT_L, current_ma)
        };
    }
    
    /**
     * @brief Build set target speed command (32-bit)
     */
    static std::pair<CANFDFrame, CANFDFrame> buildSetTargetSpeed(
        uint8_t motor_id, float speed_rpm) {
        int32_t speed_units = rpmToTargetSpeedUnits(speed_rpm);
        return {
            buildWrite32Frame(motor_id, Register::TAG_SPEED_L, speed_units),
            buildWrite32HighFrame(motor_id, Register::TAG_SPEED_L, speed_units)
        };
    }
    
    /**
     * @brief Build set target position command (32-bit)
     */
    static std::pair<CANFDFrame, CANFDFrame> buildSetTargetPosition(
        uint8_t motor_id, float position_deg) {
        int32_t pos_units = degToPositionUnits(position_deg);
        return {
            buildWrite32Frame(motor_id, Register::TAG_POSITION_L, pos_units),
            buildWrite32HighFrame(motor_id, Register::TAG_POSITION_L, pos_units)
        };
    }
    
    /**
     * @brief Build read current state command (batch read multiple registers)
     */
    static CANFDFrame buildReadState(uint8_t motor_id) {
        // Read: CUR_CURRENT_L to CUR_POSITION_H (6 registers)
        return buildReadFrame(motor_id, Register::CUR_CURRENT_L, 6);
    }
    
    /**
     * @brief Build save to flash command
     */
    static CANFDFrame buildSaveToFlash(uint8_t motor_id) {
        return buildWriteFrame(motor_id, Register::SYS_SAVE_TO_FLASH, 1);
    }
    
    /**
     * @brief Build set zero position command
     */
    static CANFDFrame buildSetZeroPosition(uint8_t motor_id) {
        return buildWriteFrame(motor_id, Register::SYS_SET_ZERO_POS, 1);
    }
    
private:
    static void validateResponse(
        const CANFDFrame& frame,
        uint8_t expected_id,
        uint8_t expected_cmd) {
        
        uint32_t expected_response_id = (expected_id & 0x1F) + ProtocolConstants::RESPONSE_ID_OFFSET;
        
        if (frame.id != expected_response_id) {
            throw std::runtime_error("Response ID mismatch");
        }
        
        if (frame.len < 2) {
            throw std::runtime_error("Response frame too short");
        }
        
        // Check for error response
        if (frame.data[0] == ProtocolConstants::CMD_ERR) {
            ProtocolError err = parseErrorResponse(frame);
            throw std::runtime_error(std::string("Protocol error: ") + 
                std::to_string(static_cast<int>(err)));
        }
        
        if (frame.data[0] != expected_cmd) {
            throw std::runtime_error("Command type mismatch in response");
        }
    }
    
    // Namespace alias for protocol constants
    struct ProtocolConsts {
        static constexpr uint8_t CMD_READ  = 0x01;
        static constexpr uint8_t CMD_WRITE = 0x02;
        static constexpr uint8_t CMD_ERR   = 0xFF;
        static constexpr uint16_t RESPONSE_ID_OFFSET = 0x100;
        static constexpr uint8_t WRITE_SUCCESS = 0x01;
    };
};

} // namespace realman_whj
