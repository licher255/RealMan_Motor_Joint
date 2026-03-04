/**
 * @file driver.hpp
 * @brief RealMan WHJ Joint Motor Driver - Main Driver Class
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 */

#pragma once

#include "realman_whj/core/command.hpp"
#include "realman_whj/core/protocol.hpp"
#include "realman_whj/core/types.hpp"
#include "realman_whj/platform/can_interface.hpp"
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <unordered_map>
#include <functional>
#include <future>

namespace realman_whj {

/**
 * @brief Main driver class for RealMan WHJ Joint Motors
 * 
 * This class provides a high-level interface for controlling RealMan WHJ motors
 * via CAN FD communication. It handles protocol details, error recovery, and
 * provides both synchronous and asynchronous APIs.
 */
class WHJDriver {
public:
    /**
     * @brief Construct driver with CAN interface
     * 
     * @param can_iface CAN interface implementation (platform-specific)
     */
    explicit WHJDriver(std::unique_ptr<ICANInterface> can_iface = nullptr);
    
    ~WHJDriver();

    // Disable copy
    WHJDriver(const WHJDriver&) = delete;
    WHJDriver& operator=(const WHJDriver&) = delete;
    
    // Enable move
    WHJDriver(WHJDriver&&) noexcept;
    WHJDriver& operator=(WHJDriver&&) noexcept;

    // =========================================================================
    // Initialization
    // =========================================================================
    
    /**
     * @brief Initialize the driver
     * 
     * @param can_interface CAN interface name (e.g., "can0", "sim")
     * @param config Driver configuration
     * @return true if initialization successful
     */
    bool init(const std::string& can_interface, const DriverConfig& config = {});
    
    /**
     * @brief Deinitialize and cleanup
     */
    void deinit();
    
    /**
     * @brief Check if driver is initialized
     */
    bool isInitialized() const;

    // =========================================================================
    // Low-Level Register Access
    // =========================================================================
    
    /**
     * @brief Read a single 16-bit register
     * 
     * @param motor_id Target motor ID (1-30)
     * @param reg Register address
     * @return std::optional<uint16_t> Register value or nullopt on error
     */
    std::optional<uint16_t> readRegister(uint8_t motor_id, Register reg);
    
    /**
     * @brief Read multiple consecutive registers
     * 
     * @param motor_id Target motor ID
     * @param start_reg Starting register address
     * @param count Number of registers to read
     * @return std::optional<std::vector<uint16_t>> Register values or nullopt
     */
    std::optional<std::vector<uint16_t>> readRegisters(
        uint8_t motor_id, Register start_reg, uint8_t count);
    
    /**
     * @brief Write a single 16-bit register
     * 
     * @param motor_id Target motor ID
     * @param reg Register address
     * @param value Value to write
     * @return true if write successful
     */
    bool writeRegister(uint8_t motor_id, Register reg, uint16_t value);
    
    /**
     * @brief Write a 32-bit value to consecutive registers (little-endian)
     * 
     * @param motor_id Target motor ID
     * @param low_reg Low 16-bit register address
     * @param value 32-bit value to write
     * @return true if write successful
     */
    bool writeInt32(uint8_t motor_id, Register low_reg, int32_t value);

    // =========================================================================
    // High-Level Motor Control
    // =========================================================================
    
    /**
     * @brief Enable or disable motor driver
     * 
     * @param motor_id Target motor ID
     * @param enable true to enable, false to disable
     * @return true if successful
     * 
     * Note: After disabling, wait 5ms before sending new commands.
     */
    bool enableMotor(uint8_t motor_id, bool enable);
    
    /**
     * @brief Clear motor errors
     * 
     * @param motor_id Target motor ID
     * @return true if successful
     */
    bool clearError(uint8_t motor_id);
    
    /**
     * @brief Set work mode
     * 
     * @param motor_id Target motor ID
     * @param mode Work mode (OPEN_LOOP, CURRENT_MODE, SPEED_MODE, POSITION_MODE)
     * @return true if successful
     */
    bool setWorkMode(uint8_t motor_id, WorkMode mode);
    
    /**
     * @brief Get current work mode
     */
    std::optional<WorkMode> getWorkMode(uint8_t motor_id);
    
    /**
     * @brief Set target current (mA)
     * 
     * Note: Only works in CURRENT_MODE. Unit is 1mA for J10/J30, 2mA for J60.
     * 
     * @param motor_id Target motor ID
     * @param current_ma Current in mA
     * @return true if successful
     */
    bool setTargetCurrent(uint8_t motor_id, int32_t current_ma);
    
    /**
     * @brief Set target speed (RPM)
     * 
     * Note: Only works in SPEED_MODE. Positive for CW, negative for CCW.
     * 
     * @param motor_id Target motor ID
     * @param speed_rpm Speed in RPM (output shaft)
     * @return true if successful
     */
    bool setTargetSpeed(uint8_t motor_id, float speed_rpm);
    
    /**
     * @brief Set target position (degrees)
     * 
     * Note: Only works in POSITION_MODE. Range is limited by position limits.
     * 
     * @param motor_id Target motor ID
     * @param position_deg Position in degrees
     * @return true if successful
     */
    bool setTargetPosition(uint8_t motor_id, float position_deg);

    // =========================================================================
    // State Query
    // =========================================================================
    
    /**
     * @brief Get complete joint state in a single operation
     * 
     * This is more efficient than querying individual values.
     * 
     * @param motor_id Target motor ID
     * @return std::optional<JointState> Joint state or nullopt on error
     */
    std::optional<JointState> getJointState(uint8_t motor_id);
    
    /**
     * @brief Get states of multiple joints efficiently
     * 
     * @param motor_ids List of motor IDs
     * @return std::vector<std::pair<uint8_t, JointState>> Valid states
     */
    std::vector<std::pair<uint8_t, JointState>> getMultipleStates(
        const std::vector<uint8_t>& motor_ids);
    
    /**
     * @brief Get motor information (model, firmware, etc.)
     */
    std::optional<MotorInfo> getMotorInfo(uint8_t motor_id);
    
    /**
     * @brief Get current error code
     */
    std::optional<uint16_t> getErrorCode(uint8_t motor_id);
    
    /**
     * @brief Parse error code to error list
     */
    std::vector<ErrorCode> parseErrors(uint16_t error_code);

    // =========================================================================
    // Configuration
    // =========================================================================
    
    /**
     * @brief Save current configuration to Flash
     * 
     * Note: Must be disabled before saving. After saving, wait 50ms.
     * 
     * @param motor_id Target motor ID
     * @return true if successful
     */
    bool saveToFlash(uint8_t motor_id);
    
    /**
     * @brief Set current position as zero
     * 
     * @param motor_id Target motor ID
     * @return true if successful
     */
    bool setZeroPosition(uint8_t motor_id);
    
    /**
     * @brief Set position limits
     * 
     * @param motor_id Target motor ID
     * @param min_deg Minimum position in degrees
     * @param max_deg Maximum position in degrees
     * @return true if successful
     */
    bool setPositionLimits(uint8_t motor_id, float min_deg, float max_deg);
    
    /**
     * @brief Get position limits
     */
    std::optional<std::pair<float, float>> getPositionLimits(uint8_t motor_id);
    
    /**
     * @brief Set maximum speed limit
     * 
     * @param motor_id Target motor ID
     * @param max_rpm Maximum speed in RPM
     * @return true if successful
     */
    bool setMaxSpeed(uint8_t motor_id, uint16_t max_rpm);
    
    /**
     * @brief Set PID parameters for position loop
     */
    bool setPositionPID(uint8_t motor_id, const PIDParams& pid);
    
    /**
     * @brief Set PID parameters for speed loop
     */
    bool setSpeedPID(uint8_t motor_id, const PIDParams& pid);
    
    /**
     * @brief Set PID parameters for current loop
     */
    bool setCurrentPID(uint8_t motor_id, const PIDParams& pid);
    
    /**
     * @brief Get all PID parameters
     */
    std::optional<ThreeLoopPID> getPIDParams(uint8_t motor_id);
    
    /**
     * @brief Set motor ID (requires power cycle to take effect)
     */
    bool setMotorID(uint8_t current_id, uint8_t new_id);

    // =========================================================================
    // Asynchronous API
    // =========================================================================
    
    /**
     * @brief Async enable/disable motor
     */
    std::future<bool> asyncEnableMotor(uint8_t motor_id, bool enable);
    
    /**
     * @brief Async set target position
     */
    std::future<bool> asyncSetTargetPosition(uint8_t motor_id, float position_deg);
    
    /**
     * @brief Async get joint state
     */
    std::future<std::optional<JointState>> asyncGetJointState(uint8_t motor_id);
    
    /**
     * @brief Set state update callback for continuous monitoring
     */
    using StateCallback = std::function<void(uint8_t, const JointState&)>;
    void setStateCallback(StateCallback callback);
    
    /**
     * @brief Start continuous state monitoring
     * 
     * @param motor_ids List of motors to monitor
     * @param interval_ms Update interval in milliseconds
     */
    void startStateMonitoring(const std::vector<uint8_t>& motor_ids, uint32_t interval_ms);
    
    /**
     * @brief Stop continuous state monitoring
     */
    void stopStateMonitoring();

    // =========================================================================
    // Utility
    // =========================================================================
    
    /**
     * @brief Ping a motor to check if it's online
     */
    bool ping(uint8_t motor_id, uint32_t timeout_ms = 100);
    
    /**
     * @brief Scan for available motors on the bus
     * 
     * @return std::vector<uint8_t> List of motor IDs that responded
     */
    std::vector<uint8_t> scan(uint32_t timeout_ms_per_motor = 50);
    
    /**
     * @brief Get driver configuration
     */
    DriverConfig getConfig() const;
    
    /**
     * @brief Set retry count for failed operations
     */
    void setRetryCount(uint8_t retries);
    
    /**
     * @brief Get last error message
     */
    std::string getLastError() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace realman_whj
