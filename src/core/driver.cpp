/**
 * @file driver.cpp
 * @brief WHJDriver Implementation
 */

#include "realman_whj/driver.hpp"
#include <thread>
#include <algorithm>
#include <sstream>
#include <future>

namespace realman_whj {

// ============================================================================
// Implementation Class
// ============================================================================

class WHJDriver::Impl {
public:
    std::unique_ptr<ICANInterface> can_;
    DriverConfig config_;
    std::atomic<bool> initialized_{false};
    std::atomic<uint8_t> retry_count_{3};
    
    // Response handling
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    std::unordered_map<uint32_t, CANFDFrame> responses_;
    
    // State monitoring
    std::thread monitor_thread_;
    std::atomic<bool> monitoring_{false};
    std::vector<uint8_t> monitor_ids_;
    uint32_t monitor_interval_ms_{100};
    std::function<void(uint8_t, const JointState&)> state_callback_;
    std::mutex callback_mutex_;
    
    std::string last_error_;
    
    Impl(std::unique_ptr<ICANInterface> can_iface) 
        : can_(can_iface ? std::move(can_iface) : createCANInterface()) {}
    
    ~Impl() {
        stopStateMonitoring();
        if (can_) {
            can_->close();
        }
    }
    
    bool init(const std::string& can_interface, const DriverConfig& config) {
        config_ = config;
        
        // Setup response callback
        can_->registerCallback([this](const CANFDFrame& frame) {
            handleResponse(frame);
        });
        
        // Initialize CAN interface
        if (!can_->init(can_interface, config_)) {
            last_error_ = "Failed to initialize CAN interface: " + can_->getLastError();
            return false;
        }
        
        initialized_ = true;
        return true;
    }
    
    void handleResponse(const CANFDFrame& frame) {
        // Store response for correlation
        std::lock_guard<std::mutex> lock(response_mutex_);
        responses_[frame.id] = frame;
        response_cv_.notify_all();
    }
    
    std::optional<CANFDFrame> sendAndWait(const CANFDFrame& request, uint32_t timeout_ms) {
        uint32_t response_id = Protocol::getResponseId(request.id);
        
        // Clear previous response
        {
            std::lock_guard<std::mutex> lock(response_mutex_);
            responses_.erase(response_id);
        }
        
        // Send request
        if (!can_->send(request)) {
            last_error_ = "Failed to send CAN frame";
            return std::nullopt;
        }
        
        // Wait for response
        std::unique_lock<std::mutex> lock(response_mutex_);
        bool received = response_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
            [this, response_id] {
                return responses_.find(response_id) != responses_.end();
            });
        
        if (!received) {
            last_error_ = "Response timeout";
            return std::nullopt;
        }
        
        auto frame = responses_[response_id];
        responses_.erase(response_id);
        return frame;
    }
    
    std::optional<uint16_t> readRegister(uint8_t motor_id, Register reg) {
        if (!initialized_) {
            last_error_ = "Driver not initialized";
            return std::nullopt;
        }
        
        if (motor_id < 1 || motor_id > ProtocolConstants::MAX_MOTOR_ID) {
            last_error_ = "Invalid motor ID";
            return std::nullopt;
        }
        
        auto frame = Protocol::buildReadFrame(motor_id, reg, 1);
        
        for (uint8_t retry = 0; retry < retry_count_; ++retry) {
            auto response = sendAndWait(frame, config_.timeout_ms);
            if (!response) continue;
            
            try {
                auto values = Protocol::parseReadResponse(*response, motor_id, reg);
                if (!values.empty()) {
                    return values[0];
                }
            } catch (const std::exception& e) {
                last_error_ = e.what();
            }
        }
        
        return std::nullopt;
    }
    
    std::optional<std::vector<uint16_t>> readRegisters(
        uint8_t motor_id, Register start_reg, uint8_t count) {
        if (!initialized_) {
            last_error_ = "Driver not initialized";
            return std::nullopt;
        }
        
        auto frame = Protocol::buildReadFrame(motor_id, start_reg, count);
        
        for (uint8_t retry = 0; retry < retry_count_; ++retry) {
            auto response = sendAndWait(frame, config_.timeout_ms);
            if (!response) continue;
            
            try {
                return Protocol::parseReadResponse(*response, motor_id, start_reg);
            } catch (const std::exception& e) {
                last_error_ = e.what();
            }
        }
        
        return std::nullopt;
    }
    
    bool writeRegister(uint8_t motor_id, Register reg, uint16_t value) {
        if (!initialized_) {
            last_error_ = "Driver not initialized";
            return false;
        }
        
        auto frame = Protocol::buildWriteFrame(motor_id, reg, value);
        
        for (uint8_t retry = 0; retry < retry_count_; ++retry) {
            auto response = sendAndWait(frame, config_.timeout_ms);
            if (!response) continue;
            
            try {
                return Protocol::parseWriteResponse(*response, motor_id);
            } catch (const std::exception& e) {
                last_error_ = e.what();
            }
        }
        
        return false;
    }
    
    bool writeInt32(uint8_t motor_id, Register low_reg, int32_t value) {
        // Write low 16-bit
        uint16_t low = value & 0xFFFF;
        if (!writeRegister(motor_id, low_reg, low)) {
            return false;
        }
        
        // Write high 16-bit
        Register high_reg = static_cast<Register>(toUnderlying(low_reg) + 1);
        uint16_t high = (value >> 16) & 0xFFFF;
        return writeRegister(motor_id, high_reg, high);
    }
    
    std::optional<JointState> getJointState(uint8_t motor_id) {
        // Batch read: current, speed, position (6 registers)
        auto values = readRegisters(motor_id, Register::CUR_CURRENT_L, 6);
        if (!values || values->size() < 6) {
            return std::nullopt;
        }
        
        JointState state;
        state.motor_id = motor_id;
        state.timestamp = std::chrono::steady_clock::now();
        state.valid = true;
        
        // Parse current (32-bit)
        state.current_ma = unpackInt32((*values)[0], (*values)[1]);
        
        // Parse speed (32-bit, 0.02 RPM units)
        int32_t speed_units = unpackInt32((*values)[2], (*values)[3]);
        state.speed_rpm = actualSpeedUnitsToRpm(speed_units);
        
        // Parse position (32-bit, 0.0001 degree units)
        int32_t pos_units = unpackInt32((*values)[4], (*values)[5]);
        state.position_deg = positionUnitsToDeg(pos_units);
        
        // Read additional status (voltage, temp, error, enable)
        auto status = readRegisters(motor_id, Register::SYS_VOLTAGE, 4);
        if (status && status->size() >= 4) {
            state.voltage_v = (*status)[0] * Units::VOLTAGE_SCALE;
            state.temperature_c = (*status)[1] * Units::TEMP_SCALE;
            state.error_code = (*status)[2];
            state.is_enabled = ((*status)[3] & 0x01) != 0;
        }
        
        // Read work mode
        auto mode = readRegister(motor_id, Register::TAG_WORK_MODE);
        if (mode) {
            state.work_mode = static_cast<uint8_t>(*mode);
        }
        
        return state;
    }
    
    void monitorLoop() {
        while (monitoring_) {
            auto start = std::chrono::steady_clock::now();
            
            for (uint8_t id : monitor_ids_) {
                if (!monitoring_) break;
                
                auto state = getJointState(id);
                if (state && state->valid) {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if (state_callback_) {
                        state_callback_(id, *state);
                    }
                }
            }
            
            // Calculate sleep time to maintain interval
            auto elapsed = std::chrono::steady_clock::now() - start;
            auto sleep_time = std::chrono::milliseconds(monitor_interval_ms_) - elapsed;
            if (sleep_time > std::chrono::milliseconds(0)) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    }
    
    void startStateMonitoring(const std::vector<uint8_t>& motor_ids, uint32_t interval_ms) {
        stopStateMonitoring();
        
        monitor_ids_ = motor_ids;
        monitor_interval_ms_ = interval_ms;
        monitoring_ = true;
        
        monitor_thread_ = std::thread(&Impl::monitorLoop, this);
    }
    
    void stopStateMonitoring() {
        monitoring_ = false;
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }
    
    bool ping(uint8_t motor_id, uint32_t timeout_ms) {
        auto result = readRegister(motor_id, Register::SYS_MODEL_TYPE);
        return result.has_value();
    }
    
    std::vector<uint8_t> scan(uint32_t timeout_ms_per_motor) {
        std::vector<uint8_t> found;
        
        for (uint8_t id = 1; id <= ProtocolConstants::MAX_MOTOR_ID; ++id) {
            if (ping(id, timeout_ms_per_motor)) {
                found.push_back(id);
            }
        }
        
        return found;
    }
};

// ============================================================================
// WHJDriver Public Methods
// ============================================================================

WHJDriver::WHJDriver(std::unique_ptr<ICANInterface> can_iface)
    : pImpl_(std::make_unique<Impl>(std::move(can_iface))) {}

WHJDriver::~WHJDriver() = default;

WHJDriver::WHJDriver(WHJDriver&&) noexcept = default;
WHJDriver& WHJDriver::operator=(WHJDriver&&) noexcept = default;

bool WHJDriver::init(const std::string& can_interface, const DriverConfig& config) {
    return pImpl_->init(can_interface, config);
}

void WHJDriver::deinit() {
    pImpl_->initialized_ = false;
    if (pImpl_->can_) {
        pImpl_->can_->close();
    }
}

bool WHJDriver::isInitialized() const {
    return pImpl_->initialized_;
}

std::optional<uint16_t> WHJDriver::readRegister(uint8_t motor_id, Register reg) {
    return pImpl_->readRegister(motor_id, reg);
}

std::optional<std::vector<uint16_t>> WHJDriver::readRegisters(
    uint8_t motor_id, Register start_reg, uint8_t count) {
    return pImpl_->readRegisters(motor_id, start_reg, count);
}

bool WHJDriver::writeRegister(uint8_t motor_id, Register reg, uint16_t value) {
    return pImpl_->writeRegister(motor_id, reg, value);
}

bool WHJDriver::writeInt32(uint8_t motor_id, Register low_reg, int32_t value) {
    return pImpl_->writeInt32(motor_id, low_reg, value);
}

bool WHJDriver::enableMotor(uint8_t motor_id, bool enable) {
    return pImpl_->writeRegister(motor_id, Register::SYS_ENABLE_DRIVER, enable ? 1 : 0);
}

bool WHJDriver::clearError(uint8_t motor_id) {
    return pImpl_->writeRegister(motor_id, Register::SYS_CLEAR_ERROR, 1);
}

bool WHJDriver::setWorkMode(uint8_t motor_id, WorkMode mode) {
    return pImpl_->writeRegister(motor_id, Register::TAG_WORK_MODE, static_cast<uint16_t>(mode));
}

std::optional<WorkMode> WHJDriver::getWorkMode(uint8_t motor_id) {
    auto value = pImpl_->readRegister(motor_id, Register::TAG_WORK_MODE);
    if (value) {
        return static_cast<WorkMode>(*value);
    }
    return std::nullopt;
}

bool WHJDriver::setTargetCurrent(uint8_t motor_id, int32_t current_ma) {
    return pImpl_->writeInt32(motor_id, Register::TAG_CURRENT_L, current_ma);
}

bool WHJDriver::setTargetSpeed(uint8_t motor_id, float speed_rpm) {
    int32_t speed_units = rpmToTargetSpeedUnits(speed_rpm);
    return pImpl_->writeInt32(motor_id, Register::TAG_SPEED_L, speed_units);
}

bool WHJDriver::setTargetPosition(uint8_t motor_id, float position_deg) {
    int32_t pos_units = degToPositionUnits(position_deg);
    return pImpl_->writeInt32(motor_id, Register::TAG_POSITION_L, pos_units);
}

std::optional<JointState> WHJDriver::getJointState(uint8_t motor_id) {
    return pImpl_->getJointState(motor_id);
}

std::vector<std::pair<uint8_t, JointState>> WHJDriver::getMultipleStates(
    const std::vector<uint8_t>& motor_ids) {
    std::vector<std::pair<uint8_t, JointState>> results;
    
    for (uint8_t id : motor_ids) {
        auto state = getJointState(id);
        if (state && state->valid) {
            results.push_back({id, *state});
        }
    }
    
    return results;
}

std::optional<MotorInfo> WHJDriver::getMotorInfo(uint8_t motor_id) {
    auto values = pImpl_->readRegisters(motor_id, Register::SYS_ID, 7);
    if (!values || values->size() < 7) {
        return std::nullopt;
    }
    
    MotorInfo info;
    info.motor_id = (*values)[0];
    info.model_type = (*values)[1];
    info.firmware_version = (*values)[2];
    // values[3] = error code
    // values[4] = voltage
    // values[5] = temp
    info.reduction_ratio = (*values)[6];
    
    return info;
}

std::optional<uint16_t> WHJDriver::getErrorCode(uint8_t motor_id) {
    return pImpl_->readRegister(motor_id, Register::SYS_ERROR);
}

std::vector<ErrorCode> WHJDriver::parseErrors(uint16_t error_code) {
    return parseErrorBitmap(error_code);
}

bool WHJDriver::saveToFlash(uint8_t motor_id) {
    return pImpl_->writeRegister(motor_id, Register::SYS_SAVE_TO_FLASH, 1);
}

bool WHJDriver::setZeroPosition(uint8_t motor_id) {
    return pImpl_->writeRegister(motor_id, Register::SYS_SET_ZERO_POS, 1);
}

bool WHJDriver::setPositionLimits(uint8_t motor_id, float min_deg, float max_deg) {
    int32_t min_units = degToPositionUnits(min_deg);
    int32_t max_units = degToPositionUnits(max_deg);
    
    // Write min position
    if (!pImpl_->writeInt32(motor_id, Register::LIT_MIN_POSITION_L, min_units)) {
        return false;
    }
    
    // Write max position
    return pImpl_->writeInt32(motor_id, Register::LIT_MAX_POSITION_L, max_units);
}

std::optional<std::pair<float, float>> WHJDriver::getPositionLimits(uint8_t motor_id) {
    auto values = pImpl_->readRegisters(motor_id, Register::LIT_MIN_POSITION_L, 4);
    if (!values || values->size() < 4) {
        return std::nullopt;
    }
    
    int32_t min_units = unpackInt32((*values)[0], (*values)[1]);
    int32_t max_units = unpackInt32((*values)[2], (*values)[3]);
    
    return std::make_pair(
        positionUnitsToDeg(min_units),
        positionUnitsToDeg(max_units)
    );
}

bool WHJDriver::setMaxSpeed(uint8_t motor_id, uint16_t max_rpm) {
    return pImpl_->writeRegister(motor_id, Register::LIT_MAX_SPEED, max_rpm);
}

bool WHJDriver::setPositionPID(uint8_t motor_id, const PIDParams& pid) {
    if (!pImpl_->writeRegister(motor_id, Register::SEV_POSITION_P, pid.kp)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_POSITION_I, pid.ki)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_POSITION_D, pid.kd)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_POSITION_DS, pid.dead_zone)) return false;
    return true;
}

bool WHJDriver::setSpeedPID(uint8_t motor_id, const PIDParams& pid) {
    if (!pImpl_->writeRegister(motor_id, Register::SEV_SPEED_P, pid.kp)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_SPEED_I, pid.ki)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_SPEED_D, pid.kd)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_SPEED_DS, pid.dead_zone)) return false;
    return true;
}

bool WHJDriver::setCurrentPID(uint8_t motor_id, const PIDParams& pid) {
    if (!pImpl_->writeRegister(motor_id, Register::SEV_CURRENT_P, pid.kp)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_CURRENT_I, pid.ki)) return false;
    if (!pImpl_->writeRegister(motor_id, Register::SEV_CURRENT_D, pid.kd)) return false;
    return true;
}

std::optional<ThreeLoopPID> WHJDriver::getPIDParams(uint8_t motor_id) {
    auto values = pImpl_->readRegisters(motor_id, Register::SEV_CURRENT_P, 13);
    if (!values || values->size() < 13) {
        return std::nullopt;
    }
    
    ThreeLoopPID pid;
    pid.current.kp = (*values)[0];
    pid.current.ki = (*values)[1];
    pid.current.kd = (*values)[2];
    pid.speed.kp = (*values)[3];
    pid.speed.ki = (*values)[4];
    pid.speed.kd = (*values)[5];
    pid.speed.dead_zone = (*values)[6];
    pid.position.kp = (*values)[7];
    pid.position.ki = (*values)[8];
    pid.position.kd = (*values)[9];
    pid.position.dead_zone = (*values)[10];
    pid.pos_smooth_coeff = (*values)[11];
    pid.speed_ff_coeff = (*values)[12];
    
    return pid;
}

bool WHJDriver::setMotorID(uint8_t current_id, uint8_t new_id) {
    return pImpl_->writeRegister(current_id, Register::SYS_ID, new_id);
}

// Async methods
std::future<bool> WHJDriver::asyncEnableMotor(uint8_t motor_id, bool enable) {
    return std::async(std::launch::async, [this, motor_id, enable]() {
        return enableMotor(motor_id, enable);
    });
}

std::future<bool> WHJDriver::asyncSetTargetPosition(uint8_t motor_id, float position_deg) {
    return std::async(std::launch::async, [this, motor_id, position_deg]() {
        return setTargetPosition(motor_id, position_deg);
    });
}

std::future<std::optional<JointState>> WHJDriver::asyncGetJointState(uint8_t motor_id) {
    return std::async(std::launch::async, [this, motor_id]() {
        return getJointState(motor_id);
    });
}

void WHJDriver::setStateCallback(StateCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl_->callback_mutex_);
    pImpl_->state_callback_ = callback;
}

void WHJDriver::startStateMonitoring(const std::vector<uint8_t>& motor_ids, uint32_t interval_ms) {
    pImpl_->startStateMonitoring(motor_ids, interval_ms);
}

void WHJDriver::stopStateMonitoring() {
    pImpl_->stopStateMonitoring();
}

bool WHJDriver::ping(uint8_t motor_id, uint32_t timeout_ms) {
    return pImpl_->ping(motor_id, timeout_ms);
}

std::vector<uint8_t> WHJDriver::scan(uint32_t timeout_ms_per_motor) {
    return pImpl_->scan(timeout_ms_per_motor);
}

DriverConfig WHJDriver::getConfig() const {
    return pImpl_->config_;
}

void WHJDriver::setRetryCount(uint8_t retries) {
    pImpl_->retry_count_ = retries;
}

std::string WHJDriver::getLastError() const {
    return pImpl_->last_error_;
}

} // namespace realman_whj
