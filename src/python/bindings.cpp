/**
 * @file bindings.cpp
 * @brief Python bindings using pybind11
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include "realman_whj/driver.hpp"

namespace py = pybind11;
using namespace realman_whj;

PYBIND11_MODULE(realman_whj, m) {
    m.doc() = "RealMan WHJ Joint Motor Driver - Python Bindings";
    
    // =========================================================================
    // Enums
    // =========================================================================
    
    py::enum_<CommandType>(m, "CommandType")
        .value("READ", CommandType::READ)
        .value("WRITE", CommandType::WRITE)
        .value("ERROR", CommandType::ERROR);
    
    py::enum_<WorkMode>(m, "WorkMode")
        .value("OPEN_LOOP", WorkMode::OPEN_LOOP)
        .value("CURRENT_MODE", WorkMode::CURRENT_MODE)
        .value("SPEED_MODE", WorkMode::SPEED_MODE)
        .value("POSITION_MODE", WorkMode::POSITION_MODE);
    
    py::enum_<JointModel>(m, "JointModel")
        .value("J14", JointModel::J14)
        .value("J17", JointModel::J17)
        .value("J20", JointModel::J20)
        .value("J25", JointModel::J25)
        .value("GRIPPER", JointModel::GRIPPER)
        .value("J3", JointModel::J3);
    
    py::enum_<ErrorCode>(m, "ErrorCode")
        .value("NONE", ErrorCode::NONE)
        .value("FOC_FREQ_HIGH", ErrorCode::FOC_FREQ_HIGH)
        .value("OVER_VOLTAGE", ErrorCode::OVER_VOLTAGE)
        .value("UNDER_VOLTAGE", ErrorCode::UNDER_VOLTAGE)
        .value("OVER_TEMPERATURE", ErrorCode::OVER_TEMPERATURE)
        .value("STARTUP_FAILED", ErrorCode::STARTUP_FAILED)
        .value("ENCODER_ERROR", ErrorCode::ENCODER_ERROR)
        .value("OVER_CURRENT", ErrorCode::OVER_CURRENT)
        .value("SOFTWARE_ERROR", ErrorCode::SOFTWARE_ERROR)
        .value("TEMP_SENSOR_ERROR", ErrorCode::TEMP_SENSOR_ERROR)
        .value("POSITION_OUT_OF_RANGE", ErrorCode::POSITION_OUT_OF_RANGE)
        .value("INVALID_ID", ErrorCode::INVALID_ID)
        .value("POSITION_TRACK_ERROR", ErrorCode::POSITION_TRACK_ERROR)
        .value("CURRENT_SENSOR_ERROR", ErrorCode::CURRENT_SENSOR_ERROR)
        .value("BRAKE_FAILED", ErrorCode::BRAKE_FAILED)
        .value("POSITION_STEP_ERROR", ErrorCode::POSITION_STEP_ERROR)
        .value("MULTI_TURN_LOST", ErrorCode::MULTI_TURN_LOST);
    
    py::enum_<Register>(m, "Register")
        .value("SYS_ID", Register::SYS_ID)
        .value("SYS_MODEL_TYPE", Register::SYS_MODEL_TYPE)
        .value("SYS_FW_VERSION", Register::SYS_FW_VERSION)
        .value("SYS_ERROR", Register::SYS_ERROR)
        .value("SYS_VOLTAGE", Register::SYS_VOLTAGE)
        .value("SYS_TEMP", Register::SYS_TEMP)
        .value("SYS_ENABLE_DRIVER", Register::SYS_ENABLE_DRIVER)
        .value("SYS_SAVE_TO_FLASH", Register::SYS_SAVE_TO_FLASH)
        .value("SYS_SET_ZERO_POS", Register::SYS_SET_ZERO_POS)
        .value("SYS_CLEAR_ERROR", Register::SYS_CLEAR_ERROR)
        .value("CUR_CURRENT_L", Register::CUR_CURRENT_L)
        .value("CUR_CURRENT_H", Register::CUR_CURRENT_H)
        .value("CUR_SPEED_L", Register::CUR_SPEED_L)
        .value("CUR_SPEED_H", Register::CUR_SPEED_H)
        .value("CUR_POSITION_L", Register::CUR_POSITION_L)
        .value("CUR_POSITION_H", Register::CUR_POSITION_H)
        .value("TAG_WORK_MODE", Register::TAG_WORK_MODE)
        .value("TAG_CURRENT_L", Register::TAG_CURRENT_L)
        .value("TAG_CURRENT_H", Register::TAG_CURRENT_H)
        .value("TAG_SPEED_L", Register::TAG_SPEED_L)
        .value("TAG_SPEED_H", Register::TAG_SPEED_H)
        .value("TAG_POSITION_L", Register::TAG_POSITION_L)
        .value("TAG_POSITION_H", Register::TAG_POSITION_H)
        .value("LIT_MAX_SPEED", Register::LIT_MAX_SPEED)
        .value("LIT_MAX_ACC", Register::LIT_MAX_ACC);
    
    // =========================================================================
    // Structs
    // =========================================================================
    
    py::class_<DriverConfig>(m, "DriverConfig")
        .def(py::init<>())
        .def_readwrite("can_bitrate", &DriverConfig::can_bitrate)
        .def_readwrite("can_fd_bitrate", &DriverConfig::can_fd_bitrate)
        .def_readwrite("timeout_ms", &DriverConfig::timeout_ms)
        .def_readwrite("default_retries", &DriverConfig::default_retries)
        .def_readwrite("enable_logging", &DriverConfig::enable_logging)
        .def_readwrite("log_level", &DriverConfig::log_level);
    
    py::class_<JointState>(m, "JointState")
        .def(py::init<>())
        .def_readwrite("motor_id", &JointState::motor_id)
        .def_readwrite("current_ma", &JointState::current_ma)
        .def_readwrite("speed_rpm", &JointState::speed_rpm)
        .def_readwrite("position_deg", &JointState::position_deg)
        .def_readwrite("voltage_v", &JointState::voltage_v)
        .def_readwrite("temperature_c", &JointState::temperature_c)
        .def_readwrite("error_code", &JointState::error_code)
        .def_readwrite("is_enabled", &JointState::is_enabled)
        .def_readwrite("work_mode", &JointState::work_mode)
        .def_readwrite("valid", &JointState::valid)
        .def("__repr__", [](const JointState& s) {
            return "<JointState id=" + std::to_string(s.motor_id) +
                   " pos=" + std::to_string(s.position_deg) + "°>";
        });
    
    py::class_<JointTarget>(m, "JointTarget")
        .def(py::init<>())
        .def_readwrite("current_ma", &JointTarget::current_ma)
        .def_readwrite("speed_rpm", &JointTarget::speed_rpm)
        .def_readwrite("position_deg", &JointTarget::position_deg)
        .def_readwrite("work_mode", &JointTarget::work_mode);
    
    py::class_<PIDParams>(m, "PIDParams")
        .def(py::init<>())
        .def(py::init<uint16_t, uint16_t, uint16_t, uint16_t>(),
             py::arg("kp"), py::arg("ki"), py::arg("kd"), py::arg("dead_zone") = 0)
        .def_readwrite("kp", &PIDParams::kp)
        .def_readwrite("ki", &PIDParams::ki)
        .def_readwrite("kd", &PIDParams::kd)
        .def_readwrite("dead_zone", &PIDParams::dead_zone);
    
    py::class_<MotorInfo>(m, "MotorInfo")
        .def(py::init<>())
        .def_readwrite("motor_id", &MotorInfo::motor_id)
        .def_readwrite("model_type", &MotorInfo::model_type)
        .def_readwrite("firmware_version", &MotorInfo::firmware_version)
        .def_readwrite("reduction_ratio", &MotorInfo::reduction_ratio)
        .def("get_model_name", &MotorInfo::getModelName)
        .def("get_firmware_string", &MotorInfo::getFirmwareString);
    
    // =========================================================================
    // Utility Functions
    // =========================================================================
    
    m.def("deg_to_position_units", &degToPositionUnits, "Convert degrees to motor units");
    m.def("position_units_to_deg", &positionUnitsToDeg, "Convert motor units to degrees");
    m.def("rpm_to_target_speed_units", &rpmToTargetSpeedUnits, "Convert RPM to target speed units");
    m.def("target_speed_units_to_rpm", &targetSpeedUnitsToRpm, "Convert target speed units to RPM");
    m.def("get_joint_model_name", &getJointModelName, "Get model name string");
    m.def("get_error_description", &getErrorDescription, "Get error description");
    m.def("work_mode_to_string", &workModeToString, "Get work mode string");
    m.def("parse_error_bitmap", &parseErrorBitmap, "Parse error bitmap to list");
    m.def("get_current_scale", &getCurrentScale, "Get current scale for model");
    m.def("list_available_interfaces", &listAvailableInterfaces, "List available CAN interfaces");
    
    // =========================================================================
    // Main Driver Class
    // =========================================================================
    
    py::class_<WHJDriver>(m, "WHJDriver")
        .def(py::init<>())
        
        // Initialization
        .def("init", &WHJDriver::init, 
             py::arg("can_interface"), py::arg("config") = DriverConfig{},
             "Initialize the driver")
        .def("deinit", &WHJDriver::deinit, "Deinitialize the driver")
        .def("is_initialized", &WHJDriver::isInitialized, "Check if initialized")
        
        // Register access
        .def("read_register", &WHJDriver::readRegister, 
             "Read a single register", py::arg("motor_id"), py::arg("reg"))
        .def("read_registers", &WHJDriver::readRegisters,
             "Read multiple registers", py::arg("motor_id"), py::arg("start_reg"), py::arg("count"))
        .def("write_register", &WHJDriver::writeRegister,
             "Write a register", py::arg("motor_id"), py::arg("reg"), py::arg("value"))
        .def("write_int32", &WHJDriver::writeInt32,
             "Write 32-bit value", py::arg("motor_id"), py::arg("low_reg"), py::arg("value"))
        
        // Motor control
        .def("enable_motor", &WHJDriver::enableMotor,
             "Enable/disable motor", py::arg("motor_id"), py::arg("enable"))
        .def("clear_error", &WHJDriver::clearError,
             "Clear errors", py::arg("motor_id"))
        .def("set_work_mode", &WHJDriver::setWorkMode,
             "Set work mode", py::arg("motor_id"), py::arg("mode"))
        .def("get_work_mode", &WHJDriver::getWorkMode,
             "Get work mode", py::arg("motor_id"))
        .def("set_target_current", &WHJDriver::setTargetCurrent,
             "Set target current", py::arg("motor_id"), py::arg("current_ma"))
        .def("set_target_speed", &WHJDriver::setTargetSpeed,
             "Set target speed", py::arg("motor_id"), py::arg("speed_rpm"))
        .def("set_target_position", &WHJDriver::setTargetPosition,
             "Set target position", py::arg("motor_id"), py::arg("position_deg"))
        
        // State query
        .def("get_joint_state", &WHJDriver::getJointState,
             "Get joint state", py::arg("motor_id"))
        .def("get_multiple_states", &WHJDriver::getMultipleStates,
             "Get multiple joint states", py::arg("motor_ids"))
        .def("get_motor_info", &WHJDriver::getMotorInfo,
             "Get motor info", py::arg("motor_id"))
        .def("get_error_code", &WHJDriver::getErrorCode,
             "Get error code", py::arg("motor_id"))
        .def("parse_errors", &WHJDriver::parseErrors,
             "Parse error code", py::arg("error_code"))
        
        // Configuration
        .def("save_to_flash", &WHJDriver::saveToFlash,
             "Save to flash", py::arg("motor_id"))
        .def("set_zero_position", &WHJDriver::setZeroPosition,
             "Set zero position", py::arg("motor_id"))
        .def("set_position_limits", &WHJDriver::setPositionLimits,
             "Set position limits", py::arg("motor_id"), py::arg("min_deg"), py::arg("max_deg"))
        .def("get_position_limits", &WHJDriver::getPositionLimits,
             "Get position limits", py::arg("motor_id"))
        .def("set_max_speed", &WHJDriver::setMaxSpeed,
             "Set max speed", py::arg("motor_id"), py::arg("max_rpm"))
        .def("set_position_pid", &WHJDriver::setPositionPID,
             "Set position PID", py::arg("motor_id"), py::arg("pid"))
        .def("set_speed_pid", &WHJDriver::setSpeedPID,
             "Set speed PID", py::arg("motor_id"), py::arg("pid"))
        .def("set_current_pid", &WHJDriver::setCurrentPID,
             "Set current PID", py::arg("motor_id"), py::arg("pid"))
        .def("get_pid_params", &WHJDriver::getPIDParams,
             "Get PID parameters", py::arg("motor_id"))
        
        // Utility
        .def("ping", &WHJDriver::ping,
             "Ping motor", py::arg("motor_id"), py::arg("timeout_ms") = 100)
        .def("scan", &WHJDriver::scan,
             "Scan for motors", py::arg("timeout_ms_per_motor") = 50)
        .def("get_config", &WHJDriver::getConfig, "Get configuration")
        .def("set_retry_count", &WHJDriver::setRetryCount,
             "Set retry count", py::arg("retries"))
        .def("get_last_error", &WHJDriver::getLastError, "Get last error");
    
    // =========================================================================
    // Constants
    // =========================================================================
    
    m.attr("MAX_MOTOR_ID") = Protocol::MAX_MOTOR_ID;
    m.attr("BROADCAST_ID") = Protocol::BROADCAST_ID;
}
