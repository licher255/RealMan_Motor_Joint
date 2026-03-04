/**
 * @file types.cpp
 * @brief Type implementation helpers
 */

#include "realman_whj/core/types.hpp"
#include "realman_whj/core/command.hpp"

namespace realman_whj {

std::string MotorInfo::getModelName() const {
    switch (static_cast<JointModel>(model_type)) {
        case JointModel::J14:     return "J14 (Joint 10)";
        case JointModel::J17:     return "J17 (Joint 30)";
        case JointModel::J20:     return "J20 (Joint 60)";
        case JointModel::J25:     return "J25 (Joint 120)";
        case JointModel::GRIPPER: return "GRIPPER";
        case JointModel::J3:      return "J3 (Joint 03)";
        default:                  return "Unknown (" + std::to_string(model_type) + ")";
    }
}

std::string MotorInfo::getFirmwareString() const {
    uint8_t major = (firmware_version >> 8) & 0xFF;
    uint8_t minor = firmware_version & 0xFF;
    return std::to_string(major) + "." + std::to_string(minor);
}

const char* commResultToString(CommResult result) {
    switch (result) {
        case CommResult::SUCCESS:           return "Success";
        case CommResult::TIMEOUT:           return "Timeout";
        case CommResult::INVALID_RESPONSE:  return "Invalid response";
        case CommResult::CHECKSUM_ERROR:    return "Checksum error";
        case CommResult::BUS_ERROR:         return "Bus error";
        case CommResult::NOT_INITIALIZED:   return "Not initialized";
        case CommResult::INVALID_PARAMETER: return "Invalid parameter";
        case CommResult::WRITE_FAILED:      return "Write failed";
        case CommResult::READ_FAILED:       return "Read failed";
        case CommResult::BUSY:              return "Busy";
        case CommResult::UNKNOWN_ERROR:     return "Unknown error";
        default:                            return "Unknown";
    }
}

} // namespace realman_whj
