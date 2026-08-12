#include "dynamixel_motion_hardware.hpp"

#include <cmath>
#include <exception>
#include <utility>

namespace irc_step {

DynamixelMotionHardware::DynamixelMotionHardware()
    : controller_(&dxl_),
      desired_rad_(Eigen::VectorXd::Zero(NUMBER_OF_DYNAMIXELS)) {}

void DynamixelMotionHardware::setError(std::string message) noexcept {
    last_error_ = std::move(message);
}

bool DynamixelMotionHardware::initialize() noexcept {
    try {
        if (!dxl_.IsReady()) {
            setError("DYNAMIXEL port is not ready");
            return false;
        }
        if (!controller_.SetTorqueEnabled(false)) {
            setError("failed to disable torque for time-profile setup");
            return false;
        }
        if (!controller_.ConfigureTimeBasedProfile()) {
            setError("failed to configure time-based profiles");
            return false;
        }
        desired_rad_ = controller_.GetJointTheta();
        if (!controller_.SetTorqueEnabled(true)) {
            setError("failed to enable motor torque");
            return false;
        }
        initialized_ = true;
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        setError(error.what());
        initialized_ = false;
        return false;
    } catch (...) {
        setError("unknown DYNAMIXEL initialization error");
        initialized_ = false;
        return false;
    }
}

bool DynamixelMotionHardware::ready() const noexcept {
    return initialized_ && dxl_.IsReady();
}

bool DynamixelMotionHardware::commandPosition(
    const JointAngles& goal_deg, const std::vector<int>& motor_ids,
    std::uint32_t duration_ms, std::uint32_t acceleration_ms) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    for (const int motor_id : motor_ids) {
        const auto goal = goal_deg.find(motor_id);
        if (motor_id < 0 || motor_id >= NUMBER_OF_DYNAMIXELS
            || goal == goal_deg.end()) {
            setError("invalid or missing motor goal");
            return false;
        }
        desired_rad_[motor_id] = goal->second * DEG2RAD;
    }
    if (!controller_.SetTimeBasedPosition(
            desired_rad_, motor_ids, duration_ms, acceleration_ms)) {
        setError("DYNAMIXEL time-profile SyncWrite failed");
        return false;
    }
    last_error_.clear();
    return true;
}

bool DynamixelMotionHardware::readPresentPositions(
    JointAngles& positions_deg) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    try {
        const auto positions_rad = controller_.GetJointTheta();
        positions_deg.clear();
        for (int motor_id = 0; motor_id < NUMBER_OF_DYNAMIXELS; ++motor_id)
            positions_deg[motor_id] = positions_rad[motor_id] / DEG2RAD;
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        setError(error.what());
        return false;
    } catch (...) {
        setError("unknown Present Position read error");
        return false;
    }
}

bool DynamixelMotionHardware::holdCurrentPosition(
    std::uint32_t stop_duration_ms) noexcept {
    JointAngles current_deg;
    if (!readPresentPositions(current_deg)) return false;
    std::vector<int> motor_ids;
    motor_ids.reserve(current_deg.size());
    for (const auto& [motor_id, unused] : current_deg)
        motor_ids.push_back(motor_id);
    return commandPosition(current_deg, motor_ids, stop_duration_ms, 0);
}

bool DynamixelMotionHardware::setTorqueEnabled(bool enabled) noexcept {
    if (!dxl_.IsReady()) {
        setError("DYNAMIXEL port is not ready");
        return false;
    }
    if (!controller_.SetTorqueEnabled(enabled)) {
        setError(enabled ? "failed to enable torque"
                         : "failed to disable torque");
        return false;
    }
    if (!enabled) initialized_ = false;
    last_error_.clear();
    return true;
}

std::string_view DynamixelMotionHardware::lastError() const noexcept {
    return last_error_;
}

}  // namespace irc_step
