#pragma once

#include "motion_hardware.hpp"

#include <string>
#include <utility>

namespace irc_step {

// ROS Motion Executor가 실제 모터 없이 성공/실패/cancel을 시험할 때 사용합니다.
class MockMotionHardware final : public IMotionHardware {
public:
    bool initialize() noexcept override {
        initialized_ = initialize_success_;
        if (!initialized_) last_error_ = "mock initialization failed";
        return initialized_;
    }

    [[nodiscard]] bool ready() const noexcept override {
        return initialized_ && ready_;
    }

    bool commandPosition(
        const JointAngles& goal_deg,
        const std::vector<int>& motor_ids,
        std::uint32_t duration_ms,
        std::uint32_t acceleration_ms) noexcept override {
        ++command_count_;
        last_duration_ms_ = duration_ms;
        last_acceleration_ms_ = acceleration_ms;
        if (!ready() || !command_success_) {
            last_error_ = "mock command failed";
            return false;
        }
        for (const int motor_id : motor_ids) {
            if (const auto goal = goal_deg.find(motor_id);
                goal != goal_deg.end()) {
                commanded_deg_[motor_id] = goal->second;
                if (auto_reach_goal_) present_deg_[motor_id] = goal->second;
            }
        }
        last_error_.clear();
        return true;
    }

    bool readPresentPositions(JointAngles& positions_deg) noexcept override {
        ++read_count_;
        if (!ready() || !read_success_) {
            last_error_ = "mock Present Position read failed";
            return false;
        }
        positions_deg = present_deg_;
        last_error_.clear();
        return true;
    }

    bool holdCurrentPosition(
        std::uint32_t stop_duration_ms) noexcept override {
        ++hold_count_;
        last_duration_ms_ = stop_duration_ms;
        if (!ready() || !hold_success_) {
            last_error_ = "mock hold failed";
            return false;
        }
        commanded_deg_ = present_deg_;
        last_error_.clear();
        return true;
    }

    bool setTorqueEnabled(bool enabled) noexcept override {
        torque_enabled_ = enabled;
        if (!enabled) initialized_ = false;
        return torque_success_;
    }

    [[nodiscard]] std::string_view lastError() const noexcept override {
        return last_error_;
    }

    void setInitializeSuccess(bool value) noexcept { initialize_success_ = value; }
    void setReady(bool value) noexcept { ready_ = value; }
    void setCommandSuccess(bool value) noexcept { command_success_ = value; }
    void setReadSuccess(bool value) noexcept { read_success_ = value; }
    void setHoldSuccess(bool value) noexcept { hold_success_ = value; }
    void setTorqueSuccess(bool value) noexcept { torque_success_ = value; }
    void setAutoReachGoal(bool value) noexcept { auto_reach_goal_ = value; }
    void setPresentPositions(JointAngles positions) {
        present_deg_ = std::move(positions);
    }

    [[nodiscard]] const JointAngles& commandedPositions() const noexcept {
        return commanded_deg_;
    }
    [[nodiscard]] int commandCount() const noexcept { return command_count_; }
    [[nodiscard]] int readCount() const noexcept { return read_count_; }
    [[nodiscard]] int holdCount() const noexcept { return hold_count_; }
    [[nodiscard]] std::uint32_t lastDurationMs() const noexcept {
        return last_duration_ms_;
    }
    [[nodiscard]] std::uint32_t lastAccelerationMs() const noexcept {
        return last_acceleration_ms_;
    }

private:
    JointAngles present_deg_;
    JointAngles commanded_deg_;
    std::string last_error_;
    bool initialize_success_{true};
    bool initialized_{false};
    bool ready_{true};
    bool command_success_{true};
    bool read_success_{true};
    bool hold_success_{true};
    bool torque_success_{true};
    bool torque_enabled_{false};
    bool auto_reach_goal_{true};
    int command_count_{0};
    int read_count_{0};
    int hold_count_{0};
    std::uint32_t last_duration_ms_{0};
    std::uint32_t last_acceleration_ms_{0};
};

}  // namespace irc_step
