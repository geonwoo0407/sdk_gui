#pragma once

#include "dynamixel_controller.hpp"
#include "motion_hardware.hpp"

#include <string>

namespace irc_step {

class DynamixelMotionHardware final : public IMotionHardware {
public:
    DynamixelMotionHardware();

    bool initialize() noexcept override;
    [[nodiscard]] bool ready() const noexcept override;
    bool commandPosition(
        const JointAngles& goal_deg,
        const std::vector<int>& motor_ids,
        std::uint32_t duration_ms,
        std::uint32_t acceleration_ms) noexcept override;
    bool readPresentPositions(JointAngles& positions_deg) noexcept override;
    bool holdCurrentPosition(
        std::uint32_t stop_duration_ms) noexcept override;
    bool setTorqueEnabled(bool enabled) noexcept override;
    [[nodiscard]] std::string_view lastError() const noexcept override;

private:
    void setError(std::string message) noexcept;

    Dxl dxl_;
    Dxl_Controller controller_;
    Eigen::VectorXd desired_rad_;
    std::string last_error_;
    bool initialized_{false};
};

}  // namespace irc_step
