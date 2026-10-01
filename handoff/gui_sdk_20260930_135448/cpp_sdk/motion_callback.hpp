#pragma once

#include "motion_pattern.hpp"

#include <chrono>
#include <functional>

namespace irc_step {

// main의 루프 또는 ROS callback에서 update()를 호출하는 비차단 실행기.
class MotionCallback {
public:
    using GoalWriter = std::function<bool(const JointAngles&)>;

    MotionCallback(const MotionPattern& pattern, GoalWriter writer,
                   std::int64_t transmit_period_ms = 30);
    void setInitialAngles(const JointAngles& angles);
    void start();
    void stop();
    void update();
    [[nodiscard]] bool running() const noexcept { return running_; }
    [[nodiscard]] const MotionTarget& lastTarget() const noexcept { return last_target_; }

private:
    using Clock = std::chrono::steady_clock;
    MotionPattern pattern_;
    GoalWriter writer_;
    std::int64_t period_ms_;
    Clock::time_point started_at_{};
    Clock::time_point last_sent_at_{};
    int repeat_{1};
    bool running_{false};
    MotionTarget last_target_;
};

}  // namespace irc_step
