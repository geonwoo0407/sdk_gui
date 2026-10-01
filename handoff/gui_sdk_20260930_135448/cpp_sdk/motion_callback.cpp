#include "motion_callback.hpp"

#include <stdexcept>
#include <utility>

namespace irc_step {

MotionCallback::MotionCallback(const MotionPattern& pattern, GoalWriter writer,
                               std::int64_t transmit_period_ms)
    : pattern_(pattern), writer_(std::move(writer)), period_ms_(transmit_period_ms) {
    if (!writer_) throw std::invalid_argument("GoalWriter is required");
    if (period_ms_ <= 0) throw std::invalid_argument("transmit period must be positive");
}

void MotionCallback::setInitialAngles(const JointAngles& angles) {
    if (running_) throw std::logic_error("set initial angles before start");
    pattern_.setInitialAngles(angles);
}

void MotionCallback::start() {
    repeat_ = 1;
    running_ = true;
    started_at_ = Clock::now();
    last_sent_at_ = started_at_ - std::chrono::milliseconds(period_ms_);
}

void MotionCallback::stop() { running_ = false; }

void MotionCallback::update() {
    if (!running_) return;
    const auto now = Clock::now();
    if (now - last_sent_at_ < std::chrono::milliseconds(period_ms_)) return;
    last_sent_at_ = now;
    const auto real_ms = std::chrono::duration<double, std::milli>(now - started_at_).count();
    const auto motion_ms = static_cast<std::int64_t>(real_ms * pattern_.playbackSpeed());
    last_target_ = pattern_.sample(motion_ms);
    if (!last_target_.angles.empty() && !writer_(last_target_.angles)) {
        running_ = false;
        throw std::runtime_error("DYNAMIXEL SyncWrite failed");
    }
    if (!last_target_.finished) return;
    if (repeat_ < pattern_.repeatCount()) {
        pattern_.setInitialAngles(last_target_.angles);
        ++repeat_;
        started_at_ = now;
    } else {
        running_ = false;
    }
}

}  // namespace irc_step
