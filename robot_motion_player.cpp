#include "robot_motion_player.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

namespace irc_step {
namespace {
constexpr std::string_view kLandingTag{"[착지]"};
constexpr double kLandingAccelerationRatio = 0.10;
constexpr std::uint32_t kLandingAccelerationMaxMs = 30;
constexpr std::uint32_t kCancelHoldDurationMs = 50;
constexpr auto kPositionCheckInterval = std::chrono::milliseconds(20);
}

RobotMotionPlayer::RobotMotionPlayer(
    const std::string& json_path, IMotionHardware& hardware)
    : library_(MotionLibrary::loadGuiJson(json_path)),
      hardware_(&hardware) {}

RobotMotionPlayer::~RobotMotionPlayer() { shutdown(); }

bool RobotMotionPlayer::initialize() noexcept {
    if (hardware_ == nullptr) {
        fail(MotionError::InternalError, "motion hardware is not configured");
        return false;
    }
    if (!hardware_->initialize()) {
        fail(MotionError::HardwareNotReady, std::string(hardware_->lastError()));
        return false;
    }
    initialized_ = true;
    status_ = MotionStatus::Idle;
    error_ = MotionError::None;
    last_error_.clear();
    return true;
}

bool RobotMotionPlayer::hardwareReady() const noexcept {
    return initialized_ && hardware_ != nullptr && hardware_->ready();
}

std::vector<std::string> RobotMotionPlayer::motionNames() const {
    return library_.names();
}

bool RobotMotionPlayer::contains(std::string_view name) const {
    return library_.contains(std::string(name));
}

MotionInfo RobotMotionPlayer::motionInfo(std::string_view name) const {
    const std::string owned_name(name);
    const auto& pattern = library_.motion(owned_name);
    const auto nominal_once_ms = static_cast<std::int64_t>(std::ceil(
        pattern.durationMs() / pattern.playbackSpeed()));
    return MotionInfo{
        owned_name,
        pattern.durationMs(),
        nominal_once_ms * pattern.repeatCount(),
        pattern.startPose(),
        pattern.endPose(),
        pattern.repeatable(),
        pattern.repeatCount(),
        pattern.playbackSpeed(),
    };
}

StartResult RobotMotionPlayer::start(std::string_view motion_name) noexcept {
    if (running()) return StartResult::RejectedBusy;
    if (!hardwareReady()) {
        fail(MotionError::HardwareNotReady, "motion hardware is not ready");
        return StartResult::HardwareNotReady;
    }
    const std::string name(motion_name);
    if (!library_.contains(name)) {
        error_ = MotionError::None;
        last_error_ = "motion not found: " + name;
        return StartResult::MotionNotFound;
    }
    try {
        pattern_ = &library_.motion(name);
        if (pattern_->frames().empty()) {
            last_error_ = "motion has no frames: " + name;
            return StartResult::InvalidMotion;
        }
        current_motion_ = name;
        next_frame_ = 0;
        repeat_ = 1;
        final_goal_deg_.clear();
        final_goal_ids_.clear();
        gated_goal_deg_.clear();
        gated_goal_ids_.clear();
        gated_frame_ = nullptr;
        frame_gate_wait_started_at_ = Clock::time_point{};
        frame_gate_last_check_at_ = Clock::time_point{};
        within_tolerance_ = false;
        started_at_ = Clock::now();
        status_ = MotionStatus::Running;
        error_ = MotionError::None;
        last_error_.clear();
        return StartResult::Accepted;
    } catch (const std::exception& error) {
        fail(MotionError::InternalError, error.what());
        return StartResult::InvalidMotion;
    } catch (...) {
        fail(MotionError::InternalError, "unknown motion start error");
        return StartResult::InvalidMotion;
    }
}

MotionStatus RobotMotionPlayer::update() noexcept {
    try {
        const auto now = Clock::now();
        if (status_ == MotionStatus::Running) return updateRunning(now);
        if (status_ == MotionStatus::Settling) return updateSettling(now);
        return status_;
    } catch (const std::exception& error) {
        fail(MotionError::InternalError, error.what());
    } catch (...) {
        fail(MotionError::InternalError, "unknown RobotMotionPlayer update error");
    }
    return status_;
}

MotionStatus RobotMotionPlayer::updateRunning(Clock::time_point now) {
    if (pattern_ == nullptr) {
        fail(MotionError::InternalError, "running motion has no pattern");
        return status_;
    }
    const double real_ms =
        std::chrono::duration<double, std::milli>(now - started_at_).count();
    const auto timeline_ms = static_cast<std::int64_t>(
        real_ms * pattern_->playbackSpeed());
    const auto& frames = pattern_->frames();

    if (!gateCurrentFrame(now, timeline_ms)) return status_;

    while (next_frame_ < frames.size()
           && frames[next_frame_].start_ms <= timeline_ms) {
        if (!sendFrame(frames[next_frame_], timeline_ms)) {
            fail(
                MotionError::FrameSendFailed,
                hardware_->lastError().empty()
                    ? "DYNAMIXEL frame transmission failed"
                    : std::string(hardware_->lastError()));
            return status_;
        }
        ++next_frame_;
    }

    if (timeline_ms < pattern_->durationMs()) return status_;
    if (repeat_ < pattern_->repeatCount()) {
        ++repeat_;
        next_frame_ = 0;
        started_at_ = now;
        return status_;
    }

    status_ = MotionStatus::Settling;
    settling_started_at_ = now;
    last_position_check_at_ = Clock::time_point{};
    within_tolerance_ = false;
    return updateSettling(now);
}

bool RobotMotionPlayer::gateCurrentFrame(
    Clock::time_point now, std::int64_t timeline_ms) {
    if (gated_frame_ == nullptr) return true;
    const auto frame_end_ms = gated_frame_->start_ms + gated_frame_->time_ms;
    if (timeline_ms < frame_end_ms) return true;

    started_at_ = now - std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::milli>(
            frame_end_ms / pattern_->playbackSpeed()));
    if (frame_gate_wait_started_at_ == Clock::time_point{})
        frame_gate_wait_started_at_ = now;
    if (frame_gate_last_check_at_ != Clock::time_point{}
        && now - frame_gate_last_check_at_ < kPositionCheckInterval) {
        return false;
    }
    frame_gate_last_check_at_ = now;

    JointAngles present_deg;
    if (!hardware_->readPresentPositions(present_deg)) {
        const auto wait_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - frame_gate_wait_started_at_).count();
        if (wait_ms <= pattern_->completion().settle_timeout_ms) return false;
        static_cast<void>(hardware_->holdCurrentPosition(kCancelHoldDurationMs));
        fail(
            MotionError::PresentPositionReadFailed,
            hardware_->lastError().empty()
                ? "failed to read frame Present Position"
                : std::string(hardware_->lastError()));
        return false;
    }

    bool reached = !gated_goal_ids_.empty();
    double worst_error = 0.0;
    int worst_id = -1;
    for (const int motor_id : gated_goal_ids_) {
        const auto goal = gated_goal_deg_.find(motor_id);
        const auto present = present_deg.find(motor_id);
        const double error = (
            goal == gated_goal_deg_.end() || present == present_deg.end())
            ? 1e9 : std::abs(goal->second - present->second);
        if (error > worst_error) {
            worst_error = error;
            worst_id = motor_id;
        }
        if (error > pattern_->completion().position_tolerance_deg)
            reached = false;
    }
    if (reached) {
        gated_frame_ = nullptr;
        gated_goal_deg_.clear();
        gated_goal_ids_.clear();
        frame_gate_wait_started_at_ = Clock::time_point{};
        frame_gate_last_check_at_ = Clock::time_point{};
        return true;
    }

    const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - frame_gate_wait_started_at_).count();
    if (wait_ms <= pattern_->completion().settle_timeout_ms) return false;
    static_cast<void>(hardware_->holdCurrentPosition(kCancelHoldDurationMs));
    fail(
        MotionError::PositionTimeout,
        "frame goal was not reached; motor "
            + std::to_string(worst_id) + " error="
            + std::to_string(worst_error) + "deg");
    return false;
}

MotionStatus RobotMotionPlayer::updateSettling(Clock::time_point now) {
    if (pattern_ == nullptr) {
        fail(MotionError::InternalError, "settling motion has no pattern");
        return status_;
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - settling_started_at_).count();
    if (elapsed_ms > pattern_->completion().settle_timeout_ms) {
        fail(
            MotionError::PositionTimeout,
            "final Present Position did not reach the goal before timeout");
        return status_;
    }
    if (last_position_check_at_ != Clock::time_point{}
        && now - last_position_check_at_ < kPositionCheckInterval) {
        return status_;
    }
    last_position_check_at_ = now;

    JointAngles present_deg;
    if (!hardware_->readPresentPositions(present_deg)) {
        fail(
            MotionError::PresentPositionReadFailed,
            hardware_->lastError().empty()
                ? "failed to read final Present Position"
                : std::string(hardware_->lastError()));
        return status_;
    }

    bool all_reached = !final_goal_ids_.empty();
    for (const int motor_id : final_goal_ids_) {
        const auto goal = final_goal_deg_.find(motor_id);
        const auto present = present_deg.find(motor_id);
        if (goal == final_goal_deg_.end() || present == present_deg.end()
            || std::abs(goal->second - present->second)
                > pattern_->completion().position_tolerance_deg) {
            all_reached = false;
            break;
        }
    }

    if (!all_reached) {
        within_tolerance_ = false;
        return status_;
    }
    if (!within_tolerance_) {
        within_tolerance_ = true;
        within_tolerance_since_ = now;
        return status_;
    }
    const auto stable_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - within_tolerance_since_).count();
    if (stable_ms < pattern_->completion().settle_duration_ms)
        return status_;

    status_ = MotionStatus::Succeeded;
    error_ = MotionError::None;
    last_error_.clear();
    std::cout << "[Info] Motion succeeded: " << current_motion_ << std::endl;
    return status_;
}

bool RobotMotionPlayer::running() const noexcept {
    return status_ == MotionStatus::Running
        || status_ == MotionStatus::Settling;
}

MotionStatus RobotMotionPlayer::status() const noexcept { return status_; }

bool RobotMotionPlayer::succeeded() const noexcept {
    return status_ == MotionStatus::Succeeded;
}

MotionError RobotMotionPlayer::result() const noexcept { return error_; }

std::string_view RobotMotionPlayer::lastError() const noexcept {
    return last_error_;
}

std::string_view RobotMotionPlayer::currentMotion() const noexcept {
    return current_motion_;
}

CancelResult RobotMotionPlayer::cancel() noexcept {
    if (!running()) return CancelResult::NotRunning;
    if (!hardwareReady()) {
        fail(MotionError::HardwareNotReady, "hardware is not ready for cancel");
        return CancelResult::HardwareNotReady;
    }
    if (!hardware_->holdCurrentPosition(kCancelHoldDurationMs)) {
        fail(
            MotionError::CancelFailed,
            hardware_->lastError().empty()
                ? "failed to hold current motor positions"
                : std::string(hardware_->lastError()));
        return CancelResult::HoldFailed;
    }
    status_ = MotionStatus::Cancelled;
    error_ = MotionError::None;
    last_error_.clear();
    pattern_ = nullptr;
    next_frame_ = 0;
    return CancelResult::Cancelled;
}

bool RobotMotionPlayer::emergencyStop() noexcept {
    if (hardware_ == nullptr) return false;
    const bool success = hardware_->setTorqueEnabled(false);
    initialized_ = false;
    status_ = success ? MotionStatus::Cancelled : MotionStatus::Failed;
    if (!success) {
        error_ = MotionError::CommunicationError;
        last_error_ = std::string(hardware_->lastError());
    }
    pattern_ = nullptr;
    next_frame_ = 0;
    return success;
}

void RobotMotionPlayer::stop() noexcept {
    if (running()) static_cast<void>(cancel());
}

bool RobotMotionPlayer::playBlocking(std::string_view motion_name) {
    if (start(motion_name) != StartResult::Accepted) return false;
    while (running()) {
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return succeeded();
}

void RobotMotionPlayer::setJointCorrection(
    int motor_id, double correction_deg) {
    if (motor_id < 0 || motor_id >= kMotionMotorCount)
        throw std::out_of_range("invalid correction motor ID");
    corrections_deg_[motor_id] = correction_deg;
}

void RobotMotionPlayer::setCorrections(const JointAngles& corrections_deg) {
    for (const auto& [motor_id, correction] : corrections_deg)
        setJointCorrection(motor_id, correction);
}

void RobotMotionPlayer::clearJointCorrections() noexcept {
    corrections_deg_.clear();
}

void RobotMotionPlayer::setFrameCorrection(
    const std::string& motion_name, const std::string& frame_name,
    int motor_id, double correction_deg) {
    if (motion_name.empty() || frame_name.empty())
        throw std::invalid_argument("motion and frame names are required");
    if (!library_.contains(motion_name))
        throw std::out_of_range("unknown correction motion: " + motion_name);
    if (motor_id < 0 || motor_id >= kMotionMotorCount)
        throw std::out_of_range("invalid frame correction motor ID");
    const auto& frames = library_.motion(motion_name).frames();
    const bool matching_target = std::any_of(
        frames.begin(), frames.end(),
        [&](const MotionFrame& frame) {
            return frame.name == frame_name && frame.angles.contains(motor_id);
        });
    if (!matching_target)
        throw std::out_of_range(
            "motion/frame does not command motor " + std::to_string(motor_id));
    frame_corrections_deg_[{motion_name, frame_name, motor_id}] = correction_deg;
}

void RobotMotionPlayer::clearFrameCorrection(
    const std::string& motion_name, const std::string& frame_name,
    int motor_id) noexcept {
    frame_corrections_deg_.erase({motion_name, frame_name, motor_id});
}

void RobotMotionPlayer::clearFrameCorrections() noexcept {
    frame_corrections_deg_.clear();
}

void RobotMotionPlayer::clearCorrections() noexcept {
    clearJointCorrections();
    clearFrameCorrections();
}

void RobotMotionPlayer::shutdown() noexcept {
    if (hardware_ != nullptr && initialized_) {
        if (running()) static_cast<void>(cancel());
        static_cast<void>(hardware_->setTorqueEnabled(false));
    }
    initialized_ = false;
    if (status_ == MotionStatus::Running
        || status_ == MotionStatus::Settling) {
        status_ = MotionStatus::Cancelled;
    }
}

void RobotMotionPlayer::fail(
    MotionError error, std::string message) noexcept {
    error_ = error;
    last_error_ = std::move(message);
    status_ = MotionStatus::Failed;
    pattern_ = nullptr;
    next_frame_ = 0;
}

JointAngles RobotMotionPlayer::correctedFrameAngles(
    const MotionFrame& frame) const {
    JointAngles corrected = frame.angles;
    for (auto& [motor_id, degree] : corrected) {
        if (const auto correction = corrections_deg_.find(motor_id);
            correction != corrections_deg_.end()) {
            degree += correction->second;
        }
        if (const auto correction = frame_corrections_deg_.find(
                {current_motion_, frame.name, motor_id});
            correction != frame_corrections_deg_.end()) {
            degree += correction->second;
        }
    }
    return corrected;
}

bool RobotMotionPlayer::sendFrame(
    const MotionFrame& frame, std::int64_t timeline_ms) {
    const auto corrected = correctedFrameAngles(frame);
    std::vector<int> target_ids;
    target_ids.reserve(corrected.size());
    for (const auto& [motor_id, degree] : corrected) {
        if (motor_id < 0 || motor_id >= kMotionMotorCount)
            throw std::runtime_error(
                "invalid motor ID in JSON: " + std::to_string(motor_id));
        target_ids.push_back(motor_id);
        final_goal_deg_[motor_id] = degree;
    }
    if (target_ids.empty()) return true;
    final_goal_ids_.clear();
    final_goal_ids_.reserve(final_goal_deg_.size());
    for (const auto& [motor_id, unused] : final_goal_deg_)
        final_goal_ids_.push_back(motor_id);

    const auto frame_end_ms = frame.start_ms + frame.time_ms;
    const auto remaining_timeline_ms =
        std::max<std::int64_t>(1, frame_end_ms - timeline_ms);
    const auto real_duration_ms = static_cast<std::uint32_t>(std::clamp(
        std::ceil(remaining_timeline_ms / pattern_->playbackSpeed()),
        1.0, static_cast<double>(kMaxTimeProfileMs)));
    const bool landing = frame.name.find(kLandingTag) != std::string::npos;
    const auto acceleration_ms = landing
        ? std::min<std::uint32_t>(
            static_cast<std::uint32_t>(
                real_duration_ms * kLandingAccelerationRatio),
            kLandingAccelerationMaxMs)
        : 0;
    if (!hardware_->commandPosition(
            corrected, target_ids, real_duration_ms, acceleration_ms)) {
        return false;
    }
    gated_frame_ = &frame;
    gated_goal_deg_ = corrected;
    gated_goal_ids_ = target_ids;
    frame_gate_wait_started_at_ = Clock::time_point{};
    frame_gate_last_check_at_ = Clock::time_point{};
    return true;
}

}  // namespace irc_step
