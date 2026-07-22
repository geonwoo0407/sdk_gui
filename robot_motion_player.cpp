#include "robot_motion_player.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>

namespace irc_step {
namespace {
constexpr std::string_view kLandingTag{"[착지]"};
constexpr double kLandingAccelerationRatio = 0.10;
constexpr uint32_t kLandingAccelerationMaxMs = 30;
}

RobotMotionPlayer::RobotMotionPlayer(const std::string& json_path)
    : library_(MotionLibrary::loadGuiJson(json_path)),
      controller_(&dxl_),
      desired_rad_(Eigen::VectorXd::Zero(NUMBER_OF_DYNAMIXELS)) {
    if (!dxl_.IsReady())
        throw std::runtime_error("DYNAMIXEL initialization failed");
    if (!controller_.SetTorqueEnabled(false))
        throw std::runtime_error("failed to disable torques for profile setup");
    if (!controller_.ConfigureTimeBasedProfile())
        throw std::runtime_error("failed to configure GUI-compatible time profiles");

    desired_rad_ = controller_.GetJointTheta();
    if (!controller_.SetTorqueEnabled(true))
        throw std::runtime_error("failed to enable all motor torques");
    torque_enabled_ = true;
}

RobotMotionPlayer::~RobotMotionPlayer() { shutdown(); }

std::vector<std::string> RobotMotionPlayer::motionNames() const {
    return library_.names();
}

bool RobotMotionPlayer::contains(const std::string& name) const {
    return library_.contains(name);
}

bool RobotMotionPlayer::start(const std::string& motion_name) {
    if (running_ || !library_.contains(motion_name)) return false;
    pattern_ = &library_.motion(motion_name);
    current_motion_ = motion_name;
    next_frame_ = 0;
    repeat_ = 1;
    started_at_ = Clock::now();
    running_ = true;
    return true;
}

void RobotMotionPlayer::update() {
    if (!running_ || pattern_ == nullptr) return;
    const auto now = Clock::now();
    const double real_ms =
        std::chrono::duration<double, std::milli>(now - started_at_).count();
    const auto timeline_ms = static_cast<std::int64_t>(
        real_ms * pattern_->playbackSpeed());
    const auto& frames = pattern_->frames();

    while (next_frame_ < frames.size()
           && frames[next_frame_].start_ms <= timeline_ms) {
        if (!sendFrame(frames[next_frame_], timeline_ms)) {
            running_ = false;
            throw std::runtime_error("DYNAMIXEL time-profile SyncWrite failed");
        }
        ++next_frame_;
    }

    if (timeline_ms < pattern_->durationMs()) return;
    if (repeat_ < pattern_->repeatCount()) {
        ++repeat_;
        next_frame_ = 0;
        started_at_ = now;
        return;
    }
    running_ = false;
    std::cout << "[Info] Motion finished: " << current_motion_ << std::endl;
}

bool RobotMotionPlayer::running() const noexcept { return running_; }

void RobotMotionPlayer::stop() noexcept {
    running_ = false;
    pattern_ = nullptr;
    next_frame_ = 0;
    current_motion_.clear();
}

bool RobotMotionPlayer::playBlocking(const std::string& motion_name) {
    if (!start(motion_name)) return false;
    while (running()) {
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

void RobotMotionPlayer::setJointCorrection(int motor_id, double correction_deg) {
    if (motor_id < 0 || motor_id >= NUMBER_OF_DYNAMIXELS)
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
    if (motor_id < 0 || motor_id >= NUMBER_OF_DYNAMIXELS)
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
    stop();
    if (torque_enabled_) {
        controller_.SetTorqueEnabled(false);
        torque_enabled_ = false;
    }
}

bool RobotMotionPlayer::sendFrame(const MotionFrame& frame,
                                  std::int64_t timeline_ms) {
    std::vector<int> target_ids;
    target_ids.reserve(frame.angles.size());
    for (const auto& [motor_id, degree] : frame.angles) {
        if (motor_id < 0 || motor_id >= NUMBER_OF_DYNAMIXELS)
            throw std::runtime_error(
                "invalid motor ID in JSON: " + std::to_string(motor_id));
        const auto correction = corrections_deg_.find(motor_id);
        const double joint_correction_deg = correction == corrections_deg_.end()
            ? 0.0 : correction->second;
        const auto frame_correction = frame_corrections_deg_.find(
            {current_motion_, frame.name, motor_id});
        const double frame_correction_deg =
            frame_correction == frame_corrections_deg_.end()
            ? 0.0 : frame_correction->second;
        desired_rad_[motor_id] = (
            degree + joint_correction_deg + frame_correction_deg) * DEG2RAD;
        if (joint_correction_deg != 0.0 || frame_correction_deg != 0.0) {
            std::cout << "[Correction] " << current_motion_ << '/' << frame.name
                      << " ID " << motor_id
                      << " base=" << degree
                      << " joint=" << joint_correction_deg
                      << " frame=" << frame_correction_deg
                      << " final="
                      << degree + joint_correction_deg + frame_correction_deg
                      << "deg" << std::endl;
        }
        target_ids.push_back(motor_id);
    }
    if (target_ids.empty()) return true;

    const auto frame_end_ms = frame.start_ms + frame.time_ms;
    const auto remaining_timeline_ms =
        std::max<std::int64_t>(1, frame_end_ms - timeline_ms);
    const auto real_duration_ms = static_cast<uint32_t>(std::clamp(
        std::ceil(remaining_timeline_ms / pattern_->playbackSpeed()),
        1.0, static_cast<double>(MAX_TIME_PROFILE_MS)));

    const bool landing = frame.name.find(kLandingTag) != std::string::npos;
    const uint32_t acceleration_ms = landing
        ? std::min<uint32_t>(
            static_cast<uint32_t>(real_duration_ms * kLandingAccelerationRatio),
            kLandingAccelerationMaxMs)
        : 0;

    std::cout << "[TimeProfile] " << frame.name
              << " arrival=" << real_duration_ms << "ms"
              << " accel=" << acceleration_ms << "ms"
              << " landing=" << (landing ? "yes" : "no")
              << " repeat=" << repeat_ << '/' << pattern_->repeatCount()
              << std::endl;
    return controller_.SetTimeBasedPosition(
        desired_rad_, target_ids, real_duration_ms, acceleration_ms);
}

}  // namespace irc_step
