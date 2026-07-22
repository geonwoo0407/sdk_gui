#pragma once

#include "dynamixel_controller.hpp"
#include "motion_pattern.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace irc_step {

// 알고리즘이 사용할 로봇 모션 API입니다. 프레임 시작마다 GUI와 동일하게
// 목표각 + 도착시간(+ [착지] 가감속)을 전송하고, 궤적/PID는 모터가 수행합니다.
class RobotMotionPlayer {
public:
    explicit RobotMotionPlayer(const std::string& json_path);
    ~RobotMotionPlayer();

    RobotMotionPlayer(const RobotMotionPlayer&) = delete;
    RobotMotionPlayer& operator=(const RobotMotionPlayer&) = delete;

    [[nodiscard]] std::vector<std::string> motionNames() const;
    [[nodiscard]] bool contains(const std::string& name) const;

    // 비차단 API: start() 후 알고리즘 루프에서 update()를 계속 호출합니다.
    bool start(const std::string& motion_name);
    void update();
    [[nodiscard]] bool running() const noexcept;
    void stop() noexcept;

    // 터미널 시험 등 카메라 처리가 필요 없는 경우에만 사용합니다.
    bool playBlocking(const std::string& motion_name);

    // 향후 라인트레이싱의 작은 관절 보정용. 최종각 = JSON각 + correction.
    void setJointCorrection(int motor_id, double correction_deg);
    void setCorrections(const JointAngles& corrections_deg);
    void clearJointCorrections() noexcept;

    // 특정 시퀀스의 특정 이름 프레임에만 적용되는 보정입니다.
    void setFrameCorrection(const std::string& motion_name,
                            const std::string& frame_name,
                            int motor_id, double correction_deg);
    void clearFrameCorrection(const std::string& motion_name,
                              const std::string& frame_name,
                              int motor_id) noexcept;
    void clearFrameCorrections() noexcept;

    // 전체 관절 보정과 프레임 전용 보정을 모두 제거합니다.
    void clearCorrections() noexcept;

    // 모션 실행을 멈추고 전체 토크를 해제합니다.
    void shutdown() noexcept;

private:
    using Clock = std::chrono::steady_clock;
    bool sendFrame(const MotionFrame& frame, std::int64_t timeline_ms);

    MotionLibrary library_;
    Dxl dxl_;
    Dxl_Controller controller_;
    Eigen::VectorXd desired_rad_;
    JointAngles corrections_deg_;
    using FrameCorrectionKey = std::tuple<std::string, std::string, int>;
    std::map<FrameCorrectionKey, double> frame_corrections_deg_;
    const MotionPattern* pattern_{nullptr};
    std::size_t next_frame_{0};
    int repeat_{1};
    Clock::time_point started_at_{};
    std::string current_motion_;
    bool running_{false};
    bool torque_enabled_{false};
};

}  // namespace irc_step
