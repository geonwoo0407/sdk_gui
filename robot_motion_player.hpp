#pragma once

#include "motion_hardware.hpp"
#include "motion_pattern.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace irc_step {

enum class MotionStatus : std::uint8_t {
    Idle,
    Running,
    Settling,
    Succeeded,
    Cancelled,
    Failed,
};

enum class StartResult : std::uint8_t {
    Accepted,
    RejectedBusy,
    MotionNotFound,
    HardwareNotReady,
    InvalidMotion,
};

// 실행 중인 모션 다음에 한 개의 모션을 미리 예약할 때의 결과입니다.
// start()의 busy 검사를 우회하지 않고 SDK가 타임라인 경계에서 전환합니다.
enum class QueueResult : std::uint8_t {
    Queued,
    RejectedNotRunning,
    RejectedQueueFull,
    MotionNotFound,
    HardwareNotReady,
    IncompatibleTransition,
    InvalidMotion,
};

enum class CancelResult : std::uint8_t {
    Cancelled,
    NotRunning,
    HardwareNotReady,
    HoldFailed,
};

enum class MotionError : std::uint8_t {
    None,
    JsonError,
    HardwareNotReady,
    CommunicationError,
    FrameSendFailed,
    PresentPositionReadFailed,
    PositionTimeout,
    CancelFailed,
    InternalError,
};

struct MotionInfo {
    std::string name;
    std::int64_t timeline_duration_ms{0};
    std::int64_t expected_duration_ms{0};
    std::string start_pose;
    std::string end_pose;
    bool repeatable{true};
    int default_repeat_count{1};
    double default_playback_speed{1.0};
};

class RobotMotionPlayer {
public:
    // 실제 Dynamixel을 소유하는 실행용 생성자.
    explicit RobotMotionPlayer(const std::string& json_path);
    // ROS/mock 테스트가 가짜 하드웨어를 주입할 때 사용하는 생성자.
    RobotMotionPlayer(const std::string& json_path, IMotionHardware& hardware);
    ~RobotMotionPlayer();

    RobotMotionPlayer(const RobotMotionPlayer&) = delete;
    RobotMotionPlayer& operator=(const RobotMotionPlayer&) = delete;

    bool initialize() noexcept;
    [[nodiscard]] bool hardwareReady() const noexcept;

    [[nodiscard]] std::vector<std::string> motionNames() const;
    [[nodiscard]] bool contains(std::string_view name) const;
    [[nodiscard]] MotionInfo motionInfo(std::string_view name) const;

    StartResult start(std::string_view motion_name) noexcept;
    QueueResult queueNext(std::string_view motion_name) noexcept;
    [[nodiscard]] bool hasQueuedMotion() const noexcept;
    [[nodiscard]] std::string_view queuedMotion() const noexcept;
    void clearQueuedMotion() noexcept;
    // 무정착 경계 전환이 발생할 때마다 증가합니다. Executor는 이 값의
    // 변화를 보고 이전 모션 명령을 완료 처리할 수 있습니다.
    [[nodiscard]] std::uint64_t completionSequence() const noexcept;
    [[nodiscard]] std::string_view lastCompletedMotion() const noexcept;
    MotionStatus update() noexcept;
    [[nodiscard]] bool running() const noexcept;
    [[nodiscard]] MotionStatus status() const noexcept;
    [[nodiscard]] bool succeeded() const noexcept;
    [[nodiscard]] MotionError result() const noexcept;
    [[nodiscard]] std::string_view lastError() const noexcept;
    [[nodiscard]] std::string_view currentMotion() const noexcept;

    // Present Position을 읽고 같은 위치를 Goal로 보내 토크를 유지한 채 정지.
    CancelResult cancel() noexcept;
    // 낙상 위험이 있으므로 비상시에만 전체 토크를 해제.
    bool emergencyStop() noexcept;
    // 이전 코드 호환용. 실제 안전정지는 cancel()을 사용합니다.
    void stop() noexcept;

    bool playBlocking(std::string_view motion_name);

    void setJointCorrection(int motor_id, double correction_deg);
    void setCorrections(const JointAngles& corrections_deg);
    void clearJointCorrections() noexcept;
    void setFrameCorrection(const std::string& motion_name,
                            const std::string& frame_name,
                            int motor_id, double correction_deg);
    void clearFrameCorrection(const std::string& motion_name,
                              const std::string& frame_name,
                              int motor_id) noexcept;
    void clearFrameCorrections() noexcept;
    void clearCorrections() noexcept;

    void shutdown() noexcept;

private:
    using Clock = std::chrono::steady_clock;
    using FrameCorrectionKey = std::tuple<std::string, std::string, int>;

    bool sendFrame(const MotionFrame& frame, std::int64_t timeline_ms);
    MotionStatus updateRunning(Clock::time_point now);
    MotionStatus updateSettling(Clock::time_point now);
    bool canSeamlesslyTransition(const MotionPattern& current,
                                 std::string_view current_name,
                                 const MotionPattern& next,
                                 std::string_view next_name) const noexcept;
    bool activateQueuedMotion(Clock::time_point now) noexcept;
    void resetMotionRuntime(const MotionPattern& pattern,
                            std::string name,
                            Clock::time_point now) noexcept;
    bool gateCurrentFrame(Clock::time_point now, std::int64_t timeline_ms);
    void fail(MotionError error, std::string message) noexcept;
    JointAngles correctedFrameAngles(const MotionFrame& frame) const;

    MotionLibrary library_;
    std::unique_ptr<IMotionHardware> owned_hardware_;
    IMotionHardware* hardware_{nullptr};
    JointAngles corrections_deg_;
    std::map<FrameCorrectionKey, double> frame_corrections_deg_;
    JointAngles final_goal_deg_;
    std::vector<int> final_goal_ids_;
    JointAngles gated_goal_deg_;
    std::vector<int> gated_goal_ids_;
    const MotionFrame* gated_frame_{nullptr};
    const MotionPattern* pattern_{nullptr};
    std::size_t next_frame_{0};
    int repeat_{1};
    Clock::time_point started_at_{};
    Clock::time_point settling_started_at_{};
    Clock::time_point within_tolerance_since_{};
    Clock::time_point last_position_check_at_{};
    Clock::time_point frame_gate_wait_started_at_{};
    Clock::time_point frame_gate_last_check_at_{};
    std::string current_motion_;
    std::string queued_motion_;
    std::string last_completed_motion_;
    const MotionPattern* queued_pattern_{nullptr};
    std::uint64_t completion_sequence_{0};
    std::string last_error_;
    MotionStatus status_{MotionStatus::Idle};
    MotionError error_{MotionError::None};
    bool initialized_{false};
    bool within_tolerance_{false};
};

}  // namespace irc_step
