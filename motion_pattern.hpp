#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace irc_step {

using JointAngles = std::unordered_map<int, double>;
using TorqueStates = std::unordered_map<int, bool>;

// Python GUI의 motion_sequence 프레임과 1:1 대응한다.
struct MotionFrame {
    std::string name;
    std::int64_t start_ms{0};
    std::int64_t time_ms{1};
    JointAngles angles;
    TorqueStates torques;
};

// callback이 SDK로 전송할 현재 시각의 목표값.
struct MotionTarget {
    std::int64_t time_ms{0};
    JointAngles angles;
    TorqueStates torques;
    std::string active_frame;
    bool finished{false};
};

struct MotionCompletion {
    double position_tolerance_deg{2.0};
    std::int64_t settle_duration_ms{80};
    std::int64_t settle_timeout_ms{3000};
};

// 새 모션을 만드는 클래스가 아니다.
// GUI가 저장한 모션을 읽고, GUI와 같은 방식으로 현재 목표각을 꺼내는 클래스다.
class MotionPattern {
public:
    static MotionPattern loadGuiJson(const std::filesystem::path& path);

    MotionPattern(std::vector<MotionFrame> frames,
                  std::int64_t max_seq_ms,
                  int repeat_count = 1,
                  double playback_speed = 1.0,
                  bool repeatable = true,
                  std::string start_pose = {},
                  std::string end_pose = {},
                  MotionCompletion completion = {});

    // 재생 시작 직전 실제 로봇의 현재각을 넣는다.
    void setInitialAngles(JointAngles angles);

    // 현재 시각의 선형/최단각 보간 결과를 반환한다.
    // 프레임 사이에 공백이 있으면 이전 프레임 종료 즉시 이동을 시작해
    // 다음 프레임 시작 시각에 목표 자세에 도달한다.
    [[nodiscard]] MotionTarget sample(std::int64_t motion_time_ms) const;

    [[nodiscard]] const std::vector<MotionFrame>& frames() const noexcept { return frames_; }
    [[nodiscard]] std::int64_t durationMs() const noexcept { return duration_ms_; }
    [[nodiscard]] std::int64_t maxSequenceMs() const noexcept { return max_seq_ms_; }
    [[nodiscard]] int repeatCount() const noexcept { return repeat_count_; }
    [[nodiscard]] double playbackSpeed() const noexcept { return playback_speed_; }
    [[nodiscard]] bool repeatable() const noexcept { return repeatable_; }
    [[nodiscard]] const std::string& startPose() const noexcept { return start_pose_; }
    [[nodiscard]] const std::string& endPose() const noexcept { return end_pose_; }
    [[nodiscard]] const MotionCompletion& completion() const noexcept {
        return completion_;
    }

private:
    struct QuinticSegment {
        std::int64_t start_ms{0};
        std::int64_t end_ms{0};
        double c0{0.0};
        double c1{0.0};
        double c2{0.0};
        double c3{0.0};
        double c4{0.0};
        double c5{0.0};
    };

    static double shortestDelta(double from_deg, double to_deg);
    static double evaluateSegment(const QuinticSegment& segment,
                                  std::int64_t motion_time_ms);
    void validate();
    void buildTrajectories();

    std::vector<MotionFrame> frames_;
    JointAngles initial_angles_;
    std::unordered_map<int, std::vector<QuinticSegment>> trajectories_;
    std::int64_t max_seq_ms_{0};
    std::int64_t duration_ms_{0};
    int repeat_count_{1};
    double playback_speed_{1.0};
    bool repeatable_{true};
    std::string start_pose_;
    std::string end_pose_;
    MotionCompletion completion_;
};

// robot_motions.json 하나에 들어 있는 여러 시퀀스를 이름으로 관리한다.
class MotionLibrary {
public:
    static MotionLibrary loadGuiJson(const std::filesystem::path& path);
    [[nodiscard]] const MotionPattern& motion(const std::string& name) const;
    [[nodiscard]] bool contains(const std::string& name) const;
    [[nodiscard]] std::vector<std::string> names() const;

private:
    std::unordered_map<std::string, MotionPattern> motions_;
};

}  // namespace irc_step
