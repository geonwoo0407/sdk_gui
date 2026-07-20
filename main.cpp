#include "dynamixel_controller.hpp"
#include "motion_pattern.hpp"

#include <chrono>
#include <atomic>
#include <csignal>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace irc_step {

std::atomic_bool stop_requested{false};
void requestStop(int) { stop_requested.store(true); }

// GUI 실기 재생과 같은 실행기:
// 프레임 시작마다 최종 Goal Position과 도착시간을 한 번 전송하고,
// 중간 궤적과 위치 PID는 DYNAMIXEL의 Time-based Profile이 담당한다.
class RobotMotionPlayer {
public:
    explicit RobotMotionPlayer(const std::string& json_path)
        : library_(MotionLibrary::loadGuiJson(json_path)),
          controller_(&dxl_),
          desired_rad_(Eigen::VectorXd::Zero(NUMBER_OF_DYNAMIXELS)) {
        if (!dxl_.IsReady())
            throw std::runtime_error("DYNAMIXEL initialization failed");

        // Drive Mode는 EEPROM이라 반드시 토크 OFF 상태에서 설정한다.
        if (!controller_.SetTorqueEnabled(false))
            throw std::runtime_error("failed to disable torques for profile setup");
        if (!controller_.ConfigureTimeBasedProfile())
            throw std::runtime_error("failed to configure GUI-compatible time profiles");

        desired_rad_ = controller_.GetJointTheta();
        if (!controller_.SetTorqueEnabled(true))
            throw std::runtime_error("failed to enable all motor torques");
        torque_enabled_ = true;
    }

    ~RobotMotionPlayer() { shutdown(); }
    RobotMotionPlayer(const RobotMotionPlayer&) = delete;
    RobotMotionPlayer& operator=(const RobotMotionPlayer&) = delete;

    [[nodiscard]] std::vector<std::string> motionNames() const {
        return library_.names();
    }
    [[nodiscard]] bool contains(const std::string& name) const {
        return library_.contains(name);
    }

    // 비차단 시작. 실행 중인 모션을 덮어쓰지 않고 완료 후 다음 명령을 받는다.
    bool start(const std::string& motion_name) {
        if (running_ || !library_.contains(motion_name)) return false;
        pattern_ = &library_.motion(motion_name);
        current_motion_ = motion_name;
        next_frame_ = 0;
        repeat_ = 1;
        started_at_ = Clock::now();
        running_ = true;
        return true;
    }

    // RealSense/알고리즘 루프에서 반복 호출한다. 블로킹하지 않는다.
    void update() {
        if (!running_ || pattern_ == nullptr) return;
        const auto now = Clock::now();
        const double real_ms =
            std::chrono::duration<double, std::milli>(now - started_at_).count();
        const auto timeline_ms = static_cast<std::int64_t>(
            real_ms * pattern_->playbackSpeed());
        const auto& frames = pattern_->frames();

        // 정상 루프에서는 한 프레임만 해당한다. 스케줄 지연으로 경계를
        // 건너뛰어도 최신 프레임까지 따라잡아 누락된 Goal을 남기지 않는다.
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

    [[nodiscard]] bool running() const noexcept { return running_; }

    void stop() noexcept {
        running_ = false;
        pattern_ = nullptr;
        next_frame_ = 0;
        current_motion_.clear();
    }

    bool playBlocking(const std::string& motion_name) {
        if (!start(motion_name)) return false;
        while (running() && !stop_requested.load()) {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (stop_requested.load()) stop();
        return true;
    }

    void shutdown() noexcept {
        stop();
        if (torque_enabled_) {
            controller_.SetTorqueEnabled(false);
            torque_enabled_ = false;
        }
    }

private:
    using Clock = std::chrono::steady_clock;

    bool sendFrame(const MotionFrame& frame, std::int64_t timeline_ms) {
        std::vector<int> target_ids;
        target_ids.reserve(frame.angles.size());
        for (const auto& [motor_id, degree] : frame.angles) {
            if (motor_id < 0 || motor_id >= NUMBER_OF_DYNAMIXELS)
                throw std::runtime_error(
                    "invalid motor ID in JSON: " + std::to_string(motor_id));
            // 향후 고정/알고리즘 보정각은 이 지점에서 degree에 더한다.
            desired_rad_[motor_id] = degree * DEG2RAD;
            target_ids.push_back(motor_id);
        }
        if (target_ids.empty()) return true;

        const auto frame_end_ms = frame.start_ms + frame.time_ms;
        const auto remaining_timeline_ms =
            std::max<std::int64_t>(1, frame_end_ms - timeline_ms);
        const auto real_duration_ms = static_cast<uint32_t>(std::clamp(
            std::ceil(remaining_timeline_ms / pattern_->playbackSpeed()),
            1.0, static_cast<double>(MAX_TIME_PROFILE_MS)));

        std::cout << "[TimeProfile] " << frame.name
                  << " arrival=" << real_duration_ms << "ms"
                  << " repeat=" << repeat_ << '/' << pattern_->repeatCount()
                  << std::endl;
        return controller_.SetTimeBasedPosition(
            desired_rad_, target_ids, real_duration_ms);
    }

    MotionLibrary library_;
    Dxl dxl_;
    Dxl_Controller controller_;
    Eigen::VectorXd desired_rad_;
    const MotionPattern* pattern_{nullptr};
    std::size_t next_frame_{0};
    int repeat_{1};
    Clock::time_point started_at_{};
    std::string current_motion_;
    bool running_{false};
    bool torque_enabled_{false};
};

}  // namespace irc_step

int main(int argc, char** argv) {
    using namespace irc_step;
    if (argc < 2) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " robot_motions.json\n"
                  << "  " << argv[0] << " robot_motions.json <motion_name>\n";
        return 1;
    }

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    try {
        RobotMotionPlayer player(argv[1]);
        if (argc > 2) {
            if (!player.playBlocking(argv[2])) {
                std::cerr << "[Error] Motion not found or player busy: " << argv[2]
                          << "\nAvailable:";
                for (const auto& name : player.motionNames()) std::cerr << ' ' << name;
                std::cerr << std::endl;
                return 1;
            }
            return 0;
        }

        std::cout << "[Ready] Enter a motion name, 'list', or 'quit'.\n";
        std::string command;
        while (std::cout << "> " && std::getline(std::cin, command)) {
            if (stop_requested.load()) break;
            if (command == "quit" || command == "exit") break;
            if (command == "list") {
                std::cout << "Available:";
                for (const auto& name : player.motionNames()) std::cout << ' ' << name;
                std::cout << std::endl;
            } else if (!command.empty() && !player.playBlocking(command)) {
                std::cerr << "[Error] Motion not found: " << command << std::endl;
            }
        }
        player.shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[Error] " << error.what() << std::endl;
        return 1;
    }
}
