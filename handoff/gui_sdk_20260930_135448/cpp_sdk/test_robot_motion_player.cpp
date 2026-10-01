#include "mock_motion_hardware.hpp"
#include "robot_motion_player.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

using namespace irc_step;

namespace {
std::filesystem::path writeTestLibrary() {
    const auto path =
        std::filesystem::temp_directory_path() / "robot_player_test.json";
    std::ofstream(path) << R"({
      "version": 1,
      "motions": [
        {
          "name": "test_walk",
          "repeatable": true,
          "start_pose": "ready",
          "end_pose": "ready",
          "max_seq_ms": 50,
          "repeat_count": 1,
          "playback_speed": 1.0,
          "completion": {
            "position_tolerance_deg": 1.0,
            "settle_duration_ms": 0,
            "settle_timeout_ms": 80
          },
          "frames": [
            {"name":"ready","start_ms":0,"time_ms":10,
             "angles":{"1":10.0},"torques":{"1":true}},
            {"name":"ready","start_ms":10,"time_ms":10,
             "angles":{"1":20.0},"torques":{"1":true}}
          ]
        }
      ]
    })";
    return path;
}

void spinUntilDone(RobotMotionPlayer& player) {
    const auto timeout =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (player.running() && std::chrono::steady_clock::now() < timeout) {
        player.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
}

int main() {
    const auto path = writeTestLibrary();

    MockMotionHardware hardware;
    RobotMotionPlayer player(path.string(), hardware);
    assert(!player.hardwareReady());
    assert(player.initialize());
    assert(player.hardwareReady());
    assert(player.contains("test_walk"));
    const auto info = player.motionInfo("test_walk");
    assert(info.timeline_duration_ms == 20);
    assert(info.expected_duration_ms == 20);
    assert(info.start_pose == "ready");
    assert(info.end_pose == "ready");
    assert(info.repeatable);

    assert(player.queueNext("test_walk") == QueueResult::RejectedNotRunning);

    assert(player.start("missing") == StartResult::MotionNotFound);
    assert(player.start("test_walk") == StartResult::Accepted);
    assert(player.start("test_walk") == StartResult::RejectedBusy);
    spinUntilDone(player);
    assert(player.status() == MotionStatus::Succeeded);
    assert(player.succeeded());
    assert(player.result() == MotionError::None);
    assert(hardware.commandCount() == 2);
    assert(player.completionSequence() == 1);
    assert(player.lastCompletedMotion() == "test_walk");

    // 같은 repeatable 보행을 미리 예약하면 첫 모션의 Settling 없이
    // 타임라인 경계에서 두 번째 보행으로 전환합니다.
    assert(player.start("test_walk") == StartResult::Accepted);
    assert(player.queueNext("missing") == QueueResult::MotionNotFound);
    assert(player.queueNext("test_walk") == QueueResult::Queued);
    assert(player.hasQueuedMotion());
    assert(player.queuedMotion() == "test_walk");
    assert(player.queueNext("test_walk") == QueueResult::RejectedQueueFull);
    spinUntilDone(player);
    assert(player.status() == MotionStatus::Succeeded);
    assert(!player.hasQueuedMotion());
    assert(hardware.commandCount() == 6);
    // 첫 경계 전환과 마지막 정상 완료가 각각 완료 이벤트로 기록됩니다.
    assert(player.completionSequence() == 3);
    assert(player.lastCompletedMotion() == "test_walk");

    assert(player.start("test_walk") == StartResult::Accepted);
    assert(player.cancel() == CancelResult::Cancelled);
    assert(player.status() == MotionStatus::Cancelled);
    assert(hardware.holdCount() == 1);

    hardware.setCommandSuccess(false);
    assert(player.start("test_walk") == StartResult::Accepted);
    player.update();
    assert(player.status() == MotionStatus::Failed);
    assert(player.result() == MotionError::FrameSendFailed);
    assert(!player.lastError().empty());
    hardware.setCommandSuccess(true);

    hardware.setAutoReachGoal(false);
    hardware.setPresentPositions({{1, -30.0}});
    const int commands_before_timeout_test = hardware.commandCount();
    assert(player.start("test_walk") == StartResult::Accepted);
    spinUntilDone(player);
    assert(player.status() == MotionStatus::Failed);
    assert(player.result() == MotionError::PositionTimeout);
    // 첫 프레임에 도달하지 않았으므로 두 번째 프레임은 절대 전송하지 않는다.
    assert(hardware.commandCount() == commands_before_timeout_test + 1);

    // GUI에서 손으로 읽은 소수 각도가 JSON 로드와 실제 frame 명령까지 보존된다.
    const auto fractional_path =
        std::filesystem::temp_directory_path() / "robot_player_fractional_test.json";
    std::ofstream(fractional_path) << R"({
      "motions": [{
        "name": "taught_pose", "max_seq_ms": 10,
        "completion": {"settle_duration_ms": 0},
        "frames": [{"name": "pose", "start_ms": 0, "time_ms": 10,
                    "angles": {"17": -12.392578125, "18": 0.087890625}}]
      }]
    })";
    MockMotionHardware fractional_hardware;
    RobotMotionPlayer fractional_player(fractional_path.string(), fractional_hardware);
    assert(fractional_player.initialize());
    assert(fractional_player.start("taught_pose") == StartResult::Accepted);
    spinUntilDone(fractional_player);
    assert(fractional_player.succeeded());
    assert(fractional_hardware.commandedPositions().at(17) == -12.392578125);
    assert(fractional_hardware.commandedPositions().at(18) == 0.087890625);
    std::filesystem::remove(fractional_path);

    std::filesystem::remove(path);
    std::cout << "RobotMotionPlayer mock tests passed\n";
}
