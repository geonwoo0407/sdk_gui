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

    assert(player.start("missing") == StartResult::MotionNotFound);
    assert(player.start("test_walk") == StartResult::Accepted);
    assert(player.start("test_walk") == StartResult::RejectedBusy);
    spinUntilDone(player);
    assert(player.status() == MotionStatus::Succeeded);
    assert(player.succeeded());
    assert(player.result() == MotionError::None);
    assert(hardware.commandCount() == 2);

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

    std::filesystem::remove(path);
    std::cout << "RobotMotionPlayer mock tests passed\n";
}
