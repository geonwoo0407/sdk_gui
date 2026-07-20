#include "motion_pattern.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace irc_step;

bool close(double a, double b) { return std::abs(a - b) < 1e-9; }

double jointAngle(const MotionPattern& pattern, std::int64_t time_ms, int joint_id = 1) {
    return pattern.sample(time_ms).angles.at(joint_id);
}

double numericalVelocity(const MotionPattern& pattern, std::int64_t time_ms,
                         int joint_id = 1, std::int64_t step_ms = 1) {
    const double step_sec = step_ms / 1000.0;
    return (jointAngle(pattern, time_ms + step_ms, joint_id)
            - jointAngle(pattern, time_ms - step_ms, joint_id))
        / (2.0 * step_sec);
}

double numericalAcceleration(const MotionPattern& pattern, std::int64_t time_ms,
                             int joint_id = 1, std::int64_t step_ms = 1) {
    const double step_sec = step_ms / 1000.0;
    return (jointAngle(pattern, time_ms + step_ms, joint_id)
            - 2.0 * jointAngle(pattern, time_ms, joint_id)
            + jointAngle(pattern, time_ms - step_ms, joint_id))
        / (step_sec * step_sec);
}

int main() {
    // Python GUI export_motion_json()이 만드는 형식 그대로 테스트한다.
    const auto path = std::filesystem::temp_directory_path() / "jetson_motion_data.json";
    std::ofstream(path) << R"({
      "max_seq_ms": 3000,
      "repeat_count": 2,
      "playback_speed": 1.0,
      "frames": [
        {"name":"A","start_ms":0,"time_ms":1000,
         "angles":{"1":90.0,"2":5.0},"torques":{"1":true}},
        {"name":"B","start_ms":1500,"time_ms":500,
         "angles":{"1":170.0},"torques":{}},
        {"name":"C","start_ms":2000,"time_ms":500,
         "angles":{"1":160.0},"torques":{}}
      ]
    })";

    auto pattern = MotionPattern::loadGuiJson(path);
    pattern.setInitialAngles({{1, 0.0}, {2, 0.0}});
    assert(pattern.repeatCount() == 2);
    assert(close(jointAngle(pattern, 0), 0.0));
    assert(close(jointAngle(pattern, 1000), 90.0));
    assert(jointAngle(pattern, 1200) > 90.0);
    assert(jointAngle(pattern, 1200) < 170.0);
    assert(close(jointAngle(pattern, 2000), 170.0));
    assert(close(jointAngle(pattern, 2500), 160.0));
    assert(pattern.sample(2500).finished);
    std::filesystem::remove(path);

    // 같은 방향의 중간 프레임에서는 멈추지 않고 속도와 가속도가 연속이다.
    MotionPattern smooth_pattern({
        MotionFrame{"20deg", 0, 1000, {{1, 20.0}}, {}},
        MotionFrame{"40deg", 1000, 1000, {{1, 40.0}}, {}},
    }, 2000);
    smooth_pattern.setInitialAngles({{1, 0.0}});
    assert(close(jointAngle(smooth_pattern, 0), 0.0));
    assert(close(jointAngle(smooth_pattern, 1000), 20.0));
    assert(close(jointAngle(smooth_pattern, 2000), 40.0));
    assert(std::abs(numericalVelocity(smooth_pattern, 1000) - 20.0) < 0.01);
    assert(std::abs(numericalAcceleration(smooth_pattern, 1000)) < 0.01);
    assert(std::abs(jointAngle(smooth_pattern, 1) - jointAngle(smooth_pattern, 0)) < 0.001);
    assert(std::abs(jointAngle(smooth_pattern, 2000) - jointAngle(smooth_pattern, 1999)) < 0.001);
    for (std::int64_t time_ms = 0; time_ms <= 2000; time_ms += 10) {
        const double angle = jointAngle(smooth_pattern, time_ms);
        assert(angle >= 0.0 && angle <= 40.0);
    }

    // 방향 전환점에서는 오버슈트를 막기 위해 중간 속도를 0으로 만든다.
    MotionPattern reversing_pattern({
        MotionFrame{"peak", 0, 1000, {{1, 20.0}}, {}},
        MotionFrame{"return", 1000, 1000, {{1, 10.0}}, {}},
    }, 2000);
    reversing_pattern.setInitialAngles({{1, 0.0}});
    assert(std::abs(numericalVelocity(reversing_pattern, 1000)) < 0.01);
    // 1ms 정수 샘플의 좌우 유한차분에는 양쪽 구간 jerk 차이의 오차가 남는다.
    assert(std::abs(numericalAcceleration(reversing_pattern, 1000)) < 1.0);

    // 프레임 공백은 다음 목표의 도달 시간까지 이어지는 이동 시간에 포함된다.
    MotionPattern gap_pattern({
        MotionFrame{"hold", 0, 1000, {{1, 0.0}}, {}},
        MotionFrame{"target", 1500, 500, {{1, 100.0}}, {}},
    }, 2000);
    gap_pattern.setInitialAngles({{1, 0.0}});
    assert(close(jointAngle(gap_pattern, 1000), 0.0));
    assert(std::abs(jointAngle(gap_pattern, 1500) - 50.0) < 1e-9);
    assert(close(jointAngle(gap_pattern, 2000), 100.0));

    // 단일 회전 Position Mode 경계를 넘는 최단각 궤적은 안전하게 거부한다.
    bool boundary_rejected = false;
    try {
        MotionPattern invalid_boundary({
            MotionFrame{"near_max", 0, 1000, {{1, 170.0}}, {}},
            MotionFrame{"wrapped", 1000, 1000, {{1, -170.0}}, {}},
        }, 2000);
    } catch (const std::runtime_error&) {
        boundary_rejected = true;
    }
    assert(boundary_rejected);

    const auto example_path =
        std::filesystem::path(TEST_SOURCE_DIR) / "robot_motions.example.json";
    const auto library = MotionLibrary::loadGuiJson(example_path);
    assert(library.contains("walk"));
    assert(library.motion("walk").frames().size() == 2);
    std::cout << "GUI-compatible pattern tests passed\n";
}
