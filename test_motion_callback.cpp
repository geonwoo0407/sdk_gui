#include "motion_callback.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace irc_step;

int main() {
    std::vector<MotionFrame> frames{
        MotionFrame{"start", 0, 20, {{1, 20.0}}, {}},
        MotionFrame{"finish", 20, 20, {{1, 40.0}}, {}},
    };
    const MotionPattern pattern(std::move(frames), 40, 2, 1.0);

    int write_count = 0;
    JointAngles last_written;
    MotionCallback callback(
        pattern,
        [&](const JointAngles& angles) {
            ++write_count;
            last_written = angles;
            return true;
        },
        1);
    callback.setInitialAngles({{1, 0.0}});
    callback.start();

    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (callback.running() && std::chrono::steady_clock::now() < timeout) {
        callback.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    assert(!callback.running());
    assert(write_count > 2);
    assert(last_written.contains(1));
    assert(callback.lastTarget().finished);

    MotionCallback failing_callback(
        MotionPattern({MotionFrame{"fail", 0, 10, {{1, 10.0}}, {}}}, 10),
        [](const JointAngles&) { return false; },
        1);
    failing_callback.setInitialAngles({{1, 0.0}});
    failing_callback.start();

    bool failure_reported = false;
    try {
        failing_callback.update();
    } catch (const std::runtime_error&) {
        failure_reported = true;
    }
    assert(failure_reported);
    assert(!failing_callback.running());

    std::cout << "Motion callback tests passed\n";
}
