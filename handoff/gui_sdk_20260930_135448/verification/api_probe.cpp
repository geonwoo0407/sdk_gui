#include "robot_motion_player.hpp"
#include <iostream>
#include <vector>

// Unevaluated API checks only: no player or hardware object is constructed.
template<class T> constexpr bool startup = requires(T& p, const std::vector<double>& a) { p.startPoseTransition(a, 1000); };
template<class T> constexpr bool update_startup = requires(T& p) { p.updateStartupPose(); };
template<class T> constexpr bool queue = requires(T& p) { p.queueNext("example"); };
template<class T> constexpr bool completion = requires(T& p) { p.completionSequence(); };
template<class T> constexpr bool tolerance = requires(T& p) { p.setPositionToleranceEnabled(false); };
template<class T> constexpr bool override_joint = requires(T& p) { p.setJointOverride(0, 0.0); };
template<class T> constexpr bool clear_override = requires(T& p) { p.clearJointOverride(0); };

int main() {
    using P = irc_step::RobotMotionPlayer;
    std::cout << std::boolalpha
              << "startPoseTransition: " << startup<P> << '\n'
              << "updateStartupPose: " << update_startup<P> << '\n'
              << "queueNext: " << queue<P> << '\n'
              << "completionSequence: " << completion<P> << '\n'
              << "setPositionToleranceEnabled: " << tolerance<P> << '\n'
              << "setJointOverride: " << override_joint<P> << '\n'
              << "clearJointOverride: " << clear_override<P> << '\n';
}
