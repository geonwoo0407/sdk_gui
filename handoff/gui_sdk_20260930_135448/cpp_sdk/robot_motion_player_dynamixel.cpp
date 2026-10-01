#include "robot_motion_player.hpp"

#include "dynamixel_motion_hardware.hpp"

#include <memory>

namespace irc_step {

RobotMotionPlayer::RobotMotionPlayer(const std::string& json_path)
    : library_(MotionLibrary::loadGuiJson(json_path)),
      owned_hardware_(std::make_unique<DynamixelMotionHardware>()),
      hardware_(owned_hardware_.get()) {}

}  // namespace irc_step
