#ifndef FAKE_ROBOT_MOTION_SDK_TEST_SUPPORT_HPP_
#define FAKE_ROBOT_MOTION_SDK_TEST_SUPPORT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace irc_step::fake_sdk
{

void reset_tracking();
void set_player_constructor_throws(bool value);
void set_player_initialize_result(bool success, std::string error_message);
void set_hardware_preflight_result(bool success, std::string error_message);
int hardware_construction_count();
int hardware_preflight_count();
int hardware_initialize_count();
int player_construction_count();
int player_initialize_count();
const std::string & hardware_device_path();
std::int64_t hardware_baud_rate();
const std::vector<int> & hardware_motor_ids();
const std::vector<std::string> & destruction_order();

}  // namespace irc_step::fake_sdk

#endif  // FAKE_ROBOT_MOTION_SDK_TEST_SUPPORT_HPP_
