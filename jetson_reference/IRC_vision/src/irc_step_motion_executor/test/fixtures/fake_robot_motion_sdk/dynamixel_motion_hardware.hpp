#ifndef FAKE_DYNAMIXEL_MOTION_HARDWARE_HPP_
#define FAKE_DYNAMIXEL_MOTION_HARDWARE_HPP_

#include "motion_hardware.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace irc_step
{

struct DynamixelMotionHardwareConfig
{
  std::string device_path;
  std::int64_t baud_rate{0};
  std::vector<int> motor_ids;
};

class DynamixelMotionHardware final : public IMotionHardware
{
public:
  DynamixelMotionHardware();
  explicit DynamixelMotionHardware(DynamixelMotionHardwareConfig config);
  ~DynamixelMotionHardware() override;

  bool preflight() noexcept;
  bool preflightReady() const noexcept;
  bool initialize() noexcept override;
  bool ready() const noexcept override;
  std::string_view lastError() const noexcept;

private:
  std::string last_error_;
};

}  // namespace irc_step

#endif  // FAKE_DYNAMIXEL_MOTION_HARDWARE_HPP_
