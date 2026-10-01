#ifndef FAKE_MOTION_HARDWARE_HPP_
#define FAKE_MOTION_HARDWARE_HPP_

namespace irc_step
{

class IMotionHardware
{
public:
  virtual ~IMotionHardware() = default;
  virtual bool initialize() noexcept = 0;
  virtual bool ready() const noexcept = 0;
};

}  // namespace irc_step

#endif  // FAKE_MOTION_HARDWARE_HPP_
