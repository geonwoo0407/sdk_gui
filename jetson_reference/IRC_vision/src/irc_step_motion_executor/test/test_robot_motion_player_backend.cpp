#include "irc_step_motion_executor/robot_motion_player_backend.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <string_view>

namespace
{

class FakeRobotMotionPlayerApi final
  : public irc_step_motion_executor::RobotMotionPlayerApi
{
public:
  irc_step::StartResult start_result{irc_step::StartResult::Accepted};
  irc_step::QueueResult queue_result{irc_step::QueueResult::Queued};
  irc_step::CancelResult cancel_result{irc_step::CancelResult::Cancelled};
  irc_step::MotionStatus motion_status{irc_step::MotionStatus::Idle};
  irc_step::MotionError motion_error{irc_step::MotionError::None};
  std::string error_message;
  std::string received_motion_name;
  bool throw_on_start{false};
  bool throw_on_cancel{false};
  bool throw_on_update{false};
  std::uint64_t sequence{0};
  int override_motor_id{-1};
  double override_target_deg{0.0};

  irc_step::StartResult start(std::string_view motion_name) override
  {
    if (throw_on_start) {
      throw std::runtime_error("fake start exception");
    }
    received_motion_name = std::string(motion_name);
    return start_result;
  }

  irc_step::QueueResult queue_next(std::string_view motion_name) override
  {
    received_motion_name = std::string(motion_name);
    return queue_result;
  }

  irc_step::CancelResult cancel() override
  {
    if (throw_on_cancel) {
      throw std::runtime_error("fake cancel exception");
    }
    return cancel_result;
  }

  irc_step::MotionStatus update() override
  {
    if (throw_on_update) {
      throw std::runtime_error("fake update exception");
    }
    return motion_status;
  }

  irc_step::MotionError result() const override
  {
    return motion_error;
  }

  std::string last_error() const override
  {
    return error_message;
  }


  std::uint64_t completion_sequence() const override
  {
    return sequence;
  }

  void set_joint_override(int motor_id, double target_deg) override
  {
    override_motor_id = motor_id;
    override_target_deg = target_deg;
  }

  void clear_joint_override(int motor_id) noexcept override
  {
    if (override_motor_id == motor_id) {
      override_motor_id = -1;
    }
  }
};

TEST(RobotMotionPlayerBackend, PassesResolvedMotionNameWithoutModification)
{
  FakeRobotMotionPlayerApi api;
  irc_step_motion_executor::RobotMotionPlayerBackend backend(api);

  const auto result = backend.start_motion("첫발");

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(api.received_motion_name, "첫발");
}

TEST(RobotMotionPlayerBackend, ForwardsAbsoluteJointOverride)
{
  FakeRobotMotionPlayerApi api;
  irc_step_motion_executor::RobotMotionPlayerBackend backend(api);

  EXPECT_TRUE(backend.set_joint_override(0, -60.0));
  EXPECT_EQ(api.override_motor_id, 0);
  EXPECT_DOUBLE_EQ(api.override_target_deg, -60.0);

  backend.clear_joint_override(0);
  EXPECT_EQ(api.override_motor_id, -1);
}

TEST(RobotMotionPlayerBackend, MapsEveryKnownStartResult)
{
  struct Case
  {
    irc_step::StartResult sdk_result;
    bool accepted;
    const char * error_code;
  };
  for (const Case & test_case : {
      Case{irc_step::StartResult::Accepted, true, ""},
      Case{irc_step::StartResult::RejectedBusy, false, "SDK_BUSY"},
      Case{
        irc_step::StartResult::MotionNotFound, false,
        "SDK_MOTION_NOT_FOUND"},
      Case{
        irc_step::StartResult::HardwareNotReady, false,
        "SDK_HARDWARE_NOT_READY"},
      Case{
        irc_step::StartResult::InvalidMotion, false,
        "SDK_INVALID_MOTION"}})
  {
    FakeRobotMotionPlayerApi api;
    api.start_result = test_case.sdk_result;
    irc_step_motion_executor::RobotMotionPlayerBackend backend(api);
    const auto result = backend.start_motion("resolved");
    EXPECT_EQ(result.accepted, test_case.accepted);
    EXPECT_EQ(result.error_code, test_case.error_code);
  }
}

TEST(RobotMotionPlayerBackend, UnknownStartResultFailsSafely)
{
  FakeRobotMotionPlayerApi api;
  api.start_result = static_cast<irc_step::StartResult>(255);
  irc_step_motion_executor::RobotMotionPlayerBackend backend(api);

  const auto result = backend.start_motion("resolved");

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.error_code, "SDK_UNKNOWN_START_RESULT");
}

TEST(RobotMotionPlayerBackend, MapsEveryKnownCancelResult)
{
  struct Case
  {
    irc_step::CancelResult sdk_result;
    bool accepted;
    const char * error_code;
  };
  for (const Case & test_case : {
      Case{irc_step::CancelResult::Cancelled, true, ""},
      Case{irc_step::CancelResult::NotRunning, false, "NOT_RUNNING"},
      Case{
        irc_step::CancelResult::HardwareNotReady, false,
        "SDK_HARDWARE_NOT_READY"},
      Case{
        irc_step::CancelResult::HoldFailed, false,
        "SDK_CANCEL_HOLD_FAILED"}})
  {
    FakeRobotMotionPlayerApi api;
    api.cancel_result = test_case.sdk_result;
    irc_step_motion_executor::RobotMotionPlayerBackend backend(api);
    const auto result = backend.cancel_motion();
    EXPECT_EQ(result.accepted, test_case.accepted);
    EXPECT_EQ(result.error_code, test_case.error_code);
  }
}

TEST(RobotMotionPlayerBackend, MapsEveryKnownMotionStatus)
{
  struct Case
  {
    irc_step::MotionStatus sdk_status;
    irc_step_motion_executor::BackendState backend_state;
  };
  for (const Case & test_case : {
      Case{
        irc_step::MotionStatus::Idle,
        irc_step_motion_executor::BackendState::IDLE},
      Case{
        irc_step::MotionStatus::Running,
        irc_step_motion_executor::BackendState::RUNNING},
      Case{
        irc_step::MotionStatus::Settling,
        irc_step_motion_executor::BackendState::SETTLING},
      Case{
        irc_step::MotionStatus::Succeeded,
        irc_step_motion_executor::BackendState::SUCCEEDED},
      Case{
        irc_step::MotionStatus::Cancelled,
        irc_step_motion_executor::BackendState::CANCELLED},
      Case{
        irc_step::MotionStatus::Failed,
        irc_step_motion_executor::BackendState::FAILED}})
  {
    FakeRobotMotionPlayerApi api;
    api.motion_status = test_case.sdk_status;
    api.motion_error = irc_step::MotionError::InternalError;
    irc_step_motion_executor::RobotMotionPlayerBackend backend(api);
    EXPECT_EQ(backend.poll_status().state, test_case.backend_state);
  }
}

TEST(RobotMotionPlayerBackend, MapsSdkFailureCodeAndMessage)
{
  FakeRobotMotionPlayerApi api;
  api.motion_status = irc_step::MotionStatus::Failed;
  api.motion_error = irc_step::MotionError::PositionTimeout;
  api.error_message = "position did not settle";
  irc_step_motion_executor::RobotMotionPlayerBackend backend(api);

  const auto status = backend.poll_status();

  EXPECT_EQ(status.state, irc_step_motion_executor::BackendState::FAILED);
  EXPECT_EQ(status.error_code, "SDK_POSITION_TIMEOUT");
  EXPECT_EQ(status.message, "position did not settle");
}

TEST(RobotMotionPlayerBackend, UnknownStatusBecomesFailed)
{
  FakeRobotMotionPlayerApi api;
  api.motion_status = static_cast<irc_step::MotionStatus>(255);
  irc_step_motion_executor::RobotMotionPlayerBackend backend(api);

  const auto status = backend.poll_status();

  EXPECT_EQ(status.state, irc_step_motion_executor::BackendState::FAILED);
  EXPECT_EQ(status.error_code, "SDK_UNKNOWN_STATUS");
}

TEST(RobotMotionPlayerBackend, ConvertsInjectedApiExceptionsSafely)
{
  FakeRobotMotionPlayerApi api;
  irc_step_motion_executor::RobotMotionPlayerBackend backend(api);

  api.throw_on_start = true;
  const auto start = backend.start_motion("전진");
  EXPECT_FALSE(start.accepted);
  EXPECT_EQ(start.error_code, "SDK_EXCEPTION");

  api.throw_on_start = false;
  api.throw_on_cancel = true;
  const auto cancel = backend.cancel_motion();
  EXPECT_FALSE(cancel.accepted);
  EXPECT_EQ(cancel.error_code, "SDK_EXCEPTION");

  api.throw_on_cancel = false;
  api.throw_on_update = true;
  const auto status = backend.poll_status();
  EXPECT_EQ(status.state, irc_step_motion_executor::BackendState::FAILED);
  EXPECT_EQ(status.error_code, "SDK_EXCEPTION");
}

}  // namespace
