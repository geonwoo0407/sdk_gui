#include "irc_step_motion_executor/simulated_motion_backend.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

using irc_step_motion_executor::BackendState;
using irc_step_motion_executor::SimulatedMotionBackend;
using irc_step_motion_executor::SimulatedMotionBackendConfig;

TEST(SimulatedMotionBackend, TransitionsByPollCountWithoutSleeping)
{
  SimulatedMotionBackend backend({2, 1, false, false});
  ASSERT_TRUE(backend.start_motion("resolved alias").accepted);
  ASSERT_TRUE(backend.active_motion_name().has_value());
  EXPECT_EQ(*backend.active_motion_name(), "resolved alias");

  EXPECT_EQ(backend.poll_status().state, BackendState::RUNNING);
  EXPECT_EQ(backend.poll_status().state, BackendState::RUNNING);
  EXPECT_EQ(backend.poll_status().state, BackendState::SETTLING);
  EXPECT_EQ(backend.poll_status().state, BackendState::SUCCEEDED);
  EXPECT_FALSE(backend.active_motion_name().has_value());
  EXPECT_EQ(backend.poll_status().state, BackendState::IDLE);
}

TEST(SimulatedMotionBackend, CancelIsReportedOnNextPoll)
{
  SimulatedMotionBackend backend({2, 1, false, false});
  ASSERT_TRUE(backend.start_motion("resolved alias").accepted);
  EXPECT_TRUE(backend.cancel_motion().accepted);
  EXPECT_EQ(backend.poll_status().state, BackendState::CANCELLED);
  EXPECT_FALSE(backend.active_motion_name().has_value());
}

TEST(SimulatedMotionBackend, RejectsCancelAndSecondStartWhenActiveStateDisallowsIt)
{
  SimulatedMotionBackend backend({2, 1, false, false});
  const auto idle_cancel = backend.cancel_motion();
  EXPECT_FALSE(idle_cancel.accepted);
  EXPECT_EQ(idle_cancel.error_code, "NOT_RUNNING");

  ASSERT_TRUE(backend.start_motion("first resolved alias").accepted);
  const auto second_start = backend.start_motion("second resolved alias");
  EXPECT_FALSE(second_start.accepted);
  EXPECT_EQ(second_start.error_code, "SIMULATED_BUSY");
  ASSERT_TRUE(backend.active_motion_name().has_value());
  EXPECT_EQ(*backend.active_motion_name(), "first resolved alias");
}

TEST(SimulatedMotionBackend, SupportsConfiguredStartAndPollFailures)
{
  SimulatedMotionBackend start_failure({2, 1, true, false});
  EXPECT_FALSE(start_failure.start_motion("resolved alias").accepted);

  SimulatedMotionBackend poll_failure({2, 1, false, true});
  ASSERT_TRUE(poll_failure.start_motion("resolved alias").accepted);
  const auto failed = poll_failure.poll_status();
  EXPECT_EQ(failed.state, BackendState::FAILED);
  EXPECT_EQ(failed.error_code, "SIMULATED_BACKEND_FAILURE");
}

}  // namespace
