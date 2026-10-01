#ifndef IRC_STEP_MOTION_EXECUTOR__TEST__FAKE_MOTION_BACKEND_HPP_
#define IRC_STEP_MOTION_EXECUTOR__TEST__FAKE_MOTION_BACKEND_HPP_

#include "irc_step_motion_executor/motion_backend.hpp"

#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class FakeMotionBackend
  : public irc_step_motion_executor::MotionBackend
{
public:
  irc_step_motion_executor::BackendStartResult start_result{
    true, "", "fake start accepted"};
  irc_step_motion_executor::BackendCancelResult cancel_result{
    true, "", "fake cancel accepted"};
  irc_step_motion_executor::BackendQueueResult queue_result{
    false, "QUEUE_UNSUPPORTED", "fake queue unsupported"};
  std::deque<irc_step_motion_executor::BackendStatus> statuses;
  std::vector<std::string> started_motion_names;
  std::vector<std::string> queued_motion_names;
  std::uint64_t sequence{0};
  int cancel_calls{0};
  int poll_calls{0};
  bool throw_on_start{false};
  bool throw_on_cancel{false};
  bool throw_on_poll{false};

  irc_step_motion_executor::BackendStartResult start_motion(
    const std::string & resolved_motion_name) override
  {
    if (throw_on_start) {
      throw std::runtime_error("fake start failure");
    }
    started_motion_names.push_back(resolved_motion_name);
    return start_result;
  }

  irc_step_motion_executor::BackendCancelResult cancel_motion() override
  {
    ++cancel_calls;
    if (throw_on_cancel) {
      throw std::runtime_error("fake cancel failure");
    }
    return cancel_result;
  }

  irc_step_motion_executor::BackendStatus poll_status() override
  {
    ++poll_calls;
    if (throw_on_poll) {
      throw std::runtime_error("fake poll failure");
    }
    if (statuses.empty()) {
      return {
        irc_step_motion_executor::BackendState::RUNNING, "", "fake running"};
    }
    auto status = std::move(statuses.front());
    statuses.pop_front();
    return status;
  }

  irc_step_motion_executor::BackendQueueResult queue_motion(
    const std::string & resolved_motion_name) override
  {
    queued_motion_names.push_back(resolved_motion_name);
    return queue_result;
  }

  std::uint64_t completion_sequence() const override
  {
    return sequence;
  }
};

#endif  // IRC_STEP_MOTION_EXECUTOR__TEST__FAKE_MOTION_BACKEND_HPP_
