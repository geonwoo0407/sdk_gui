#ifndef IRC_STEP_MOTION_EXECUTOR__CATALOG_ONLY_CORE_HPP_
#define IRC_STEP_MOTION_EXECUTOR__CATALOG_ONLY_CORE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace irc_step_motion_executor
{

struct MotionRequest
{
  std::int64_t request_id{0};
  std::string motion_id;
  std::optional<std::int64_t> command_id;
  std::optional<std::int64_t> event_id;
  std::optional<std::string> action;
};

struct MotionStatus
{
  std::string status{"REJECTED"};
  std::optional<std::string> action;
  std::optional<std::int64_t> command_id;
  std::optional<std::int64_t> event_id;
  std::int64_t request_id{0};
  std::string motion_id;
  std::string error_code;
  std::string message;
};

class MotionAliasCatalog
{
public:
  bool load(const std::string & path, std::string & error_message);
  bool contains(const std::string & motion_id) const;
  std::optional<std::string> resolve(const std::string & motion_id) const;
  std::size_t size() const;

private:
  std::map<std::string, std::string> aliases_;
};

class CatalogOnlyCore
{
public:
  explicit CatalogOnlyCore(MotionAliasCatalog catalog);
  MotionStatus handle_request(const std::string & payload) const;
  MotionStatus handle_cancel(const std::string & payload) const;
  static std::string to_json(const MotionStatus & status);

private:
  MotionAliasCatalog catalog_;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__CATALOG_ONLY_CORE_HPP_
