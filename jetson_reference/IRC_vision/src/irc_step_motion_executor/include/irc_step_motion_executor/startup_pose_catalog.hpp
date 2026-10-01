#ifndef IRC_STEP_MOTION_EXECUTOR__STARTUP_POSE_CATALOG_HPP_
#define IRC_STEP_MOTION_EXECUTOR__STARTUP_POSE_CATALOG_HPP_

#include <string>
#include <vector>

namespace irc_step_motion_executor
{

bool load_startup_pose_angles(
  const std::string & json_path, const std::string & pose_name,
  std::vector<double> & angles_deg, std::string & error_message);

}  // namespace irc_step_motion_executor

#endif
