#include "irc_step_motion_executor/startup_pose_catalog.hpp"

#include <json-c/json.h>

#include <cmath>
#include <fstream>
#include <iterator>

namespace irc_step_motion_executor
{

bool load_startup_pose_angles(
  const std::string & json_path, const std::string & pose_name,
  std::vector<double> & angles_deg, std::string & error_message)
{
  angles_deg.clear();
  std::ifstream stream(json_path);
  if (!stream) {error_message = "cannot open motion catalog: " + json_path; return false;}
  const std::string source{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  json_object * root = json_tokener_parse(source.c_str());
  json_object * motions = nullptr;
  if (root == nullptr || !json_object_object_get_ex(root, "motions", &motions) ||
    json_object_get_type(motions) != json_type_array)
  {
    if (root != nullptr) {json_object_put(root);}
    error_message = "invalid motion catalog JSON";
    return false;
  }
  std::size_t matching_frames = 0;
  bool inconsistent_duplicate = false;
  std::vector<double> canonical;
  const auto motion_count = json_object_array_length(motions);
  for (std::size_t i = 0; i < motion_count; ++i) {
    json_object * frames = nullptr;
    json_object * motion = json_object_array_get_idx(motions, i);
    if (!json_object_object_get_ex(motion, "frames", &frames) ||
      json_object_get_type(frames) != json_type_array) {continue;}
    const auto frame_count = json_object_array_length(frames);
    for (std::size_t j = 0; j < frame_count; ++j) {
      json_object * frame = json_object_array_get_idx(frames, j);
      json_object * name = nullptr;
      if (!json_object_object_get_ex(frame, "name", &name) ||
        json_object_get_type(name) != json_type_string ||
        pose_name != json_object_get_string(name)) {continue;}
      ++matching_frames;
      json_object * angles = nullptr;
      if (!json_object_object_get_ex(frame, "angles", &angles) ||
        json_object_get_type(angles) != json_type_object ||
        json_object_object_length(angles) != 23)
      {
        inconsistent_duplicate = true;
        continue;
      }
      std::vector<double> candidate(23);
      bool valid = true;
      for (int id = 0; id <= 22; ++id) {
        json_object * value = nullptr;
        const std::string key = std::to_string(id);
        if (!json_object_object_get_ex(angles, key.c_str(), &value) ||
          (json_object_get_type(value) != json_type_int &&
          json_object_get_type(value) != json_type_double) ||
          !std::isfinite(candidate[id] = json_object_get_double(value))) {
          valid = false; break;
        }
      }
      if (!valid) {
        inconsistent_duplicate = true;
      } else if (canonical.empty()) {
        canonical = std::move(candidate);
      } else if (candidate != canonical) {
        inconsistent_duplicate = true;
      }
    }
  }
  json_object_put(root);
  if (matching_frames == 0U) {
    error_message = "startup pose not found: " + pose_name;
    return false;
  }
  if (inconsistent_duplicate || canonical.empty()) {
    error_message = "startup pose name is ambiguous: " + pose_name;
    return false;
  }
  angles_deg = std::move(canonical);
  return true;
}

}  // namespace irc_step_motion_executor
