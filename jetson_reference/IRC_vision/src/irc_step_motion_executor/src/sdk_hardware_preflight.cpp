#include "irc_step_motion_executor/sdk_hardware_preflight_core.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  try {
    std::vector<std::string> arguments;
    arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0U);
    for (int index = 1; index < argc; ++index) {
      arguments.emplace_back(argv[index]);
    }

    const auto parsed =
      irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);
    if (!parsed) {
      std::cerr << parsed.error << '\n'
                << irc_step_motion_executor::hardware_preflight_usage() << '\n';
      return 2;
    }

    irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
    const auto result =
      irc_step_motion_executor::run_hardware_preflight(factory, parsed.config);
    if (!result.output.empty()) {
      std::cout << result.output;
    }
    if (!result.error.empty()) {
      std::cerr << result.error << '\n';
    }
    return result.exit_code;
  } catch (const std::exception & exception) {
    std::cerr << "unexpected preflight CLI error: " << exception.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "unexpected preflight CLI error: unknown exception\n";
    return 1;
  }
}
