#include "robot_motion_player.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic_bool stop_requested{false};
void requestStop(int) { stop_requested.store(true); }

bool runMotion(irc_step::RobotMotionPlayer& player, const std::string& name) {
    using irc_step::CancelResult;
    using irc_step::StartResult;
    const auto start_result = player.start(name);
    if (start_result != StartResult::Accepted) {
        std::cerr << "[Error] Motion start rejected ("
                  << static_cast<int>(start_result) << "): "
                  << player.lastError() << std::endl;
        return false;
    }
    while (player.running() && !stop_requested.load()) {
        player.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (stop_requested.load()) {
        const auto cancel_result = player.cancel();
        return cancel_result == CancelResult::Cancelled;
    }
    if (!player.succeeded()) {
        std::cerr << "[Error] Motion failed: " << player.lastError() << std::endl;
        return false;
    }
    return true;
}
}

int main(int argc, char** argv) {
    using irc_step::RobotMotionPlayer;
    if (argc < 2) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " robot_motions.json\n"
                  << "  " << argv[0] << " robot_motions.json <motion_name>\n";
        return 1;
    }

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    try {
        RobotMotionPlayer player(argv[1]);
        if (!player.initialize()) {
            std::cerr << "[Error] Hardware initialization failed: "
                      << player.lastError() << std::endl;
            return 1;
        }
        if (argc > 2) {
            if (!runMotion(player, argv[2])) {
                std::cerr << "[Error] Motion not found or player busy: " << argv[2]
                          << "\nAvailable:";
                for (const auto& name : player.motionNames()) std::cerr << ' ' << name;
                std::cerr << std::endl;
                return 1;
            }
            return 0;
        }

        std::cout << "[Ready] Enter a motion name, 'list', or 'quit'.\n";
        std::string command;
        while (!stop_requested.load()
               && std::cout << "> " && std::getline(std::cin, command)) {
            if (command == "quit" || command == "exit") break;
            if (command == "list") {
                std::cout << "Available:";
                for (const auto& name : player.motionNames()) std::cout << ' ' << name;
                std::cout << std::endl;
            } else if (!command.empty() && !runMotion(player, command)) {
                std::cerr << "[Error] Motion not found: " << command << std::endl;
            }
        }
        player.shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[Error] " << error.what() << std::endl;
        return 1;
    }
}
