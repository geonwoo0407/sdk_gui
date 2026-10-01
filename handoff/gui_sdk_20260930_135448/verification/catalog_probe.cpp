#include "motion_pattern.hpp"
#include <exception>
#include <iostream>

// JSON loader only. This executable does not link robot_control or open USB.
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    try {
        const auto library = irc_step::MotionLibrary::loadGuiJson(argv[1]);
        std::cout << "Loaded motions: " << library.names().size() << '\n';
        for (const auto& name : library.names()) {
            const auto& motion = library.motion(name);
            if (name == "찐찐전진45(8회)" || name == "찐찐라인복귀좌회전45도(4회)")
                std::cout << name << ": frames=" << motion.frames().size()
                          << ", duration_ms=" << motion.durationMs()
                          << ", speed=" << motion.playbackSpeed()
                          << ", repeat=" << motion.repeatCount() << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
