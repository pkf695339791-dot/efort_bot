#include "robot_control.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program
        << " [--points FILE] [--config FILE] [--real-robot]\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string points_path = "../samples/tie_points.json";
    std::string config_path;
    bool real_robot = false;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--points" && index + 1 < argc) {
            points_path = argv[++index];
        } else if (argument == "--config" && index + 1 < argc) {
            config_path = argv[++index];
        } else if (argument == "--real-robot") {
            real_robot = true;
        } else if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown or incomplete argument: " << argument << '\n';
            print_usage(argv[0]);
            return 2;
        }
    }

    try {
        tie::RobotControlConfig config =
            config_path.empty() ? tie::RobotControlConfig{} : tie::load_config_json(config_path);
        if (real_robot) config.dry_run = false;

        const auto points = tie::load_tie_points_json(points_path);
        tie::RobotControlSystem system(config);
        const auto queue = system.run(points);

        std::cout << "Generated and sent " << queue.size()
                  << " queue points from " << points.size() << " tie points.\n";
        if (!system.skipped().empty()) {
            std::cout << "Skipped points:\n";
            for (const std::string& item : system.skipped()) {
                std::cout << "  - " << item << '\n';
            }
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Execution failed: " << error.what() << '\n';
        return 1;
    }
}

