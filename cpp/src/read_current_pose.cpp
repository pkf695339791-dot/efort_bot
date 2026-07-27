#include "EfortSdk.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Options {
    std::string ip;
    bool verify_version{true};
};

void check(int result, const std::string& operation) {
    if (result != ERROR_OK) {
        throw std::runtime_error(
            operation + " failed, SDK error code=" + std::to_string(result));
    }
}

void print_usage(const char* program) {
    std::cout
        << "Usage:\n  " << program
        << " --ip CONTROLLER_ADDRESS [--skip-version-check]\n\n"
        << "Reads the current robot state without commanding motion or IO.\n"
        << "The printed Cartesian TCP pose is expressed in the robot base frame.\n"
        << "  --skip-version-check  connect despite an SDK/controller version mismatch\n";
}

Options parse_options(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (arg == "--ip") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--ip requires an address");
            }
            options.ip = argv[++i];
            continue;
        }
        if (arg == "--skip-version-check") {
            options.verify_version = false;
            continue;
        }
        throw std::invalid_argument("unknown option: " + arg);
    }

    if (options.ip.empty()) {
        throw std::invalid_argument(
            "--ip is required; use the actual controller address shown on site");
    }
    return options;
}

void print_state(
    const RobotAPI::RobotPos& pose,
    const RobotAPI::RobotJoint& joints,
    const std::string& tool,
    const std::string& user_frame) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Current tool       : " << tool << '\n';
    std::cout << "Current user frame : " << user_frame << '\n';
    std::cout << "Base TCP XYZABC    : "
              << pose.x << ' ' << pose.y << ' ' << pose.z << ' '
              << pose.a << ' ' << pose.b << ' ' << pose.c << '\n';
    std::cout << "CFG                : "
              << pose.cfgx << ' ' << pose.cfg1 << ' '
              << pose.cfg4 << ' ' << pose.cfg6 << '\n';
    std::cout << "Robot joints J1-J6 : "
              << joints.j[0] << ' ' << joints.j[1] << ' ' << joints.j[2] << ' '
              << joints.j[3] << ' ' << joints.j[4] << ' ' << joints.j[5] << '\n';
    std::cout << "External axes E1-E6: "
              << pose.ej1 << ' ' << pose.ej2 << ' ' << pose.ej3 << ' '
              << pose.ej4 << ' ' << pose.ej5 << ' ' << pose.ej6 << "\n\n";

    // These two lines can be copied directly into run_single_point_tie arguments.
    std::cout << "Copy for the tying program:\n";
    std::cout << "--pose "
              << pose.x << ' ' << pose.y << ' ' << pose.z << ' '
              << pose.a << ' ' << pose.b << ' ' << pose.c << '\n';
    std::cout << "--cfg "
              << pose.cfgx << ' ' << pose.cfg1 << ' '
              << pose.cfg4 << ' ' << pose.cfg6 << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    unsigned device_id = 0;
    bool connected = false;

    try {
        const Options options = parse_options(argc, argv);

        std::cout << "Connecting to controller " << options.ip << " ...\n";
        if (!options.verify_version) {
            std::cerr
                << "WARNING: SDK/controller version verification is disabled.\n";
        }
        // This is a read-only utility. It intentionally does not enable API
        // control, servo power, motion, or digital outputs.
        check(RobotAPI::ConnectRobot(
                  options.ip, device_id, true, false, 2, options.verify_version),
              "ConnectRobot");
        connected = true;

        RobotAPI::RobotPos pose;
        RobotAPI::RobotJoint joints;
        std::string tool;
        std::string user_frame;

        check(RobotAPI::BlockGetBaseCoordinatePos(pose, device_id),
              "BlockGetBaseCoordinatePos");
        check(RobotAPI::BlockGetJointPos(joints, device_id),
              "BlockGetJointPos");
        check(RobotAPI::GetCurrentToolName(tool, device_id),
              "GetCurrentToolName");
        check(RobotAPI::GetCurrentUframeName(user_frame, device_id),
              "GetCurrentUframeName");

        print_state(pose, joints, tool, user_frame);

        RobotAPI::DisconnectRobot(device_id);
        connected = false;
        return 0;
    } catch (const std::exception& error) {
        if (connected) RobotAPI::DisconnectRobot(device_id);
        std::cerr << "Read current pose failed: " << error.what() << '\n';
        return 1;
    }
}
