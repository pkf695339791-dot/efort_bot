#include "EfortSdk.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Pose {
    double x{};
    double y{};
    double z{};
    double a{};
    double b{};
    double c{};
    int cfgx{};
    int cfg1{};
    int cfg4{};
    int cfg6{};
};

struct Options {
    std::string ip{"192.168.1.12"};
    std::string tool{"tool_tie"};
    // wobj0 is the controller base frame. Use the same calibrated tool that was
    // active when the point was taught.
    std::string workobject{"wobj0"};
    std::vector<Pose> targets;
    std::array<int, 4> common_cfg{};
    std::array<double, 3> approach_direction{0.0, 0.0, 1.0};
    double approach_distance_mm{50.0};
    int joint_speed{5};
    int linear_speed{5};
    unsigned global_speed_percent{10};
    unsigned do_index{};
    unsigned pulse_ms{500};
    double position_accuracy_mm{1.0};
    double arrival_timeout_seconds{15.0};
    bool cfg_supplied{false};
    bool do_supplied{false};
    bool execute{false};
    bool verify_version{true};
};

void check(int result, const std::string& operation) {
    if (result != ERROR_OK) {
        throw std::runtime_error(
            operation + " failed, SDK error code=" + std::to_string(result));
    }
}

double parse_double(const char* text, const std::string& name) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (!end || *end != '\0' || !std::isfinite(value)) {
        throw std::invalid_argument(name + " must be a finite number");
    }
    return value;
}

long parse_long(const char* text, const std::string& name) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (!end || *end != '\0') {
        throw std::invalid_argument(name + " must be an integer");
    }
    return value;
}

void require_values(int index, int argc, int count, const std::string& option) {
    if (index + count >= argc) {
        throw std::invalid_argument(option + " has too few values");
    }
}

void print_usage(const char* program) {
    std::cout
        << "Usage:\n  " << program
        << " --pose X Y Z A B C [--pose X Y Z A B C ...]"
        << " --cfg CFGX CFG1 CFG4 CFG6 [options]\n\n"
        << "Required for movement:\n"
        << "  --pose ...              repeat once per target, in execution order (mm/deg)\n"
        << "  --cfg ...               configuration applied to every supplied target\n"
        << "  --execute               auto power on and move; omit for connected preflight only\n\n"
        << "Options:\n"
        << "  --ip ADDRESS            controller address (default 192.168.1.12)\n"
        << "  --tool NAME             calibrated tool name (default tool_tie)\n"
        << "  --wobj NAME             frame name (default wobj0 = robot base)\n"
        << "  --joint-speed N         direct MJOINT speed profile (default 5)\n"
        << "  --approach ...          reserved; approach/retreat motion is currently disabled\n"
        << "  --linear-speed N        reserved; linear motion is currently disabled\n"
        << "  --global-speed PERCENT  controller speed cap, 1..100 (default 10)\n"
        << "  --do-index N            reserved; tying-tool output is currently disabled\n"
        << "  --pulse-ms MS           reserved; tying-tool output is currently disabled\n"
        << "  --accuracy MM           arrival tolerance (default 1.0)\n"
        << "  --wait-timeout SEC      maximum wait per movement (default 15.0)\n"
        << "  --skip-version-check    connect despite an SDK/controller version mismatch\n";
}

Options parse_options(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (arg == "--ip" || arg == "--tool" || arg == "--wobj") {
            require_values(i, argc, 1, arg);
            const std::string value = argv[++i];
            if (arg == "--ip") options.ip = value;
            if (arg == "--tool") options.tool = value;
            if (arg == "--wobj") options.workobject = value;
        } else if (arg == "--pose") {
            require_values(i, argc, 6, arg);
            Pose target;
            target.x = parse_double(argv[++i], "X");
            target.y = parse_double(argv[++i], "Y");
            target.z = parse_double(argv[++i], "Z");
            target.a = parse_double(argv[++i], "A");
            target.b = parse_double(argv[++i], "B");
            target.c = parse_double(argv[++i], "C");
            options.targets.push_back(target);
        } else if (arg == "--cfg") {
            require_values(i, argc, 4, arg);
            options.common_cfg[0] = static_cast<int>(parse_long(argv[++i], "CFGX"));
            options.common_cfg[1] = static_cast<int>(parse_long(argv[++i], "CFG1"));
            options.common_cfg[2] = static_cast<int>(parse_long(argv[++i], "CFG4"));
            options.common_cfg[3] = static_cast<int>(parse_long(argv[++i], "CFG6"));
            options.cfg_supplied = true;
        } else if (arg == "--approach") {
            require_values(i, argc, 4, arg);
            options.approach_direction[0] = parse_double(argv[++i], "NX");
            options.approach_direction[1] = parse_double(argv[++i], "NY");
            options.approach_direction[2] = parse_double(argv[++i], "NZ");
            options.approach_distance_mm = parse_double(argv[++i], "approach distance");
        } else if (arg == "--do-index") {
            require_values(i, argc, 1, arg);
            const long value = parse_long(argv[++i], "DO index");
            if (value < 0) throw std::invalid_argument("DO index cannot be negative");
            options.do_index = static_cast<unsigned>(value);
            options.do_supplied = true;
        } else if (arg == "--joint-speed" || arg == "--linear-speed") {
            require_values(i, argc, 1, arg);
            const int value = static_cast<int>(parse_long(argv[++i], arg));
            if (value <= 0) throw std::invalid_argument(arg + " must be positive");
            if (arg == "--joint-speed") options.joint_speed = value;
            if (arg == "--linear-speed") options.linear_speed = value;
        } else if (arg == "--global-speed") {
            require_values(i, argc, 1, arg);
            const long value = parse_long(argv[++i], arg);
            if (value < 1 || value > 100) {
                throw std::invalid_argument("--global-speed must be in [1, 100]");
            }
            options.global_speed_percent = static_cast<unsigned>(value);
        } else if (arg == "--pulse-ms") {
            require_values(i, argc, 1, arg);
            const long value = parse_long(argv[++i], arg);
            if (value < 1) throw std::invalid_argument("--pulse-ms must be positive");
            options.pulse_ms = static_cast<unsigned>(value);
        } else if (arg == "--accuracy" || arg == "--wait-timeout") {
            require_values(i, argc, 1, arg);
            const double value = parse_double(argv[++i], arg);
            if (value <= 0.0) throw std::invalid_argument(arg + " must be positive");
            if (arg == "--accuracy") options.position_accuracy_mm = value;
            if (arg == "--wait-timeout") options.arrival_timeout_seconds = value;
        } else if (arg == "--execute") {
            options.execute = true;
        } else if (arg == "--skip-version-check") {
            options.verify_version = false;
        } else {
            throw std::invalid_argument("unknown option: " + arg);
        }
    }
    if (options.targets.empty()) throw std::invalid_argument("at least one --pose is required");
    if (!options.cfg_supplied) throw std::invalid_argument("--cfg is required");
    for (Pose& target : options.targets) {
        target.cfgx = options.common_cfg[0];
        target.cfg1 = options.common_cfg[1];
        target.cfg4 = options.common_cfg[2];
        target.cfg6 = options.common_cfg[3];
    }
    if (options.approach_distance_mm <= 0.0) {
        throw std::invalid_argument("approach distance must be positive");
    }
    const double nx = options.approach_direction[0];
    const double ny = options.approach_direction[1];
    const double nz = options.approach_direction[2];
    const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (length < 1e-9) throw std::invalid_argument("approach direction cannot be zero");
    for (double& component : options.approach_direction) component /= length;
    return options;
}

RobotAPI::PointC point_c(const Pose& pose) {
    RobotAPI::PointC value;
    value.x = pose.x;
    value.y = pose.y;
    value.z = pose.z;
    value.a = pose.a;
    value.b = pose.b;
    value.c = pose.c;
    value.cfgx = static_cast<unsigned>(pose.cfgx);
    value.cfg1 = pose.cfg1;
    value.cfg4 = pose.cfg4;
    value.cfg6 = pose.cfg6;
    return value;
}

RobotAPI::RobotPos robot_pos(const Pose& pose) {
    RobotAPI::RobotPos value;
    value.x = pose.x;
    value.y = pose.y;
    value.z = pose.z;
    value.a = pose.a;
    value.b = pose.b;
    value.c = pose.c;
    value.cfgx = pose.cfgx;
    value.cfg1 = pose.cfg1;
    value.cfg4 = pose.cfg4;
    value.cfg6 = pose.cfg6;
    return value;
}

std::array<double, 6> pose_array(const Pose& pose) {
    return {pose.x, pose.y, pose.z, pose.a, pose.b, pose.c};
}

std::array<int, 4> cfg_array(const Pose& pose) {
    return {pose.cfgx, pose.cfg1, pose.cfg4, pose.cfg6};
}

void print_pose(const std::string& label, const Pose& pose) {
    std::cout << std::fixed << std::setprecision(3)
              << label << " XYZABC=[" << pose.x << ", " << pose.y << ", "
              << pose.z << ", " << pose.a << ", " << pose.b << ", " << pose.c
              << "] CFG=[" << pose.cfgx << ", " << pose.cfg1 << ", "
              << pose.cfg4 << ", " << pose.cfg6 << "]\n";
}

void assert_robot_safe(unsigned device_id, bool require_servo) {
    bool alarm = false;
    bool emergency_stop = false;
    bool servo_on = false;
    check(RobotAPI::GetCurrentAlarmStatus(alarm, device_id), "GetCurrentAlarmStatus");
    check(RobotAPI::GetCurrentEmgStatus(emergency_stop, device_id), "GetCurrentEmgStatus");
    check(RobotAPI::GetCurrentServoStatus(servo_on, device_id), "GetCurrentServoStatus");
    if (alarm) throw std::runtime_error("robot has an active alarm");
    if (emergency_stop) throw std::runtime_error("emergency stop is active");
    if (require_servo && !servo_on) {
        throw std::runtime_error("servo is off; power on manually before --execute");
    }
}

void ensure_servo_on(unsigned device_id) {
    bool servo_on = false;
    check(RobotAPI::GetCurrentServoStatus(servo_on, device_id),
          "GetCurrentServoStatus(before PowerOn)");
    if (servo_on) {
        std::cout << "Servo is already on.\n";
        return;
    }

    // PowerOn requests servo power through the controller. Hardware emergency
    // stops, safety doors and the controller operating mode remain authoritative.
    std::cout << "Servo is off; requesting automatic PowerOn.\n";
    check(RobotAPI::PowerOn(device_id), "PowerOn");

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        assert_robot_safe(device_id, false);
        check(RobotAPI::GetCurrentServoStatus(servo_on, device_id),
              "GetCurrentServoStatus(after PowerOn)");
        if (servo_on) {
            std::cout << "Automatic PowerOn confirmed.\n";
            return;
        }
    }
    throw std::runtime_error(
        "automatic PowerOn was not confirmed within 10 seconds; check auto/remote mode and safety circuit");
}

void preflight_pose(
    const Pose& pose,
    const Options& options,
    unsigned device_id,
    const char* label) {
    check(RobotAPI::CheckTarget(
              point_c(pose), options.tool, options.workobject, device_id),
          std::string("CheckTarget(") + label + ")");
    RobotAPI::RobotJoint joints;
    check(RobotAPI::IkSolver(
              robot_pos(pose), joints, options.tool, options.workobject, device_id),
          std::string("IkSolver(") + label + ")");
}

void wait_at(const Pose& pose, const Options& options, unsigned device_id) {
    check(RobotAPI::WaitCartPosition(
              robot_pos(pose),
              options.position_accuracy_mm,
              device_id,
              options.arrival_timeout_seconds),
          "WaitCartPosition");
    assert_robot_safe(device_id, true);
}

}  // namespace

int main(int argc, char* argv[]) {
    unsigned device_id = 0;
    bool connected = false;
    bool api_control = false;
    bool output_active = false;
    unsigned output_index = 0;

    try {
        const Options options = parse_options(argc, argv);
        output_index = options.do_index;

        for (std::size_t i = 0; i < options.targets.size(); ++i) {
            print_pose("target " + std::to_string(i + 1), options.targets[i]);
        }
        std::cout << "tool=" << options.tool << " wobj=" << options.workobject
                  << " global_speed=" << options.global_speed_percent << "%\n";
        if (!options.verify_version) {
            std::cerr
                << "WARNING: SDK/controller version verification is disabled. "
                << "Use connected preflight before considering motion.\n";
        }

        // Use the five-argument form shared by the Windows V2.8 header and the
        // older header bundled with the Linux demo. This also keeps VS Code
        // IntelliSense correct if it happens to discover both SDK copies.
        check(RobotAPI::ConnectRobot(
                  options.ip, device_id, true, false, 2, options.verify_version),
              "ConnectRobot");
        connected = true;
        // The vendor examples wait after connection before writing the API
        // control variable. Older controllers need time to finish attaching
        // the communication environment.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        check(RobotAPI::EnableApiControl(true, device_id), "EnableApiControl(true)");
        api_control = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        check(RobotAPI::SetCurrentToolByName(options.tool, device_id),
              "SetCurrentToolByName");
        check(RobotAPI::SetCurrentUframeByName(options.workobject, device_id),
              "SetCurrentUframeByName");
        // Reachability is checked before requesting servo power. Alarm and
        // emergency-stop conditions still stop execution immediately.
        assert_robot_safe(device_id, false);

        for (std::size_t i = 0; i < options.targets.size(); ++i) {
            const std::string label = "target " + std::to_string(i + 1);
            preflight_pose(options.targets[i], options, device_id, label.c_str());
        }
        std::cout << "Preflight passed: all " << options.targets.size()
                  << " Cartesian target(s) are reachable.\n";

        if (!options.execute) {
            std::cout << "No movement performed. Add --execute after validating the values.\n";
        } else {
            ensure_servo_on(device_id);
            assert_robot_safe(device_id, true);
            check(RobotAPI::SetGlobalSpeed(options.global_speed_percent, device_id),
                  "SetGlobalSpeed");
            // Tying-head IO is intentionally disabled until the physical IO
            // wiring and signal polarity have been commissioned.
#if 0
            bool initial_output = false;
            check(RobotAPI::ReadDOut(options.do_index, initial_output, device_id),
                  "ReadDOut");
            if (initial_output) {
                throw std::runtime_error(
                    "tie-tool DO is already active; turn it off and inspect the IO mapping");
            }
#endif

            for (std::size_t i = 0; i < options.targets.size(); ++i) {
                const Pose& target = options.targets[i];
                auto target_values = pose_array(target);
                auto config_values = cfg_array(target);
                const std::string label = "target " + std::to_string(i + 1);

                std::cout << (i + 1) << '/' << options.targets.size()
                          << " MJOINT directly to " << label << '\n';
                check(RobotAPI::MJOINT_CFG_TWS(
                          target_values.data(), config_values.data(), options.tool,
                          options.workobject, options.joint_speed, device_id),
                      "MJOINT_CFG_TWS(" + label + ")");
                wait_at(target, options, device_id);
            }

            std::cout << "Tying-tool output disabled; no DO was written.\n";
#if 0
            // Re-enable this block only after the tying-head IO index, voltage,
            // polarity and pulse duration have been verified on site.
            std::cout << "3/4 pulse DO[" << options.do_index << "] for "
                      << options.pulse_ms << " ms\n";
            check(RobotAPI::WriteDOut(options.do_index, true, device_id),
                  "WriteDOut(true)");
            output_active = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(options.pulse_ms));
            check(RobotAPI::WriteDOut(options.do_index, false, device_id),
                  "WriteDOut(false)");
            output_active = false;
#endif
            std::cout << "Sequential target movement completed.\n";
        }

        check(RobotAPI::EnableApiControl(false, device_id), "EnableApiControl(false)");
        api_control = false;
        RobotAPI::DisconnectRobot(device_id);
        connected = false;
        return 0;
    } catch (const std::exception& error) {
#if 0
        // Keep disabled until the tying-head IO wiring and signal polarity are verified.
        if (connected && api_control && output_active) {
            RobotAPI::WriteDOut(output_index, false, device_id);
        }
#endif
        if (connected && api_control) RobotAPI::MOVECLEAR(device_id);
        if (connected && api_control) RobotAPI::EnableApiControl(false, device_id);
        if (connected) RobotAPI::DisconnectRobot(device_id);
        std::cerr << "Single-point tying failed: " << error.what() << '\n';
        return 1;
    }
}
