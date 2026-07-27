#include "robot_control.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

class HandshakeBackend final : public tie::RobotBackend {
public:
    void connect() override { connected = true; }
    void disconnect() noexcept override { connected = false; }
    void prepare() override {}
    std::vector<tie::MotionSegment> prepare_targets(
        const std::vector<tie::MotionSegment>& segments) override {
        ++prepare_target_calls;
        if (reject_preflight) {
            throw std::runtime_error("target 2 rejected by preflight");
        }
        auto prepared = segments;
        for (auto& segment : prepared) {
            if (segment.motion_type == tie::MotionType::MJoint) {
                tie::JointPose joints;
                joints.joints = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
                segment.joint_target = joints;
            }
        }
        return prepared;
    }
    void set_int(unsigned index, int value) override { ints[index] = value; }
    int get_int(unsigned index) override {
        if (force_controller_error && index == forced_error_code_index) {
            return forced_error_code;
        }
        const auto item = ints.find(index);
        return item == ints.end() ? 0 : item->second;
    }
    void set_bool(unsigned index, bool value) override { bools[index] = value; }
    bool get_bool(unsigned index) override {
        if (force_controller_error && index == forced_error_bool_index) return true;
        if (index == 0 || index == 1 || index == 2) return true;
        const auto item = bools.find(index);
        return item != bools.end() && item->second;
    }
    void set_motion_batch(
        const std::vector<tie::MotionSegment>& segments,
        std::size_t target_start_index,
        unsigned motion_type_start_index) override {
        target_starts.push_back(target_start_index);
        type_starts.push_back(motion_type_start_index);
        std::vector<int> batch_types;
        for (const auto& segment : segments) {
            batch_types.push_back(static_cast<int>(segment.motion_type));
            if (segment.motion_type == tie::MotionType::MJoint &&
                !segment.joint_target.has_value()) {
                all_joint_targets_resolved = false;
            }
        }
        type_batches.push_back(std::move(batch_types));
    }
    tie::RobotStatus read_status() override {
        return {connected, false, false, true, false};
    }

    bool connected{true};
    std::unordered_map<unsigned, bool> bools;
    std::unordered_map<unsigned, int> ints;
    std::vector<std::size_t> target_starts;
    std::vector<unsigned> type_starts;
    std::vector<std::vector<int>> type_batches;
    bool force_controller_error{};
    unsigned forced_error_bool_index{5};
    unsigned forced_error_code_index{3};
    int forced_error_code{9001};
    bool reject_preflight{};
    int prepare_target_calls{};
    bool all_joint_targets_resolved{true};
};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

std::vector<tie::TiePoint> make_points(int count) {
    std::vector<tie::TiePoint> points;
    for (int index = 0; index < count; ++index) {
        tie::CartesianPose pose;
        pose.x = 100.0 + index * 10.0;
        pose.y = -50.0;
        pose.z = 250.0;
        pose.cfgx = 1;
        points.push_back({"P" + std::to_string(index), pose, 0.9, "test"});
    }
    return points;
}

std::string read_text_file(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open template: " + path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void require_tokens(const std::string& text,
                    const std::vector<std::string>& tokens,
                    const std::string& template_name) {
    for (const auto& token : tokens) {
        require(text.find(token) != std::string::npos,
                template_name + " is missing contract token: " + token);
    }
}

void test_controller_templates_support_mixed_motion() {
#ifdef TEST_PGM_PATH
    require_tokens(read_text_file(TEST_PGM_PATH),
                   {"PC_INT[10", "PC_POINTJ[", "MJOINT", "PC_POINTC[", "MLIN",
                    "PC_BOOL[5]", "PC_INT[3]", "v100perc", "v100", "v800",
                    "9001", "9002"},
                   "PGM template");
#endif
#ifdef TEST_XPL_PATH
    require_tokens(read_text_file(TEST_XPL_PATH),
                   {"PC_INT[10", "PC_POINTJ[", "<mjoint>", "PC_POINTC[", "<mlin>",
                    "PC_BOOL[5]", "PC_INT[3]", "v100perc", "v100", "v800",
                    "9001", "9002"},
                   "XPL template");
#endif
}

void test_five_stage_plan() {
    tie::RobotControlConfig config;
    config.tie_dwell_s = 0.0;
    tie::MotionPlanBuilder builder(config);
    const auto plan = builder.build(make_points(2));
    require(plan.size() == 10, "two tie points must create ten motion segments");
    require(plan[0].stage == tie::MotionStage::SafeEntry, "first stage is safe entry");
    require(plan[0].motion_type == tie::MotionType::MJoint,
            "safe entry uses MJOINT");
    require(plan[1].stage == tie::MotionStage::Approach, "second stage is approach");
    require(plan[2].stage == tie::MotionStage::Tie, "third stage is tie");
    require(plan[4].stage == tie::MotionStage::SafeExit, "fifth stage is safe exit");
    require(near(plan[0].target_pose.z, 400.0), "safe entry clearance is applied");
    require(near(plan[4].target_pose.z, 400.0), "safe exit clearance is applied");
}

void test_batch_boundaries() {
    tie::RobotControlConfig config;
    tie::MotionPlanBuilder builder(config);
    require(builder.batches(builder.build(make_points(5))).size() == 1,
            "25 motion segments fit in one batch");
    const auto batches = builder.batches(builder.build(make_points(6)));
    require(batches.size() == 2, "30 motion segments require two batches");
    require(batches[0].size() == 25 && batches[1].size() == 5,
            "batch sizes must be 25 and 5");
}

void test_safety() {
    tie::RobotControlConfig config;
    tie::SafetyChecker checker(config);
    tie::CartesianPose normal;
    tie::CartesianPose far = normal;
    far.x = 9999.0;
    tie::CartesianPose non_finite = normal;
    non_finite.y = std::numeric_limits<double>::quiet_NaN();
    const auto issues = checker.validate_tie_points({
        {"LOW", normal, 0.1, "test"},
        {"FAR", far, 0.9, "test"},
        {"NAN", non_finite, 0.9, "test"},
    });
    bool confidence = false;
    bool outside = false;
    bool finite = false;
    for (const auto& issue : issues) {
        confidence = confidence || issue.reason.find("confidence") != std::string::npos;
        outside = outside || issue.reason.find("outside") != std::string::npos;
        finite = finite || issue.reason.find("finite") != std::string::npos;
    }
    require(confidence, "low confidence must be rejected");
    require(outside, "out-of-range pose must be rejected");
    require(finite, "non-finite pose must be rejected");
}

void test_dry_run_sender() {
    tie::RobotControlConfig config;
    config.tie_dwell_s = 0.0;
    auto backend = std::make_unique<tie::DryRunBackend>(config);
    auto* inspection = backend.get();
    tie::RobotControlSystem system(config, std::move(backend));
    const auto plan = system.run(make_points(10));
    require(plan.size() == 50, "ten tie points create fifty motion segments");
    require(inspection->last_vector().size() == 25,
            "last dry-run batch has twenty-five segments");
    require(inspection->last_start_index() == 25, "second batch targets buffer B");
}

void test_dry_run_logs_motion_details() {
    tie::RobotControlConfig config;
    tie::MotionPlanBuilder builder(config);
    const auto plan = builder.build(make_points(1));
    tie::DryRunBackend backend(config);
    backend.connect();
    std::ostringstream captured;
    auto* previous = std::cout.rdbuf(captured.rdbuf());
    backend.set_motion_batch(plan, 0, config.rpl.motion_type_int_start);
    std::cout.rdbuf(previous);
    backend.disconnect();
    const auto log = captured.str();
    require(log.find("stage=safe_entry") != std::string::npos,
            "dry-run log must include the motion stage");
    require(log.find("motion_type=MJOINT") != std::string::npos,
            "dry-run log must include the motion type");
    require(log.find("xyz=(") != std::string::npos && log.find("abc=(") != std::string::npos,
            "dry-run log must include XYZ and ABC");
    require(log.find("velocity_profile=100") != std::string::npos &&
                log.find("zone=-1") != std::string::npos,
            "dry-run log must include velocity profile and zone");
}

void test_double_buffer_wraps_to_a() {
    tie::RobotControlConfig config;
    config.tie_dwell_s = 0.0;
    auto backend = std::make_unique<tie::DryRunBackend>(config);
    auto* inspection = backend.get();
    tie::RobotControlSystem system(config, std::move(backend));
    system.run(make_points(11));
    require(inspection->last_vector().size() == 5, "third batch has five segments");
    require(inspection->last_start_index() == 0, "third batch wraps to buffer A");
    require(config.rpl.batch_done_bool == 0, "completion flag matches XPL PC_BOOL[0]");
}

void test_real_handshake_buffer_sequence() {
    tie::RobotControlConfig config;
    config.dry_run = false;
    config.tie_dwell_s = 0.0;
    tie::MotionPlanBuilder builder(config);
    const auto plan = builder.build(make_points(11));
    HandshakeBackend backend;
    tie::RplBatchSender(config, backend).send_queue(plan);
    require(backend.target_starts == std::vector<std::size_t>({0, 25, 0}),
            "real handshake must alternate A/B buffer offsets");
    require(backend.type_starts == std::vector<unsigned>({10, 35, 10}),
            "motion type buffers must use matching A/B offsets");
    require(backend.type_batches.front().at(0) ==
                static_cast<int>(tie::MotionType::MJoint),
            "safe entry type code must be MJOINT");
    require(backend.type_batches.front().at(1) ==
                static_cast<int>(tie::MotionType::MLinear),
            "approach type code must be MLIN");
    require(backend.ints.at(config.rpl.transfer_velocity_profile_int) == 100,
            "transfer velocity profile must be initialized");
    require(backend.ints.at(config.rpl.local_velocity_profile_int) == 100,
            "local velocity profile must be initialized");
    require(backend.ints.at(config.rpl.error_code_int) == 0,
            "controller error code must be cleared before start");
}

void test_controller_error_is_reported() {
    tie::RobotControlConfig config;
    config.dry_run = false;
    config.handshake_poll_s = 0.0;
    config.execution_timeout_s = 1.0;
    tie::MotionPlanBuilder builder(config);
    const auto plan = builder.build(make_points(1));
    HandshakeBackend backend;
    backend.force_controller_error = true;
    backend.forced_error_bool_index = config.rpl.controller_error_bool;
    backend.forced_error_code_index = config.rpl.error_code_int;
    try {
        tie::RplBatchSender(config, backend).send_queue(plan);
    } catch (const std::runtime_error& error) {
        require(std::string(error.what()).find("9001") != std::string::npos,
                "controller error message must contain its code");
        return;
    }
    throw std::runtime_error("controller error flag must fail queue execution");
}

void test_preflight_rejects_before_any_batch() {
    tie::RobotControlConfig config;
    config.tie_dwell_s = 0.0;
    auto backend = std::make_unique<HandshakeBackend>();
    auto* inspection = backend.get();
    inspection->reject_preflight = true;
    tie::RobotControlSystem system(config, std::move(backend));
    try {
        system.run(make_points(1));
    } catch (const std::runtime_error& error) {
        require(std::string(error.what()).find("preflight") != std::string::npos,
                "preflight rejection must be reported");
        require(inspection->target_starts.empty(),
                "preflight failure must occur before any batch write");
        return;
    }
    throw std::runtime_error("invalid target must reject the complete task");
}

void test_preflight_resolves_mjoint_targets() {
    tie::RobotControlConfig config;
    config.tie_dwell_s = 0.0;
    auto backend = std::make_unique<HandshakeBackend>();
    auto* inspection = backend.get();
    tie::RobotControlSystem system(config, std::move(backend));
    system.run(make_points(1));
    require(inspection->prepare_target_calls == 1,
            "target preflight must run exactly once");
    require(inspection->all_joint_targets_resolved,
            "every MJOINT batch target must contain a joint solution");
}

void test_json_loading() {
#ifdef TEST_DATA_PATH
    const auto points = tie::load_tie_points_json(TEST_DATA_PATH);
    require(points.size() == 10, "sample JSON contains ten points");
    require(points.front().point_id == "P001", "first point id is parsed");
#endif
#ifdef TEST_CONFIG_PATH
    const auto config = tie::load_config_json(TEST_CONFIG_PATH);
    require(near(config.surface.normal.z, 1.0), "surface normal is loaded");
    require(near(config.motion.transfer_clearance_mm, 150.0), "clearance is loaded");
    require(config.motion.transfer_velocity_profile == 100,
            "transfer profile is loaded");
    require(config.motion.local_velocity_profile == 100,
            "local profile is loaded");
    require(config.rpl.motion_type_int_start == 10, "motion type base is loaded");
    require(config.rpl.controller_error_bool == 5, "controller error flag is loaded");
#endif
}

}  // namespace

int main() {
    try {
        test_five_stage_plan();
        test_batch_boundaries();
        test_safety();
        test_dry_run_sender();
        test_dry_run_logs_motion_details();
        test_double_buffer_wraps_to_a();
        test_real_handshake_buffer_sequence();
        test_controller_error_is_reported();
        test_preflight_rejects_before_any_batch();
        test_preflight_resolves_mjoint_targets();
        test_json_loading();
        test_controller_templates_support_mixed_motion();
        std::cout << "All C++ robot control tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
