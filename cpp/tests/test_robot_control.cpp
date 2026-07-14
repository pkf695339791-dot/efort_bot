#include "robot_control.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

class HandshakeBackend final : public tie::RobotBackend {
public:
    void connect() override { connected = true; }
    void disconnect() noexcept override { connected = false; }
    void prepare() override {}
    void set_int(unsigned, int) override {}
    int get_int(unsigned) override { return 0; }
    void set_bool(unsigned index, bool value) override { bools[index] = value; }
    bool get_bool(unsigned index) override {
        if (index == 0 || index == 1 || index == 2) return true;
        const auto item = bools.find(index);
        return item != bools.end() && item->second;
    }
    void set_pointc_vector(
        const std::vector<tie::MotionSegment>&, std::size_t start_index) override {
        starts.push_back(start_index);
    }
    tie::RobotStatus read_status() override {
        return {connected, false, false, true, false};
    }

    bool connected{true};
    std::unordered_map<unsigned, bool> bools;
    std::vector<std::size_t> starts;
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
    const auto issues = checker.validate_tie_points({
        {"LOW", normal, 0.1, "test"},
        {"FAR", far, 0.9, "test"},
    });
    bool confidence = false;
    bool outside = false;
    for (const auto& issue : issues) {
        confidence = confidence || issue.reason.find("confidence") != std::string::npos;
        outside = outside || issue.reason.find("outside") != std::string::npos;
    }
    require(confidence, "low confidence must be rejected");
    require(outside, "out-of-range pose must be rejected");
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
    const auto plan = builder.build(make_points(16));
    HandshakeBackend backend;
    tie::RplBatchSender(config, backend).send_queue(plan);
    require(backend.starts == std::vector<std::size_t>({0, 25, 0, 25}),
            "real handshake must alternate A/B buffer offsets");
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
        test_double_buffer_wraps_to_a();
        test_real_handshake_buffer_sequence();
        test_json_loading();
        std::cout << "All C++ robot control tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
