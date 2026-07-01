#include "robot_control.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::vector<tie::TiePoint> make_points(int count) {
    std::vector<tie::TiePoint> points;
    for (int index = 0; index < count; ++index) {
        tie::CartesianPose pose;
        pose.x = 100.0 + index * 10.0;
        pose.y = -50.0;
        pose.z = 250.0;
        pose.cfgx = 1;
        points.push_back(
            {"P" + std::to_string(index), pose, 0.9, "test"});
    }
    return points;
}

void test_three_segments() {
    tie::RobotControlConfig config;
    config.tie_dwell_s = 0.0;
    tie::TiePointQueue builder(config);
    const auto queue = builder.build(make_points(2));
    require(queue.size() == 6, "two tie points must create six queue points");
    require(queue[0].kind == tie::QueuePointKind::Approach, "first point is approach");
    require(queue[1].kind == tie::QueuePointKind::Tie, "second point is tie");
    require(queue[2].kind == tie::QueuePointKind::Retreat, "third point is retreat");
    require(std::abs(queue[0].pose.z - 300.0) < 1e-9, "approach offset is applied");
    require(std::abs(queue[1].pose.z - 250.0) < 1e-9, "tie pose is unchanged");
}

void test_batch_boundaries() {
    tie::RobotControlConfig config;
    tie::TiePointQueue builder(config);
    require(builder.batches(builder.build(make_points(8))).size() == 1,
            "24 queue points fit in one batch");
    const auto batches = builder.batches(builder.build(make_points(9)));
    require(batches.size() == 2, "27 queue points require two batches");
    require(batches[0].size() == 25 && batches[1].size() == 2,
            "batch sizes must be 25 and 2");
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
    const auto queue = system.run(make_points(10));
    require(queue.size() == 30, "ten tie points create thirty queue points");
    require(inspection->last_vector().size() == 5, "last dry-run batch has five points");
}

void test_json_loading() {
#ifdef TEST_DATA_PATH
    const auto points = tie::load_tie_points_json(TEST_DATA_PATH);
    require(points.size() == 10, "sample JSON contains ten points");
    require(points.front().point_id == "P001", "first point id is parsed");
#endif
}

}  // namespace

int main() {
    try {
        test_three_segments();
        test_batch_boundaries();
        test_safety();
        test_dry_run_sender();
        test_json_loading();
        std::cout << "All C++ robot control tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}

