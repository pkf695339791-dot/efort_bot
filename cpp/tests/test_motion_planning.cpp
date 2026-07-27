#include "motion_planning.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

template <typename Function>
void require_throws(Function function, const char* message) {
    try {
        function();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
}

void test_normalization_and_cross_product() {
    const tie::Vector3 z = tie::normalized({0.0, 0.0, 4.0});
    require(near(z.x, 0.0) && near(z.y, 0.0) && near(z.z, 1.0),
            "normalization must produce unit z");
    const tie::Vector3 y = tie::cross(z, {1.0, 0.0, 0.0});
    require(near(y.x, 0.0) && near(y.y, 1.0) && near(y.z, 0.0),
            "cross product must use right-hand rule");
}

void test_horizontal_surface_pose() {
    tie::SurfaceFrameConfig config;
    config.normal = {0.0, 0.0, 1.0};
    config.x_direction = {1.0, 0.0, 0.0};
    config.tool_axis_sign = 1;
    tie::SurfacePoseCorrector corrector(config);
    tie::CartesianPose input;
    input.x = 100.0;
    input.y = 200.0;
    input.z = 300.0;
    const auto result = corrector.corrected_pose(input);
    require(near(result.x, 100.0) && near(result.y, 200.0) && near(result.z, 300.0),
            "pose correction must preserve position");
    require(near(result.a, 0.0) && near(result.b, 0.0) && near(result.c, 0.0),
            "identity surface frame must produce zero ZYX angles");
}

void test_invalid_surface_frame() {
    tie::SurfaceFrameConfig zero;
    zero.normal = {0.0, 0.0, 0.0};
    require_throws([&] { tie::SurfacePoseCorrector value(zero); },
                   "zero normal must be rejected");

    tie::SurfaceFrameConfig parallel;
    parallel.normal = {0.0, 0.0, 1.0};
    parallel.x_direction = {0.0, 0.0, 2.0};
    require_throws([&] { tie::SurfacePoseCorrector value(parallel); },
                   "parallel directions must be rejected");

    tie::SurfaceFrameConfig bad_sign;
    bad_sign.tool_axis_sign = 0;
    require_throws([&] { tie::SurfacePoseCorrector value(bad_sign); },
                   "invalid tool axis sign must be rejected");
}

void test_surface_tilt_and_roll() {
    tie::SurfaceFrameConfig config;
    config.tool_axis_sign = 1;
    config.tool_tilt_deg = 15.0;
    config.tool_roll_deg = 20.0;
    tie::SurfacePoseCorrector corrector(config);
    tie::CartesianPose input;
    input.cfgx = 4;
    const auto result = corrector.corrected_pose(input);
    require(near(result.b, 15.0, 1e-8), "tilt must be represented in B");
    require(near(result.a, 20.0, 1e-8), "roll must be represented in A");
    require(result.cfgx == 4, "pose correction must preserve CFG");
}

tie::TiePoint make_tie_point(const std::string& id, double x) {
    tie::TiePoint point;
    point.point_id = id;
    point.pose.x = x;
    point.pose.y = 20.0;
    point.pose.z = 30.0;
    point.pose.cfgx = 2;
    point.confidence = 0.9;
    return point;
}

void test_five_stage_plan_on_tilted_surface() {
    tie::SurfaceFrameConfig surface;
    surface.normal = {1.0, 0.0, 1.0};
    surface.x_direction = {0.0, 1.0, 0.0};

    tie::MotionPlanningConfig motion;
    motion.approach_distance_mm = 10.0;
    motion.retreat_distance_mm = 20.0;
    motion.transfer_clearance_mm = 50.0;
    tie::SafeTransferPlanner planner(surface, motion);
    const auto segments = planner.plan({make_tie_point("P1", 10.0)});

    const std::vector<tie::MotionStage> expected_stages{
        tie::MotionStage::SafeEntry,
        tie::MotionStage::Approach,
        tie::MotionStage::Tie,
        tie::MotionStage::Retreat,
        tie::MotionStage::SafeExit,
    };
    const std::vector<tie::MotionType> expected_types{
        tie::MotionType::MJoint,
        tie::MotionType::MLinear,
        tie::MotionType::MLinear,
        tie::MotionType::MLinear,
        tie::MotionType::MLinear,
    };
    require(segments.size() == 5, "one tie point must produce five segments");
    for (std::size_t index = 0; index < segments.size(); ++index) {
        require(segments[index].stage == expected_stages[index],
                "five-stage order must be stable");
        require(segments[index].motion_type == expected_types[index],
                "motion-type order must be MJOINT then MLIN");
        require(segments[index].sequence_index == index,
                "sequence indices must start at zero");
    }
    const double diagonal = 50.0 / std::sqrt(2.0);
    require(near(segments[0].target_pose.x, 10.0 + diagonal),
            "safe entry must shift X along tilted normal");
    require(near(segments[0].target_pose.z, 30.0 + diagonal),
            "safe entry must shift Z along tilted normal");
    require(near(segments[2].target_pose.x, 10.0) &&
                near(segments[2].target_pose.z, 30.0),
            "tie position must remain unchanged");
}

void test_multiple_points_have_continuous_sequence_indices() {
    tie::SafeTransferPlanner planner({}, {});
    const auto segments = planner.plan(
        {make_tie_point("P1", 10.0), make_tie_point("P2", 100.0)});
    require(segments.size() == 10, "two tie points must produce ten segments");
    for (std::size_t index = 0; index < segments.size(); ++index) {
        require(segments[index].sequence_index == index,
                "sequence indices must remain continuous");
    }
}

void test_invalid_motion_configuration() {
    tie::MotionPlanningConfig bad_transfer;
    bad_transfer.transfer_velocity_profile = 800;
    require_throws(
        [&] { tie::SafeTransferPlanner planner({}, bad_transfer); },
        "unsupported transfer profile must be rejected");

    tie::MotionPlanningConfig bad_local;
    bad_local.local_velocity_profile = 123;
    require_throws(
        [&] { tie::SafeTransferPlanner planner({}, bad_local); },
        "unsupported local profile must be rejected");

    tie::MotionPlanningConfig negative_distance;
    negative_distance.approach_distance_mm = -1.0;
    require_throws(
        [&] { tie::SafeTransferPlanner planner({}, negative_distance); },
        "negative approach distance must be rejected");

    tie::MotionPlanningConfig blended_zone;
    blended_zone.zone = 1.0;
    require_throws(
        [&] { tie::SafeTransferPlanner planner({}, blended_zone); },
        "non-fine zone must be rejected");

    tie::MotionPlanningConfig low_clearance;
    low_clearance.approach_distance_mm = 200.0;
    require_throws(
        [&] { tie::SafeTransferPlanner planner({}, low_clearance); },
        "clearance below approach distance must be rejected");
}

}  // namespace

int main() {
    try {
        test_normalization_and_cross_product();
        test_horizontal_surface_pose();
        test_invalid_surface_frame();
        test_surface_tilt_and_roll();
        test_five_stage_plan_on_tilted_surface();
        test_multiple_points_have_continuous_sequence_indices();
        test_invalid_motion_configuration();
        std::cout << "All motion planning tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
