#include "motion_planning.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

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

}  // namespace

int main() {
    try {
        test_normalization_and_cross_product();
        test_horizontal_surface_pose();
        test_invalid_surface_frame();
        test_surface_tilt_and_roll();
        std::cout << "All motion planning tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
