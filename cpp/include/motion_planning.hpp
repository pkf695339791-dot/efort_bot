#pragma once

#include "robot_types.hpp"

#include <array>

namespace tie {

struct Vector3 {
    double x{};
    double y{};
    double z{};
};

struct Matrix3 {
    std::array<std::array<double, 3>, 3> value{};
};

double dot(const Vector3& lhs, const Vector3& rhs);
Vector3 cross(const Vector3& lhs, const Vector3& rhs);
double norm(const Vector3& value);
Vector3 normalized(const Vector3& value);
Vector3 operator+(const Vector3& lhs, const Vector3& rhs);
Vector3 operator-(const Vector3& lhs, const Vector3& rhs);
Vector3 operator*(double scale, const Vector3& value);

enum class AbcConvention { ZyxIntrinsic };

struct SurfaceFrameConfig {
    Vector3 normal{0.0, 0.0, 1.0};
    Vector3 x_direction{1.0, 0.0, 0.0};
    int tool_axis_sign{1};
    double tool_tilt_deg{};
    double tool_roll_deg{};
    AbcConvention abc_convention{AbcConvention::ZyxIntrinsic};
};

class SurfacePoseCorrector {
public:
    explicit SurfacePoseCorrector(SurfaceFrameConfig config);
    CartesianPose corrected_pose(const CartesianPose& input) const;
    const Vector3& normal() const;

private:
    SurfaceFrameConfig config_;
    Vector3 normal_;
    Matrix3 tool_rotation_;
};

}  // namespace tie
