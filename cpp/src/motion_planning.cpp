#include "motion_planning.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tie {
namespace {
constexpr double kEpsilon = 1e-9;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

Matrix3 columns(const Vector3& x, const Vector3& y, const Vector3& z) {
    Matrix3 result;
    result.value[0] = {x.x, y.x, z.x};
    result.value[1] = {x.y, y.y, z.y};
    result.value[2] = {x.z, y.z, z.z};
    return result;
}

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) {
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int index = 0; index < 3; ++index) {
                result.value[row][column] +=
                    lhs.value[row][index] * rhs.value[index][column];
            }
        }
    }
    return result;
}

Matrix3 rotation_y(double radians) {
    Matrix3 result{};
    result.value[0] = {std::cos(radians), 0.0, std::sin(radians)};
    result.value[1] = {0.0, 1.0, 0.0};
    result.value[2] = {-std::sin(radians), 0.0, std::cos(radians)};
    return result;
}

Matrix3 rotation_z(double radians) {
    Matrix3 result{};
    result.value[0] = {std::cos(radians), -std::sin(radians), 0.0};
    result.value[1] = {std::sin(radians), std::cos(radians), 0.0};
    result.value[2] = {0.0, 0.0, 1.0};
    return result;
}

std::array<double, 3> to_zyx_degrees(const Matrix3& matrix) {
    const double b = std::asin(std::clamp(-matrix.value[2][0], -1.0, 1.0));
    const double cos_b = std::cos(b);
    double a{};
    double c{};
    if (std::abs(cos_b) > kEpsilon) {
        a = std::atan2(matrix.value[1][0], matrix.value[0][0]);
        c = std::atan2(matrix.value[2][1], matrix.value[2][2]);
    } else {
        a = std::atan2(-matrix.value[0][1], matrix.value[1][1]);
    }
    return {a * kRadiansToDegrees, b * kRadiansToDegrees, c * kRadiansToDegrees};
}
}  // namespace

double dot(const Vector3& lhs, const Vector3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vector3 cross(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

double norm(const Vector3& value) { return std::sqrt(dot(value, value)); }

Vector3 normalized(const Vector3& value) {
    const double length = norm(value);
    if (!std::isfinite(length) || length <= kEpsilon) {
        throw std::invalid_argument("surface direction must be a finite non-zero vector");
    }
    return {value.x / length, value.y / length, value.z / length};
}

Vector3 operator+(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vector3 operator-(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vector3 operator*(double scale, const Vector3& value) {
    return {scale * value.x, scale * value.y, scale * value.z};
}

SurfacePoseCorrector::SurfacePoseCorrector(SurfaceFrameConfig config)
    : config_(config), normal_(normalized(config.normal)) {
    if (config_.tool_axis_sign != -1 && config_.tool_axis_sign != 1) {
        throw std::invalid_argument("tool_axis_sign must be -1 or 1");
    }
    Vector3 x = config_.x_direction - dot(config_.x_direction, normal_) * normal_;
    x = normalized(x);
    const Vector3 z = static_cast<double>(config_.tool_axis_sign) * normal_;
    Vector3 y = normalized(cross(z, x));
    x = normalized(cross(y, z));
    const Matrix3 base_rotation = columns(x, y, z);
    tool_rotation_ = multiply(
        base_rotation,
        multiply(rotation_z(config_.tool_roll_deg * kDegreesToRadians),
                 rotation_y(config_.tool_tilt_deg * kDegreesToRadians)));
}

CartesianPose SurfacePoseCorrector::corrected_pose(const CartesianPose& input) const {
    CartesianPose output = input;
    const auto abc = to_zyx_degrees(tool_rotation_);
    output.a = abc[0];
    output.b = abc[1];
    output.c = abc[2];
    return output;
}

const Vector3& SurfacePoseCorrector::normal() const { return normal_; }

}  // namespace tie
