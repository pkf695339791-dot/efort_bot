#include "motion_planning.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

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

namespace {

CartesianPose shifted_along(
    const CartesianPose& pose, const Vector3& normal, double distance_mm) {
    CartesianPose shifted = pose;
    shifted.x += normal.x * distance_mm;
    shifted.y += normal.y * distance_mm;
    shifted.z += normal.z * distance_mm;
    return shifted;
}

bool finite_non_negative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

}  // namespace

SafeTransferPlanner::SafeTransferPlanner(
    SurfaceFrameConfig surface, MotionPlanningConfig motion)
    : pose_corrector_(surface), motion_(motion) {
    if (motion_.transfer_velocity_profile != 100) {
        throw std::invalid_argument("transfer_velocity_profile must be 100");
    }
    if (motion_.local_velocity_profile != 100 &&
        motion_.local_velocity_profile != 800) {
        throw std::invalid_argument("local_velocity_profile must be 100 or 800");
    }
    if (!finite_non_negative(motion_.approach_distance_mm) ||
        !finite_non_negative(motion_.retreat_distance_mm) ||
        !finite_non_negative(motion_.transfer_clearance_mm)) {
        throw std::invalid_argument("motion distances must be finite and non-negative");
    }
    if (motion_.zone != -1.0) {
        throw std::invalid_argument("zone must be -1.0 (fine)");
    }
    if (motion_.transfer_clearance_mm < motion_.approach_distance_mm ||
        motion_.transfer_clearance_mm < motion_.retreat_distance_mm) {
        throw std::invalid_argument(
            "transfer clearance must cover approach and retreat distances");
    }
}

std::vector<MotionSegment> SafeTransferPlanner::plan(
    const std::vector<TiePoint>& points) const {
    std::vector<TiePoint> ordered = points;
    std::sort(ordered.begin(), ordered.end(), [](const TiePoint& lhs, const TiePoint& rhs) {
        if (lhs.pose.x != rhs.pose.x) return lhs.pose.x < rhs.pose.x;
        if (lhs.pose.y != rhs.pose.y) return lhs.pose.y < rhs.pose.y;
        if (lhs.pose.z != rhs.pose.z) return lhs.pose.z < rhs.pose.z;
        return lhs.point_id < rhs.point_id;
    });

    std::vector<MotionSegment> result;
    result.reserve(ordered.size() * 5);
    for (const TiePoint& point : ordered) {
        const CartesianPose tie_pose = pose_corrector_.corrected_pose(point.pose);
        const std::array<std::tuple<MotionStage, MotionType, double, int>, 5> stages{{
            {MotionStage::SafeEntry, MotionType::MJoint,
             motion_.transfer_clearance_mm, motion_.transfer_velocity_profile},
            {MotionStage::Approach, MotionType::MLinear,
             motion_.approach_distance_mm, motion_.local_velocity_profile},
            {MotionStage::Tie, MotionType::MLinear, 0.0,
             motion_.local_velocity_profile},
            {MotionStage::Retreat, MotionType::MLinear,
             motion_.retreat_distance_mm, motion_.local_velocity_profile},
            {MotionStage::SafeExit, MotionType::MLinear,
             motion_.transfer_clearance_mm, motion_.local_velocity_profile},
        }};
        for (const auto& stage : stages) {
            MotionSegment segment;
            segment.sequence_index = result.size();
            segment.tie_point_id = point.point_id;
            segment.stage = std::get<0>(stage);
            segment.motion_type = std::get<1>(stage);
            segment.target_pose = shifted_along(
                tie_pose, pose_corrector_.normal(), std::get<2>(stage));
            segment.velocity_profile_code = std::get<3>(stage);
            segment.zone = motion_.zone;
            segment.confidence = point.confidence;
            result.push_back(std::move(segment));
        }
    }
    return result;
}

std::string motion_stage_name(MotionStage stage) {
    switch (stage) {
        case MotionStage::SafeEntry: return "safe_entry";
        case MotionStage::Approach: return "approach";
        case MotionStage::Tie: return "tie";
        case MotionStage::Retreat: return "retreat";
        case MotionStage::SafeExit: return "safe_exit";
    }
    return "unknown";
}

std::string motion_type_name(MotionType type) {
    switch (type) {
        case MotionType::MLinear: return "MLIN";
        case MotionType::MJoint: return "MJOINT";
    }
    return "UNKNOWN";
}

}  // namespace tie
