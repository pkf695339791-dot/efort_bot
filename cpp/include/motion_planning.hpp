#pragma once

#include "robot_types.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

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

enum class MotionType { MLinear = 0, MJoint = 1 };
enum class MotionStage { SafeEntry, Approach, Tie, Retreat, SafeExit };

struct MotionPlanningConfig {
    double approach_distance_mm{50.0};
    double retreat_distance_mm{50.0};
    double transfer_clearance_mm{150.0};
    int transfer_velocity_profile{100};
    int local_velocity_profile{100};
    double zone{-1.0};
};

struct MotionSegment {
    std::size_t sequence_index{};
    std::string tie_point_id;
    MotionStage stage{MotionStage::SafeEntry};
    MotionType motion_type{MotionType::MJoint};
    CartesianPose target_pose;
    std::optional<JointPose> joint_target;
    int velocity_profile_code{100};
    double zone{-1.0};
    double confidence{};
};

class SafeTransferPlanner {
public:
    SafeTransferPlanner(SurfaceFrameConfig surface, MotionPlanningConfig motion);
    std::vector<MotionSegment> plan(const std::vector<TiePoint>& points) const;

private:
    SurfacePoseCorrector pose_corrector_;
    MotionPlanningConfig motion_;
};

std::string motion_stage_name(MotionStage stage);
std::string motion_type_name(MotionType type);

}  // namespace tie
