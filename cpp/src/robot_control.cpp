#include "robot_control.hpp"
#include "json_value.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

#ifdef EFORT_WITH_SDK
#include "EfortSdk.h"
#endif

namespace tie {
namespace {

void sleep_seconds(double seconds) {
    if (seconds > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    }
}

std::string issue_message(const SafetyIssue& issue) {
    return issue.point_id + ": " + issue.reason;
}

double required_number(const json::Value& value, const std::string& key) {
    const json::Value* item = value.find(key);
    if (!item) throw std::runtime_error("required JSON field is missing: " + key);
    return item->number_or(0.0);
}

AxisLimit read_limit(const json::Value& parent, const std::string& key, AxisLimit fallback) {
    const json::Value* value = parent.find(key);
    if (!value || !value->is_array() || value->array().size() != 2) return fallback;
    return {value->array()[0].number_or(fallback.lower),
            value->array()[1].number_or(fallback.upper)};
}

Vector3 read_vector3(
    const json::Value& parent, const std::string& key, const Vector3& fallback) {
    const json::Value* value = parent.find(key);
    if (!value) return fallback;
    if (!value->is_array() || value->array().size() != 3) {
        throw std::runtime_error(key + " must contain exactly three numbers");
    }
    const double missing = std::numeric_limits<double>::quiet_NaN();
    Vector3 result{
        value->array()[0].number_or(missing),
        value->array()[1].number_or(missing),
        value->array()[2].number_or(missing),
    };
    if (!std::isfinite(result.x) || !std::isfinite(result.y) ||
        !std::isfinite(result.z)) {
        throw std::runtime_error(key + " must contain finite numbers");
    }
    return result;
}

}  // namespace

CartesianPose CartesianPose::shifted(double dz) const {
    CartesianPose result = *this;
    result.z += dz;
    return result;
}

CartesianPose CoordinateManager::to_workobject_pose(const TiePoint& point) const {
    // Identity until the site-specific hand-eye calibration matrix is supplied.
    return point.pose;
}

SafetyChecker::SafetyChecker(const RobotControlConfig& config) : config_(config) {}

double SafetyChecker::distance(const CartesianPose& first, const CartesianPose& second) {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    const double dz = first.z - second.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<SafetyIssue> SafetyChecker::check_pose_ranges(
    const std::string& point_id, const CartesianPose& pose) const {
    std::vector<SafetyIssue> issues;
    const std::array<std::pair<const char*, std::pair<double, AxisLimit>>, 6> checks{{
        {"x", {pose.x, config_.workspace.x}},
        {"y", {pose.y, config_.workspace.y}},
        {"z", {pose.z, config_.workspace.z}},
        {"a", {pose.a, config_.workspace.a}},
        {"b", {pose.b, config_.workspace.b}},
        {"c", {pose.c, config_.workspace.c}},
    }};
    for (const auto& check : checks) {
        const double value = check.second.first;
        const AxisLimit limit = check.second.second;
        if (value < limit.lower || value > limit.upper) {
            std::ostringstream reason;
            reason << check.first << '=' << value << " outside ["
                   << limit.lower << ", " << limit.upper << ']';
            issues.push_back({point_id, reason.str()});
        }
    }
    return issues;
}

std::vector<SafetyIssue> SafetyChecker::validate_tie_points(
    const std::vector<TiePoint>& points) const {
    std::vector<SafetyIssue> issues;
    std::vector<const TiePoint*> seen;
    for (const TiePoint& point : points) {
        if (point.point_id.empty()) issues.push_back({"<missing>", "point_id is required"});
        if (point.confidence < config_.min_confidence) {
            std::ostringstream reason;
            reason << "confidence " << point.confidence << " below " << config_.min_confidence;
            issues.push_back({point.point_id, reason.str()});
        }
        auto range_issues = check_pose_ranges(point.point_id, point.pose);
        issues.insert(issues.end(), range_issues.begin(), range_issues.end());
        for (const TiePoint* previous : seen) {
            if (distance(point.pose, previous->pose) < config_.duplicate_distance_mm) {
                issues.push_back(
                    {point.point_id, "duplicate or near-duplicate of " + previous->point_id});
                break;
            }
        }
        seen.push_back(&point);
    }
    return issues;
}

std::vector<SafetyIssue> SafetyChecker::validate_plan(
    const std::vector<MotionSegment>& segments) const {
    if (segments.empty()) return {{"<plan>", "motion plan is empty"}};
    std::vector<SafetyIssue> issues;
    const CartesianPose* last_tie_pose = nullptr;
    for (const MotionSegment& segment : segments) {
        auto range_issues = check_pose_ranges(segment.tie_point_id, segment.target_pose);
        issues.insert(issues.end(), range_issues.begin(), range_issues.end());
        if (segment.stage == MotionStage::Tie) {
            if (last_tie_pose) {
                const double spacing = distance(segment.target_pose, *last_tie_pose);
                if (spacing < config_.minimum_point_spacing_mm) {
                    std::ostringstream reason;
                    reason << "tie point spacing " << spacing << " mm below minimum";
                    issues.push_back({segment.tie_point_id, reason.str()});
                }
            }
            last_tie_pose = &segment.target_pose;
        }
    }
    return issues;
}

MotionPlanBuilder::MotionPlanBuilder(const RobotControlConfig& config)
    : config_(config), safety_checker_(config), planner_(config.surface, config.motion) {}

std::vector<MotionSegment> MotionPlanBuilder::build(const std::vector<TiePoint>& points) {
    skipped_.clear();
    const auto issues = safety_checker_.validate_tie_points(points);
    std::vector<std::string> blocked_ids;
    for (const SafetyIssue& issue : issues) {
        skipped_.push_back(issue_message(issue));
        if (issue.severity == "error") blocked_ids.push_back(issue.point_id);
    }
    std::vector<TiePoint> usable;
    for (const TiePoint& point : points) {
        if (std::find(blocked_ids.begin(), blocked_ids.end(), point.point_id) ==
            blocked_ids.end()) {
            usable.push_back(point);
        }
    }
    auto plan = planner_.plan(usable);
    const auto plan_issues = safety_checker_.validate_plan(plan);
    if (!plan_issues.empty()) {
        std::ostringstream message;
        message << "motion plan failed safety validation: ";
        for (std::size_t index = 0; index < plan_issues.size(); ++index) {
            if (index) message << "; ";
            message << issue_message(plan_issues[index]);
        }
        throw std::runtime_error(message.str());
    }
    return plan;
}

std::vector<std::vector<MotionSegment>> MotionPlanBuilder::batches(
    const std::vector<MotionSegment>& segments) const {
    if (config_.batch_size == 0) throw std::invalid_argument("batch_size must be positive");
    std::vector<std::vector<MotionSegment>> result;
    for (std::size_t start = 0; start < segments.size(); start += config_.batch_size) {
        const std::size_t end = std::min(segments.size(), start + config_.batch_size);
        result.emplace_back(segments.begin() + static_cast<std::ptrdiff_t>(start),
                            segments.begin() + static_cast<std::ptrdiff_t>(end));
    }
    return result;
}

const std::vector<std::string>& MotionPlanBuilder::skipped() const { return skipped_; }

DryRunBackend::DryRunBackend(const RobotControlConfig& config) : config_(config) {}
void DryRunBackend::connect() {
    connected_ = true;
    std::cout << "dry-run robot connected ip=" << config_.controller_ip << '\n';
}
void DryRunBackend::disconnect() noexcept {
    connected_ = false;
    std::cout << "dry-run robot disconnected\n";
}
void DryRunBackend::ensure_connected() const {
    if (!connected_) throw std::runtime_error("dry-run backend is not connected");
}
void DryRunBackend::prepare() {
    ensure_connected();
    std::cout << "dry-run prepare tool=" << config_.tool_name
              << " workobject=" << config_.workobject_name << '\n';
}
std::vector<MotionSegment> DryRunBackend::prepare_targets(
    const std::vector<MotionSegment>& segments) {
    ensure_connected();
    const auto issues = SafetyChecker(config_).validate_plan(segments);
    if (!issues.empty()) {
        throw std::runtime_error(
            "dry-run target validation failed: " + issue_message(issues.front()));
    }
    for (const MotionSegment& segment : segments) {
        if (segment.motion_type == MotionType::MJoint) {
            std::cout << "dry-run preflight sequence=" << segment.sequence_index
                      << " joint_target=unresolved(dry-run)\n";
        }
    }
    return segments;
}
void DryRunBackend::set_int(unsigned index, int value) {
    ensure_connected();
    ints_[index] = value;
}
int DryRunBackend::get_int(unsigned index) {
    ensure_connected();
    const auto item = ints_.find(index);
    return item == ints_.end() ? 0 : item->second;
}
void DryRunBackend::set_bool(unsigned index, bool value) {
    ensure_connected();
    bools_[index] = value;
}
bool DryRunBackend::get_bool(unsigned index) {
    ensure_connected();
    const auto item = bools_.find(index);
    return item != bools_.end() && item->second;
}
void DryRunBackend::set_motion_batch(
    const std::vector<MotionSegment>& segments,
    std::size_t target_start_index,
    unsigned) {
    ensure_connected();
    last_vector_ = segments;
    last_start_index_ = target_start_index;
}
RobotStatus DryRunBackend::read_status() {
    ensure_connected();
    return {connected_, false, false, true, false};
}
const std::vector<MotionSegment>& DryRunBackend::last_vector() const {
    return last_vector_;
}
std::size_t DryRunBackend::last_start_index() const { return last_start_index_; }

EfortSdkBackend::EfortSdkBackend(const RobotControlConfig& config) : config_(config) {}
void EfortSdkBackend::check(int result, const std::string& operation) {
    if (result != 0) {
        throw std::runtime_error(operation + " failed with code " + std::to_string(result));
    }
}

#ifdef EFORT_WITH_SDK
void EfortSdkBackend::connect() {
    check(RobotAPI::ConnectRobot(config_.controller_ip, device_id_, true, false, 2, false),
          "ConnectRobot");
}
void EfortSdkBackend::disconnect() noexcept {
    if (device_id_ != 0) RobotAPI::DisconnectRobot(device_id_);
}
void EfortSdkBackend::prepare() {
    check(RobotAPI::EnableApiControl(true, device_id_), "EnableApiControl");
    check(RobotAPI::SetCurrentToolByName(config_.tool_name, device_id_),
          "SetCurrentToolByName");
    check(RobotAPI::SetCurrentUframeByName(config_.workobject_name, device_id_),
          "SetCurrentUframeByName");
    const RobotStatus status = read_status();
    if (status.alarm) throw std::runtime_error("robot has active alarm");
    if (status.emergency_stop) throw std::runtime_error("robot emergency stop is active");
    if (config_.require_servo_on && !status.servo_on) {
        throw std::runtime_error("servo is not on");
    }
}
std::vector<MotionSegment> EfortSdkBackend::prepare_targets(
    const std::vector<MotionSegment>& segments) {
    RobotAPI::RobotJoint current_joints;
    RobotAPI::RobotPos current_pose;
    check(RobotAPI::GetJointPos(current_joints, device_id_), "GetJointPos");
    check(RobotAPI::GetBaseCoordinatePos2(current_pose, device_id_),
          "GetBaseCoordinatePos2");
    std::cout << "robot start tcp="
              << current_pose.x << ',' << current_pose.y << ',' << current_pose.z
              << ',' << current_pose.a << ',' << current_pose.b << ',' << current_pose.c
              << " joints=" << current_joints.j[0] << ',' << current_joints.j[1]
              << ',' << current_joints.j[2] << ',' << current_joints.j[3]
              << ',' << current_joints.j[4] << ',' << current_joints.j[5] << '\n';

    auto prepared = segments;
    for (MotionSegment& segment : prepared) {
        RobotAPI::PointC point_c;
        point_c.x = segment.target_pose.x;
        point_c.y = segment.target_pose.y;
        point_c.z = segment.target_pose.z;
        point_c.a = segment.target_pose.a;
        point_c.b = segment.target_pose.b;
        point_c.c = segment.target_pose.c;
        point_c.cfgx = static_cast<unsigned>(segment.target_pose.cfgx);
        point_c.cfg1 = segment.target_pose.cfg1;
        point_c.cfg4 = segment.target_pose.cfg4;
        point_c.cfg6 = segment.target_pose.cfg6;
        check(RobotAPI::CheckTarget(
                  point_c, config_.tool_name, config_.workobject_name, device_id_),
              "CheckTarget sequence " + std::to_string(segment.sequence_index));

        if (segment.motion_type != MotionType::MJoint) continue;
        RobotAPI::RobotPos robot_pos;
        robot_pos.x = segment.target_pose.x;
        robot_pos.y = segment.target_pose.y;
        robot_pos.z = segment.target_pose.z;
        robot_pos.a = segment.target_pose.a;
        robot_pos.b = segment.target_pose.b;
        robot_pos.c = segment.target_pose.c;
        robot_pos.cfgx = segment.target_pose.cfgx;
        robot_pos.cfg1 = segment.target_pose.cfg1;
        robot_pos.cfg4 = segment.target_pose.cfg4;
        robot_pos.cfg6 = segment.target_pose.cfg6;
        RobotAPI::RobotJoint robot_joint;
        check(RobotAPI::IkSolver(
                  robot_pos,
                  robot_joint,
                  config_.tool_name,
                  config_.workobject_name,
                  device_id_),
              "IkSolver sequence " + std::to_string(segment.sequence_index));
        JointPose joint_target;
        for (std::size_t axis = 0; axis < joint_target.joints.size(); ++axis) {
            joint_target.joints[axis] = robot_joint.j[axis];
        }
        segment.joint_target = joint_target;
    }
    return prepared;
}
void EfortSdkBackend::set_int(unsigned index, int value) {
    check(RobotAPI::SetIntVariable(index, value, device_id_), "SetIntVariable");
}
int EfortSdkBackend::get_int(unsigned index) {
    int value{};
    check(RobotAPI::GetIntVariable(index, value, device_id_), "GetIntVariable");
    return value;
}
void EfortSdkBackend::set_bool(unsigned index, bool value) {
    check(RobotAPI::SetBoolVariable(index, value, device_id_), "SetBoolVariable");
}
bool EfortSdkBackend::get_bool(unsigned index) {
    bool value{};
    check(RobotAPI::GetBoolVariable(index, value, device_id_), "GetBoolVariable");
    return value;
}
void EfortSdkBackend::set_motion_batch(
    const std::vector<MotionSegment>& segments,
    std::size_t target_start_index,
    unsigned motion_type_start_index) {
    std::vector<RobotAPI::PointC> point_c_values;
    std::vector<RobotAPI::PointJ> point_j_values;
    std::vector<int> motion_types;
    point_c_values.reserve(segments.size());
    point_j_values.reserve(segments.size());
    motion_types.reserve(segments.size());
    for (std::size_t index = 0; index < segments.size(); ++index) {
        const MotionSegment& item = segments[index];
        const int slot = static_cast<int>(target_start_index + index);
        RobotAPI::PointC point_c{};
        point_c.index = slot;
        point_c.x = item.target_pose.x;
        point_c.y = item.target_pose.y;
        point_c.z = item.target_pose.z;
        point_c.a = item.target_pose.a;
        point_c.b = item.target_pose.b;
        point_c.c = item.target_pose.c;
        point_c.cfgx = item.target_pose.cfgx;
        point_c.cfg1 = item.target_pose.cfg1;
        point_c.cfg4 = item.target_pose.cfg4;
        point_c.cfg6 = item.target_pose.cfg6;
        point_c_values.push_back(point_c);

        RobotAPI::PointJ point_j{};
        point_j.index = slot;
        if (item.motion_type == MotionType::MJoint) {
            if (!item.joint_target) {
                throw std::runtime_error(
                    "MJOINT segment is missing SDK-resolved joint target");
            }
            point_j.j1 = item.joint_target->joints[0];
            point_j.j2 = item.joint_target->joints[1];
            point_j.j3 = item.joint_target->joints[2];
            point_j.j4 = item.joint_target->joints[3];
            point_j.j5 = item.joint_target->joints[4];
            point_j.j6 = item.joint_target->joints[5];
        }
        point_j_values.push_back(point_j);
        motion_types.push_back(static_cast<int>(item.motion_type));
    }
    check(RobotAPI::SetPointCVector(point_c_values, device_id_, false),
          "SetPointCVector");
    check(RobotAPI::SetPointJVector(point_j_values, device_id_, false),
          "SetPointJVector");
    check(RobotAPI::SetIntVariable(
              motion_type_start_index,
              static_cast<unsigned>(motion_types.size()),
              motion_types.data(),
              device_id_),
          "SetIntVariable(motion types)");
}
RobotStatus EfortSdkBackend::read_status() {
    bool alarm{}, emergency{}, servo{}, moving{};
    check(RobotAPI::GetCurrentAlarmStatus(alarm, device_id_), "GetCurrentAlarmStatus");
    check(RobotAPI::GetCurrentEmgStatus(emergency, device_id_), "GetCurrentEmgStatus");
    check(RobotAPI::GetCurrentServoStatus(servo, device_id_), "GetCurrentServoStatus");
    check(RobotAPI::GetMoveState(moving, device_id_), "GetMoveState");
    return {true, alarm, emergency, servo, moving};
}
#else
void EfortSdkBackend::connect() {
    throw std::runtime_error(
        "real robot backend is unavailable; rebuild with -DEFORT_WITH_SDK=ON");
}
void EfortSdkBackend::disconnect() noexcept {}
void EfortSdkBackend::prepare() { connect(); }
std::vector<MotionSegment> EfortSdkBackend::prepare_targets(
    const std::vector<MotionSegment>&) {
    connect();
    return {};
}
void EfortSdkBackend::set_int(unsigned, int) { connect(); }
int EfortSdkBackend::get_int(unsigned) { connect(); return 0; }
void EfortSdkBackend::set_bool(unsigned, bool) { connect(); }
bool EfortSdkBackend::get_bool(unsigned) { connect(); return false; }
void EfortSdkBackend::set_motion_batch(
    const std::vector<MotionSegment>&, std::size_t, unsigned) { connect(); }
RobotStatus EfortSdkBackend::read_status() { connect(); return {}; }
#endif

std::unique_ptr<RobotBackend> create_backend(const RobotControlConfig& config) {
    if (config.dry_run) return std::make_unique<DryRunBackend>(config);
    return std::make_unique<EfortSdkBackend>(config);
}

RplBatchSender::RplBatchSender(
    const RobotControlConfig& config, RobotBackend& backend)
    : config_(config), backend_(backend) {}

void RplBatchSender::send_batch(
    const std::vector<MotionSegment>& batch,
    std::size_t batch_index,
    std::size_t buffer_start) {
    backend_.set_motion_batch(
        batch,
        buffer_start,
        config_.rpl.motion_type_int_start + static_cast<unsigned>(buffer_start));
    std::cout << "sent batch=" << batch_index << " size=" << batch.size()
              << " buffer_start=" << buffer_start
              << " sequence_range=" << batch.front().sequence_index << '-'
              << batch.back().sequence_index << '\n';
}

void RplBatchSender::wait_for_request(unsigned bool_index) {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(config_.handshake_timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        throw_if_controller_error();
        const RobotStatus status = backend_.read_status();
        if (status.alarm || status.emergency_stop) {
            throw std::runtime_error("robot unsafe while waiting for XPL buffer request");
        }
        if (backend_.get_bool(bool_index)) return;
        sleep_seconds(config_.handshake_poll_s);
    }
    throw std::runtime_error("timed out waiting for XPL buffer request");
}

void RplBatchSender::wait_for_completion() {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(config_.execution_timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        throw_if_controller_error();
        const RobotStatus status = backend_.read_status();
        if (status.alarm || status.emergency_stop) {
            throw std::runtime_error("robot unsafe while waiting for queue completion");
        }
        if (backend_.get_bool(config_.rpl.batch_done_bool)) return;
        sleep_seconds(config_.monitor_poll_s);
    }
    throw std::runtime_error("timed out waiting for XPL queue completion");
}

void RplBatchSender::throw_if_controller_error() {
    if (!backend_.get_bool(config_.rpl.controller_error_bool)) return;
    const int code = backend_.get_int(config_.rpl.error_code_int);
    throw std::runtime_error("controller error code " + std::to_string(code));
}

void RplBatchSender::send_queue(const std::vector<MotionSegment>& segments) {
    if (segments.empty()) throw std::invalid_argument("cannot send an empty motion plan");
    if (config_.batch_size != 25) {
        throw std::invalid_argument(
            "batch_size must be 25 because XPL buffers are fixed at 0-24 and 25-49");
    }
    MotionPlanBuilder plan_builder(config_);
    const auto batches = plan_builder.batches(segments);
    backend_.set_int(config_.rpl.total_points_int, static_cast<int>(segments.size()));
    backend_.set_int(
        config_.rpl.transfer_velocity_profile_int,
        config_.motion.transfer_velocity_profile);
    backend_.set_int(
        config_.rpl.local_velocity_profile_int,
        config_.motion.local_velocity_profile);
    backend_.set_int(config_.rpl.error_code_int, 0);
    backend_.set_bool(config_.rpl.controller_error_bool, false);
    backend_.set_bool(config_.rpl.batch_done_bool, false);
    backend_.set_bool(config_.rpl.stop_bool, false);
    backend_.set_bool(config_.rpl.request_buffer_a_bool, false);
    backend_.set_bool(config_.rpl.request_buffer_b_bool, false);

    send_batch(batches.front(), 0, 0);
    if (batches.size() > 1) {
        send_batch(batches[1], 1, config_.batch_size);
    }
    backend_.set_bool(config_.rpl.start_bool, true);
    for (std::size_t index = 2; index < batches.size(); ++index) {
        const bool target_a = index % 2 == 0;
        const unsigned request = target_a
            ? config_.rpl.request_buffer_a_bool
            : config_.rpl.request_buffer_b_bool;
        if (!config_.dry_run) {
            wait_for_request(request);
        }
        send_batch(batches[index], index, target_a ? 0 : config_.batch_size);
        if (!config_.dry_run) backend_.set_bool(request, false);
    }
    if (!config_.dry_run) wait_for_completion();
}
void RplBatchSender::request_stop() {
    backend_.set_bool(config_.rpl.stop_bool, true);
}

ExecutionMonitor::ExecutionMonitor(
    const RobotControlConfig& config, RobotBackend& backend)
    : config_(config), backend_(backend) {}
void ExecutionMonitor::assert_ready() {
    const RobotStatus status = backend_.read_status();
    if (status.alarm) throw std::runtime_error("robot has active alarm");
    if (status.emergency_stop) throw std::runtime_error("emergency stop is active");
    if (config_.require_servo_on && !status.servo_on) {
        throw std::runtime_error("servo is not on");
    }
}
void ExecutionMonitor::wait_until_idle(double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        const RobotStatus status = backend_.read_status();
        if (status.alarm || status.emergency_stop) {
            throw std::runtime_error("robot entered unsafe state during execution");
        }
        if (!status.moving) return;
        sleep_seconds(config_.monitor_poll_s);
    }
    throw std::runtime_error("robot did not become idle before timeout");
}

TieToolInterface::TieToolInterface(const RobotControlConfig& config) : config_(config) {}
void TieToolInterface::tie(const MotionSegment& segment) const {
    std::cout << "simulated tie action point=" << segment.tie_point_id
              << " sequence_index=" << segment.sequence_index << '\n';
    sleep_seconds(config_.tie_dwell_s);
}

RobotControlSystem::RobotControlSystem(
    RobotControlConfig config, std::unique_ptr<RobotBackend> backend)
    : config_(std::move(config)),
      backend_(backend ? std::move(backend) : create_backend(config_)),
      plan_builder_(config_) {}

std::vector<MotionSegment> RobotControlSystem::run(const std::vector<TiePoint>& points) {
    auto plan = plan_builder_.build(points);
    std::cout << "built motion plan tie_points=" << points.size()
              << " segments=" << plan.size()
              << " skipped=" << plan_builder_.skipped().size() << '\n';
    backend_->connect();
    try {
        backend_->prepare();
        ExecutionMonitor(config_, *backend_).assert_ready();
        plan = backend_->prepare_targets(plan);
        RplBatchSender(config_, *backend_).send_queue(plan);
        if (config_.dry_run) {
            TieToolInterface tool(config_);
            for (const MotionSegment& segment : plan) {
                if (segment.stage == MotionStage::Tie) tool.tie(segment);
            }
        }
        backend_->disconnect();
        return plan;
    } catch (...) {
        try {
            RplBatchSender(config_, *backend_).request_stop();
        } catch (...) {
        }
        backend_->disconnect();
        throw;
    }
}

const std::vector<std::string>& RobotControlSystem::skipped() const {
    return plan_builder_.skipped();
}

std::vector<TiePoint> load_tie_points_json(const std::string& path) {
    const json::Value root = json::read_file(path);
    const json::Value* values = root.is_array() ? &root : root.find("points");
    if (!values || !values->is_array()) {
        throw std::runtime_error("points JSON must be an array or contain a points array");
    }
    std::vector<TiePoint> points;
    for (const json::Value& item : values->array()) {
        if (!item.is_object()) throw std::runtime_error("each tie point must be an object");
        const json::Value* pose = item.find("pose");
        if (!pose || !pose->is_object()) pose = &item;
        TiePoint point;
        point.point_id = json::string(item, "point_id", json::string(item, "id", ""));
        point.pose.x = required_number(*pose, "x");
        point.pose.y = required_number(*pose, "y");
        point.pose.z = required_number(*pose, "z");
        point.pose.a = json::number(*pose, "a", 180.0);
        point.pose.b = json::number(*pose, "b", 0.0);
        point.pose.c = json::number(*pose, "c", 180.0);
        point.pose.cfgx = static_cast<int>(json::number(*pose, "cfgx", 0));
        point.pose.cfg1 = static_cast<int>(json::number(*pose, "cfg1", 0));
        point.pose.cfg4 = static_cast<int>(json::number(*pose, "cfg4", 0));
        point.pose.cfg6 = static_cast<int>(json::number(*pose, "cfg6", 0));
        point.confidence = json::number(item, "confidence", 1.0);
        point.source = json::string(item, "source", "offline");
        points.push_back(std::move(point));
    }
    return points;
}

RobotControlConfig load_config_json(const std::string& path) {
    const json::Value root = json::read_file(path);
    if (!root.is_object()) throw std::runtime_error("config JSON must be an object");
    RobotControlConfig config;
    config.controller_ip = json::string(root, "controller_ip", config.controller_ip);
    config.tool_name = json::string(root, "tool_name", config.tool_name);
    config.workobject_name = json::string(root, "workobject_name", config.workobject_name);
    config.batch_size = static_cast<std::size_t>(
        json::number(root, "batch_size", static_cast<double>(config.batch_size)));
    config.dry_run = json::boolean(root, "dry_run", config.dry_run);
    config.require_servo_on =
        json::boolean(root, "require_servo_on", config.require_servo_on);
    config.min_confidence = json::number(root, "min_confidence", config.min_confidence);
    config.duplicate_distance_mm =
        json::number(root, "duplicate_distance_mm", config.duplicate_distance_mm);
    config.minimum_point_spacing_mm =
        json::number(root, "minimum_point_spacing_mm", config.minimum_point_spacing_mm);
    config.tie_dwell_s = json::number(root, "tie_dwell_s", config.tie_dwell_s);
    config.handshake_poll_s =
        json::number(root, "handshake_poll_s", config.handshake_poll_s);
    config.handshake_timeout_s =
        json::number(root, "handshake_timeout_s", config.handshake_timeout_s);
    config.execution_timeout_s =
        json::number(root, "execution_timeout_s", config.execution_timeout_s);
    config.monitor_poll_s =
        json::number(root, "monitor_poll_s", config.monitor_poll_s);
    if (const json::Value* surface = root.find("surface")) {
        if (!surface->is_object()) throw std::runtime_error("surface must be an object");
        config.surface.normal = read_vector3(*surface, "normal", config.surface.normal);
        config.surface.x_direction =
            read_vector3(*surface, "x_direction", config.surface.x_direction);
        config.surface.tool_axis_sign = static_cast<int>(
            json::number(*surface, "tool_axis_sign", config.surface.tool_axis_sign));
        config.surface.tool_tilt_deg =
            json::number(*surface, "tool_tilt_deg", config.surface.tool_tilt_deg);
        config.surface.tool_roll_deg =
            json::number(*surface, "tool_roll_deg", config.surface.tool_roll_deg);
        const std::string convention =
            json::string(*surface, "abc_convention", "ZYX_INTRINSIC");
        if (convention != "ZYX_INTRINSIC") {
            throw std::runtime_error("only abc_convention ZYX_INTRINSIC is supported");
        }
    }
    if (const json::Value* motion = root.find("motion")) {
        if (!motion->is_object()) throw std::runtime_error("motion must be an object");
        config.motion.approach_distance_mm = json::number(
            *motion, "approach_distance_mm", config.motion.approach_distance_mm);
        config.motion.retreat_distance_mm = json::number(
            *motion, "retreat_distance_mm", config.motion.retreat_distance_mm);
        config.motion.transfer_clearance_mm = json::number(
            *motion, "transfer_clearance_mm", config.motion.transfer_clearance_mm);
        config.motion.transfer_velocity_profile = static_cast<int>(json::number(
            *motion, "transfer_velocity_profile",
            config.motion.transfer_velocity_profile));
        config.motion.local_velocity_profile = static_cast<int>(json::number(
            *motion, "local_velocity_profile", config.motion.local_velocity_profile));
        config.motion.zone = json::number(*motion, "zone", config.motion.zone);
    } else {
        bool used_legacy_distance = false;
        if (const json::Value* approach = root.find("approach_offset_z_mm")) {
            config.motion.approach_distance_mm =
                approach->number_or(config.motion.approach_distance_mm);
            used_legacy_distance = true;
        }
        if (const json::Value* retreat = root.find("retreat_offset_z_mm")) {
            config.motion.retreat_distance_mm =
                retreat->number_or(config.motion.retreat_distance_mm);
            used_legacy_distance = true;
        }
        config.motion.zone = json::number(root, "zone", config.motion.zone);
        if (used_legacy_distance) {
            std::cerr << "deprecated Z-offset fields are treated as surface-normal distances\n";
        }
    }
    if (const json::Value* workspace = root.find("workspace")) {
        config.workspace.x = read_limit(*workspace, "x", config.workspace.x);
        config.workspace.y = read_limit(*workspace, "y", config.workspace.y);
        config.workspace.z = read_limit(*workspace, "z", config.workspace.z);
        config.workspace.a = read_limit(*workspace, "a", config.workspace.a);
        config.workspace.b = read_limit(*workspace, "b", config.workspace.b);
        config.workspace.c = read_limit(*workspace, "c", config.workspace.c);
    }
    if (const json::Value* rpl = root.find("rpl")) {
        config.rpl.total_points_int = static_cast<unsigned>(
            json::number(*rpl, "total_points_int", config.rpl.total_points_int));
        config.rpl.transfer_velocity_profile_int = static_cast<unsigned>(json::number(
            *rpl, "transfer_velocity_profile_int",
            config.rpl.transfer_velocity_profile_int));
        config.rpl.local_velocity_profile_int = static_cast<unsigned>(json::number(
            *rpl, "local_velocity_profile_int",
            config.rpl.local_velocity_profile_int));
        config.rpl.error_code_int = static_cast<unsigned>(
            json::number(*rpl, "error_code_int", config.rpl.error_code_int));
        config.rpl.motion_type_int_start = static_cast<unsigned>(json::number(
            *rpl, "motion_type_int_start", config.rpl.motion_type_int_start));
        config.rpl.request_buffer_a_bool = static_cast<unsigned>(
            json::number(*rpl, "request_buffer_a_bool", config.rpl.request_buffer_a_bool));
        config.rpl.request_buffer_b_bool = static_cast<unsigned>(
            json::number(*rpl, "request_buffer_b_bool", config.rpl.request_buffer_b_bool));
        config.rpl.start_bool = static_cast<unsigned>(
            json::number(*rpl, "start_bool", config.rpl.start_bool));
        config.rpl.stop_bool = static_cast<unsigned>(
            json::number(*rpl, "stop_bool", config.rpl.stop_bool));
        config.rpl.batch_done_bool = static_cast<unsigned>(
            json::number(*rpl, "batch_done_bool", config.rpl.batch_done_bool));
        config.rpl.controller_error_bool = static_cast<unsigned>(json::number(
            *rpl, "controller_error_bool", config.rpl.controller_error_bool));
    }
    return config;
}

}  // namespace tie
