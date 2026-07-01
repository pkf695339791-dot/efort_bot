#include "robot_control.hpp"
#include "json_value.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
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

std::vector<SafetyIssue> SafetyChecker::validate_queue(
    const std::vector<QueuePoint>& points) const {
    if (points.empty()) return {{"<queue>", "queue is empty"}};
    std::vector<SafetyIssue> issues;
    const CartesianPose* last_tie_pose = nullptr;
    for (const QueuePoint& point : points) {
        auto range_issues = check_pose_ranges(point.tie_point_id, point.pose);
        issues.insert(issues.end(), range_issues.begin(), range_issues.end());
        if (point.kind == QueuePointKind::Tie) {
            if (last_tie_pose) {
                const double spacing = distance(point.pose, *last_tie_pose);
                if (spacing < config_.minimum_point_spacing_mm) {
                    std::ostringstream reason;
                    reason << "tie point spacing " << spacing << " mm below minimum";
                    issues.push_back({point.tie_point_id, reason.str()});
                }
            }
            last_tie_pose = &point.pose;
        }
    }
    return issues;
}

TiePointQueue::TiePointQueue(const RobotControlConfig& config)
    : config_(config), safety_checker_(config) {}

std::vector<QueuePoint> TiePointQueue::build(const std::vector<TiePoint>& points) {
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
    std::sort(usable.begin(), usable.end(), [](const TiePoint& lhs, const TiePoint& rhs) {
        if (lhs.pose.x != rhs.pose.x) return lhs.pose.x < rhs.pose.x;
        if (lhs.pose.y != rhs.pose.y) return lhs.pose.y < rhs.pose.y;
        if (lhs.pose.z != rhs.pose.z) return lhs.pose.z < rhs.pose.z;
        return lhs.point_id < rhs.point_id;
    });

    std::vector<QueuePoint> queue;
    for (const TiePoint& point : usable) {
        const CartesianPose base = coordinate_manager_.to_workobject_pose(point);
        const std::array<std::pair<QueuePointKind, CartesianPose>, 3> segments{{
            {QueuePointKind::Approach, base.shifted(config_.approach_offset_z_mm)},
            {QueuePointKind::Tie, base},
            {QueuePointKind::Retreat, base.shifted(config_.retreat_offset_z_mm)},
        }};
        for (const auto& segment : segments) {
            queue.push_back({
                queue.size(), point.point_id, segment.first, segment.second, point.confidence});
        }
    }
    const auto queue_issues = safety_checker_.validate_queue(queue);
    if (!queue_issues.empty()) {
        std::ostringstream message;
        message << "queue failed safety validation: ";
        for (std::size_t index = 0; index < queue_issues.size(); ++index) {
            if (index) message << "; ";
            message << issue_message(queue_issues[index]);
        }
        throw std::runtime_error(message.str());
    }
    return queue;
}

std::vector<std::vector<QueuePoint>> TiePointQueue::batches(
    const std::vector<QueuePoint>& points) const {
    if (config_.batch_size == 0) throw std::invalid_argument("batch_size must be positive");
    std::vector<std::vector<QueuePoint>> result;
    for (std::size_t start = 0; start < points.size(); start += config_.batch_size) {
        const std::size_t end = std::min(points.size(), start + config_.batch_size);
        result.emplace_back(points.begin() + static_cast<std::ptrdiff_t>(start),
                            points.begin() + static_cast<std::ptrdiff_t>(end));
    }
    return result;
}

const std::vector<std::string>& TiePointQueue::skipped() const { return skipped_; }

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
void DryRunBackend::set_pointc_vector(const std::vector<QueuePoint>& points) {
    ensure_connected();
    last_vector_ = points;
}
RobotStatus DryRunBackend::read_status() {
    ensure_connected();
    return {connected_, false, false, true, false};
}
const std::vector<QueuePoint>& DryRunBackend::last_vector() const { return last_vector_; }

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
void EfortSdkBackend::set_pointc_vector(const std::vector<QueuePoint>& points) {
    std::vector<RobotAPI::PointC> values;
    values.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        const QueuePoint& item = points[index];
        RobotAPI::PointC point{};
        point.index = static_cast<int>(index % 50);
        point.x = item.pose.x;
        point.y = item.pose.y;
        point.z = item.pose.z;
        point.a = item.pose.a;
        point.b = item.pose.b;
        point.c = item.pose.c;
        point.cfgx = item.pose.cfgx;
        point.cfg1 = item.pose.cfg1;
        point.cfg4 = item.pose.cfg4;
        point.cfg6 = item.pose.cfg6;
        values.push_back(point);
    }
    check(RobotAPI::SetPointCVector(values, device_id_, false), "SetPointCVector");
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
void EfortSdkBackend::set_int(unsigned, int) { connect(); }
int EfortSdkBackend::get_int(unsigned) { connect(); return 0; }
void EfortSdkBackend::set_bool(unsigned, bool) { connect(); }
bool EfortSdkBackend::get_bool(unsigned) { connect(); return false; }
void EfortSdkBackend::set_pointc_vector(const std::vector<QueuePoint>&) { connect(); }
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
    const std::vector<QueuePoint>& batch, std::size_t batch_index) {
    backend_.set_pointc_vector(batch);
    std::cout << "sent batch=" << batch_index << " size=" << batch.size()
              << " queue_range=" << batch.front().queue_index << '-'
              << batch.back().queue_index << '\n';
}

void RplBatchSender::wait_for_request(unsigned bool_index) {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(config_.handshake_timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        const RobotStatus status = backend_.read_status();
        if (status.alarm || status.emergency_stop) {
            throw std::runtime_error("robot unsafe while waiting for XPL buffer request");
        }
        if (backend_.get_bool(bool_index)) return;
        sleep_seconds(config_.handshake_poll_s);
    }
    throw std::runtime_error("timed out waiting for XPL buffer request");
}

void RplBatchSender::send_queue(const std::vector<QueuePoint>& points) {
    if (points.empty()) throw std::invalid_argument("cannot send an empty queue");
    TiePointQueue queue_builder(config_);
    const auto batches = queue_builder.batches(points);
    backend_.set_int(config_.rpl.total_points_int, static_cast<int>(points.size()));
    send_batch(batches.front(), 0);
    backend_.set_bool(config_.rpl.start_bool, true);
    for (std::size_t index = 1; index < batches.size(); ++index) {
        if (!config_.dry_run) {
            const unsigned request = index % 2 == 1
                ? config_.rpl.request_buffer_a_bool
                : config_.rpl.request_buffer_b_bool;
            wait_for_request(request);
            send_batch(batches[index], index);
            backend_.set_bool(request, false);
        } else {
            send_batch(batches[index], index);
        }
    }
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
void TieToolInterface::tie(const QueuePoint& point) const {
    std::cout << "simulated tie action point=" << point.tie_point_id
              << " queue_index=" << point.queue_index << '\n';
    sleep_seconds(config_.tie_dwell_s);
}

RobotControlSystem::RobotControlSystem(
    RobotControlConfig config, std::unique_ptr<RobotBackend> backend)
    : config_(std::move(config)),
      backend_(backend ? std::move(backend) : create_backend(config_)),
      queue_builder_(config_) {}

std::vector<QueuePoint> RobotControlSystem::run(const std::vector<TiePoint>& points) {
    auto queue = queue_builder_.build(points);
    std::cout << "built queue tie_points=" << points.size()
              << " queue_points=" << queue.size()
              << " skipped=" << queue_builder_.skipped().size() << '\n';
    backend_->connect();
    try {
        backend_->prepare();
        ExecutionMonitor(config_, *backend_).assert_ready();
        RplBatchSender(config_, *backend_).send_queue(queue);
        if (config_.dry_run) {
            TieToolInterface tool(config_);
            for (const QueuePoint& point : queue) {
                if (point.kind == QueuePointKind::Tie) tool.tie(point);
            }
        }
        backend_->disconnect();
        return queue;
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
    return queue_builder_.skipped();
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
    config.speed = static_cast<int>(json::number(root, "speed", config.speed));
    config.zone = json::number(root, "zone", config.zone);
    config.dry_run = json::boolean(root, "dry_run", config.dry_run);
    config.require_servo_on =
        json::boolean(root, "require_servo_on", config.require_servo_on);
    config.min_confidence = json::number(root, "min_confidence", config.min_confidence);
    config.duplicate_distance_mm =
        json::number(root, "duplicate_distance_mm", config.duplicate_distance_mm);
    config.minimum_point_spacing_mm =
        json::number(root, "minimum_point_spacing_mm", config.minimum_point_spacing_mm);
    config.approach_offset_z_mm =
        json::number(root, "approach_offset_z_mm", config.approach_offset_z_mm);
    config.retreat_offset_z_mm =
        json::number(root, "retreat_offset_z_mm", config.retreat_offset_z_mm);
    config.tie_dwell_s = json::number(root, "tie_dwell_s", config.tie_dwell_s);
    config.handshake_poll_s =
        json::number(root, "handshake_poll_s", config.handshake_poll_s);
    config.handshake_timeout_s =
        json::number(root, "handshake_timeout_s", config.handshake_timeout_s);
    config.monitor_poll_s =
        json::number(root, "monitor_poll_s", config.monitor_poll_s);
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
        config.rpl.current_point_int = static_cast<unsigned>(
            json::number(*rpl, "current_point_int", config.rpl.current_point_int));
        config.rpl.error_code_int = static_cast<unsigned>(
            json::number(*rpl, "error_code_int", config.rpl.error_code_int));
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
    }
    return config;
}

std::string queue_point_kind_name(QueuePointKind kind) {
    switch (kind) {
        case QueuePointKind::Approach: return "approach";
        case QueuePointKind::Tie: return "tie";
        case QueuePointKind::Retreat: return "retreat";
    }
    return "unknown";
}

}  // namespace tie
