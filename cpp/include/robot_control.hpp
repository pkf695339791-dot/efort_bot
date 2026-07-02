#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace tie {

struct AxisLimit {
    double lower;
    double upper;
};

struct WorkspaceLimits {
    AxisLimit x{-1000.0, 1000.0};
    AxisLimit y{-1000.0, 1000.0};
    AxisLimit z{-500.0, 1500.0};
    AxisLimit a{-180.0, 180.0};
    AxisLimit b{-180.0, 180.0};
    AxisLimit c{-180.0, 180.0};
};

struct RplVariableMap {
    unsigned total_points_int{0};
    unsigned current_point_int{1};
    unsigned error_code_int{2};
    unsigned request_buffer_a_bool{1};
    unsigned request_buffer_b_bool{2};
    unsigned start_bool{3};
    unsigned stop_bool{4};
    unsigned batch_done_bool{0};
};

struct RobotControlConfig {
    std::string controller_ip{"192.168.1.12"};
    std::string tool_name{"tool_tie"};
    std::string workobject_name{"wobj_rebar"};
    std::size_t batch_size{25};
    int speed{10};
    double zone{-1.0};
    bool dry_run{true};
    bool require_servo_on{false};
    double min_confidence{0.55};
    double duplicate_distance_mm{1.0};
    double minimum_point_spacing_mm{0.5};
    double approach_offset_z_mm{50.0};
    double retreat_offset_z_mm{50.0};
    double tie_dwell_s{0.25};
    double handshake_poll_s{0.05};
    double handshake_timeout_s{120.0};
    double execution_timeout_s{300.0};
    double monitor_poll_s{0.1};
    WorkspaceLimits workspace{};
    RplVariableMap rpl{};
};

struct CartesianPose {
    double x{};
    double y{};
    double z{};
    double a{180.0};
    double b{};
    double c{180.0};
    int cfgx{};
    int cfg1{};
    int cfg4{};
    int cfg6{};

    CartesianPose shifted(double dz) const;
};

struct TiePoint {
    std::string point_id;
    CartesianPose pose;
    double confidence{1.0};
    std::string source{"offline"};
};

enum class QueuePointKind { Approach, Tie, Retreat };

struct QueuePoint {
    std::size_t queue_index{};
    std::string tie_point_id;
    QueuePointKind kind{QueuePointKind::Approach};
    CartesianPose pose;
    double confidence{};
};

struct SafetyIssue {
    std::string point_id;
    std::string reason;
    std::string severity{"error"};
};

struct RobotStatus {
    bool connected{};
    bool alarm{};
    bool emergency_stop{};
    bool servo_on{};
    bool moving{};
};

class CoordinateManager {
public:
    CartesianPose to_workobject_pose(const TiePoint& point) const;
};

class SafetyChecker {
public:
    explicit SafetyChecker(const RobotControlConfig& config);
    std::vector<SafetyIssue> validate_tie_points(const std::vector<TiePoint>& points) const;
    std::vector<SafetyIssue> validate_queue(const std::vector<QueuePoint>& points) const;

private:
    const RobotControlConfig& config_;
    std::vector<SafetyIssue> check_pose_ranges(
        const std::string& point_id, const CartesianPose& pose) const;
    static double distance(const CartesianPose& first, const CartesianPose& second);
};

class TiePointQueue {
public:
    explicit TiePointQueue(const RobotControlConfig& config);
    std::vector<QueuePoint> build(const std::vector<TiePoint>& points);
    std::vector<std::vector<QueuePoint>> batches(const std::vector<QueuePoint>& points) const;
    const std::vector<std::string>& skipped() const;

private:
    const RobotControlConfig& config_;
    CoordinateManager coordinate_manager_;
    SafetyChecker safety_checker_;
    std::vector<std::string> skipped_;
};

class RobotBackend {
public:
    virtual ~RobotBackend() = default;
    virtual void connect() = 0;
    virtual void disconnect() noexcept = 0;
    virtual void prepare() = 0;
    virtual void set_int(unsigned index, int value) = 0;
    virtual int get_int(unsigned index) = 0;
    virtual void set_bool(unsigned index, bool value) = 0;
    virtual bool get_bool(unsigned index) = 0;
    virtual void set_pointc_vector(
        const std::vector<QueuePoint>& points, std::size_t start_index) = 0;
    virtual RobotStatus read_status() = 0;
};

class DryRunBackend final : public RobotBackend {
public:
    explicit DryRunBackend(const RobotControlConfig& config);
    void connect() override;
    void disconnect() noexcept override;
    void prepare() override;
    void set_int(unsigned index, int value) override;
    int get_int(unsigned index) override;
    void set_bool(unsigned index, bool value) override;
    bool get_bool(unsigned index) override;
    void set_pointc_vector(
        const std::vector<QueuePoint>& points, std::size_t start_index) override;
    RobotStatus read_status() override;

    const std::vector<QueuePoint>& last_vector() const;
    std::size_t last_start_index() const;

private:
    const RobotControlConfig& config_;
    bool connected_{false};
    std::unordered_map<unsigned, bool> bools_;
    std::unordered_map<unsigned, int> ints_;
    std::vector<QueuePoint> last_vector_;
    std::size_t last_start_index_{};
    void ensure_connected() const;
};

class EfortSdkBackend final : public RobotBackend {
public:
    explicit EfortSdkBackend(const RobotControlConfig& config);
    void connect() override;
    void disconnect() noexcept override;
    void prepare() override;
    void set_int(unsigned index, int value) override;
    int get_int(unsigned index) override;
    void set_bool(unsigned index, bool value) override;
    bool get_bool(unsigned index) override;
    void set_pointc_vector(
        const std::vector<QueuePoint>& points, std::size_t start_index) override;
    RobotStatus read_status() override;

private:
    const RobotControlConfig& config_;
    unsigned device_id_{};
    static void check(int result, const std::string& operation);
};

std::unique_ptr<RobotBackend> create_backend(const RobotControlConfig& config);

class RplBatchSender {
public:
    RplBatchSender(const RobotControlConfig& config, RobotBackend& backend);
    void send_queue(const std::vector<QueuePoint>& points);
    void request_stop();

private:
    const RobotControlConfig& config_;
    RobotBackend& backend_;
    void send_batch(
        const std::vector<QueuePoint>& batch,
        std::size_t batch_index,
        std::size_t buffer_start);
    void wait_for_request(unsigned bool_index);
    void wait_for_completion();
};

class ExecutionMonitor {
public:
    ExecutionMonitor(const RobotControlConfig& config, RobotBackend& backend);
    void assert_ready();
    void wait_until_idle(double timeout_s);

private:
    const RobotControlConfig& config_;
    RobotBackend& backend_;
};

class TieToolInterface {
public:
    explicit TieToolInterface(const RobotControlConfig& config);
    void tie(const QueuePoint& point) const;

private:
    const RobotControlConfig& config_;
};

class RobotControlSystem {
public:
    explicit RobotControlSystem(
        RobotControlConfig config, std::unique_ptr<RobotBackend> backend = nullptr);
    std::vector<QueuePoint> run(const std::vector<TiePoint>& points);
    const std::vector<std::string>& skipped() const;

private:
    RobotControlConfig config_;
    std::unique_ptr<RobotBackend> backend_;
    TiePointQueue queue_builder_;
};

std::vector<TiePoint> load_tie_points_json(const std::string& path);
RobotControlConfig load_config_json(const std::string& path);
std::string queue_point_kind_name(QueuePointKind kind);

}  // namespace tie
