# ER25-2700 Motion Planning and Pose Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the C++ ER25-2700 controller with construction-surface pose correction, five-stage safe motion planning, and synchronized `MJOINT/MLIN` double-buffer execution.

**Architecture:** Add a focused motion-planning library that converts configured surface directions and tie points into typed motion segments. Keep the existing RPL double-buffer transport, but synchronize Cartesian `PointC`, resolved joint `PointJ`, integer motion-type, and global velocity-profile/error fields. The SDK resolves each transfer target with `IkSolver`; the controller reads `PointJ` for `MJOINT` transfers and `PointC` for `MLIN` local work.

**Tech Stack:** C++17, CMake/CTest, Efort C++ SDK V2.8, RPL/XPL controller templates, JSON configuration.

## Global Constraints

- The implementation is C++ only; do not add or restore a Python control prototype.
- Do not apply a site calibration or hand-eye transformation in this phase.
- Treat planning as deterministic safe-segment planning, not link-level collision avoidance.
- Do not implement RRT, RRT*, PRM, online replanning, or a local DH/URDF kinematics solver.
- Use `MJOINT` only for `SafeEntry` transfer targets and `MLIN` for `Approach`, `Tie`, `Retreat`, and `SafeExit`.
- Keep two 25-slot buffer ranges and the existing Boolean refill handshake; use matching indices in `PC_POINTC`, `PC_POINTJ`, and the motion-type array.
- Use `PC_INT[10..59]` for per-slot motion types, `PC_INT[1]` for the transfer velocity-profile code, `PC_INT[2]` for the local velocity-profile code, `PC_INT[3]` for controller error code, and `PC_BOOL[5]` for controller error.
- Accept transfer profile code `100` only and map it to the SDK-demonstrated joint-speed object `v100perc`. Accept local profile codes `100` and `800` and map them to Cartesian-speed objects `v100` and `v800`. Default both profile codes to `100`.
- Keep `fine` for every target in this phase; configuration `zone` must equal `-1.0`.
- Preserve unrelated user changes and do not stage or commit files outside each task's explicit file list.

## File Structure

- Create `cpp/include/motion_planning.hpp`: vector/matrix primitives, surface configuration, motion enums, segment data, pose-correction and planner interfaces.
- Create `cpp/include/robot_types.hpp`: shared `CartesianPose`, `JointPose`, and `TiePoint` definitions used by planning and execution.
- Create `cpp/src/motion_planning.cpp`: normalization, orthonormal surface-frame construction, ZYX conversion, normal offsets, and five-stage planning.
- Create `cpp/tests/test_motion_planning.cpp`: isolated mathematical and planner tests.
- Modify `cpp/include/robot_control.hpp`: replace queue-facing `QueuePoint` APIs with `MotionSegment`, add protocol fields and backend validation/type-buffer methods.
- Modify `cpp/src/robot_control.cpp`: configuration compatibility, motion-segment integration, PointC/PointJ protocol transmission, SDK inverse-kinematics preflight, and dry-run logging.
- Modify `cpp/tests/test_robot_control.cpp`: transport, safety, batch, error, and regression tests.
- Modify `cpp/CMakeLists.txt`: compile the new source and register the new test executable.
- Modify `samples/real_config.json` and `samples/sim_config.json`: add explicit surface/motion/protocol settings.
- Modify `TIE_QUEUE_SIM.pgm`: readable mixed-motion controller source.
- Modify `TIE_QUEUE_SIM.XPL`: deployable mixed-motion controller program.
- Modify `TIE_QUEUE_SIM_README.md` and `cpp/README.md`: document the five-stage sequence and safety boundary.

---

### Task 1: Surface Math and Pose Correction

**Files:**
- Create: `cpp/include/robot_types.hpp`
- Create: `cpp/include/motion_planning.hpp`
- Create: `cpp/src/motion_planning.cpp`
- Create: `cpp/tests/test_motion_planning.cpp`
- Modify: `cpp/include/robot_control.hpp`
- Modify: `cpp/CMakeLists.txt`

**Interfaces:**
- Consumes: `tie::CartesianPose` and `tie::JointPose` from the new shared `robot_types.hpp`.
- Produces: `tie::Vector3`, `tie::Matrix3`, `tie::SurfaceFrameConfig`, `tie::SurfacePoseCorrector::corrected_pose(const CartesianPose&)`, and `tie::SurfacePoseCorrector::normal()`.

- [ ] **Step 1: Add failing vector and horizontal-surface tests**

Add `cpp/tests/test_motion_planning.cpp` with a local `require()` helper and these first tests:

```cpp
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

}  // namespace

int main() {
    try {
        test_normalization_and_cross_product();
        test_horizontal_surface_pose();
        std::cout << "All motion planning tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
```

- [ ] **Step 2: Register and run the test to verify it fails**

Add the target to `cpp/CMakeLists.txt` without adding the missing implementation yet:

```cmake
add_executable(motion_planning_tests tests/test_motion_planning.cpp)
target_link_libraries(motion_planning_tests PRIVATE robot_control)
add_test(NAME motion_planning_tests COMMAND motion_planning_tests)
```

Run:

```powershell
cmake -S cpp -B build -G Ninja
cmake --build build
```

Expected: compilation fails because `motion_planning.hpp` does not exist.

- [ ] **Step 3: Add the minimal public math and correction API**

Create `cpp/include/motion_planning.hpp`:

```cpp
#pragma once

#include "robot_types.hpp"

#include <array>
#include <string>

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
```

Implement in `cpp/src/motion_planning.cpp`:

```cpp
#include "motion_planning.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tie {
namespace {
constexpr double kEpsilon = 1e-9;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

Matrix3 columns(const Vector3& x, const Vector3& y, const Vector3& z) {
    return {{{{x.x, y.x, z.x}}, {{x.y, y.y, z.y}}, {{x.z, y.z, z.z}}}};
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
    Vector3 z = static_cast<double>(config_.tool_axis_sign) * normal_;
    Vector3 y = normalized(cross(z, x));
    x = normalized(cross(y, z));
    tool_rotation_ = columns(x, y, z);
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
```

Add `src/motion_planning.cpp` to `add_library(robot_control ...)`.

Before adding `motion_planning.hpp`, create `robot_types.hpp`, move the existing `CartesianPose` and `TiePoint` definitions there without changing their fields, and add:

```cpp
struct JointPose {
    std::array<double, 6> joints{};
};
```

Include `robot_types.hpp` from `robot_control.hpp`. This establishes the shared-type dependency before Task 2 adds `std::optional<JointPose>`.

- [ ] **Step 4: Run the new test and confirm it passes**

Run:

```powershell
cmake --build build
ctest --test-dir build -C Debug --output-on-failure -R motion_planning_tests
```

Expected: `motion_planning_tests` passes.

- [ ] **Step 5: Add tilt/roll and invalid-frame tests before implementing rotations**

Add the following helpers and tests:

```cpp
template <typename Function>
void require_throws(Function function, const char* message) {
    try {
        function();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
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
```

Implement matrix multiplication and local Y/Z rotations in `motion_planning.cpp`:

```cpp
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
    return {{{{std::cos(radians), 0.0, std::sin(radians)}},
              {{0.0, 1.0, 0.0}},
              {{-std::sin(radians), 0.0, std::cos(radians)}}}};
}

Matrix3 rotation_z(double radians) {
    return {{{{std::cos(radians), -std::sin(radians), 0.0}},
              {{std::sin(radians), std::cos(radians), 0.0}},
              {{0.0, 0.0, 1.0}}}};
}
```

After constructing the base frame, apply local tilt and roll:

```cpp
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
tool_rotation_ = multiply(
    tool_rotation_,
    multiply(rotation_z(config_.tool_roll_deg * kDegreesToRadians),
             rotation_y(config_.tool_tilt_deg * kDegreesToRadians)));
```

Run the test and expect all cases to pass without changing position or CFG fields.

- [ ] **Step 6: Commit Task 1**

```powershell
git add cpp/include/robot_types.hpp cpp/include/robot_control.hpp cpp/include/motion_planning.hpp cpp/src/motion_planning.cpp cpp/tests/test_motion_planning.cpp cpp/CMakeLists.txt
git commit -m "feat: add construction surface pose correction"
```

---

### Task 2: Five-Stage Safe Motion Planner

**Files:**
- Modify: `cpp/include/motion_planning.hpp`
- Modify: `cpp/src/motion_planning.cpp`
- Modify: `cpp/tests/test_motion_planning.cpp`

**Interfaces:**
- Consumes: `SurfacePoseCorrector`, `std::vector<TiePoint>`, and configured distances/velocity-profile codes.
- Produces: `MotionSegment`, `MotionType`, `MotionStage`, and `SafeTransferPlanner::plan(const std::vector<TiePoint>&)`.

- [ ] **Step 1: Write failing five-stage tests**

Add tests asserting one tie point produces exactly:

```cpp
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
```

For normal `(1, 0, 1)`, require normalized offsets to change both X and Z. Also require two tie points to produce ten segments and sequence indices `0..9`. Add `require_throws` cases for transfer profile `800`, local profile `123`, negative distance, non-`fine` zone, and transfer clearance smaller than approach distance.

- [ ] **Step 2: Run to verify the planner tests fail**

Run:

```powershell
cmake --build build
ctest --test-dir build -C Debug --output-on-failure -R motion_planning_tests
```

Expected: compilation fails because motion segment and planner types are absent.

- [ ] **Step 3: Add motion types and planner configuration**

Append to `motion_planning.hpp`:

```cpp
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
```

Add `#include <optional>`. The planner leaves `joint_target` empty; Task 5 populates it for real `MJoint` execution using the controller SDK.

- [ ] **Step 4: Implement validation and five targets**

Implement the planner using one helper:

```cpp
CartesianPose shifted_along(
    const CartesianPose& pose, const Vector3& normal, double distance_mm) {
    CartesianPose shifted = pose;
    shifted.x += normal.x * distance_mm;
    shifted.y += normal.y * distance_mm;
    shifted.z += normal.z * distance_mm;
    return shifted;
}
```

Validate `transfer_velocity_profile == 100`, require `local_velocity_profile` to be a member of `{100, 800}`, require distances to be finite and non-negative, require `zone == -1.0`, and require transfer clearance not less than approach/retreat distances. Assign the transfer profile to `MJoint` segments and the local profile to `MLinear` segments. For each corrected tie pose, append `SafeEntry`, `Approach`, `Tie`, `Retreat`, and `SafeExit` with the exact motion types above.

- [ ] **Step 5: Run planner tests**

Run:

```powershell
cmake --build build
ctest --test-dir build -C Debug --output-on-failure -R motion_planning_tests
```

Expected: all surface and planner tests pass.

- [ ] **Step 6: Commit Task 2**

```powershell
git add cpp/include/motion_planning.hpp cpp/src/motion_planning.cpp cpp/tests/test_motion_planning.cpp
git commit -m "feat: plan five-stage robot motions"
```

---

### Task 3: Integrate Motion Segments and Configuration

**Files:**
- Modify: `cpp/include/robot_control.hpp`
- Modify: `cpp/src/robot_control.cpp`
- Modify: `cpp/tests/test_robot_control.cpp`
- Modify: `cpp/CMakeLists.txt`
- Modify: `samples/real_config.json`
- Modify: `samples/sim_config.json`

**Interfaces:**
- Consumes: `SafeTransferPlanner::plan()` from Task 2.
- Produces: `RobotControlSystem::run()` returning `std::vector<MotionSegment>` and JSON-loaded `surface`/`motion` configuration.

- [ ] **Step 1: Replace the old three-point assertions with failing five-stage assertions**

Update `test_three_segments()` to `test_five_stage_plan()` and require two tie points to produce ten segments. Require `SafeEntry` and `SafeExit` Z values to use `transfer_clearance_mm`, and require the first segment to use `MJoint`.

- [ ] **Step 2: Add failing JSON configuration assertions**

Add the `TEST_CONFIG_PATH` compile definition in `cpp/CMakeLists.txt`, then add `test_json_loading()` so the test loads `samples/real_config.json` and asserts:

```cpp
require(near(config.surface.normal.z, 1.0), "surface normal is loaded");
require(near(config.motion.transfer_clearance_mm, 150.0), "clearance is loaded");
require(config.motion.transfer_velocity_profile == 100, "transfer profile is loaded");
require(config.motion.local_velocity_profile == 100, "local profile is loaded");
require(config.rpl.motion_type_int_start == 10, "motion type base is loaded");
require(config.rpl.controller_error_bool == 5, "controller error flag is loaded");
```

- [ ] **Step 3: Run tests to verify the integration fails**

Run `cmake --build build` and expect compilation failures for missing config members and changed return types.

- [ ] **Step 4: Update the control configuration and system**

Confirm the Task 1 dependency direction remains `robot_types.hpp -> motion_planning.hpp -> robot_control.hpp`; do not reintroduce shared type definitions into `robot_control.hpp`.

Add to `RobotControlConfig`:

```cpp
SurfaceFrameConfig surface{};
MotionPlanningConfig motion{};
```

Add to `RplVariableMap`:

```cpp
unsigned transfer_velocity_profile_int{1};
unsigned local_velocity_profile_int{2};
unsigned error_code_int{3};
unsigned motion_type_int_start{10};
unsigned controller_error_bool{5};
```

Remove `TiePointQueue` and the old `QueuePointKind` from public execution paths. Replace them with `SafeTransferPlanner` and `MotionSegment`; keep a small `MotionBatcher` if batching needs a dedicated class.

- [ ] **Step 5: Implement JSON parsing and legacy compatibility**

Add `Vector3 read_vector3(const JsonValue&, const char* field_name)` that requires an array of exactly three finite numbers. Load `surface.normal`, `surface.x_direction`, `surface.tool_axis_sign`, tilt/roll and `ZYX_INTRINSIC`; reject every other convention string. Load the three distances, the two velocity-profile fields, `zone`, and the five RPL indices. If the new distance fields are absent, map `approach_offset_z_mm` and `retreat_offset_z_mm` to the corresponding normal distances and emit one deprecation warning; do not treat the old values as base-Z offsets.

Update both sample configs with this structure (preserve their connection-specific values), and set `samples/sim_config.json` to `"dry_run": true`:

```json
"surface": {
  "normal": [0.0, 0.0, 1.0],
  "x_direction": [1.0, 0.0, 0.0],
  "tool_axis_sign": -1,
  "tool_tilt_deg": 0.0,
  "tool_roll_deg": 0.0,
  "abc_convention": "ZYX_INTRINSIC"
},
"motion": {
  "approach_distance_mm": 50.0,
  "retreat_distance_mm": 50.0,
  "transfer_clearance_mm": 150.0,
  "transfer_velocity_profile": 100,
  "local_velocity_profile": 100,
  "zone": -1.0
},
"rpl": {
  "transfer_velocity_profile_int": 1,
  "local_velocity_profile_int": 2,
  "error_code_int": 3,
  "motion_type_int_start": 10,
  "controller_error_bool": 5
}
```

- [ ] **Step 6: Run all C++ tests**

Run:

```powershell
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

Expected: the planner and existing safety/batching tests pass with five-stage counts.

- [ ] **Step 7: Commit Task 3**

```powershell
git add cpp/include/robot_control.hpp cpp/src/robot_control.cpp cpp/tests/test_robot_control.cpp cpp/CMakeLists.txt samples/real_config.json samples/sim_config.json
git commit -m "feat: integrate typed motion plans"
```

---

### Task 4: Synchronize PointC, PointJ, and Motion-Type Buffers

**Files:**
- Modify: `cpp/include/robot_control.hpp`
- Modify: `cpp/src/robot_control.cpp`
- Modify: `cpp/tests/test_robot_control.cpp`

**Interfaces:**
- Consumes: batches of `MotionSegment`.
- Produces: `RobotBackend::set_motion_batch(...)`, synchronized PointC/PointJ/type writes, velocity-profile initialization, and controller error polling.

- [ ] **Step 1: Add failing backend recording assertions**

Extend `HandshakeBackend` to record PointC and PointJ start slots, type start indices, motion-type arrays, and integer writes. Use fabricated `JointPose` values for test-only `MJoint` segments. Assert a three-batch queue uses PointC/PointJ starts `{0,25,0}`, type starts `{10,35,10}`, and codes match `MotionType` enum values.

- [ ] **Step 2: Add failing controller-error test**

Make a fake backend return `true` for `controller_error_bool` and `9001` for `error_code_int`. Require `RplBatchSender::send_queue()` to throw a message containing `9001` rather than reporting completion.

- [ ] **Step 3: Run to verify failures**

Run `ctest --test-dir build -C Debug --output-on-failure -R robot_control_tests`.

Expected: failures because the backend does not send type arrays or inspect the controller error flag.

- [ ] **Step 4: Change the backend contract atomically**

Replace separate point-only writes with:

```cpp
virtual void set_motion_batch(
    const std::vector<MotionSegment>& segments,
    std::size_t target_start_index,
    unsigned motion_type_start_index) = 0;
```

`DryRunBackend` records Cartesian targets, optional joint targets, and integer type codes. `EfortSdkBackend` builds indexed `RobotAPI::PointC` and `RobotAPI::PointJ` vectors. Every `MJoint` segment must have a resolved joint target; missing resolution is a hard error. It calls `SetPointCVector`, `SetPointJVector`, then the SDK integer-array overload:

```cpp
check(RobotAPI::SetPointCVector(point_c_values, device_id_, false),
      "SetPointCVector");
check(RobotAPI::SetPointJVector(point_j_values, device_id_, false),
      "SetPointJVector");
RobotAPI::SetIntVariable(
    motion_type_start_index,
    static_cast<unsigned>(types.size()),
    types.data(),
    device_id_);
```

Unused entries in either target vector may contain zero placeholders because the motion-type branch never reads them, but both vectors must retain identical slot alignment. Only after all three writes succeed may `RplBatchSender` release the buffer through the Boolean handshake.

- [ ] **Step 5: Initialize velocity profiles and monitor controller errors**

Before sending the initial buffers, write the validated transfer profile code (`100`) and local profile code (`100` or `800`), then clear the error code/error Boolean. While waiting for refill or completion, check `PC_BOOL[5]`; when set, read `PC_INT[3]` and throw `controller error code <n>`.

- [ ] **Step 6: Run transport tests**

Run:

```powershell
cmake --build build
ctest --test-dir build -C Debug --output-on-failure -R robot_control_tests
```

Expected: synchronized start-index, velocity-profile, error, and buffer-wrap tests pass.

- [ ] **Step 7: Commit Task 4**

```powershell
git add cpp/include/robot_control.hpp cpp/src/robot_control.cpp cpp/tests/test_robot_control.cpp
git commit -m "feat: send synchronized motion type buffers"
```

---

### Task 5: Add SDK Target Preflight

**Files:**
- Modify: `cpp/include/robot_control.hpp`
- Modify: `cpp/src/robot_control.cpp`
- Modify: `cpp/tests/test_robot_control.cpp`

**Interfaces:**
- Consumes: the complete planned `std::vector<MotionSegment>`.
- Produces: `RobotBackend::prepare_targets(...)`, resolved joint targets for every transfer, and current-state logging before motion starts.

- [ ] **Step 1: Add failing preflight rejection test**

Add a fake backend that reports the third target as invalid. Require `RobotControlSystem::run()` to reject the complete task before any batch is written and require the recorded batch count to remain zero. Add a second test requiring every `MJoint` segment returned by fake preflight to contain `joint_target` before batching.

- [ ] **Step 2: Add the backend validation interface**

```cpp
virtual std::vector<MotionSegment> prepare_targets(
    const std::vector<MotionSegment>& segments) = 0;
```

The dry-run backend checks finite values and workspace limits, returns a copy, and explicitly logs that inverse kinematics was not resolved. The SDK backend converts each Cartesian target to `RobotAPI::RobotPos`, calls `CheckTarget`, and for every `MJoint` segment calls `IkSolver` to obtain `RobotAPI::RobotJoint`. Copy the six solved axes into `JointPose`; reject the entire task on any non-zero SDK result.

```cpp
RobotAPI::CheckTarget(point_c, config_.tool_name, config_.workobject_name, device_id_);
RobotAPI::IkSolver(robot_pos, robot_joint,
                   config_.tool_name, config_.workobject_name, device_id_);
```

Also call `GetJointPos` and `GetBaseCoordinatePos2` once after `prepare()` and log the actual start state. Use controller-returned inverse kinematics only; do not implement a local solver or fabricate real-execution joint solutions.

- [ ] **Step 3: Run the preflight and regression tests**

Run all tests and expect task rejection before `set_motion_batch()` when any target is invalid.

- [ ] **Step 4: Build the SDK variant**

Run:

```powershell
cmake -S cpp -B cpp/build-sdk-vs18 -DEFORT_WITH_SDK=ON
cmake --build cpp/build-sdk-vs18 --config Release
```

Expected: SDK-backed `robot_control` and `run_tie_queue` compile successfully.

- [ ] **Step 5: Commit Task 5**

```powershell
git add cpp/include/robot_control.hpp cpp/src/robot_control.cpp cpp/tests/test_robot_control.cpp
git commit -m "feat: preflight robot motion targets"
```

---

### Task 6: Mixed MJOINT/MLIN Controller Program

**Files:**
- Modify: `TIE_QUEUE_SIM.pgm`
- Modify: `TIE_QUEUE_SIM.XPL`
- Modify: `cpp/tests/test_robot_control.cpp`
- Modify: `cpp/CMakeLists.txt`

**Interfaces:**
- Consumes: `PC_POINTC[slot]`, `PC_POINTJ[slot]`, `PC_INT[10+slot]`, and velocity-profile codes in `PC_INT[1..2]`.
- Produces: controller execution with type code `1 -> MJOINT`, `0 -> MLIN`, and invalid-type error reporting.

- [ ] **Step 1: Add a controller-template contract test**

Pass `TIE_QUEUE_SIM.pgm` and `TIE_QUEUE_SIM.XPL` paths to `robot_control_tests` using compile definitions. Read both files and require the presence of:

```text
PC_INT[10
PC_POINTJ[
MJOINT
MLIN
PC_BOOL[5]
PC_INT[3]
```

Also require the named velocity objects `v100perc`, `v100`, and `v800`, plus explicit invalid-profile error code `9002`.

- [ ] **Step 2: Run to verify the contract test fails**

Run the robot control test and expect failure because the templates still contain only `MLIN` and do not branch by profile code.

- [ ] **Step 3: Update the readable PGM source**

At task start, reset:

```text
comutil.PC_BOOL[5] := false;
comutil.PC_INT[3] := 0;
```

Before entering the buffer loop, require transfer profile code `100` and local profile code `100` or `800`; otherwise set error code `9002`, set `PC_BOOL[5]`, and stop. For each slot, read `motionType := comutil.PC_INT[10 + i]` and use explicit named-velocity branches:

```text
IF motionType = 1 THEN
    IF comutil.PC_INT[1] = 100 THEN
        MJOINT(comutil.PC_POINTJ[i], v100perc, fine, tool1);
    ELSE
        comutil.PC_INT[3] := 9002;
        comutil.PC_BOOL[5] := true;
        running := false;
    END_IF;
ELSIF motionType = 0 THEN
    IF comutil.PC_INT[2] = 100 THEN
        MLIN(comutil.PC_POINTC[i], v100, fine, tool1);
    ELSIF comutil.PC_INT[2] = 800 THEN
        MLIN(comutil.PC_POINTC[i], v800, fine, tool1);
    ELSE
        comutil.PC_INT[3] := 9002;
        comutil.PC_BOOL[5] := true;
        running := false;
    END_IF;
ELSE
    comutil.PC_INT[3] := 9001;
    comutil.PC_BOOL[5] := true;
    running := false;
END_IF;
```

Do not pass `PC_INT` directly to a motion instruction: the integer is only a selector for the controller's existing named velocity object.

- [ ] **Step 4: Apply the same branch structure to XPL**

Use `DEMO_JOINT.XPL` as the authoritative XML shape for `<mjoint>` and `PC_POINTJ`, and the current `TIE_QUEUE_SIM.XPL` as the buffer-loop base. Preserve all three existing execution blocks and insert equivalent type/error branches in each. The motion leaves must use these exact target/speed pairs (with the existing zone/tool/refsys children retained):

```xml
<mjoint>
  <target>comutil.PC_POINTJ[i_0]</target>
  <speed>v100perc</speed>
  <zone>fine</zone>
  <tool>tool1</tool>
</mjoint>
<mlin>
  <target>comutil.PC_POINTC[i_0]</target>
  <speed>v100</speed>
  <zone>fine</zone>
  <tool>tool1</tool>
  <refsys>wobj_cvy</refsys>
</mlin>
<mlin>
  <target>comutil.PC_POINTC[i_0]</target>
  <speed>v800</speed>
  <zone>fine</zone>
  <tool>tool1</tool>
  <refsys>wobj_cvy</refsys>
</mlin>
```

- [ ] **Step 5: Run the contract test and inspect XML well-formedness**

Run:

```powershell
cmake --build build
ctest --test-dir build -C Debug --output-on-failure -R robot_control_tests
```

Then parse `TIE_QUEUE_SIM.XPL` with a standard XML parser. Expected: tests pass and XML parsing reports no error.

- [ ] **Step 6: Commit Task 6**

```powershell
git add TIE_QUEUE_SIM.pgm TIE_QUEUE_SIM.XPL cpp/tests/test_robot_control.cpp cpp/CMakeLists.txt
git commit -m "feat: execute mixed joint and linear queue motions"
```

---

### Task 7: Documentation, Dry-Run Evidence, and Final Verification

**Files:**
- Modify: `cpp/README.md`
- Modify: `TIE_QUEUE_SIM_README.md`
- Modify: `README.md`
- Preserve: `assets/robot_models/ER25-2700/**`

**Interfaces:**
- Consumes: completed planner, transport, and controller protocol.
- Produces: operator-facing setup instructions and reproducible verification evidence.

- [ ] **Step 1: Update documentation with the exact motion sequence**

Document:

```text
current -> MJOINT SafeEntry
SafeEntry -> MLIN Approach -> MLIN Tie -> MLIN Retreat -> MLIN SafeExit
SafeExit -> MJOINT next SafeEntry
```

State that STEP geometry is downloaded and verified but has not yet been converted to URDF or link-level collision shapes. Remove Python instructions and legacy three-point descriptions.

- [ ] **Step 2: Run a dry-run with a horizontal surface**

Run:

```powershell
build\run_tie_queue.exe --points samples\tie_points.json --config samples\sim_config.json
```

Expected: the checked-in simulation configuration has `dry_run=true`; the program does not connect to a controller. The log contains five segments per accepted tie point and each line includes stage, motion type, XYZ, ABC, velocity-profile code, and zone. `MJoint` lines explicitly say `joint_target=unresolved(dry-run)` so the output cannot be mistaken for controller-validated inverse kinematics.

- [ ] **Step 3: Run a dry-run with a tilted surface**

Use normal `[0.5, 0.0, 0.866025403784]` and verify approach/retreat/safe targets change X and Z while tie positions remain unchanged.

- [ ] **Step 4: Run the complete non-SDK verification**

```powershell
cmake -S cpp -B build -G Ninja
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all tests pass, including motion math, five-stage planning, synchronized buffering, error handling, and controller-template contract tests.

- [ ] **Step 5: Run the SDK compile verification**

```powershell
cmake --build cpp/build-sdk-vs18 --config Release
ctest --test-dir cpp/build-sdk-vs18 -C Release --output-on-failure
```

Expected: SDK build succeeds and unit tests pass without connecting to a robot.

- [ ] **Step 6: Review the final diff for safety claims**

Run:

```powershell
git diff --check
rg -n "已完成自动避障|碰撞检测完成|仿真验证通过|实机验证通过" README.md cpp TIE_QUEUE_SIM_README.md
```

Expected: no wording claims that rule-based planning provides completed collision avoidance, simulation validation, or real-robot validation.

- [ ] **Step 7: Commit Task 7**

```powershell
git add README.md cpp/README.md TIE_QUEUE_SIM_README.md samples/real_config.json samples/sim_config.json
git commit -m "docs: describe safe mixed-motion execution"
```

## Deferred Follow-Up Plan

After obtaining or deriving joint axes, link transforms, collision shapes, and a registered site obstacle model, create a separate implementation plan for URDF generation, collision-scene validation, and RRT-Connect. Do not fold that independent subsystem into this rule-based planner implementation.
