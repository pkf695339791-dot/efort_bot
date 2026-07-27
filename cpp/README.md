# ER25-2700 机械臂 C++ 控制程序

本目录包含纯 C++17 的绑扎机械臂控制实现，不包含 Python 原型。程序完成点位筛选、施工面姿态修正、五阶段运动规划、控制器目标预检和 25 槽双缓冲下发。

## 运动控制逻辑

每个合格绑扎点生成五个 `MotionSegment`：

```text
当前位置 -> MJOINT SafeEntry
SafeEntry -> MLIN Approach
Approach  -> MLIN Tie
Tie       -> MLIN Retreat
Retreat   -> MLIN SafeExit
SafeExit  -> MJOINT 下一个 SafeEntry
```

- `SafeEntry`：位于绑扎点沿施工面法向的安全高度，用 `MJOINT` 从当前状态转移。
- `Approach`：沿法向下降到接近点。
- `Tie`：到达作业点；绑扎 IO/工艺动作仍需现场接入。
- `Retreat`：沿法向撤离作业面。
- `SafeExit`：回到安全高度，为下一个绑扎点转移做准备。

`surface.normal` 定义施工面法向，`surface.x_direction` 定义面内 X 方向。程序将二者正交化为施工面坐标架，并按 `tool_axis_sign`、`tool_tilt_deg`、`tool_roll_deg` 生成工具 ZYX 姿态。接近、撤离和安全高度都沿法向计算，所以倾斜面会同时改变 X/Y/Z，而不是只沿世界坐标 Z 轴移动。

当前 SafeEntry 之间的转移是 `MJOINT` 目标转移，不是带障碍物约束的全局路径搜索。STEP 模型尚未转换成 URDF/碰撞体，现场障碍物尚未注册，因此不要把该逻辑视为完整避障。

## 构建与测试

普通 dry-run 版本：

```powershell
cmake -S cpp -B cpp\build -A x64
cmake --build cpp\build --config Release
ctest --test-dir cpp\build -C Release --output-on-failure
```

运行水平面示例：

```powershell
cpp\build\Release\run_tie_queue.exe `
  --points samples\tie_points.json `
  --config samples\sim_config.json
```

`sim_config.json` 固定 `dry_run=true`，不会连接控制器。输出逐段包含阶段、运动类型、XYZ、ABC、速度档位和 zone；`MJOINT` 预检会明确显示 `joint_target=unresolved(dry-run)`，表示没有做真实控制器逆解。

真实 SDK 版本：

```powershell
cmake -S cpp -B cpp\build-sdk -A x64 `
  -DEFORT_WITH_SDK=ON `
  "-DEFORT_SDK_ROOT=C:\path\to\SDK V2.8\V2.8.0"
cmake --build cpp\build-sdk --config Release
ctest --test-dir cpp\build-sdk -C Release --output-on-failure
```

只有在配置与现场检查完成后才运行：

```powershell
cpp\build-sdk\Release\run_tie_queue.exe `
  --config samples\real_config.json `
  --points samples\tie_points.json `
  --real-robot
```

真实模式会先读取当前关节/基坐标状态，对全部笛卡尔目标调用 `CheckTarget`，并对所有 `MJOINT` 目标调用控制器 `IkSolver`。任一目标失败时，整条任务会在首批运动下发前终止。

## 配置重点

- `surface`：施工面法向、面内方向、工具轴方向及附加倾角/滚角。
- `motion`：接近距离、撤离距离、安全高度、速度档位和 `fine` zone。
- `workspace`：上位机侧笛卡尔范围，只是第一层边界检查。
- `rpl`：双缓冲、运动类型、速度档位与错误反馈变量索引。
- `tool_name` / `workobject_name`：必须与控制器中实际名称一致。

当前仅支持传递速度档 `100 -> v100perc`，局部直线速度档 `100 -> v100` 或 `800 -> v800`，所有点使用 `fine`（配置值 `zone=-1.0`）。

## 与视觉 C++ 集成

视觉模块构造 `std::vector<tie::TiePoint>` 后调用：

```cpp
tie::RobotControlConfig config;
tie::RobotControlSystem controller(config);
const auto plan = controller.run(detected_points);
```

现场标定变换暂不在本阶段实现。接入时应在进入 `RobotControlSystem` 前把视觉点统一变换到配置所声明的工件坐标系，并保证只有控制线程调用 SDK。
