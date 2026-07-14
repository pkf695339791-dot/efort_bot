# ER25-2700 路径规划与施工面姿态修正设计

## 1. 目标

在现有 C++ 机械臂控制系统上增加可落地的规则化路径规划和施工面姿态修正，使 ER25-2700 不再从任意当前位置直接以一段 `MLIN` 进入作业点，而是按安全转场、法向接近、捆扎、法向撤离的分段策略执行。

本阶段保持现有 C++、埃夫特 SDK、RPL/XPL 双缓冲架构，不引入 Python 原型，不接入现场标定变换。

## 2. 已确认的设备能力

ER25-2700 产品资料确认该机器人为六轴工业机器人，额定负载 25 kg，可达半径 2701 mm。产品单页提供关节范围和最大单轴速度，但不包含完整 DH 参数、零位定义或连杆碰撞几何。

埃夫特 C++ SDK 提供：

- `GetJointPos`：读取当前关节位置；
- `GetBaseCoordinatePos2`：读取带构型参数的当前 TCP 位姿；
- `FkSolver`、`IkSolver`：调用控制器运动学求解；
- `MJOINT`、`MLIN`：关节插补和笛卡尔直线运动；
- `SetPointCVector`、整型数组变量以及 RPL `comutil.PC_POINTC[]`：支持扩展现有半控队列协议。

## 3. 当前实现与问题

当前 `TiePointQueue` 为每个捆扎点生成：

```text
approach -> tie -> retreat
```

接近点和撤离点仅通过基准位姿的 `z` 分量加固定偏移得到。控制器端对全部 `PointC` 使用 `MLIN(..., fine, ...)`。

这会产生以下问题：

1. 当前 TCP 到第一个接近点直接执行直线运动，没有显式安全转场点。
2. 上一个撤离点到下一个接近点直接执行直线运动。
3. 倾斜施工面下，基坐标 `Z` 偏移不等于施工面法向偏移。
4. 输入姿态没有根据施工面法向和主钢筋方向统一修正。
5. 队列数据没有表达 `MJOINT` 与 `MLIN` 的运动类型。

## 4. 设计边界

### 4.1 本阶段实现

- 用配置给出的施工面法向量和面内参考方向计算捆扎工具姿态。
- 沿施工面法向生成接近点、撤离点和更远的安全转场点。
- 将运动队列扩展为带运动类型的运动段。
- 首次进入和捆扎点之间转场使用 `MJOINT`。
- 接近、捆扎、撤离和法向退出使用 `MLIN`。
- 为运动类型扩展 PC 端与 RPL/XPL 端的双缓冲协议。
- 保留现有点位校验、报警、急停、伺服和握手监控。
- 提供 dry-run 输出和单元测试，能在无实机环境验证生成的运动序列。

### 4.2 本阶段不实现

- 现场手眼标定或工件坐标变换矩阵；
- 基于 DH/URDF 的自主正逆运动学实现；
- 机械臂连杆级碰撞检测；
- 任意障碍物自动绕行；
- RRT、RRT*、PRM 等采样式规划；
- 在线动态重规划；
- 未经现场确认的自动高速运行。

因此，本阶段的“路径规划”定义为基于已验证安全区域的规则化分段规划，不宣称是完整三维避障规划。

## 5. 总体运动策略

每个有效捆扎点扩展为五个运动段目标：

```text
当前位置
  -> MJOINT -> SafeEntry
  -> MLIN   -> Approach
  -> MLIN   -> Tie
  -> MLIN   -> Retreat
  -> MLIN   -> SafeExit
  -> MJOINT -> 下一个 SafeEntry
```

其中：

- `SafeEntry`：下一捆扎点沿施工面法向外侧的安全转场点；
- `Approach`：沿施工面法向外侧的局部接近点；
- `Tie`：捆扎作业目标点；
- `Retreat`：完成捆扎后的法向撤离点；
- `SafeExit`：继续沿法向退出到安全转场距离的点。

`SafeEntry` 和 `SafeExit` 可以在几何上相同，但保留不同语义，便于日志、状态机和后续规划扩展。

所有转场点必须位于现场已验证的无障碍安全区域。`MJOINT` 仅保证关节插补，不保证 TCP 路径为直线，也不替代整机碰撞检测。

## 6. 数据模型

### 6.1 基础向量和施工面描述

新增 `Vector3`，支持归一化、点积、叉积和标量运算。

新增 `SurfaceFrameConfig`：

```text
normal              施工面单位法向的输入值
x_direction         施工面内主钢筋或参考 X 方向
tool_axis_sign       工具工作轴朝向法向或反法向
tool_tilt_deg        工具相对法向的附加倾角
tool_roll_deg        工具绕法向的滚转角
```

`normal` 与 `x_direction` 必须使用和输入捆扎点相同的坐标系。两者不得为零向量或近似平行。

### 6.2 运动段

新增：

```text
MotionType: MJoint | MLinear
MotionStage: SafeEntry | Approach | Tie | Retreat | SafeExit
MotionSegment:
  sequence_index
  tie_point_id
  stage
  motion_type
  target_pose
  speed
  zone
  confidence
```

现有 `QueuePoint` 的外部职责由 `MotionSegment` 取代。为减少无关改动，双缓冲批次和安全校验继续围绕该运动段集合工作。

## 7. 姿态修正算法

### 7.1 建立正交施工面坐标架

输入施工面法向 `n_raw` 和面内参考方向 `x_raw`：

```text
z_surface = normalize(n_raw)
x_projected = x_raw - dot(x_raw, z_surface) * z_surface
x_surface = normalize(x_projected)
y_surface = normalize(cross(z_surface, x_surface))
```

计算后再次使用叉积修正 `x_surface`，避免数值误差破坏正交性。

### 7.2 工具方向

工具工作轴根据 `tool_axis_sign` 对准 `z_surface` 或 `-z_surface`。然后按配置施加：

1. 围绕施工面切向轴的 `tool_tilt_deg`；
2. 围绕施工面法向的 `tool_roll_deg`。

内部使用旋转矩阵完成组合，不直接对欧拉角逐项相加。

### 7.3 A/B/C 输出约定

埃夫特 SDK 文档确认 `A/B/C` 是当前用户坐标系下的欧拉角，但现有资料未明确旋转顺序。因此配置中增加 `abc_convention`，本阶段只实现一个明确命名并有单元测试的约定，默认值为 `ZYX_INTRINSIC`，禁止隐式猜测。

现场首次运动前必须用一个已示教的已知姿态核对该约定。若控制器实际约定不同，只替换矩阵到欧拉角的适配器，不改变法向计算和路径规划模块。

### 7.4 姿态连续性

同一施工平面默认对全部 `SafeEntry`、`Approach`、`Tie`、`Retreat` 和 `SafeExit` 使用相同工具姿态。角度归一化到连续表示，避免相邻目标出现等价欧拉角的 `+180/-180` 跳变。

构型参数 `cfgx/cfg1/cfg4/cfg6` 暂时继承输入捆扎点。现场接入时使用 SDK `IkSolver` 和当前关节状态验证构型连续性；本阶段 dry-run 不伪造逆解结果。

## 8. 位置规划算法

设修正后的捆扎点位置为 `p`，单位施工面法向为 `n`：

```text
Approach = p + approach_distance_mm * n
Tie      = p
Retreat  = p + retreat_distance_mm * n
SafeEntry = p + transfer_clearance_mm * n
SafeExit  = p + transfer_clearance_mm * n
```

必须满足：

```text
transfer_clearance_mm >= approach_distance_mm
transfer_clearance_mm >= retreat_distance_mm
```

距离必须为非负值。法向正负由 `normal` 和 `tool_axis_sign` 分开表达：`normal` 决定机械臂从施工面退出的空间方向，`tool_axis_sign` 决定工具工作轴朝向。

第一目标为第一个 `SafeEntry`，执行时机器人从当前实际位姿以 `MJOINT` 到达该目标。一个捆扎点的 `SafeExit` 到下一个捆扎点的 `SafeEntry` 同样使用 `MJOINT`。

本阶段保持现有确定性点位排序，避免同时引入任务排序优化。后续可用规划代价替换现有坐标排序。

## 9. 控制器队列协议

### 9.1 PointC 缓冲

目标位姿继续写入：

```text
comutil.PC_POINTC[0..49]
```

保持 A/B 两个 25 点缓冲区和现有续传握手。

### 9.2 运动类型缓冲

为每个 PointC 同步写入运动类型编码：

```text
PC_INT[10 + slot]
0 = MLIN
1 = MJOINT
```

其中 `slot` 为 0 到 49。`PC_INT[0]` 继续表示总点数；`PC_INT[1..3]` 用于本设计的全局控制字段；`PC_INT[4..9]` 保留。

本设计占用以下全局控制字段：

```text
PC_INT[1] = transfer_speed
PC_INT[2] = local_speed
PC_INT[3] = controller_error_code
PC_BOOL[5] = controller_error
```

速度值由 C++ 在启动队列前写入，控制器校验范围为 1 到 100。控制器错误标志和错误码在新任务开始时清零；出现非法运动类型或运动指令失败时，控制器置位错误标志、写入非零错误码并终止队列。

C++ 后端增加批量写入运动类型的能力。每次写入 PointC 批次时，必须先写完整点位和类型，再通过现有布尔握手通知控制器使用该缓冲区。

### 9.3 RPL/XPL 执行

控制器读取每个槽位后按运动类型分支：

```text
type == 1 -> MJOINT(PointC target, transfer speed, fine, tool)
type == 0 -> MLIN(PointC target, local speed, fine, tool)
```

本阶段所有目标继续使用 `fine`，优先确保现场测试动作可观察、可停止。控制器分别读取 `PC_INT[1]` 和 `PC_INT[2]` 作为转场速度和局部作业速度，不再使用写死的 `v100/v800`，也不再让 C++ 配置与 RPL 固定速度互相矛盾。

控制器不识别的运动类型必须触发停止和错误标志，不得回退为任意运动。

## 10. 配置

在现有 JSON 配置中增加：

```json
{
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
    "transfer_speed": 5,
    "local_speed": 3,
    "zone": -1.0
  },
  "rpl": {
    "motion_type_int_start": 10
  }
}
```

默认速度保持保守，仅用于初始配置示例；现场值必须由项目组按工具、负载和风险评估确认。

本阶段 `zone` 只允许 `-1.0`，明确映射为控制器 `fine`。其他过渡值留到现场单点分段运动验证完成后再开放，避免安全转场初期出现轨迹拐角偏离。

旧配置字段 `approach_offset_z_mm`、`retreat_offset_z_mm` 在加载时可兼容映射到新距离字段，并输出弃用提示；新配置不再使用“Z 偏移”命名。

## 11. 校验与失败处理

规划前校验：

- 法向量和参考方向有限且可归一化；
- 两个方向不近似平行；
- 距离、速度和 zone 合法；
- 安全距离不小于接近和撤离距离；
- 生成的全部位置和姿态为有限数；
- 全部目标位于配置工作空间内；
- 每个捆扎点生成完整的五阶段序列。

实机执行前增加：

- 读取当前 TCP 和关节状态；
- 使用 SDK `CheckTarget` 或 `IkSolver` 校验所有目标；
- 任一目标无解时整条任务拒绝执行，不跳过后继续；
- 报警、急停、伺服异常或控制器程序错误时请求停止；
- motion type 与 PointC 批次写入失败时不得启动缓冲区。

## 12. 测试策略

### 12.1 数学单元测试

- 水平面法向 `(0,0,1)` 的偏移结果；
- 倾斜法向的 X/Y/Z 联合偏移；
- 非单位法向自动归一化；
- 面内方向投影和正交性；
- 零向量和平行方向拒绝；
- 垂直姿态、倾角和滚转角的矩阵结果；
- 欧拉角往返和边界角连续性。

### 12.2 规划单元测试

- 单个捆扎点生成五段；
- 运动类型顺序为 `MJOINT, MLIN, MLIN, MLIN, MLIN`；
- 多个捆扎点之间由下一点的 `SafeEntry/MJOINT` 承接；
- `SafeEntry/SafeExit` 使用安全距离；
- 非法安全距离拒绝；
- 批次边界在五段模型下仍正确。

### 12.3 协议测试

- PointC 和 motion type 使用相同槽位；
- A/B 缓冲区起始索引同步；
- 缓冲区复用前等待控制器请求；
- 转场速度和局部速度在启动前写入并校验；
- 未知运动类型拒绝；
- 控制器错误标志和错误码能够被 C++ 监控并转成失败结果；
- dry-run 日志明确输出 stage、motion type、位置和姿态。

### 12.4 回归测试

- 现有报警、急停、伺服和双缓冲测试继续通过；
- JSON 点位加载继续兼容；
- C++ 无 SDK 构建和 SDK 构建均可编译。

## 13. 现场验证门槛

代码测试通过不等于可直接自动运行。现场首次测试必须：

1. 确认工具 TCP 和工具工作轴方向；
2. 确认 `A/B/C` 欧拉角约定；
3. 确认施工面法向方向指向安全退出侧；
4. 逐个验证 `SafeEntry`、`Approach`、`Tie`、`Retreat`、`SafeExit`；
5. 使用手动或低速模式、低全局倍率和单步执行；
6. 确认安全转场区域无障碍物；
7. 在没有完整碰撞模型时，不允许将规则化规划描述为自动避障。

## 14. 后续扩展

获得 ER25-2700 的 DH/URDF、连杆碰撞模型和现场障碍物几何后，可以在不改变 `MotionSegment` 和执行协议的前提下新增 `RrtConnectPlanner`：读取当前关节角，使用 SDK 或本地运动学求解候选目标构型，执行关节空间采样、整机碰撞检测、路径平滑和时间参数化，再输出多个 `MJOINT` 运动段。
