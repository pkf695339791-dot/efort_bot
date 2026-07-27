# efort_bot

埃夫特 ER25-2700 钢筋绑扎机械臂 C++ 控制工程。

当前实现的重点是机械臂运动控制：根据施工面法向修正工具姿态，为每个绑扎点生成五阶段安全运动，并通过埃夫特 SDK 与控制器端 RPL/XPL 双缓冲程序同步执行 `MJOINT` 和 `MLIN`。

```text
当前位置 -> MJOINT SafeEntry
SafeEntry -> MLIN Approach -> MLIN Tie -> MLIN Retreat -> MLIN SafeExit
SafeExit -> MJOINT 下一个 SafeEntry
```

- [cpp/README.md](cpp/README.md)：C++ 构建、配置、运动规划与运行说明
- [TIE_QUEUE_SIM_README.md](TIE_QUEUE_SIM_README.md)：控制器程序、变量映射与现场导入说明
- `samples/tie_points.json`：示例绑扎点
- `samples/sim_config.json`：不连接控制器的 dry-run 配置
- `samples/real_config.json`：真实控制器配置模板
- `assets/robot_models/ER25-2700/`：厂商 ER25-2700 STEP 几何资料

工程运行时不依赖 Python。上位机程序为 C++17，真实版本通过厂商 `EftSdk.dll` 通信。

## 当前能力边界

现阶段是基于施工面法向和安全高度的确定性分段规划，并在真实 SDK 模式下调用控制器 `CheckTarget` 与 `IkSolver` 做整条任务预检。厂商 STEP 几何已经获取，但尚未转换为 URDF、各连杆碰撞体或可用于规划的关节运动学模型；现场障碍物也尚未建模。因此当前版本不具备连杆级碰撞检测、RRT/PRM 避障、在线重规划，也没有完成仿真或实机验证。

7 月 20 日现场测试前，必须确认工具坐标、工件坐标、施工面参数、关节限位、工作空间、低速倍率和急停链路；先单点、低速、示教模式验证，再逐步增加点数。
