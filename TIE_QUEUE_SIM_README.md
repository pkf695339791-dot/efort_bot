# TIE_QUEUE_SIM 控制器程序说明

`TIE_QUEUE_SIM.XPL` 是可导入墨斗/控制器的混合运动双缓冲模板，`TIE_QUEUE_SIM.pgm` 是便于审阅的等价逻辑。上位机同时下发笛卡尔目标 `PointC`、关节目标 `PointJ` 和运动类型；控制器按槽位选择 `MJOINT` 或 `MLIN`。

## 执行序列

```text
当前位置 -> MJOINT SafeEntry
SafeEntry -> MLIN Approach -> MLIN Tie -> MLIN Retreat -> MLIN SafeExit
SafeExit -> MJOINT 下一个 SafeEntry
```

类型码 `1` 读取 `PC_POINTJ[i]` 并执行 `MJOINT(..., v100perc, fine, tool1)`；类型码 `0` 读取 `PC_POINTC[i]`，根据局部速度档执行 `MLIN(..., v100/v800, fine, tool1)`。

## 变量映射

| 变量 | 方向 | 含义 |
|---|---|---|
| `PC_INT[0]` | PC -> 控制器 | 本次运动段总数 |
| `PC_INT[1]` | PC -> 控制器 | MJOINT 速度档，仅支持 `100` |
| `PC_INT[2]` | PC -> 控制器 | MLIN 速度档，支持 `100`、`800` |
| `PC_INT[3]` | 控制器 -> PC | 错误码 |
| `PC_INT[10..59]` | PC -> 控制器 | 与槽位对应的运动类型，`0=MLIN`、`1=MJOINT` |
| `PC_POINTC[0..49]` | PC -> 控制器 | 笛卡尔目标双缓冲 |
| `PC_POINTJ[0..49]` | PC -> 控制器 | 关节目标双缓冲 |
| `PC_BOOL[0]` | 控制器 -> PC | 整条队列完成 |
| `PC_BOOL[1]` | 控制器 -> PC | A 缓冲区 `0..24` 可重填 |
| `PC_BOOL[2]` | 控制器 -> PC | B 缓冲区 `25..49` 可重填 |
| `PC_BOOL[3]` | PC -> 控制器 | 初始缓冲已装载，开始执行 |
| `PC_BOOL[4]` | PC -> 控制器 | 请求停止 |
| `PC_BOOL[5]` | 控制器 -> PC | 控制器程序错误 |

错误码：`9001` 表示运动类型无效，`9002` 表示速度档位无效。控制器置位 `PC_BOOL[5]` 后，上位机读取 `PC_INT[3]` 并终止任务。

## 导入与现场检查

1. 在墨斗 IDE 中导入 `TIE_QUEUE_SIM.XPL`；如控制器版本不兼容，以同版本厂商示例的 XML 结构重新保存。
2. 将模板中的 `tool1`、`wobj_cvy` 改为现场已标定的工具和工件坐标名称，或使上位机配置与模板保持一致。
3. 检查控制器已有命名速度对象 `v100perc`、`v100`、`v800` 和 `fine`。
4. 确认 50 个 PointC/PointJ 槽位及 `PC_INT[10..59]` 没有与其他程序冲突。
5. 先使用 `samples/sim_config.json` 完成 C++ dry-run；这不是机器人仿真。
6. 现场首次联调只保留一个绑扎点，以示教/低速倍率验证 SafeEntry、Approach、Tie、Retreat、SafeExit 的方向和姿态，再逐步增加点数。

## 安全边界

- SDK 真实模式会在下发运动前使用控制器 `CheckTarget` 和 `IkSolver` 预检，但这不等于连杆级碰撞检测。
- 当前没有 URDF、连杆碰撞体或现场障碍物几何，未实现 RRT/PRM 或在线避障。
- 末端绑扎 DO/DI 工艺逻辑尚未加入模板；`Tie` 段只负责运动到作业位。
- 本模板尚未完成仿真验证或实机验证，必须按现场安全规程低速、单点、有人监护地测试。
