# TIE_QUEUE_SIM.XPL 使用说明

`TIE_QUEUE_SIM.XPL` 是一个给墨斗/控制器使用的半控队列程序模板，基于 SDK 示例 `DEMO_TRACK.XPL` 改名整理。

它不是单独硬编码轨迹的空跑程序。它需要上位机先通过 SDK 下发 `PointC` 点位，然后它在控制器里读取点位并运动。

## 运行链路

```text
上位机 Python/C++ 程序
  -> SetPointCVector 下发 PointC 点位
  -> PC_INT[0] 写入点位总数
  -> PC_BOOL[3] = true 通知开始

控制器 TIE_QUEUE_SIM.XPL
  -> 等待 PC_BOOL[3]
  -> comutil.GetPC_POINTC 读取点位
  -> mlin 直线运动到点
  -> PC_BOOL[1]/PC_BOOL[2] 请求下一批缓冲
  -> PC_BOOL[0] 表示执行结束
```

## 变量约定

- `comutil.PC_INT[0]`: 点位总数
- `comutil.PC_BOOL[3]`: 上位机置位，开始运行
- `comutil.PC_BOOL[1]`: XPL 请求上位机发送前半缓冲
- `comutil.PC_BOOL[2]`: XPL 请求上位机发送后半缓冲
- `comutil.PC_BOOL[0]`: XPL 置位，表示队列结束
- `comutil.PC_POINTC[0..49]`: 控制器侧 PointC 缓冲

## 使用步骤

1. 在墨斗 IDE 中打开或导入 `TIE_QUEUE_SIM.XPL`。
2. 检查程序里的工具坐标和工件坐标：
   - 当前工具名: `tool1`
   - 当前工件坐标: `wobj_cvy`
3. 如果现场使用的是 `tool_tie` / `wobj_rebar`，需要在墨斗里把 `tool1` 和 `wobj_cvy` 改成对应名字。
4. 上传/保存到控制器。
5. 控制器上伺服，低速/仿真模式运行该 XPL 程序。
6. 电脑运行上位机程序，向控制器下发点位并启动。

## 和当前 Python 原型配合

当前 Python 原型默认是 dry-run，不会连接真实机器人。

真实联调时才使用：

```powershell
python run_tie_queue.py --points samples\tie_points.json --real-robot
```

第一次联调建议只放 3 个点，也就是一个绑扎点的：

```text
approach -> tie -> retreat
```

确认方向、姿态、速度、坐标系都正确后，再增加点数。

## 注意

- 该模板里的运动指令是 `mlin`，速度是 `v800`，zone 是 `fine`。
- 真实机械臂首次运行前请把速度倍率调低。
- 如果墨斗打开后提示语法或版本不兼容，请用墨斗重新保存一次，或以厂家 `DEMO_TRACK.XPL` 为模板手工替换标题和坐标系。
- 末端绑扎 IO 尚未加入本模板。后续可以在每三个点中的第二个点后插入 DO/DI 逻辑。
