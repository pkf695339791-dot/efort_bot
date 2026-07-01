# 埃夫特绑扎机械臂 C++ 控制程序

该目录是原 Python 原型的 C++17 迁移版，包含：

- JSON 绑扎点与配置读取
- `approach -> tie -> retreat` 三段运动队列
- 置信度、重复点、间距和工作空间检查
- 25 点分批与 `PC_INT` / `PC_BOOL` 握手
- dry-run 和真实埃夫特 SDK 两种后端
- 报警、急停、伺服状态检查及异常停止
- C++ 单元测试

控制器端继续使用仓库根目录的 `TIE_QUEUE_SIM.XPL`。当前 XPL 仍以 `mlin`
执行所有 `PointC`，尚未加入 `PointJ/mjoint` 混合运动。

## 构建 dry-run

在安装了 CMake 和 Visual Studio C++ Build Tools 的终端中执行：

```powershell
cmake -S cpp -B cpp\build
cmake --build cpp\build --config Release
ctest --test-dir cpp\build -C Release --output-on-failure
```

运行示例：

```powershell
cpp\build\Release\run_tie_queue.exe --points samples\tie_points.json
```

## 构建真实 SDK 版本

```powershell
cmake -S cpp -B cpp\build-sdk -DEFORT_WITH_SDK=ON -A x64
cmake --build cpp\build-sdk --config Release
```

SDK 版本会链接仓库中的 `EftSdk.lib`，并在构建后复制 `EftSdk.dll`。实机运行：

```powershell
cpp\build-sdk\Release\run_tie_queue.exe `
  --points samples\tie_points.json `
  --real-robot
```

首次实机运行前必须确认控制器 IP、工具名、工件坐标系、姿态、工作空间和低速倍率。

## 与视觉 C++ 集成

视觉模块只需构造 `std::vector<tie::TiePoint>`，然后调用：

```cpp
tie::RobotControlConfig config;
tie::RobotControlSystem controller(config);
controller.run(detected_points);
```

坐标标定矩阵应接入 `CoordinateManager::to_workobject_pose()`。机械臂控制权应只由
`RobotControlSystem` 持有，避免视觉线程与控制线程同时调用 SDK。

