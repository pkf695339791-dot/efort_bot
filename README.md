# efort_bot

埃夫特绑扎机械臂 C++ 控制代码。

- `cpp/`：当前 C++17 实现，详见 [cpp/README.md](cpp/README.md)
- `TIE_QUEUE_SIM.XPL`：控制器端 PointC 队列执行模板
- `samples/tie_points.json`：示例绑扎点
- `samples/real_config.json`：真机连接配置

本项目运行时不依赖 Python。上位机程序由 C++ 编译，并通过埃夫特
`EftSdk.dll` 与控制器通信。
