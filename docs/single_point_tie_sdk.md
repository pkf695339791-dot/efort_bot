# Efort SDK 单点捆扎程序

程序入口为 `cpp/src/single_point_tie.cpp`，执行流程如下：

```text
连接与安全检查
  -> CheckTarget + IkSolver 预检接近点和捆扎点
  -> MJOINT 到接近点
  -> MLIN 到捆扎点
  -> 捆扎机数字输出 DO 脉冲
  -> MLIN 原路撤回接近点
```

## 坐标约定

`--pose X Y Z A B C` 直接填写示教器显示的 TCP 笛卡尔坐标，单位为 mm 和度。因为这些数据位于机器人基座坐标系，程序默认使用 `wobj0`。`--tool` 必须填写示教该点时使用的、已经正确标定的工具名称；默认值是 `tool_tie`。

`--cfg CFGX CFG1 CFG4 CFG6` 应填写示教点保存的构型参数，避免笛卡尔点存在多组逆解时选到另一种机械臂姿态。若示教器没有记录这些值，应重新读取完整点位数据，不要凭经验猜测。

## 读取当前完整位姿

先用示教器低速移动到目标点，并保持实际运行时希望采用的关节姿态。然后运行只读工具：

```powershell
.\cpp\build-sdk-msvc\read_current_pose.exe --ip 控制器实际IP
```

程序输出基座坐标系下的当前 TCP `XYZABC`、`CFGX/CFG1/CFG4/CFG6`、`J1~J6`、外部轴以及当前工具和用户坐标系，并生成可直接复制到单点捆扎程序的 `--pose` 和 `--cfg` 两行参数。该工具不使能 API 运动控制、不控制伺服、不运动机械臂，也不写数字输出。

厂家 `EftSdk.dll` 是 Release 版 C++ SDK，并且接口跨 DLL 使用了 `std::string`。实机程序必须使用 `build-sdk-msvc` 中的 Release 可执行文件；不要运行 `build-sdk-debug` 中依赖 `/MDd` 的版本，否则可能在 SDK 调用处发生 `0xC0000005` 访问冲突。

若日志明确显示 `Sdk Version: 280`、`Ob Version: 200`、`Version Mismatch` 和错误码 `10035`，可在只读或无运动预检命令后添加 `--skip-version-check`。本地 SDK 自带的 C30 示例同样以 `verCheck=false` 连接。该参数只跳过连接版本门禁，不证明所有运动和 IO 接口都兼容；真正运动前应由埃夫特确认 SDK 2.8 与现场 OBB 2.00 的兼容性。

每段运动使用 `WaitCartPosition` 阻塞等待到位，默认单段超时 15 秒，可通过 `--wait-timeout SEC` 调整。不要将该值设为 0；SDK 的默认 `waitTime=0.0` 只会立即检查一次，机械臂尚未到位时会返回错误码 `10032`。

当前版本暂时禁用了捆扎头数字输出：不会读取、置高或置低任何 DO。可以重复传入 `--pose`，程序会先检查所有目标点可达性，再按命令行顺序使用 `MJOINT` 逐点运动并等待到位；不生成接近点，也不执行下探或退回。所有目标共用一次 `--cfg`。完成捆扎头 IO 接线、编号、电压、极性和脉冲时长验证后，再恢复源码中由 `#if 0` 包围的 IO 代码。

带 `--execute` 时，如果伺服尚未上电，程序会在无报警、无急停且目标可达性检查通过后调用 SDK `PowerOn()`，并等待最多 10 秒确认伺服状态。该调用不能绕过急停、安全门、安全回路或控制器自动/远程模式要求；程序结束后不会自动调用 `PowerOff()`。

默认接近方向是基座坐标系 `+Z`，距离为 50 mm。若捆扎工具应沿其他方向进退，用 `--approach NX NY NZ D` 指定。例如沿基座 `-Y` 方向退开 80 mm：`--approach 0 -1 0 80`。

## 构建

在项目根目录执行：

```powershell
cmake -S cpp -B cpp\build-sdk-single -A x64 `
  -DEFORT_WITH_SDK=ON `
  "-DEFORT_SDK_ROOT=$PWD\埃弗顿机器人\SDK V2.8\V2.8.0"
cmake --build cpp\build-sdk-single --config Release --target run_single_point_tie
```

SDK 的运行时 DLL 会自动复制到可执行文件目录。

## 先做预检

下面的数字仅用于说明参数格式，必须换成现场实测值。没有 `--execute` 时，程序会连接控制器并执行状态、`CheckTarget` 和 `IkSolver` 预检，但不会运动，也不会切换 DO：

```powershell
cpp\build-sdk-single\Release\run_single_point_tie.exe `
  --ip 192.168.1.12 `
  --pose 500 0 350 180 0 180 `
  --cfg 0 0 0 0 `
  --tool tool_tie `
  --wobj wobj0 `
  --approach 0 0 1 50
```

## 低速执行一次

确认急停、软限位、工具 TCP、捆扎机 DO 编号、进退方向以及周边无人员后，再补充真实 DO 编号和 `--execute`：

```powershell
cpp\build-sdk-single\Release\run_single_point_tie.exe `
  --ip 192.168.1.12 `
  --pose 500 0 350 180 0 180 `
  --cfg 0 0 0 0 `
  --tool tool_tie `
  --wobj wobj0 `
  --approach 0 0 1 50 `
  --do-index 0 `
  --pulse-ms 500 `
  --joint-speed 5 `
  --linear-speed 5 `
  --global-speed 10 `
  --execute
```

程序不会自动清除报警或自动上电。执行模式要求机械臂已经手动上电且无报警、无急停；异常时会尝试关闭捆扎 DO、清除运动队列并释放 API 控制权。

> 注意：SDK 的 DO 编号是否从 0 开始必须以当前控制器 IO 映射为准。首次联调应先断开捆扎动力，单独观察 DO 指示灯，确认编号和有效电平后再接执行机构。
