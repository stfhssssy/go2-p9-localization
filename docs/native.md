# 从 GitHub 源码在主机原生运行

本路线在 Linux、ROS 2 Jazzy 主机上从源码编译，不需要启动 Docker。比赛 Go2 的计算机是 ARM64；其他 x86-64 Jazzy 主机也可以原生编译用于开发和录包检查，但结果、性能和最终 ELF 架构需要分别记录。不能复制历史 x86-64 二进制到 ARM64 机器人。

比赛主机没有 sudo。两个脚本都不会安装依赖、下载文件或启动相机驱动；缺少依赖时先与有权限的维护者确认，或使用已经构建好的 [Docker 路线](deployment.md)。不要在比赛机器人上执行 Dockerfile 中的系统安装操作。

## 准备依赖与源码

在本机有权限的环境或管理员准备的主机上，按仓库根 `Dockerfile` 的 apt 列表预装依赖。该列表是构建所用的版本化记录，主要包括：

- ROS 2 Jazzy 与 CycloneDDS；PCL ROS/conversions、TF2、diagnostic/std services、RViz。
- C++ 编译器、CMake、colcon、Eigen、PCL、OpenMP 和 Python 开发头文件。
- 系统 Python 的 NumPy、PyYAML；运行测试另需 pytest。
- rosbag2 MCAP 与 SQLite/default storage 插件，供离线回放使用。

主机已有 ROS 不代表所有开发包和存储插件都已安装。比赛离线运行前完成依赖和源码下载；私有 GitHub 的认证由自己的 Git/SSH 工具处理，不把 token 或 SSH 私钥写进配置文件。

```bash
git clone git@github.com:stfhssssy/go2-p9-localization.git
cd go2-p9-localization
BUILD_JOBS=2 ./scripts/build-native.sh
```

脚本从仓库根执行 `colcon build`，只选择 `point_lio_unilidar` 和 `go2_p9`，顺序构建包。默认编译并发为 2，`BUILD_JOBS` 接受 1–16 的整数；内存紧张时设为 1。它创建 Point-LIO 必需的 `src/point_lio_unilidar/Log`、`PCD` 目录，产物位于当前仓库的 `build/`、`install/`、`log/`。不要搬用另一架构或另一工作树的 build/install 缓存。

每次修改源码后重新构建。使用干净的 Jazzy 终端，避免旧 ROS 工作空间或不同 Python 环境覆盖当前依赖。脚本不修改主机的 shell 初始化文件。

## 连接比赛机器人

在相机驱动终端与 P9 终端设置相同的接口：

```bash
test -n "$SAR_ROBOT_IFACE"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'"$SAR_ROBOT_IFACE"'"/></Interfaces></General></Domain></CycloneDDS>'
```

`scripts/p9` 会 source `/opt/ros/jazzy/setup.bash` 和本仓库 `install/setup.bash`，把 `P9_HOME` 指向本仓库，再执行 `ros2 run go2_p9 p9`。如果未提供 `CYCLONEDDS_URI`，它绑定 `SAR_ROBOT_IFACE`；无此变量时仅绑定 `lo`，便于独立离线实验，不会默认在所有 Wi-Fi 接口发现机器人。已有显式 DDS 配置会被保留。

相机驱动按照 [部署文档](deployment.md) 使用主机 RSUSB 安装，在另一个终端只启动一个 `realsense2_camera` 进程，并开启 gyro、accel 与 `unite_imu_method=2`。P9 脚本不打开相机，也不启动足式控制。

## 诊断与实时定位

先创建存放配置和结果的持久目录：

```bash
mkdir -p "$HOME/p9-config" "$HOME/p9-runs"
cp config/live.example.yaml "$HOME/p9-config/robot.yaml"
./scripts/p9 diagnose --seconds 10
```

编辑 `robot.yaml`，填写该机器人的 `calibration.rotation`、`translation`、准确的标定状态、传感器 frame 与时钟设置。空外参会拒绝启动。`live-p9-experimental.yaml` 是另一个显式实验选项：它带历史拟合 R、未知 T=0 与未验证 clock=0；不能将它重命名后声称已完成机器人标定。

```bash
./scripts/p9 live --config "$HOME/p9-config/robot.yaml" \
  --output "$HOME/p9-runs/live-001"
```

该命令的输出目录必须不存在或为空；重新实验换一个目录。需要 RViz 时添加 `--rviz`。在比赛远程桌面中可按指南使用 VirtualGL 启动整个入口，让子进程 RViz 继承其环境：

```bash
vglrun ./scripts/p9 live --config "$HOME/p9-config/robot.yaml" \
  --output "$HOME/p9-runs/live-002" --rviz
```

在同一 DDS 配置的另一终端运行 `./scripts/p9 save` 可保存当前活动实例的地图。`/p9/save` 是固定服务名，因此同一 domain 内只运行一个 P9 实例。原始时间戳、原生 IMU 位姿和对应地图的解释见 [标定约定](calibration.md)。

## 离线回放与 RViz

回放应使用独立网络/domain，避免同名真实传感器同时送数据。下面例子在单机 loopback 上使用 domain 42；检查终端中没有遗留覆盖它的 DDS 配置：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
cp config/replay.example.yaml "$HOME/p9-config/replay.yaml"
./scripts/p9 replay --bag /path/to/prepared-bag \
  --config "$HOME/p9-config/replay.yaml" --rate 0.5 \
  --output "$HOME/p9-runs/replay-001" --rviz
```

`replay.example.yaml` 只对应历史 localization 派生 bag。它使用已插值的 merged IMU，cloud offset 为该包的 2167.645092767 秒；其他 bag 需单独审计配置。`raw_linear` 模式要求 bag 包含原始 gyro、accel 和正确的静态 TF，不能把缺少 raw 数据的派生 bag 当原始包使用。

默认播放结束后保留 RViz 和节点。调用 `./scripts/p9 save` 或正常 Ctrl-C 后保存结果；也可加 `--exit-on-finish`，EOF 后等待 5 秒再退出。最终检查输入计数、拒绝原因和尾部时间；自动退出不能证明处理没有积压。回放使用 `/clock`，live 使用墙钟，这两个选择不替代消息头时间基准校准。

所有原生结果直接保存在指定主机目录，无容器卷迁移步骤。需要跨 SSH 断线保留运行时，按比赛指南使用 `tmux`；时段结束仍会停止进程。原生构建通过和运行完成只证明接口可用，不是比赛真机精度或导航闭环验收。
