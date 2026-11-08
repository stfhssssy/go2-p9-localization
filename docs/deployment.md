# P9 部署与复现

P9 是 Go2 L1 点云与 D435i IMU 的 Point-LIO 定位、配准点云和几何地图接口。历史证据来自 x86-64、ROS 2 Jazzy 的公开录包回放。ARM64 构建、在线 IMU 适配和比赛机器人运行是分别需要验收的阶段；镜像能够构建或程序能够启动，不等于定位精度已验证。

发布目标为私有 GitHub [`stfhssssy/go2-p9-localization`](https://github.com/stfhssssy/go2-p9-localization) 和私有 Hugging Face 数据集 [`sssssssssssssssy/go2-p9-arm64`](https://huggingface.co/datasets/sssssssssssssssy/go2-p9-arm64)。镜像文件名、SHA-256、源码版本与构建平台以该次发布清单为准。凭据只由本机登录工具读取，不写入配置、镜像层或源码。

## 比赛机器人的运行环境

随项目审阅的比赛指南描述了 Jetson AGX Orin、Ubuntu 24.04、ROS 2 Jazzy、CycloneDDS 和 rootless Docker。机器人仅在分配时段可用，无主机 sudo。现场离线：提前下载镜像、源码和需要的录包，并核对发布 SHA-256。运行入口不应执行 apt、pip、git clone 或模型下载。

基础镜像为 `go2-control:arm64-v4`。在 ARM64 构建机上可使用仓库根 Dockerfile：

```bash
docker build --platform linux/arm64 \
  --build-arg BASE_IMAGE=go2-control:arm64-v4 \
  --build-arg BUILD_JOBS=2 -t go2-p9:arm64 .
docker image inspect go2-p9:arm64 --format '{{.Os}}/{{.Architecture}}'
```

应得到 `linux/arm64`。在 x86 构建机上需要另行配置可用的 ARM64 构建器；不能把历史 x86-64 的 `pointlio_mapping` 复制到 ARM64 镜像。镜像中应包含构建后的 Point-LIO、P9 Python 包、ROS 消息/TF、rosbag2 MCAP 与 SQLite 存储插件，以及运行所需 Python 库。构建证据和目标机运行证据分别保留。

本次使用的基础镜像归档 SHA-256 为 `f2ec60225d0d07f9d66663a93005aeafca7e56979ed6a04809cf178578a2e99d`；这是下载归档的哈希，不是最终 P9 镜像的 image ID 或 registry digest。

## DDS 与唯一相机进程

在比赛机器人主机和容器中使用相同的网络设置：

```bash
test -n "$SAR_ROBOT_IFACE"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'"$SAR_ROBOT_IFACE"'"/></Interfaces></General></Domain></CycloneDDS>'
```

使用 `sar-run` 的 host 网络与主目录挂载。不要在比赛 Wi-Fi 的所有接口上使用默认 DDS 发现。容器中检查变量确实继承；必要时在 `sar-run IMAGE bash` 内重新导出以上设置。GUI 可用 `sar-run --gui`，无需给定位节点新增 GPU 依赖。

**推荐相机驱动运行在主机**，使用组织方已安装的 RSUSB librealsense 和 `realsense2_camera`：

```bash
source /opt/ros/jazzy/setup.bash
source /opt/sar/realsense_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

核对该驱动实际发布的 merged IMU、raw gyro/raw accel 话题及 frame。P9 只订阅数据，不应再次打开相机。一个 D435i 同时只由一个进程采集；不要同时启动主机驱动、另一个驱动容器或 `realsense-viewer`。只有选择在容器内运行驱动时才需要 `sar-run --camera`，并且该驱动必须使用 `FORCE_RSUSB_BACKEND=ON` 的 librealsense。比赛指南明确指出普通 `ros-jazzy-librealsense2` 或常规 pip wheel 在其内核上不能保证 IMU 工作；可见图像不能证明 IMU 可用。

## 实机定位

先复制并审阅 `config/live.example.yaml`，填入本机话题、frame、外参、时间约定及独立输出目录。不要把录包专用的约 2167 秒偏移用于实时数据。`config/p9-pointlio.yaml` 保留历史算法参数，其存在不意味着示例外参已适用于当前机器人。参见 [标定约定](calibration.md)。

```bash
sar-run go2-p9:arm64 p9 diagnose
sar-run go2-p9:arm64 p9 live --config /path/robot.yaml
```

`p9 diagnose` 默认只读，不启动估计器。`live` 的配置必须与当前传感器匹配，使用墙钟而非录包 `/clock`。启动后先静止检查输入频率、连续时间、加速度模长、Point-LIO 初始化和位姿/扫描同时间戳配对，再进行受控移动。保存本次生效参数、拒绝计数、源与输出计数、二进制哈希和地图。

该发行范围是定位/地图接口，不能仅凭 `/state_estimation` 存在就认为机身 TF、规划、足式控制和比赛安全停止链已接通。历史 P9 位姿参考点为相机 IMU；转换为 `base_link` 需要独立的、实测的机身外参。使用 `tmux` 或按比赛指南创建不依赖 SSH 会话的进程；不要配置 `restart=always` 或 `unless-stopped`。

## 离线回放

历史 P9 输入是已经把相机加速度线性插值到 merged IMU 时间戳的**派生 bag**；在线版本新增的等待双侧加速度样本逻辑不能对该数据重复应用。复制 `config/replay.example.yaml`，明确是原始数据还是派生数据，并与其 conversion manifest 匹配。

```bash
sar-run go2-p9:arm64 p9 replay \
  --bag /path/bag --config /path/replay.yaml --rate 0.5
```

回放使用 `use_sim_time=true` 和播放器 `/clock`。它们不修正两个传感器消息头的设备时钟差。原始点云 header 若尚未修正，应按该 bag 的 manifest 加一次偏移；已修正的 bag 不得再加。相机 IMU 与点云逐点相对时间保持其声明的约定。

等待播放器正常结束和队列处理完成后，在相同的运行上下文调用 `p9 save`，保留地图、轨迹与报告。输入原始计数不等于估计输出计数：初始化、输入覆盖不足、适配器拒绝和算法缺帧必须分开记录，不能把所有缺失统称初始化。新一次运行使用新输出目录。

## 验收边界

完成检查应覆盖有效输入和拒绝原因、时间单调、姿态有限性、位姿/扫描精确时间配对、TF 唯一父节点、文件完整性以及 ARM64 CPU/内存和处理积压。在线线性插值需等待未来一个加速度样本，会增加有界等待；记录这个等待与总端到端延迟。断流、重启、时钟跳变的处理也要在目标机验证。

历史 localization 回放为 1779 雷达输入、23099 相机 IMU、1771 位姿/扫描配对；这些只是该包的计数，不能写成实时运行的固定预期。宇树里程计仅是设备参考，不是 GT。构建通过、连续轨迹或地图看起来整齐均不能替代独立精度测量。
