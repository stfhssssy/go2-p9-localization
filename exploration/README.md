# P9 calibrated localization, mapping and TARE exploration

This release adds a separate `p9explore` entrypoint and an optional attended
`p9explore-control` transport to the retained P9/FAR image. The original `p9` and
`p9nav` commands and their profiles remain available. Do not launch FAR and TARE
on the same ROS domain at the same time: they share the local-planning topics.

```text
Go2 LiDAR + raw D435i gyro/accel
  → linear IMU merging + C extrinsics + B1 gyro correction
  → Point-LIO → accumulated map + terrain
  → TARE exploration → localPlanner/pathFollower → exploration guard
  → /p9/exploration/cmd_vel (isolated by default)
  → optional attended bridge → Unitree Sport Move/StopMove
```

The default live profile is `/opt/p9/config/p9nav-live.yaml`. Its SHA256 is
`663b31f4cd230e06b842e2d83ab5f4d00211420a70e91a28e7ac7736c35c2984`.
It contains the selected C LiDAR-to-camera-IMU rotation and translation and B1
static gyroscope correction, applied once in the adapter. Acceleration correction
is zero. This is not a completed six-position intrinsic calibration. Current-boot
clock offsets remain unverified zero initial values and the body transform is
derived from a nominal body installation. Check these on the current hardware.
Never copy the public bag's approximately 2167-second offset into live operation.

## 明天真机测试的步骤

这次发布提供真机入口与可选运动桥；软件/仿真验证不等于真机闭环验收。
之前 TARE 自动返航在隔板附近未成功，因此本版 **live 在探索完成时停车并锁定**，
不执行自动返航。现场测试使用清楚划定的小范围、在场观察人员和独立停止手段。

### 1. 导入发布的 ARM64 镜像

从本仓库发布清单对应的 Hugging Face 目录下载镜像与校验文件：
`releases/p9-explore-20261112/`，然后：

```bash
sha256sum -c SHA256SUMS
zstd -dc go2-p9-explore-arm64-20261112.tar.zst | docker load
sar-run go2-p9-explore:arm64-20261112 p9explore --help
```

校验目录必须包含 SHA256SUMS 列出的全部发布文件；只下载镜像时，可从清单取出镜像行校验。
镜像不启动第二个 RealSense 驱动。使用赛事提供的现有相机采集服务，确认有
`/camera/camera/gyro/sample`、`/camera/camera/accel/sample`、内部静态 TF
及 `/utlidar/cloud`。DDS 使用赛事分配的 `$SAR_ROBOT_IFACE` 和 ROS domain，
Tailscale 负责远程终端连接，不代替机器人的传感器网卡。

### 2. 配置和仅运行定位、建图、探索规划

按赛事说明，`sar-run` 应挂载当前用户 home 并传递 DDS 环境；本轮没有连接真机验证该启动器。先导出一份可编辑配置：

```bash
mkdir -p "$HOME/p9-explore/config" "$HOME/p9-explore/runs"
sar-run go2-p9-explore:arm64-20261112 cp /opt/p9/config/p9nav-live.yaml \
  "$HOME/p9-explore/config/live.yaml"
```

核对这份配置的当前话题、frame、时钟以及机身变换后，在 TurboVNC 终端运行：

```bash
p9_run="$HOME/p9-explore/runs/$(date -u +%Y%m%dT%H%M%SZ)"
sar-run --gui go2-p9-explore:arm64-20261112 p9explore live \
  --config "$HOME/p9-explore/config/live.yaml" \
  --output "$p9_run" --boundary -2 2 -2 2 --rviz
```

上面 `--boundary -2 2 -2 2` 只是首测示例：`camera_init` 坐标系中
X、Y 各 -2 到 2 米的矩形。它不是根据场地自动推断的安全边界，必须按照 RViz 中
真实可测试区域修改；边界发布给 TARE，并由输出守卫检查。启动后先静止初始化并检查地图。
镜像不会自动起立，也不会因为启动此入口而让机器人移动。

第二个同 DDS 环境终端：

```bash
sar-run go2-p9-explore:arm64-20261112 p9explore status
sar-run go2-p9-explore:arm64-20261112 p9explore start
```

`start` 只有在定位、点云、地形和 tracking 输入新鲜时才被接受。它开始产生探索路线和
隔离的速度建议；此时仍没有真机运动桥。传感器断流或 tracking 异常会停车并锁定，
live 不会自动恢复运动。每次运行都保存 `effective-exploration.json`、有效 YAML 和数据记录。

### 3. 现场确认后，单独启用运动桥

先通过已有控制方式让狗站稳，然后退出其他会发布运动指令的控制程序。
在有交互终端的会话（远程 SSH 使用 `ssh -t`）中执行：

```bash
sar-run go2-p9-explore:arm64-20261112 p9explore-control --enable-motion
```

- 启动只发停止指令。按 **g** 请求授权；收到匹配的 StopMove 成功应答后才可授权。
- **持续按住 h** 才允许运动；松开后许可到期会停止，恢复输入不会自动重新授权。
- **空格** 停止并解除授权；**q / Ctrl+C** 退出并尝试停止。
- 默认运动桥限速：前后 0.15 m/s、左右 0.10 m/s、旋转 0.25 rad/s；
  上游探索守卫还有独立限速。实际输出取两级限速后的结果。
- 不带 `--enable-motion` 的 `p9explore-control` 是只读观察，不创建 Sport 请求发布端。

进一步说明与接口依据见 [control/README.md](control/README.md)。软件停止不能覆盖
网络失联、进程被强杀或另一控制器持续发命令，现场仍须保留独立停止方式。

正常停止探索：

```bash
sar-run go2-p9-explore:arm64-20261112 p9explore stop
```

完成或停止后确认机体静止，再退出各终端。地图也可用同 DDS 环境的 `p9 save` 保存。

## Replay, simulation and validation

`p9explore replay` requires `--bag`, a matching replay profile and an explicit
boundary. It never enables the hardware bridge. `p9explore sim` expects a separate
Gazebo instance and the P9 simulation sensor adapter; `--sim-command-output` is
accepted only in sim mode. `--allow-return-home` is also simulation-only and is
explicitly an unaccepted return experiment. Ground truth is evaluation-only.

The AMD64 validation image is built from the retained simulation base. The
ARM64 deployment image does not bundle the external Gazebo/CHAMP workspace.
Both use the same TARE source, integration code and OR-Tools 9.8.3296, compiled
for their own architecture. Live sensor parameters and simulated sensor parameters
are deliberately separate.

Read the published `validation.json` and `release.json` for actual performed
checks, exact source commit, image ID, architecture and limitations. No hardware
test is claimed by a successful build, protocol mock or simulation run.

## Build and provenance

See [build/BUILDING.md](build/BUILDING.md). Download pinned inputs with
`python3 build/fetch_build_inputs.py`; archives are excluded from Git but retained
in the build context and image for traceability. Build `build/Dockerfile.tare`
with the matching base image and platform. Official TARE commit:
`b6760adb4280b379567c76f4e684ce628e28ef71`.

The algorithm keeps the upstream TSP solver; the original bundled x86 library is
replaced with Google's same-version architecture-specific OR-Tools SDK. Existing
patches adapt frame labels, QoS, initial hold and read-only diagnostics. The P9
map centroid-export optimization preserves voxel centroids and does not change
Point-LIO estimates. Original P9/FAR source files are retained; the image records
the mapping patch and TARE build provenance separately.

This release also fixes the upstream ROS 2 `kUseFrontier` declaration/read defect.
An undeclared switch previously left two boolean members uninitialized and could
make TARE end exploration without choosing a route. Both enabled and disabled
parameter values are checked against the actual binary; details are in
`validation/frontier-diagnosis/README.txt`.
