# Go2 P9 localization and geometry mapping

独立 ROS 2 Jazzy / ARM64 发行包，基于 `go2-control:arm64-v4`，包含实际历史 P9 使用的 Point-LIO 源码快照、新的传感器接口、地图累积器和 RViz 配置。

```
Go2 /utlidar/cloud + D435i gyro/acceleration
  -> P9 时间/字段/数据有效性接口 -> Point-LIO
  -> camera_init 下的 registered scan + 同时刻 IMU 原点位姿
  -> 5 cm 体素累积地图 /p9/map + /p9/path -> RViz / PCD / CSV
```

不使用 `robot_odom` 作为估计输入。相机图像不参与。已经变换到世界系的点云不再次乘里程计。默认不启动相机、规划器或机器人运动控制。

## 两个入口

下载发行镜像后 `docker load -i go2-p9-arm64.tar`（若下载的是 `.tar.zst`，先 `zstd -d`），镜像名与 SHA256 以同版本 `release.json` / `SHA256SUMS` 为准。

```bash
# 实机：已有主机相机驱动，且 unite_imu_method=2；domain 0，绑定 SAR_ROBOT_IFACE。
# SAR_ROBOT_IFACE 由比赛环境提供，勿自行猜测网络接口。
sar-run go2-p9:arm64 p9 diagnose
sar-run --gui go2-p9:arm64 p9 live --config "$HOME/p9/robot.yaml" \
  --output "$HOME/p9/runs/live-001" --rviz
# 另一终端保存实机运行，保持同一 domain 0 / 机器人接口：
sar-run go2-p9:arm64 p9 save

# 回放：隔离到 domain 42 + loopback，避免与真机同名话题混流。
# 历史 localization 已线性插值的派生包，使用该包专用配置。
sar-run --gui go2-p9:arm64 env ROS_DOMAIN_ID=42 \
  CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>' \
  p9 replay --bag "$HOME/bags/localization-linear" \
  --config /opt/p9/config/replay.example.yaml --rate 0.5 \
  --output "$HOME/p9/runs/replay-001" --rviz
# 另一终端保存该回放，必须使用同一 domain 42 / loopback：
sar-run go2-p9:arm64 env ROS_DOMAIN_ID=42 \
  CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>' \
  p9 save
```

回放命令中的 `env` 只覆盖该容器进程的 DDS 设置，不改变实机相机驱动的 domain 0。不要在机器人 domain 0 上播放历史 bag；同一回放 domain 内只运行一个 P9 实例。

`config/live.example.yaml` 需要填写本机测量的 LiDAR→相机 IMU 旋转和平移。`config/live-p9-experimental.yaml` 提供旧 P9 的实验旋转和假设零平移，时间偏移为零；它**不是比赛真机完成标定的证明**。`replay.example.yaml` 中约 2167 秒偏移只属于那份公开 localization 派生包，不用于其他 bag 或实时数据。所有启动必须明确选择配置，默认入口仅做 10 秒只读诊断。

如果直接回放原始 bag，把 `sensor.imu_mode` 改成 `raw_linear`；默认订阅相机 `gyro/sample`、`accel/sample`，同时需要原始 `/tf_static`。接口仅在 TF 证实原始 IMU 轴相同时重用合并 IMU frame，并等待双侧加速度样本线性插值，不外推。已有派生 IMU 或 RealSense `unite_imu_method=2` 使用 `merged_linear`，消息原样验证转发。

## 产物和检查

输出目录必须为空。有效配置、ROS 日志、`trajectory.csv`、`mapping_status.json`、`mapping_summary.json` 与 `PCD/map.pcd` 存入 `--output`。Ctrl-C 正常退出或 `p9 save` 保存地图；默认回放结束后保留 RViz，`--exit-on-finish` 可在 EOF 后退出。使用容器挂载的持久目录，不能只保存在临时容器可写层。

地图与轨迹默认参考系是 `camera_init`，位姿参考点是相机 IMU `aft_mapped`；没有测量 IMU→机身外参时，不把它误称为 `base_link`。输出 `/p9/input_diagnostics` 和 `/p9/mapping_status` 分别用于排查输入与配对问题。映射容量或配对错误会明确停止接收并报 FAULT，不以静默丢弃隐藏数据缺失。

历史录包回放、ARM64 镜像构建/测试、比赛真机效果是三个独立验收层次。无独立 GT，地图整齐不能证明绝对定位精度。`use_imu_as_input=false` 是此版 Point-LIO 的 IMU 测量更新模式，**并非关闭 IMU**。

## 构建与验证

```bash
# 首先导入发布说明所列、SHA256 已核验的 go2-control:arm64-v4 基础镜像。
docker build --platform linux/arm64 --build-arg BUILD_JOBS=2 -t go2-p9:arm64 .
docker image inspect go2-p9:arm64 --format '{{.Os}}/{{.Architecture}}'
# 在已安装 numpy/PyYAML 的源码环境执行纯合同测试：
PYTHONPATH=src/go2_p9 python3 -m unittest discover -s tests -p 'test_*.py' -v
```

x86 主机需 ARM64 QEMU/binfmt 或 ARM64 远端构建器；不得复用 x86 编译的历史二进制。运行入口不联网安装依赖。源码、配置和构建依赖均随镜像部署，不依赖作者或本地实验目录挂载。

原生 GitHub 路线使用 `scripts/build-native.sh` 构建、`scripts/p9` 启动；见 [原生安装](docs/native.md)。

完整说明：[部署](docs/deployment.md) · [标定](docs/calibration.md) · [来源](docs/provenance.md) · [第三方许可](THIRD_PARTY_NOTICES.md)。保留上游完整源文件和许可证；本项目新代码 GPL-2.0-only。实际 Point-LIO 源码哈希见 `provenance/source-manifest.json`，镜像内编译产物哈希见 `/opt/p9/provenance/binary.sha256`。
