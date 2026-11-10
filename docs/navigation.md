# P9 B1 定位、地图与 FAR 规划

本发行在独立入口 `p9nav` 中连接 Point-LIO、几何地图、Zhang Ji 地形分析、FAR 路线规划及局部路径跟随。原 `p9 replay/live` 入口和历史 P9 配置仍保留；旧 GitHub main、旧镜像及旧 HF 发布文件不由本发行替换。新源码分支为 `p9-navigation-20261110`，镜像为 `go2-p9-nav:arm64-20261110`，HF 归档目标是既有数据集中的 `releases/p9-nav-20261110/`，保留用户当前设置的可见性。实际发布版本与验证结果以对应 `release.json` 和验证清单为准。

## 数据流程与坐标

```text
Go2 雷达 + D435i gyro/accel
          ↓ 话题、逐点时间、时钟、IMU 合并和 B1 gyro 补偿
       Point-LIO
          ↓ 同一时刻的位姿 + 已配准的世界系点云
     P9 地图累积器
          ├─ /p9/map、/p9/path、保存地图与轨迹
          └─ /state_estimation、/registered_scan
                    ↓ 地形分析与扩展地形分析
                    ↓ FAR：目标点 → 路线／中间航点
                    ↓ local_planner、pathFollower
                 隔离的规划速度建议
```

Point-LIO 的原生位姿参考点是 IMU。P9 使用配置中的 `T_imu_base` 得到 `base_link` 位姿，并将它提供给规划。`/registered_scan` 已在 `camera_init` 世界坐标系中，不能再用当前机身位姿乘一次。地图累积与地形输入保持同一时间戳的点云／位姿配对；地形分析不是第二个定位器。

规划使用同一个 `camera_init` 世界坐标系。局部规划器的数学模型只使用 yaw，导航守卫按 `/state_estimation` 的同一个整数时间戳广播 `camera_init → vehicle`：位置相同，姿态只保留 yaw。它不是 `base_link` 的静态身份别名；IMU→`base_link` 的变换仍由地图接口独占发布。真实机器人 profile 中的机身外参来自历史名义变换，尚未实测验收。不要将相机光学坐标系的方向直接当作机身前向。

## 三种入口的用途

| 入口 | 输入与时钟 | 可验证的内容 | 运动输出 |
|---|---|---|---|
| `p9nav replay` | 指定 bag、播放器 `/clock` | 数据适配、定位建图、已录轨迹上接收到的环境与规划接口 | 默认仅隔离的规划建议；不能改变 bag 中的运动 |
| `p9nav live` | 当前传感器话题、墙钟 | 当前机器的在线定位、地图与规划建议 | 默认不发送真狗动作 |
| `p9nav sim` | 外部仿真器及其 `/clock` | 当仿真传感器和控制接口接好后，可进行闭环导航验证 | 仅显式 `--sim-command-output` 才允许输出仿真 `/cmd_vel` |

`sim` 入口本身不是 Gazebo 安装器，也不会创建比赛场景。仿真传感器模型、消息桥和传感器适配器必须与 `config/p9nav-sim.yaml` 的说明相符；原始 Gazebo 点云没有所需逐点时间字段，不能绕过适配直接当作真实 Go2 数据。

回放不能验证目标引导下的闭环避障：即使规划选择了另一条路线，bag 仍播放录制时走过的路线。仿真闭环成功也不代表真实狗的机身外参、时钟、吞吐和运动控制已经验收。

## B1 配置究竟改了什么

真实数据的 `config/p9nav-replay.yaml` 和 `config/p9nav-live.yaml` 使用固定 C 组 LiDAR→IMU 的 `R/t` 候选，并增加 B1 静态陀螺仪补偿：

```text
omega_corrected = omega_input - b_g
b_g = [0.00028568964388210946,
      -0.0018381138448739292,
       0.0005906477368247819] rad/s
```

该候选从已保存的静止候选段冻结，不在目标 bag 上重新拟合。补偿只在适配器应用一次；不再修改 prepared bag，也不将同一常数重复写入滤波器初值。加速度没有乘新的尺度或额外加入静态偏置。原 P9 的配置文件保持原样，选用导航 profile 才选用此组参数。

公开 bag 与近期标定来自同一条狗，安装关系一直未变。但 B1 是当时的静态零偏候选，随开机／温度变化的稳定性尚未验收；C 组 `R/t` 也不是完整外参精度认证。该发行没有完成 D435i 的完整六面内参标定、没有向 EEPROM 写标定表，也不将地图外观当作 GT 精度。

回放 profile 的 `2167.645092767 s` 只用于指定的公开 `localization` 派生包，修正点云 header；相机线性合并输入不应重复插值或重复平移时钟。不能将该偏移套到 two-laps、其他 bag 或 live。

live profile 使用原始 `/camera/camera/gyro/sample` 与 `/camera/camera/accel/sample` 做双侧线性插值，并检查内部静态 TF；不依赖之前实际观察到的 COPY merged IMU。其时钟偏移默认零只是待测初值，必须针对当前开机确认。数据年龄、倒退时间戳、加速度覆盖和异常输入拒绝逻辑仍生效。相机采集进程由主机或外部容器运行，本入口不会启动第二个 RealSense 驱动。

仿真 profile 使用模型声明的虚拟传感器安装、零 gyro 偏置和零 epoch 偏移。真实机器人的 B1 常数不适用于合成 IMU。

## 回放与目标输入

运行环境需已导入新镜像。以下 `/path` 位置替换为 `sar-run` 已挂载的主机路径，输出目录必须不存在或为空；每次试验使用新目录。回放在独立 DDS domain 和 loopback 中运行：

```bash
sar-run go2-p9-nav:arm64-20261110 env ROS_DOMAIN_ID=42 \
  CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>' \
  p9nav replay --bag /path/prepared-localization-bag \
  --config /opt/p9/config/p9nav-replay.yaml \
  --output /path/new-navigation-replay --rate 0.25 --rviz
```

在具有相同 DDS domain/interface 的另一终端提交 XY 目标。目标坐标的单位为米，默认坐标系是 `camera_init`，不会自动将像素、经纬度或另一张地图的坐标转换过来：

```bash
p9nav goal 1.0 0.5 --frame camera_init
p9nav stop
```

这些命令需在同一 ROS 环境／容器上下文执行。`p9nav stop` 停止本规划接口的目标／速度输出，不替代独立机器人控制系统的急停。地图可通过原有 `p9 save` 接口保存，同样需要匹配运行中的 domain/interface。

## 在线和仿真

在线前先复制 live profile 到挂载目录，填写并核对当前话题、frame、时钟及机身安装关系。DDS 使用比赛分配的 `SAR_ROBOT_IFACE` 和 domain 0。审阅配置后启动：

```bash
sar-run --gui go2-p9-nav:arm64-20261110 p9nav live \
  --config /path/p9nav-live-robot.yaml \
  --output /path/new-navigation-live --rviz
```

默认只有外部目标驱动规划；不会主动探索，也不会自动起立／转圈。FAR 与局部路径跟随的建议速度被隔离到 `/p9/navigation/cmd_vel`。本发行从 `pathFollower` 移除了直接 Unitree SportClient/API 发送，不能仅因规划输出了非零速度就声称狗已受控行走。

仿真器和传感器适配就绪后，可启动：

```bash
p9nav sim --config /opt/p9/config/p9nav-sim.yaml \
  --output /path/new-navigation-sim --rviz
```

仅在独立的仿真 DDS 上下文中为该命令添加 `--sim-command-output`，才将受守卫约束的命令送给仿真器 `/cmd_vel`。`replay` 和 `live` 不接受用该开关启用真狗控制。尚未实现／验收真实运动桥时，真实狗的自主导航仍未完成。

规划守卫依据输入新鲜度、Point-LIO tracking、图／路径、目标和命令的状态限制输出；当前阈值在 `go2_p9_navigation/config/navigation.yaml`。守卫拒绝或归零应先排查传感器与时间问题，不应通过关闭检查使机器人继续运动。

## 本发行的 FAR 启动修复

接入回放时发现两项启动缺陷，代码已经针对它们作了修改，验证结果需要单独记录：

- 地形 KD-tree 尚未建立有效索引时，执行地面邻域查询可能崩溃。新增 `terrain_kdtree.h` 保护空索引查询，并给两个地面查询调用点接入该保护；随源码提供确定性的 CTest 回归用例。保护本身不意味着有地面数据，不应把空查询结果当作已确认可通行区域。
- FAR 的 `/runtime` 消息只表示主循环运行，不等于导航图已经初始化。此前据此过早接受目标可能丢失目标。新增 `/p9/navigation/far_ready`，使用 transient-local Bool 明确表达图就绪状态；初始化、重置或失效时为 false。守卫同时检查其就绪状态与新鲜度、runtime 状态，以及发送目标前的 FAR 订阅连接，失效时清除目标。

以上说明记录的是发现的问题与代码改动，不代表本次 ARM64 构建、回放、闭环仿真或真狗导航已经通过。各项执行结果以对应版本的验证证据为准。

## 源码、构建与验证记录

Point-LIO 保留历史 P9 源码记录，并包含已单独测试的重力对齐 SO(3) 修复。规划的实际来源是服务器工作目录导出的 `planner-source-reference.tar`，SHA256 为 `885a46f0a1c1e558fe99a5c3d85c738b21847fe054ea3bf10ee6ac3fc14efac0`。服务器副本没有 Git 元数据；`jizhang-cmu/autonomy_stack_go2` 的 `43d5f54b389b251713f0097893c30fa76c870d54` 仅作为底层版本对照，不能把服务器副本称作该提交的干净 checkout。本发行保留服务器的 Jazzy 适配，再加入接口和移除直接机器人动作发送等改动。`provenance/planning-upstream.json` 同时记录 Git HEAD、服务器原始快照和最终文件的哈希及差异。

`provenance/source-manifest.json` 和前序 `experiment-delta.json` 保持原字节。所有源文件和配置完成后，维护者显式运行：

```bash
python3 scripts/freeze-navigation-provenance.py --freeze \
  --upstream-root /path/reviewed-autonomy_stack_go2 \
  --server-source-root /path/extracted-server-snapshot \
  --server-archive /path/planner-source-reference.tar \
  --server-workspace /data/t9/liuxu/go2-autonomy/src/autonomy_stack_go2
python3 scripts/freeze-navigation-provenance.py --check
```

冻结器记录完整 runtime、文档／许可证、五个上游包的 Git HEAD 文件哈希和本地差异，并将清单哈希固定进发布测试；任何后续未记录变更均会失败。冻结仅表示字节可追溯，不是算法精度验收。

发布验证分别记录源码测试、ARM64 构建、bag 输入完整性、地图／位姿配对、规划话题链、仿真闭环和真实机器人运行。不得把旧 P9 的构建／回放结果写成这次新镜像的结果，不得把 QEMU 的速度当作 Jetson 性能。真实内参、可靠机身外参、在线时钟与真机闭环导航仍需各自验证。
