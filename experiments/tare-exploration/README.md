# P9 + TARE 自动探索仿真实验

实际仿真证据与各次修复记录见 [RESULTS.md](RESULTS.md)。

本目录保存独立的 AMD64 Gazebo 实验封装。**TARE 替代 FAR 负责探索路线选择；继续使用 P9 的 Point-LIO 定位、5 cm 体素地图累积、地形分析、localPlanner 和 pathFollower。** 地图导出增加数值等价的批量计算，并降低 RViz 全图与完整轨迹的重发频率；探索入口单独缩小近点停车距离，不改变原 FAR 入口及 Point-LIO 算法参数。本入口不同时启动 FAR，也不向真机发送控制。

当前结果：自动探索运行并达到 TARE 的完成条件，自动返航尚未通过。详见 [RESULTS.md](RESULTS.md)。这份封装用于重现实验，不表示自动探索已完整通过验收。实际结果以对应运行目录的状态、观察记录和原始录像为准；尚不能由它推断比赛真机性能、完整覆盖率或无碰撞认证。Dockerfile 和脚本是新增的复现配方；除非另有构建记录，不应把配方存在当作新镜像已经构建或发布。

## 来源与数据链路

- 基础镜像：`go2-p9-nav:amd64-20261110`，保留原 P9/FAR 功能。
- 官方 [TARE 仓库](https://github.com/caochao39/tare_planner)，`humble-jazzy` 分支，固定提交 `b6760adb4280b379567c76f4e684ce628e28ef71`。
- `P9_TARE.patch`：统一 `camera_init` 坐标约定、把初始化前行 12 米改为保持当前位置、适配传感器 QoS，并增加只读诊断。实际应用的补丁及二进制哈希随构建保存。
- `mapping-points.patch`：只将累积地图的逐体素质心导出改为 NumPy 批量除法；原始每帧累积、体素大小、加权方式、点的顺序和 float32 输出保持一致。构建前先对旧实现和修改版执行逐字节等价测试，再通过 `apply_mapping_patch.py` 同步修改源码与实际安装的 Python 模块。
- `/ws`：现有已编译的 `go2-autonomy` 仿真工作区，运行时把**整个工作区**只读挂载；它提供 Gazebo/CHAMP 机器人模型与依赖，不提供另一个定位或全局规划器。当前工作区的 `install/` 符号链接依赖 `/ws/src` 和 `/ws/build`，不能仅复制或挂载 `install/`；基础 ROS/Gazebo 二进制依赖由镜像提供。

```text
Gazebo LiDAR + 相机安装位置的虚拟 gyro/accel
  → 原 P9 传感器适配 → Point-LIO → 世界系点云与 body 位姿
  → 原地图累积、terrainAnalysis / terrainAnalysisExt
  → TARE 覆盖 / frontier / TSP 路线及连续 lookahead
  → 探索守卫 → 原 localPlanner → 原 pathFollower
  → 探索守卫限速与新鲜度检查 → 仅 Gazebo /cmd_vel
```

TARE 的 `local_path` 提供局部规划路径；`global_path_full` 是全局规划器传给前瞻点选择函数的完整路线。上游 `/global_path` 是裁剪后用于显示的路线，`/exploration_path` 的发布语句则被注释。TARE 约 1 Hz 的 lookahead 是移动的路径跟随点，不会转交 FAR 当作固定目标。

## 探索近点与停车距离

`exploration.launch.py` 仅给本次探索的 pathFollower 覆盖 `stopDisThre: 0.10`（10 cm）。原 FAR 导航入口仍使用原来的 0.30 m，公共 `planner-defaults.yaml` 未修改；`goalCloseDis`、减速距离、速度上限、避障检查和守卫新鲜度门限均保持原值。

原因是 TARE 可以在拐点、视线受阻前或局部视点处选择小于 0.30 m 的前瞻点，`kLookAheadDistance: 1.5` 并不保证前瞻点至少 1.5 m 远。当前关闭了 waypoint 外推，localPlanner 又把局部路径裁到前瞻点距离；若 pathFollower 提前按 0.30 m 停车，就可能一直停在有效近点前。run04 出现了约 0.262 m 的前瞻点和约 0.255 m 的局部路径终点，触发了这一条件。run05 仅缩小该停车阈值进行对照；这不是探索完成的声明。

`goalCloseDis` 用于近点方向/横移判断，不是停车距离，不应混同修改。现有 C++ pathFollower 仅在启动时读取参数；要验证新值应新建运行，不能仅凭 `ros2 param set` 返回成功认定行为已改变。10 cm 也不是返航验收半径：完成判据仍要求 TARE 发出完成标志，并在记录的 home 附近稳定停下。

## 全局路径回退与近家返航

TARE 官方 `GetLookAheadPoint()` 在局部路径过短时可以使用完整全局路线。因此守卫优先使用新鲜、至少两点的局部路径；局部为空/单点或不可用时，可改用 `/p9/exploration/global_path_full`。两路均检查 `camera_init`、有限坐标、XY 边界和 3 秒新鲜度；没有任一真实有效路线时保持零输出，超过路径续流期限则锁停。单纯发布 `finished=true`、局部控制器仍有路径或持续重发旧 global，都不能代替有效 TARE 路线。

局部/全局路径与 waypoint 分属不同回调，守卫容许短暂到达顺序差异；首次有效路线后的 waypoint 等待也有 3 秒上限。停止、时间倒退、传感器失效后的恢复栅栏同样适用于全局路线，旧计划不能重新取得控制权。状态字段 `selected_tare_path` 显示当前采用 `tare_path` 还是 `tare_global_path`，并分别记录点数与时效。

run06 还恢复官方近家返航分支：`kRushHome: true`，`kRushHomeDist: 3.0`（上游默认 10 m）。仅在 TARE 已报告完成且进入该距离后，由 TARE 发布 home 作为 waypoint，解决短局部路径可能选择机身当前位置的问题。这没有在守卫里合成 home 直线路径；仍要求真实全局路线和 localPlanner 的有效避障路径，速度继续经过守卫。完成判据保持“完成标志 + 距记录 home 0.5 m 内 + 稳定停下”，不把进入 3 m 范围当作到家。run06 已实测使用全局路径回退，但返航在隔板附近未收敛，因此仍不能视为完整探索验收通过；详细结果见 [RESULTS.md](RESULTS.md)。

估计、点云、地形与边界统一在 `camera_init`；机身为 `base_link`，局部路径为 yaw-only `vehicle`。点云已在世界坐标系，不能再次乘机身位姿。仿真使用虚拟传感器的精确外参、零偏和时钟配置，不把真实 D435i 标定常数套入合成 IMU。

`exploration.sdf` 是约 13 米见方的有界场景，包含内隔板和遮挡。任务只向 TARE 提供外部区域边界；内部障碍通过雷达观察。`/odom/ground_truth` 仅供只读观察器评价，不进入 Point-LIO、TARE、守卫、目标选择或运动控制。

## 地图导出与实时输入

运行时 `make_config.py` 基于原 `p9nav-sim.yaml` 生成 `p9-exploration-sim.yaml`。地图仍按每帧有效点云与对应位姿累积，体素大小仍为 **0.05 m（5 cm）**；这表示地图离散分辨率，不是绝对定位或测量精度。Point-LIO 参数、传感器采样与实时局部点云链路没有因此降频。

只有完整地图与完整轨迹的展示发布周期改变：`map_publish_period_sec: 5.0`、`path_publish_period_sec: 1.0`，即 RViz 全图每 5 秒、轨迹每 1 秒更新一次。`/state_estimation` 与 `/registered_scan` 仍随有效配对输入发布，terrain 和局部规划继续使用这些实时输入。它们的新鲜度门限没有放宽。

这两项改动共同减少 Python mapper 在同一线程上重复导出和序列化完整历史地图的开销。批量质心导出通过随机、重复更新、极值、空图等逐字节等价测试；200k 体素基准只测 `points()` 导出速度，不能单独证明长时间 ROS 管线已恢复实时。最终需要检查较长运行中的下游点云/位姿时延和守卫状态，不能仅看地图画面正常。

## 构建实验镜像

在具备 NVIDIA Container Toolkit 的 AMD64 Linux Docker 主机操作。先确保基础镜像已经导入，并有原先可以运行 Gazebo 的已编译工作区，例如 `/data/t9/liuxu/go2-autonomy`。本配方不是独立安装整个仿真器的发行版，也不是 ARM64/Jetson 镜像。

```bash
cd /path/to/this/integration
bash build_exploration.sh
```

默认输出 `go2-p9-tare:amd64-20261111`。可用 `P9_BUILD_JOBS=2` 控制编译并发。配方先检查基础镜像中的 ROS/Gazebo 运行依赖（缺包则明确失败），再固定上游提交、验证并应用 TARE 补丁、检查动态依赖；随后先运行 mapper 等价测试，再应用地图导出补丁，并运行离线守卫与观察器测试。OR-Tools 库从官方源码内的 `or-tools/lib` 加载。原导航镜像不覆盖；新镜像不自动上传。

APT 软件源并未冻结为历史快照，因此这是可追溯源码的构建配方，不承诺跨日期逐字节相同的镜像。构建记录保存基础镜像 ID、TARE 提交/补丁、二进制哈希、依赖版本；`provenance/mapping-equivalence.json` 保存等价测试与微基准，`provenance/mapping/` 保存修改前源码及源码/安装模块前后哈希。`/ws` 的实际运行来源另由运行清单记录。

## 启动 RViz，先保持停止

每次使用新的输出目录与容器名。不要将一个进行中的运行目录复用于另一实验。

```bash
P9_EXPLORATION_CONTAINER=p9-tare-experiment \
P9_NOVNC_PORT=16097 \
bash run_container.sh \
  /mnt/hdd/ssy/p9-tare-new-experiment \
  /data/t9/liuxu/go2-autonomy
```

默认 GPU 0、8 CPU、20 GB 内存、900 秒墙钟上限；可设置 `P9_GPU_DEVICE`、`P9_EXPLORATION_SECONDS`。网络为独立 Docker bridge，DDS 使用 loopback/domain 94，图形端口仅绑定主机 `127.0.0.1`。Docker 正常停止容器预留 30 秒，供 supervisor 请求停止探索、关闭子进程和写完运行清单。访问 `http://127.0.0.1:16097/vnc.html?autoconnect=1&resize=scale`；远程主机可通过已有 SSH 连接转发这个端口。

容器把镜像内 `/opt/p9-exploration` 复制到新的 `/output/integration`，生成运行 YAML；`/tare_ws` 的源码与编译产物来自镜像；`/ws` 只读。运行输出在 `/output/run01`。

```bash
docker logs --tail 40 p9-tare-experiment
docker exec p9-tare-experiment cat /output/run01/navigation/exploration-status.json
```

等待 `input_ready: true`。首次启动模型和传感器通常需要数十秒；此时不会自动发探索开始或固定目标。

## 先开始记录，再开始探索

所有新 ROS 命令都通过 `inside.sh`，包括 `docker exec`。仅 source ROS setup 不会继承容器入口设置的 DDS 接口，可能导致观察器收不到任何消息。

```bash
docker exec -d p9-tare-experiment bash /output/integration/inside.sh \
  python3 /output/integration/observe_exploration.py \
  --output /output/run01/observation --seconds 480

docker exec -d p9-tare-experiment \
  python3 /output/integration/capture_video.py \
  --output /output/run01/exploration-rviz.mp4 \
  --metadata /output/run01/recording.json --max-seconds 600 \
  --stop-file /output/run01/stop-recording

docker exec p9-tare-experiment bash /output/integration/inside.sh \
  ros2 service call /p9/exploration/start std_srvs/srv/Trigger '{}'
```

开始服务会检查传感器和 TARE/localPlanner 连接。守卫等待新的有效探索路径和 lookahead 后才放行，不会把启动期间的保持点当作探索结果。

录像来自真实 RViz X display `:94`，1440×900、10 fps、原速。`recording.json` 记录时间与补帧情况；不存在绘制假轨迹或生成替代 RViz 画面的步骤。启动录像失败时查看对应文件和进程状态，不能仅凭后台命令返回就声称录制成功。

停止探索、结束记录并保存地图：

```bash
docker exec p9-tare-experiment bash /output/integration/inside.sh \
  ros2 service call /p9/exploration/stop std_srvs/srv/Trigger '{}'
docker exec p9-tare-experiment touch /output/run01/stop-recording
docker exec p9-tare-experiment touch /output/run01/observation/stop
docker exec p9-tare-experiment bash /output/integration/inside.sh p9 save
```

观察器默认停止文件是 `observation/stop`；也可让它按 480 秒时限完成。录像停止文件生效后，等待编码器收尾再拷贝 MP4。达到 supervisor 时限或容器收到正常停止信号时，它先请求探索停止，再关闭自身启动的进程。运行目录保留。

## 判断结果，而非只看机器人动了

- `/p9/exploration/status`：输入新鲜度、TARE/局部路径状态、限速、停止原因、估计 home 距离。
- `observation/summary.json`、CSV、`events.jsonl`：实际收到的自主 waypoint、路径、命令、估计轨迹、只用于评价的 GT 位移和观察点云增长。
- `runtime/`、`run.json`：展开机器人模型、场景/profile 哈希、overlay 来源、进程与退出记录。
- `exploration-rviz.mp4` 与 `recording.json`：可回看的真实 UI 证据。

用只读时延探针检查 15 秒窗口；命令需通过同一 DDS 环境执行：

```bash
docker exec p9-tare-experiment bash /output/integration/inside.sh \
  python3 /output/integration/probe_latency.py
```

探针对比 `/p9/points`、`/pointlio/odom`、`/pointlio/registered_scan` 与下游 `/state_estimation`、`/registered_scan`、`/terrain_map` 的消息时间戳年龄。如果 Point-LIO 输出新鲜而下游持续积压，应检查 mapper 处理与全图导出开销；不能通过提高超时门限掩盖积压。单个窗口也不能替代完整运行的时延记录。

若数据新鲜但机器人停住，可同时检查真实加载参数和近点链路：

```bash
docker exec p9-tare-experiment bash /output/integration/inside.sh \
  ros2 param get /p9_path_follower stopDisThre
docker exec p9-tare-experiment bash /output/integration/inside.sh \
  python3 /output/integration/probe_paths.py
```

参数应为 `0.1`。路径探针读取约 4 秒后输出各话题最新样本：TARE 路径/前瞻点、局部 `/path`、估计位姿和原始速度命令。样本不是严格同步的，应结合时间戳分析；先确认局部路径有效且守卫未停机，再判断是否由停车距离导致原始速度为零。对照需观察能否越过原停点并继续更新探索路径，同时检查返航停车、时延与避障，不能只用恢复移动作为成功标准。

TARE `exploration_finish=true` 表示进入返航阶段，不等于已经回到家。守卫只有在记录的估计起点附近（默认 0.5 米以内）且运动稳定约 2 秒后标记 `returned_home`。观察器应另外检查零速度命令尾段与实际 GT 停止；点云 voxel 增长仅表示观察增长，不是已知真值覆盖率。数据超时、探索停止、未返航和失败试验都应保留，不能仅展示成功片段。

真机控制桥、在线传感器时钟/外参、真机资源与安全机制没有在本封装中验收。不能把此 Docker 的 `/cmd_vel` 仿真入口直接当作比赛 Go2 控制接口。

## English overview

This is an experimental **AMD64 Gazebo** reproduction bundle. Official TARE at the pinned commit replaces FAR for autonomous exploration, while the existing P9 Point-LIO, mapping, terrain processing and local controller remain in use. It does not deploy to or control a physical Go2.

Only the exploration launch overrides the local follower's `stopDisThre` to **0.10 m**, because TARE may produce a valid lookahead inside the original 0.30 m terminal-goal tolerance. The FAR entry and shared defaults stay unchanged. Speed, slowdown, obstacle and freshness checks are retained. This interface adjustment still requires end-to-end validation; a resumed trajectory is not proof of completed exploration.

The guard accepts the actual, untrimmed `global_path_full` as a validated fallback when the local route is unavailable. It never synthesizes a home route or treats the finish flag as planning authority. The official near-home branch is enabled with `kRushHome=true` and a 3 m radius, while local obstacle checking and the separate stable-at-home completion test remain required. Both return-interface changes still need run evidence.

The mapper still accumulates every valid input frame at the original **5 cm voxel resolution**. Only centroid export is batched, with byte-identical output verified before the image applies the patch to both source and installed modules. Full-map display publication is every 5 seconds and the full trajectory every 1 second; paired local scan/odometry publication and Point-LIO parameters remain unchanged. Resolution is not an absolute accuracy claim. Export microbenchmarks do not establish end-to-end real-time performance; use the bundled latency probe and saved run evidence.

Build with `build_exploration.sh`, then launch with `run_container.sh NEW_OUTPUT COMPILED_SIM_WORKSPACE`. The simulator starts stopped. Use `/p9/exploration/start` and `/p9/exploration/stop` through the bundled `inside.sh`. Record the actual RViz display with `capture_video.py` and collect passive telemetry with `observe_exploration.py` before starting motion.

Ground truth is evaluation-only. TARE's finish flag means the return phase; it is not proof of arrival, complete physical coverage, or collision-free motion. Inspect the saved run evidence and retain failed or incomplete attempts. The Docker recipe has not itself been built or published unless accompanied by a separate build record.

## Simulation input recovery

The simulation launch explicitly enables `sim_auto_resume_inputs`. A short input timeout immediately commands zero and discards all planning/control authority. After every input is fresh for 0.5 s, a new TARE path, waypoint, local path and velocity newer than the recovery timestamp are required. A 5 s interruption, explicit stop, invalid odometry, lost tracking or clock reversal remains latched. This option defaults off and is rejected outside explicit simulation command output. It does not correct a sustained processing backlog; the mapper export changes above require separate long-run verification.
