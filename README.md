# Go2 P9: Localization, Mapping, and FAR Planning

English | [中文说明](docs/README.zh-CN.md)

A ROS 2 Jazzy integration for **Go2 LiDAR + D435i IMU → Point-LIO localization → accumulated 3D mapping → FAR and local planning**. The navigation release adds calibrated parameter candidates and a separate `p9nav` entrypoint while retaining the original P9 configuration and `p9` entrypoint.

**Status: experimental.** The planner has driven a robot model in Gazebo through goals selected in RViz. The latest complete goal test did not pass arrival validation, and input freshness issues remain. **The released `live` mode produces isolated velocity suggestions; it does not command the physical Go2.** A real-robot motion bridge and hardware validation are still required.

## Versions

| Component | Version / location |
|---|---|
| Navigation branch | `p9-navigation-20261110` |
| Runtime source used for the published images and RViz test | `0a53bff69c8bd3debbe4c2a915faefca0eb19b30` |
| Published ARM64 image | `go2-p9-nav:arm64-20261110` |
| Docker build file | [`Dockerfile.navigation`](Dockerfile.navigation) |
| Original P9 | Preserved on `main`, with the original `go2-p9:arm64` image |
| Image archive and release records | [Hugging Face release directory](https://huggingface.co/datasets/sssssssssssssssy/go2-p9-arm64/tree/main/releases/p9-nav-20261110) |

The English README was updated on **2026-11-11**. This documentation update does not rebuild or replace the published images. Use the release directory's `release.json`, `validation.json`, and `SHA256SUMS` to identify and verify those artifacts. Access follows the repository's existing permissions.

## Data flow

```text
Go2 LiDAR points + D435i gyroscope / accelerometer
  -> sensor adapter: fields, per-point time, clock alignment,
     linear IMU interpolation, and optional B1 gyro-bias correction
  -> Point-LIO
  -> registered world-frame scan + timestamp-matched IMU pose
       |-> P9 map accumulator -> /p9/map, /p9/path, PCD, CSV
       `-> body-pose bridge + terrain analysis
            -> FAR route / waypoint planning
            -> localPlanner + pathFollower
            -> navigation guard -> /p9/navigation/cmd_vel
                                      `-> simulation /cmd_vel
                                          only when explicitly enabled
```

- `/utlidar/robot_odom` is **not** an estimator input in this P9 pipeline. Camera images are not used for localization.
- Registered scans are already expressed in the `camera_init` world frame. The map accumulator does not transform them by the current pose a second time; its default voxel size is 5 cm.
- Point-LIO's native pose refers to the IMU origin. The configured `T_imu_base` supplies a `base_link` pose for planning. The planner's `vehicle` frame uses the same position and yaw-only orientation; it is not a static identity alias of `base_link`.
- Geometric map accumulation, terrain analysis, and route planning are separate stages. The system follows externally supplied goals; autonomous exploration is not included in this entrypoint.

## Requirements

- ROS 2 Jazzy for a native build, or the supplied Linux/ARM64 Docker image for deployment. The navigation image builds on the retained P9 image, which is based on `go2-control:arm64-v4`.
- An external working sensor driver. `p9nav live` does not start a second RealSense driver.
- Correct sensor topics, frame transforms, per-point LiDAR timestamps, and a common time basis. `use_sim_time` and a bag player's `/clock` do not repair message header timestamps.
- A persistent mounted output directory, empty or new for each run.
- For the commands below, `sar-run` is the competition environment's existing container launcher; it is not installed by this repository. Replace `/path/...` with host paths that the launcher mounts into the container. GUI commands require the launcher's GUI support.

## Load the navigation image

Download `go2-p9-nav-arm64-20261110.tar.zst`, `release.json`, and `SHA256SUMS` from the same release directory. In the download directory:

```bash
# Check the downloaded files against that release's checksum list.
sha256sum --ignore-missing -c SHA256SUMS
zstd -dc go2-p9-nav-arm64-20261110.tar.zst | docker load
docker image inspect go2-p9-nav:arm64-20261110 \
  --format '{{.Os}}/{{.Architecture}} {{.Id}}'
```

The published deployment image is ARM64. Its build checks and a separate AMD64 simulation test are different forms of evidence; an x86 or QEMU run does not establish Jetson throughput.

## Choose the correct profile

| Mode | Profile | Purpose | Motion output |
|---|---|---|---|
| `p9nav replay` | [`config/p9nav-replay.yaml`](config/p9nav-replay.yaml) | Prepared public `localization` bag; recorded sensor data and bag clock | Isolated suggestions; cannot alter the recorded motion |
| `p9nav live` | [`config/p9nav-live.yaml`](config/p9nav-live.yaml) | Current Go2 and D435i streams; current-device clocks and transforms must be checked | Isolated suggestions; no physical robot control |
| `p9nav sim` | [`config/p9nav-sim.yaml`](config/p9nav-sim.yaml) | An already running simulator with compatible sensors and `/clock` | Simulation `/cmd_vel` only with `--sim-command-output` |

All three modes require an explicit `--config`. The original `p9 replay/live` commands remain available for localization and mapping without the navigation stack.

### Replay

Run historical data on a separate DDS domain and loopback interface. The example profile is specific to the prepared public `localization` bag, not an arbitrary rosbag:

```bash
sar-run --gui go2-p9-nav:arm64-20261110 env ROS_DOMAIN_ID=42 \
  CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>' \
  p9nav replay --bag /path/prepared-localization-bag \
  --config /opt/p9/config/p9nav-replay.yaml \
  --output /path/new-navigation-replay --rate 0.25 --rviz
```

This profile applies a LiDAR header offset of **2167.645092767 seconds** for that particular prepared bag. Do not copy this offset to live data, `two-laps`, or another bag. For raw camera IMU topics, use a reviewed `raw_linear` profile with the required static TF instead of treating raw data as an already merged stream.

Replay can validate sensor adaptation, localization, mapping, and planning interfaces. It cannot validate closed-loop navigation: the bag continues along its recorded trajectory regardless of the selected goal.

### Live localization, mapping, and planning preview

Copy and review the live profile first. Confirm current topics, clocks, LiDAR-to-IMU extrinsics, and the IMU-to-body transform. Use the competition-provided `SAR_ROBOT_IFACE` and the assigned DDS configuration; the existing deployment uses domain 0.

```bash
sar-run go2-p9-nav:arm64-20261110 p9 diagnose
sar-run --gui go2-p9-nav:arm64-20261110 p9nav live \
  --config /path/p9nav-live-robot.yaml \
  --output /path/new-navigation-live --rviz
```

The supplied live profile combines `/camera/camera/gyro/sample` and `/camera/camera/accel/sample` using bracketed linear interpolation and checks their internal static TF. It does not extrapolate missing accelerometer coverage. Its zero clock offset is a value to verify on the current boot, not proof that the clocks are synchronized.

**This command does not make the Go2 walk, stand up, or rotate.** The direct Unitree SportClient/API sender was removed from the integrated path follower. `/p9/navigation/cmd_vel` is an isolated suggestion topic. Physical autonomous navigation still needs a separately implemented and validated motion bridge.

### Simulation

Start the simulator, sensor bridges, and the compatible sensor adapter first. The `sim` entrypoint neither installs Gazebo nor creates a world. Raw Gazebo point clouds must follow the declared virtual sensor timing contract; they must not bypass the real-sensor timestamp checks.

In the simulator's isolated ROS environment:

```bash
p9nav sim --config /opt/p9/config/p9nav-sim.yaml \
  --output /path/new-navigation-sim --rviz --sim-command-output
```

Without `--sim-command-output`, commands remain isolated. `live` and `replay` reject attempts to use this flag to enable actuation. The simulation profile uses virtual sensor transforms, zero gyro bias, and zero epoch offset instead of the physical robot's calibration constants. The [`simulation/`](simulation) directory contains the integration and validation tools used with the external simulator workspace.

## Set goals and save results

In the supplied RViz configuration, use **2D Goal Pose**. The tool publishes `/p9/navigation/goal_pose` in `camera_init`. Alternatively, run these commands in the same ROS environment, DDS domain, and network interface as the active instance:

```bash
p9nav goal 1.0 0.5 --frame camera_init
p9nav stop
p9 save
```

Goal coordinates are in metres in the current map frame. `p9nav goal` reports acceptance or rejection; acceptance does not mean arrival. `p9nav stop` stops this navigation interface, not an independent robot controller.

When using an existing Docker container, `docker exec` does not automatically repeat its entrypoint's exported ROS/DDS setup. Run helper commands through `/usr/local/bin/p9-entrypoint` and preserve the active instance's domain and interface. A mismatched interface can make an observer or goal client see no messages.

Outputs include effective configurations, ROS logs, `trajectory.csv`, `mapping_status.json`, `mapping_summary.json`, and `PCD/map.pcd` under the localization output directory. Navigation adds `effective-navigation.json` and `navigation-status.json`. Use `p9 save` to save the active map; normal shutdown also saves localization results. Keep output on a persistent mount. Only one P9 instance should use the fixed service/topic names in a given DDS domain.

The native trajectory tracks the IMU origin unless explicitly converted; map points use `camera_init`. Use the configured body pose when evaluating the robot's goal distance; do not compare paths with different reference points as if they were identical.

## Calibration in this version

The navigation profiles use the C-group LiDAR-to-camera-IMU rotation and translation candidate plus the B1 static gyroscope-bias candidate:

```text
omega_corrected = omega_input - b_g
b_g = [ 0.00028568964388210946,
       -0.0018381138448739292,
        0.0005906477368247819 ] rad/s
```

The adapter applies this correction once. The target bag is not refitted, and the same bias must not be subtracted again in a prepared bag or filter initializer. No new accelerometer scale or static-bias correction is applied by B1.

The public bag and recent calibration were recorded on the same dog with unchanged sensor mounting, as confirmed by the operator. This makes the installation relationship relevant, but does not certify parameter accuracy or bias stability across temperature and power cycles. Full D435i six-position intrinsic calibration and EEPROM updates were not performed. The body transform still needs hardware verification. See [calibration details](docs/calibration.md) and [navigation parameter notes](docs/navigation.md) (Chinese).

In this Point-LIO version, `use_imu_as_input=false` selects the IMU measurement-update formulation; it does **not** disable the IMU.

## Validation status and known issues

The **2026-11-10 RViz test** used the published runtime source at `0a53bff` in the separate AMD64 image `go2-p9-nav:amd64-20261110`, with actual RViz goal clicks and a Gazebo robot model. It was not a real Go2 run.

| Observation | What it establishes |
|---|---|
| Outbound and return RViz demonstration, with motion and stop | The localization/map/planner/control chain can drive the simulated robot; this recording did not have complete passive telemetry for both legs |
| Additional goal with complete passive telemetry: accepted goal, 663 nonzero `/cmd_vel` samples, approximately 5.62 m maximum GT displacement | Goal delivery, route following, and physical motion in the simulator were observed |
| FAR's last waypoint was approximately 0.532 m from the original clicked goal; the final estimated body position was approximately 0.809 m from that original goal | The complete test **failed** the 0.5 m original-goal arrival criterion, even though FAR reported arrival near its adjusted waypoint and the robot stopped |
| A later body-odometry age of approximately 1.192 s exceeded the 1 s freshness threshold | The guard cleared the goal; this occurred after motion had already stopped and is not established as the initial stopping cause |
| A first run stopped receiving point clouds in the adapter while raw data was still available | An unresolved input/subscription stability issue; a successful restart does not prove a fix |

The 0.809 m figure is **estimated body-to-goal distance**, not localization error against ground truth. The tests do not certify full-trajectory localization accuracy, collision-free operation, hardware throughput, or real-robot navigation. Earlier empty observer recordings were caused by test-tool setup errors, including a DDS interface mismatch; they are not treated as planner success or failure.

Remaining work includes reconciling FAR's adjusted target with the local follower and guard arrival criteria, investigating input stalls and odometry delays, and validating real-device timing, body transforms, and the motion bridge. These findings are documented here; this README update does not claim to fix them. Map appearance alone is not ground truth.

## Build from source

After installing the ROS 2 Jazzy dependencies described in [native setup](docs/native.md):

```bash
git clone --branch p9-navigation-20261110 \
  git@github.com:stfhssssy/go2-p9-localization.git
cd go2-p9-localization
BUILD_JOBS=2 ./scripts/build-navigation-native.sh
./scripts/p9nav --help
```

Use `./scripts/p9nav` instead of `p9nav` for the native examples, and use checkout-relative configuration paths. The original `scripts/build-native.sh` and `scripts/p9` remain the localization-only route.

To build the navigation container, first load the verified original `go2-p9:arm64` base image:

```bash
docker build --platform linux/arm64 -f Dockerfile.navigation \
  --build-arg BUILD_JOBS=2 --build-arg PLANNER_BUILD_JOBS=2 \
  --build-arg SOURCE_REVISION="$(git rev-parse HEAD)" \
  -t go2-p9-nav:arm64-local .
```

An x86 host requires a configured ARM64 builder or QEMU/binfmt support. Do not reuse x86 binaries for ARM64. Runtime startup does not install dependencies from the network.

For source verification in a Python environment with the test dependencies installed:

```bash
python3 scripts/freeze-navigation-provenance.py --check
PYTHONPATH=src/go2_p9:src/go2_p9_navigation python3 -m pytest -q tests
```

The provenance check verifies recorded source/document bytes, not algorithm quality. The archived image and its release records remain tied to their original source commit; a documentation commit has its own updated document inventory.

## Origins and license

The integrated planning packages come from the existing server workspace, with ROS 2 Jazzy adaptations, based on [Zhang Ji's autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2). That server snapshot had no Git metadata; it is not represented as a clean upstream checkout. The exact source snapshot, upstream comparison, local changes, and file hashes are recorded in [`provenance/planning-upstream.json`](provenance/planning-upstream.json).

Original P9 provenance remains in [`provenance/source-manifest.json`](provenance/source-manifest.json); the common changes remain in [`provenance/experiment-delta.json`](provenance/experiment-delta.json). Existing upstream source and license notices are retained. New project code is GPL-2.0-only; see [LICENSE](LICENSE) and [third-party notices](THIRD_PARTY_NOTICES.md).

Further documentation (Chinese): [navigation](docs/navigation.md) · [deployment](docs/deployment.md) · [native setup](docs/native.md) · [calibration](docs/calibration.md) · [provenance](docs/provenance.md).
