# Simulation validation record — 2026-11-11

This is an integration experiment in an isolated Gazebo scene, not a physical-robot deployment. Official TARE (`b6760adb4280b379567c76f4e684ce628e28ef71`, `humble-jazzy`) supplies exploration and return-home goals. P9 supplies Point-LIO state estimation, accumulated geometry and terrain inputs; the existing localPlanner/pathFollower supplies obstacle-aware local motion. FAR is not launched in exploration mode. The original navigation release and entry points remain unchanged.

## What was actually run

The existing `go2-p9-nav:amd64-20261110` container image was used with a separately compiled TARE overlay and this integration directory mounted into it. The new `Dockerfile.exploration` is a reproduction recipe; it has not been built or uploaded as a new image. The runtime used 12 allocated CPUs, a 28 GB memory limit and GPU 0. ROS domain 94, loopback DDS and a separate Gazebo partition isolate this experiment from robot hardware.

The 13 m × 13 m room contains internal partitions and boxes. Only the outer exploration boundary is supplied to TARE; internal geometry comes from virtual LiDAR. Virtual IMU/extrinsics use the simulator model. `/odom/ground_truth` is subscribed only by the passive observer, never by the estimator, planner or command guard.

The video is a real recording of the RViz X display: 1440 × 900, 10 fps, original speed. `recording.json` records actual capture duration and duplicate frames. Faster previews, when supplied, are explicitly labeled. Green points are the accumulated map, orange is the estimated body trajectory, magenta is TARE's local route, cyan is its global display route, blue is the local collision-checked path, and yellow is the exploration boundary. Grid spacing is 1 m.

## Debugging trials retained

| Run | Configuration / finding | Observed result |
| --- | --- | --- |
| run02 | Initial integration; input interruption latched the guard | Actual autonomous motion, raw GT path about 21.4 m; terrain freshness stop; no completion or return |
| run03 | Simulation-only recovery after short input gaps | Actual motion about 51.9 m; persistent mapper backlog later caused an odometry freshness stop |
| run04 | Vectorized map centroid export; full map every 5 s and full trajectory every 1 s | About 61.4 m; passed the previous ~300 s backlog point; stopped progressing at a 0.262 m TARE lookahead inside the 0.30 m follower stop tolerance |
| run05 | Exploration-only stop tolerance 0.10 m | About 67.3 m; TARE declared exploration complete and began returning; guard rejected a singleton local path despite a valid global return route, stopping about 2.86 m from estimated home |
| run06 | Full global-route fallback and official near-home return branch restored | TARE declared completion; actual global fallback occurred; return did not converge near the partition; operator-issued experiment stop |

These are sequential debugging runs, not statistically repeated algorithm comparisons. GT path lengths are sums of high-rate simulator samples and include body jitter. They are not localization accuracy or map quality scores. Run05 did not contain sampled lookahead distances between 0.10 and 0.30 m, so it does not independently prove traversal of the exact run04 near-goal condition.

## Changes justified by the trials

1. **Map publication cost:** in run03, Point-LIO output was fresh (≤58 ms in a 15 s window), while mapper-derived state/scan/terrain were about 4.1 s old. Turning off the RViz map subscription reduced downstream delay to ≤84 ms in the next diagnostic window. The mapper's centroid export was vectorized without changing float32 bytes, point order, weighted means or the 5 cm voxel map. Full-map / full-trajectory display publication was reduced to every 5 s / 1 s; paired local scan and pose publication remain per-frame. A run05 15 s probe measured ≤275 ms mapper scan age and ≤278 ms terrain age; this window is not a guarantee for every frame.
2. **Intermediate target tolerance:** TARE may legitimately emit a nearby viewpoint. The original FAR terminal tolerance of 0.30 m stopped a valid 0.255 m local path. Exploration now overrides only `path_follower.stopDisThre=0.10`; shared FAR defaults, speed limits and obstacle checks are unchanged.
3. **Local/global route semantics:** `GetLookAheadPoint` can use its complete global route when the local coverage route is short. The guard now accepts fresh, finite, in-bound `global_path_full` (not the trimmed global display path), while still requiring a new local collision-checked path and command. Empty routes cannot authorize motion or renew its watchdog. Callback ordering gets the existing bounded waiting window; stale data, clock changes and explicit stop still latch zero output.
4. **Near-home behavior:** the official `kRushHome` branch is enabled, with its distance reduced to 3 m for this scene. It gives the real home target instead of repeatedly selecting an intermediate near-home coverage viewpoint. It does not bypass local obstacle checking or the command guard.

## Verification scope

All 46 offline tests pass: 34 guard behavioral tests, 6 mapper equivalence tests and 6 observer tests. Tests cover stale/cached routes, empty/singleton local routes, global fallback, callback ordering, bounded startup, recovery fences, limits, explicit stop and finish-versus-arrival semantics. Runtime launch parameter inspection confirms the exploration follower's 0.10 m tolerance. The original navigation source/document provenance check still passes (235 runtime files, 11 documents).

A TARE finish flag means it has entered return mode according to its own map. It is not proof of complete physical coverage. The guard requires a stable estimated pose near home before reporting `RETURNED_HOME`; the passive GT observer is checked separately. No collision-free certification or target-hardware timing claim is made from this single scene. Real Go2 sensor clocks, calibrations, control bridge and compute load still need separate validation.

## Final run06 evidence

**Result: autonomous exploration ran and reached TARE’s own completion condition; autonomous return-home did not pass.**

- One explicit start. The first recorded finish/return transition occurred approximately **218.0 wall seconds after start**. Neither TARE nor the guard reported returned-home at any time.
- The guard selected the complete global fallback route in **209 recorded status samples**. This verifies that the newly supported input was used in the real run, beyond its offline tests.
- The robot continued moving near the partition instead of converging to home. The captured `return-route-probe.json` shows the complete global route as `[current keypose, home, current keypose]`, without intermediate points around the partition. The near-home branch also sends the direct home goal. Together with the RViz geometry and repeated motion, this points to return-route connectivity / shortcut handling and the limits of local obstacle avoidance. The precise cause of the global graph connection remains unresolved; this evidence does **not** isolate a Point-LIO defect.
- After the experiment was explicitly stopped, the estimated XY distance to recorded home was about **2.30 m**. Independent simulator GT, using the GT sample nearest the recorded start event, gives a final XY offset of **2.43 m**. This is final position offset, not trajectory ATE.
- The observer received about **21.9 s of consecutive zero commands** after the explicit stop. This is controlled experiment shutdown, **not automatic arrival**. The final recording is limited to 600.1 s, so the observer record, rather than a claim about the last video frame, establishes the shutdown tail.
- There were **12 short input pauses and 12 recoveries**, with a single mission start. Maximum observer-received odometry header age was **2.133 s**. A separate 15 s diagnostic window measured ≤55 ms Point-LIO age, ≤251 ms mapper scan age and ≤262 ms terrain age. Intermittent timing issues remain; the good diagnostic window does not erase them.
- Full original-speed recording: **600.1 s, 1440 × 900, 10 fps**, no duplicate capture frames. The separate **4× preview** contains the same run with only playback speed/frame sampling changed.
- Accumulated observed voxel growth is recorded for traceability, not reported as coverage percentage. No metric in this run establishes complete physical coverage, robust return, collision-free operation, or real-robot readiness.

The tested code and unresolved behavior are retained together. Next work should audit global keypose-graph collision/connectivity and near-home shortcut behavior against the recorded point clouds, then repeat the same scene with a return route that respects the partition. Extending the run time or weakening the command guard would not establish correctness.
