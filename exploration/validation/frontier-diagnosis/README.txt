TARE frontier parameter diagnosis, 2026-11-12

Observed failure: candidate viewpoints and uncovered frontier points were present,
but the first local route contained one pose, no waypoint was forwarded, and TARE
declared completion after its five-second minimum. The guard stopped correctly.
In sim-smoke01, frontier counts were 204 then 212 before completion. Zero counts
after completion were an effect of kNoExplorationReturnHome, not the root cause.
Earlier start (sim-smoke02) and a larger +/-7 m rectangle (sim-smoke03) did not fix it.

Root cause in the upstream ROS2 port:
- YAML contains kUseFrontier=true, but ReadParameters never declares that key.
- Default rclcpp NodeOptions do not automatically declare YAML overrides.
- ViewPointManagerParameter and PlanningEnvParameters ignored the false return
  from get_parameter and left their bool members indeterminate.
- LocalCoveragePlanner copied the viewpoint-manager bool over its true default.
  The environment could therefore extract frontiers while the local planner
  ignored them. Different builds/memory layouts can expose different behavior.
- The actual sim03 ROS parameter dump lacks kUseFrontier. A static scan of every
  non-OR-Tools CPP found this as the only literal read without a declaration.

Fix: separate TARE_FRONTIER_PARAMETER.patch adds one explicit declaration with
true default before initialization, initializes both reader bools to false and
throws if the read fails. Explicit false remains supported. No coverage scoring,
path solver, motion command, obstacle checks or completion criteria are changed.
The original P9_TARE.patch remains byte-identical for provenance and build reuse.

Validation: test_tare_frontier_source_contract.py checks declaration order and all
literal parameter reads, then compiles the exact patched reader preambles against
a minimal parameter API. It exercises true, false and missing for both readers.
The separate actual-binary ROS test, test_tare_frontier_parameter.py, has also
passed both kUseFrontier=true and explicit false through the real parameter
service. The source/API contract alone does not establish closed-loop behavior.

Actual Gazebo validation: sim-smoke05-frontier-fixed is a PARTIAL success. The
corrected TARE produced multi-pose routes (up to 89 poses), forwarded waypoints,
nonzero commands and independently observed robot motion (maximum displacement
about 4.59 m from its initial ground-truth position). This establishes that the
previous startup/no-route failure was resolved. It does not establish complete
coverage or successful autonomous return.

At simulation time 115.2 s, the raw ROS LiDAR stream /unitree_lidar/points stopped.
The simulator adapter's received-cloud counter remained at 1025 while gyro
reception continued from 49277 to 58148. No bad-frame, nonincreasing-stamp or
too-small-cloud rejection was recorded. P9 had completed 739 exact scan/odometry
pairs with no pending backlog. This localizes the outage upstream of Point-LIO,
to the Gazebo LiDAR producer or its ROS bridge; saved logs cannot distinguish
those two. The later cloud/IMU timestamp separation is a consequence of the
stopped cloud stream, not evidence of a new calibration/time-offset problem.

The exploration guard detected stale input and latched STOPPED. The run ended
with zero commands confirmed. TARE had not reported exploration completion.
Do not present this partial run as full-area coverage, successful return, or
real-robot validation. Real-robot exploration remains pending the field test.
