P9 navigation isolated Gazebo reproduction

Environment: AMD64 navigation image with Gazebo/CHAMP dependencies, /ws mounted
read-only from the existing go2-autonomy workspace, /opt/p9 new release, and
/opt/p9_ws/install new compiled overlay. /output must be a writable host mount.
This harness does not install packages or modify /ws. No real robot is required.

Start (choose a NEW directory below the writable mount):
  /opt/p9/simulation/inside.sh python3 /opt/p9/simulation/run_simulation.py \
    --output /output/nav-run-01 --sim-command-output

Omit --sim-command-output for isolated planning visualization only. The explicit
flag lets the new navigation guard publish Twist to Gazebo /cmd_vel. No old
planner/guard/command bridge is launched. Xvfb :94 and RViz start by default;
--no-rviz disables RViz and --existing-display uses an existing X display.
inside.sh sets ROS_DOMAIN_ID=94 and GZ_PARTITION=ssy-p9-nav-20261110 and sources
Jazzy, /ws/install, then /opt/p9_ws/install, in that order.

Validate from a second terminal in the same container:
  /opt/p9/simulation/inside.sh python3 /opt/p9/simulation/validate_navigation.py \
    --startup-timeout 240 --seconds 240 --goal-forward 6.5 --goal-left 0 \
    --stop-on-arrival --output /output/nav-run-01/validation-forward.json

The validator waits for a stationary estimate plus ready guard, requests one
camera_init goal, requires the matching goal acknowledgement, records raw and
guarded commands separately, and calls /p9/navigation/stop on completion. Its
early finish uses estimated movement, FAR arrival, stopped estimated motion and
stopped commands. Ground truth cannot generate a goal or trigger stopping.
JSON reports pre-stop observations separately from the forced post-stop tail.
CSV series are new received data, never copies of the previous experiment.
No contact-based collision validation is implemented; arrival is not a claim
of collision-free operation. The return code means recording usability, not
accuracy or navigation success. Inspect navigation_observations and metrics.

GT evaluation: compare body-origin /state_estimation against /odom/ground_truth
at common timestamps, 10 Hz grid, linear position interpolation + quaternion
SLERP, maximum 0.2 s gap, no extrapolation. Align once at the first covered
pre-goal stationary pose using full SE(3), fixed unit scale; do not fit the whole
trajectory. Both raw series and alignment matrix are preserved.

Capture the current RViz desktop without resizing or recoloring:
  DISPLAY=:94 xwd -root -silent -out /output/nav-run-01/rviz.xwd
  python3 /opt/p9/simulation/xwd_to_png.py /output/nav-run-01/rviz.xwd \
    /output/nav-run-01/rviz.png --metadata /output/nav-run-01/rviz-capture.json

Stopping run_simulation.py asks the guard to stop, then shuts down only its own
process groups. run.json preserves commands, process exits and stop response.
runtime/ preserves expanded URDF, verified model, world copy and source hashes.
runtime/sensor_summary.json and navigation/navigation-status.json expose state.

Simulation uses exact virtual mounting geometry and zero bias/time offset.
The 30-row 10 Hz GPU LiDAR samples a whole frame at once: all point time values
are zero. The two physical virtual optical IMUs are gyro 200 Hz and accel 100 Hz;
the normalizer interpolates accel onto gyro timestamps with bounded brackets.
It does not inject Gazebo orientation or ground-truth pose into Point-LIO.
The simulated sensor differs from actual D435i/L1 mechanics and does not verify
competition robot calibration, timing, or achievable real-world accuracy.

Offline checks (Python with numpy/scipy/PyYAML/Pillow; no ROS required):
  python3 /opt/p9/simulation/test_contracts.py
  python3 /opt/p9/simulation/validate_navigation.py --self-test
  python3 /opt/p9/simulation/xwd_to_png.py --self-test
