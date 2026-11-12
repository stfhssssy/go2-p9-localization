# Unitree ROS message bindings for the optional control bridge

Source: [official Unitree ROS 2 repository, pinned commit
668d1ec5a05d1c38d3306bdca7d59f2ba3581a88](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree).

`unitree_api` and `unitree_go` retain every upstream `.msg` byte and their package
names. The 8 API schemas and the bridge's `SportModeState`, `TimeSpec`, and
`IMUState` schemas are also byte-identical to the inspected autonomy_stack_go2
copy. Only CMake generation changes: standard ROSIDL generates C, C++ and Python
bindings on Jazzy, without the upstream legacy Connext-specific helper. Original
BSD 3-Clause license notices are included. No hardware SDK or control executable
is included in these message packages.

`SOURCE_PROVENANCE.json` records the commit, package paths, exact file hashes and
comparison scope. This is an offline schema/build adaptation, not a new hardware
compatibility test or a claim that firmware follows every schema behavior.

Example image build, after placing these packages in `/opt/p9_control_ws/src`:

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/p9_control_ws
colcon build --merge-install --packages-select unitree_api unitree_go \
  --cmake-args -DBUILD_TESTING=OFF
source /opt/p9_control_ws/install/setup.bash
python3 -c 'from unitree_api.msg import Request, Response; from unitree_go.msg import SportModeState; print(Request(), Response(), SportModeState())'
```

Build dependencies: `ament_cmake`, `rosidl_default_generators`, `geometry_msgs`,
colcon, CMake/C++ compiler and Python development headers. Runtime dependencies:
`rosidl_default_runtime`, `geometry_msgs`. Install ROS packages from the same
Jazzy distribution as the base image; standard ROSIDL dependencies supply the
Python generator and its NumPy/EmPy requirements. No custom DDS generator is
required. The release wrapper must source this install overlay.
