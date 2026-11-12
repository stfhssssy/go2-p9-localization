# Optional attended exploration control bridge

This bridge is **not launched by `p9explore live`**. The exploration pipeline only
publishes a guarded body-frame velocity. A second, explicitly started process may
forward that velocity through the existing Unitree Sport high-level interface.
It never requests standing, crouching, a gait change, controller release, or mode
switching. It does not use low-level motor control.

## Verified interface and scope

The inspected control-v4 source uses `SportClient::Move(vx, vy, vyaw)` and
`StopMove()` and records their ROS/DDS equivalents:

| ROS topic | Type | Direction |
|---|---|---|
| `/p9/exploration/cmd_vel` | `geometry_msgs/msg/Twist` | Guarded body-frame input |
| `/p9/exploration/status` | `std_msgs/msg/String` JSON | Guard activity and freshness input |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | Robot telemetry input |
| `/api/sport/request` | `unitree_api/msg/Request` | Optional actual high-level control output |
| `/api/sport/response` | `unitree_api/msg/Response` | Matching StopMove acknowledgement input |

Move uses API **1008**, JSON `{ "x": vx, "y": vy, "z": yaw_rate }`, no reply.
StopMove uses API **1003**, JSON `{}`, requires a matching request identity and
status code 0. These match the inspected control-v4 behavior and the [official
Unitree ROS 2 Sport client](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/common/ros2_sport_client.cpp).
A transport/API success is not evidence that the physical robot obeyed a command.
The bridge itself has **not been tested on hardware**.

## Operator workflow

Use the same robot-facing ROS domain, network interface, and CycloneDDS setup as
the sensor stack. Do not use the simulation-only domain/interface settings.
Choose the actual local interface name; a Tailscale SSH connection is not the
robot DDS transport interface.

1. Keep the first test bounded and attended. Verify the robot is already standing,
   a clear area is available, and an onsite independent stop is ready. Stop all
   other velocity controllers before starting this bridge.
2. Start the exploration pipeline and its explicit exploration-start operation.
   A healthy `live` guard must be actively `EXPLORING`; merely opening RViz does
   not arm robot motion.
3. In a second interactive terminal, inspect without output first:

   ```bash
   p9explore-control
   ```

   Without `--enable-motion` there is **no Sport request publisher**. It cannot
   arm, stop, stand, or move the robot.
4. Exit preview, then explicitly enable the control transport:

   ```bash
   p9explore-control --enable-motion
   ```

   Startup sends StopMove only. Press **g** to request a matching successful
   StopMove acknowledgement and arm. Then **hold h** to grant motion permission
   using terminal key autorepeat. Releasing h expires permission after 0.6 s plus
   scheduling/network/API latency. If autorepeat starts later, the bridge may
   stop and require a new g arm. This is deliberate, not an automatic speed boost.
5. **Space** stops and disarms. **q**, Escape, Ctrl+C, SIGHUP or normal shutdown
   attempts StopMove with up to three acknowledgements/retries. After any stale
   input or deadman expiry, hold h alone cannot resume; press g again only after
   inspecting the reason.

Shutdown attempts StopMove before restoring terminal settings. A closed SSH/PTY
or failed terminal restore cannot skip the subsequent ROS resource cleanup.

Default caps are forward 0.15 m/s, lateral 0.10 m/s, yaw 0.25 rad/s. The optional
`--max-forward`, `--max-lateral`, `--max-yaw` arguments are positive finite caps
(linear at most 0.5 m/s, yaw at most 1 rad/s). Caps are application limits, not
validated safe operating speeds. Low requested speeds may not produce walking;
inspect the controller and actual motion rather than silently increasing them.

The bridge requires fresh advancing robot telemetry (0.5 s), guarded commands
(0.35 s), and stamped sequenced live status (1.5 s). It rejects simulation status,
nonplanar/nonfinite commands, repeated/stale status, and changed guard instances.
After arm, all three input streams must update again before any Move request.
An observed competing Sport action also disarms. This detects coexistence but
cannot arbitrate an independent controller that keeps sending commands.

Network failure, SIGKILL, a dead host/container, firmware faults, or another
controller can prevent a software stop. Keep an independent onsite stopping
method. **Do not represent the keyboard or this bridge as a certified e-stop.**
Current exploration release stops when TARE reports finish; autonomous return
home was not accepted in simulation and is not enabled for this first live test.

## Validation

```bash
python3 -m unittest discover -s /opt/p9-exploration/control -p 'test_control_*.py'
python3 /opt/p9-exploration/control/mock_control_test.py
```

The second test internally forces CycloneDDS to loopback `lo` and domain 101,
starts a fake Sport service and an interactive pseudo-terminal, and verifies
startup/arm/deadman/freshness/stop behavior without a robot. It also interrupts an
actively forwarding child with SIGHUP and closes its PTY, requiring a shutdown
StopMove and successful process exit in both cases. Run it in an isolated
container with `--network none`. It must be run in the
new image, where `unitree_api`, `unitree_go`, and ROS Python are present. A passing
mock is protocol and software validation only, not a physical-control result.
