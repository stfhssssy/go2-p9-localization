#!/usr/bin/env python3
"""Optional attended ROS2 -> Unitree Sport high-level interface. Never auto-started."""
import argparse
from collections import deque
import json
import math
import os
import select
import signal
import sys
import termios
import time
import tty

from control_core import ControlPolicy
from control_cleanup import cleanup_session

MOVE_API = 1008
STOP_API = 1003


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--enable-motion', action='store_true', help='Create the actual Sport request publisher; otherwise read-only preview')
    p.add_argument('--max-forward', type=float, default=.15)
    p.add_argument('--max-lateral', type=float, default=.10)
    p.add_argument('--max-yaw', type=float, default=.25)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    policy = ControlPolicy(args.enable_motion, args.max_forward, args.max_lateral, args.max_yaw)
    if not sys.stdin.isatty():
        raise SystemExit('An attended interactive terminal is required. Use docker exec -it / ssh -t.')
    # Delay ROS imports until after help/argument validation; no DDS connection for --help.
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from unitree_api.msg import Request, Response
    from unitree_go.msg import SportModeState

    quit_requested = False
    terminal_output_lost = False
    def report(message, **_):
        nonlocal quit_requested, terminal_output_lost
        if terminal_output_lost: return
        try:
            print(message, flush=True)
        except (OSError, ValueError):
            # A disconnected PTY must not interrupt StopMove or its retries.
            terminal_output_lost = True
            quit_requested = True
            # Avoid a second failing Python final flush after the terminal vanished.
            try:
                with open(os.devnull, 'w') as sink:
                    for stream in (sys.stdout, sys.stderr):
                        try: os.dup2(sink.fileno(), stream.fileno())
                        except (OSError, ValueError): pass
            except OSError:
                pass
    def interrupted(*_):
        nonlocal quit_requested
        quit_requested = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('p9_exploration_control')
    if bool(node.get_parameter('use_sim_time').value):
        node.destroy_node(); rclpy.shutdown()
        raise SystemExit('This optional real-hardware bridge refuses use_sim_time=true.')
    sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    request_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
    pub = node.create_publisher(Request, '/api/sport/request', request_qos) if args.enable_motion else None
    own_ids = deque(maxlen=512)
    pending_stop = None
    last_id = time.time_ns()
    last_report = None

    def send_request(api, velocity=(0., 0., 0.)):
        nonlocal last_id
        if pub is None: return None
        if api not in (MOVE_API, STOP_API): raise ValueError('Only Move and StopMove are supported')
        request = Request()
        last_id = max(last_id + 1, time.time_ns())
        request.header.identity.id = last_id
        request.header.identity.api_id = api
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = api == MOVE_API
        request.parameter = (json.dumps(dict(zip(('x', 'y', 'z'), velocity)), allow_nan=False)
                             if api == MOVE_API else '{}')
        own_ids.append(last_id)
        pub.publish(request)
        return last_id

    def stop_request(arm=False, attempt=1):
        nonlocal pending_stop
        request_id = send_request(STOP_API)
        if request_id is not None:
            pending_stop = dict(id=request_id, arm=arm, attempt=attempt, deadline=time.monotonic()+.45)
            report('StopMove requested; awaiting matching API acknowledgement', flush=True)

    def response(msg):
        nonlocal pending_stop
        if not pending_stop or msg.header.identity.api_id != STOP_API or msg.header.identity.id != pending_stop['id']:
            return
        code = int(msg.header.status.code)
        if code == 0:
            arm = pending_stop['arm']; pending_stop = None
            if arm: policy.stop_ack(True)
            report('StopMove acknowledged (API acceptance; not a physical standstill guarantee)', flush=True)
        else:
            report(f'StopMove response code={code}; motion remains disabled', flush=True)
            pending_stop['deadline'] = time.monotonic()

    def retry_stop(now):
        nonlocal pending_stop
        if pending_stop and now >= pending_stop['deadline']:
            old = pending_stop; pending_stop = None
            if old['attempt'] < 3:
                stop_request(old['arm'], old['attempt']+1)
            else:
                policy.stop_ack(False)
                policy.fault('StopMove not confirmed after 3 attempts; use independent stop')
                policy.stop_due = False  # Avoid an unbounded automatic retry loop.
                report('STOP NOT CONFIRMED. Use the independent physical stop.', flush=True)

    def robot(msg):
        stamp = int(msg.stamp.sec)*1000000000 + int(msg.stamp.nanosec)
        policy.robot(stamp,time.monotonic(),[msg.body_height,*msg.velocity,msg.yaw_speed])

    def guard(msg):
        try: data=json.loads(msg.data)
        except (ValueError,TypeError): data={}
        policy.guard(data,time.monotonic(),time.time_ns())

    def command(msg):
        policy.velocity([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],time.monotonic())

    def other_request(msg):
        if msg.header.identity.id not in own_ids and 1001 <= msg.header.identity.api_id <= 1100:
            policy.competing_control()

    subscriptions = [
        node.create_subscription(SportModeState,'/lf/sportmodestate',robot,sensor_qos),
        node.create_subscription(String,'/p9/exploration/status',guard,sensor_qos),
        node.create_subscription(Twist,'/p9/exploration/cmd_vel',command,sensor_qos),
        node.create_subscription(Response,'/api/sport/response',response,sensor_qos),
        node.create_subscription(Request,'/api/sport/request',other_request,sensor_qos),
    ]
    report(('LIVE transport enabled; startup StopMove only.' if pub else 'READ-ONLY preview: no Sport request publisher.'),flush=True)
    report('No automatic standing or mode change. Verify standing and independent stop with the onsite operator.',flush=True)
    report('g: request StopMove and arm after acknowledgement. Then HOLD h: deadman permission.',flush=True)
    report('Space: stop/disarm. q/Esc/Ctrl+C: stop and exit. Release h: stops after <=0.6 s input expiry plus transport latency.',flush=True)
    report(f'Command caps: forward={args.max_forward} lateral={args.max_lateral} m/s; yaw={args.max_yaw} rad/s.',flush=True)
    report('After any stale input, guard finish, deadman expiry, restart, or competing action: fresh g arm is required.',flush=True)
    original_terminal = None
    try:
        original_terminal = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        if pub: stop_request()  # Zero/stop only at startup, never posture or nonzero velocity.
        next_tick = time.monotonic()
        while not quit_requested and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1)
                if key in ('q','\x1b',''):
                    quit_requested=True
                elif key == ' ':
                    policy.stop()
                elif key == 'g':
                    if policy.request_arm(time.monotonic()): stop_request(arm=True)
                    else: report('Arm rejected: '+policy.reason,flush=True)
                elif key == 'h': policy.deadman(time.monotonic())
            if quit_requested: break
            now=time.monotonic()
            retry_stop(now)
            if now >= next_tick:
                next_tick=now+.05
                decision=policy.tick(now)
                if decision.kind == 'stop': stop_request()
                elif decision.kind == 'move' and pending_stop is None: send_request(MOVE_API,decision.velocity)
                state_report=(policy.state,policy.reason)
                if state_report != last_report:
                    report('STATE '+state_report[0]+': '+state_report[1]);last_report=state_report
    except (OSError, termios.error):
        # PTY read/select failure is a request to stop, not an exit shortcut.
        report('Terminal disconnected; stopping and releasing ROS resources')
    finally:
        def stop_on_exit():
            policy.stop('process shutdown')
            if pub and rclpy.ok():
                stop_request()
                deadline=time.monotonic()+1.6
                while pending_stop and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.02)
                    retry_stop(time.monotonic())
        cleanup_session(
            stop_on_exit,
            lambda: termios.tcsetattr(sys.stdin,termios.TCSANOW,original_terminal) if original_terminal is not None else None,
            node.destroy_node,
            lambda: rclpy.shutdown() if rclpy.ok() else None,
        )


if __name__ == '__main__': main()
