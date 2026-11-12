#!/usr/bin/env python3
"""Own a bounded, loopback-only Gazebo/P9/TARE smoke run, initially STOPPED.

Run through /opt/p9/simulation/inside.sh after the P9 image entrypoint. The
operator/parent separately calls p9explore start; this script never starts a
mission or publishes motion. Ground truth is consumed only by the observer.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import time
import uuid

SIMULATION = Path('/opt/p9/simulation')
RUNTIME = Path('/opt/p9-exploration')
LOOPBACK = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'


def utc(): return datetime.now(timezone.utc).isoformat()
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def tagged_groups(tag):
    """Only process groups carrying this run's unique inherited environment tag."""
    marker = ('P9_EXPLORATION_SMOKE_RUN='+tag).encode()
    groups = set()
    for path in Path('/proc').iterdir():
        if not path.name.isdigit(): continue
        try:
            if marker in (path/'environ').read_bytes().split(b'\0'):
                group = os.getpgid(int(path.name))
                if group > 1 and group != os.getpgrp(): groups.add(group)
        except (OSError, ProcessLookupError): pass
    return groups


def clean_groups(tag):
    sent = []
    for sig, wait in ((signal.SIGINT, 5.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
        groups = tagged_groups(tag)
        sent.append(dict(signal=int(sig), groups=sorted(groups)))
        for group in groups:
            try: os.killpg(group, sig)
            except ProcessLookupError: pass
        deadline = time.monotonic()+wait
        while time.monotonic() < deadline and tagged_groups(tag): time.sleep(.1)
    return dict(signals=sent, remaining_groups=sorted(tagged_groups(tag)))


def watchdog(read_fd, tag, deadline, evidence):
    """A separate session survives parent death and bounds initialization/cleanup."""
    os.setsid()
    parent_pid = os.getppid()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, signal.SIG_IGN)
    reason = 'parent_pipe_closed'
    while time.monotonic() < deadline:
        if select.select([read_fd], [], [], min(1., max(0., deadline-time.monotonic())))[0]:
            data = os.read(read_fd, 128)
            if data == b'clean':
                os.close(read_fd); os._exit(0)
            if not data: break
    else: reason = 'outer_wall_time_deadline'
    result = clean_groups(tag)
    write_json(evidence/'watchdog.json', dict(reason=reason, utc=utc(), **result))
    if reason == 'outer_wall_time_deadline' and os.getppid() == parent_pid:
        # Also bound a supervisor stuck in a library call; all owned children
        # have received their shutdown sequence before the parent is terminated.
        try: os.kill(parent_pid, signal.SIGTERM)
        except ProcessLookupError: pass
        time.sleep(2.)
        if os.getppid() == parent_pid:
            try: os.kill(parent_pid, signal.SIGKILL)
            except ProcessLookupError: pass
    os.close(read_fd)
    os._exit(0)


def bounded(command, env, timeout):
    process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, start_new_session=True)
    try:
        stdout, stderr = process.communicate(timeout=timeout)
        return dict(command=command, returncode=process.returncode, stdout=stdout, stderr=stderr)
    except subprocess.TimeoutExpired:
        try: os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError: pass
        stdout, stderr = process.communicate(timeout=2.)
        return dict(command=command, returncode=process.returncode, stdout=stdout, stderr=stderr, timeout=True)


def confirm_zero(timeout=5.):
    """Read-only observation: both actual simulator and isolated guard outputs."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('p9_exploration_shutdown_observer')
    samples = {topic: dict(count=0, since=None, last=None, zero=False) for topic in
               ('/cmd_vel', '/p9/exploration/cmd_vel')}
    state = dict(active=None, received=None)
    def velocity(topic, msg):
        now = time.monotonic(); item = samples[topic]
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        zero = all(math.isfinite(v) and abs(v) <= 1e-8 for v in values)
        if not zero or item['last'] is None or now-item['last'] > .25: item['since'] = None
        if zero and item['since'] is None: item['since'] = now
        item.update(count=item['count']+1, last=now, zero=zero)
    def status(msg):
        try:
            value = json.loads(msg.data)
            state.update(active=value['active'], state=value['state'], mode=value['mode'], received=time.monotonic())
        except (ValueError, KeyError, TypeError): state.update(active=None, received=None)
    subscriptions = [node.create_subscription(Twist, topic, lambda msg, topic=topic: velocity(topic, msg),
                     qos_profile_sensor_data) for topic in samples]
    subscriptions.append(node.create_subscription(String, '/p9/exploration/status', status, qos_profile_sensor_data))
    deadline = time.monotonic()+timeout; confirmed = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05); now = time.monotonic()
            confirmed = (state['active'] is False and state.get('mode') == 'sim'
                         and state['received'] is not None and now-state['received'] < 1.
                         and all(v['zero'] and v['count'] >= 10 and v['since'] is not None
                                 and now-v['since'] >= 1. and now-v['last'] <= .25 for v in samples.values()))
            if confirmed: break
        now = time.monotonic()
        return dict(confirmed=confirmed, consecutive_zero_required_sec=1., observed_status=state,
                    streams={topic: dict(messages=v['count'], consecutive_zero_sec=(now-v['since'])
                              if v['since'] is not None else 0., last_receipt_age_sec=(now-v['last'])
                              if v['last'] is not None else None) for topic, v in samples.items()},
                    interpretation='Command output observation only, not physical standstill certification')
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, help='New directory; reuse is rejected')
    p.add_argument('--seconds', type=float, default=240., help='Maximum run wall time including startup (30..600)')
    p.add_argument('--profile', default='/opt/p9/config/p9nav-sim.yaml')
    p.add_argument('--world', default=str(RUNTIME/'exploration.sdf'))
    p.add_argument('--display', default=':96')
    p.add_argument('--boundary', nargs=4, type=float, default=[-7., 7., -7., 7.],
                   metavar=('MIN_X','MAX_X','MIN_Y','MAX_Y'), help='camera_init task bounds; default encloses the 13 m simulation scene')
    args = p.parse_args()
    if not all(math.isfinite(x) for x in args.boundary) or args.boundary[0] >= args.boundary[1] or args.boundary[2] >= args.boundary[3]:
        p.error('Boundary must be a finite nonempty rectangle')
    if not math.isfinite(args.seconds) or not 30 <= args.seconds <= 600: p.error('--seconds must be in [30, 600]')
    if not re.fullmatch(r':[0-9]+', args.display): p.error('--display must be a local :NUMBER')
    for path in (SIMULATION/'prepare_runtime.py', SIMULATION/'simulator.launch.py', Path(args.profile),
                 Path(args.world), RUNTIME/'observe_exploration.py'):
        if not path.is_file(): p.error('Missing input '+str(path))
    if not Path('/ws/install/setup.bash').is_file(): p.error('Mount the compiled /ws simulation workspace read-only')
    if not shutil.which('Xvfb') or not shutil.which('fluxbox'): p.error('Xvfb and fluxbox are required')
    # Set before any ROS imports. No hardware NIC/DDS domain is ever selected.
    os.environ.update(ROS_DOMAIN_ID='94', GZ_PARTITION='ssy-p9-nav-20261110', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI=LOOPBACK, SAR_ROBOT_IFACE='lo', ROS_LOCALHOST_ONLY='1', DISPLAY=args.display)
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False); (output/'logs').mkdir()
    tag = str(uuid.uuid4()); child_env = dict(os.environ, P9_EXPLORATION_SMOKE_RUN=tag)
    child_env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    started = time.monotonic(); children = {}; logs = []; requested_signal = [None]
    read_fd, write_fd = os.pipe()
    watcher = os.fork()
    if watcher == 0:
        os.close(write_fd); watchdog(read_fd, tag, started+args.seconds+35., output)
    os.close(read_fd)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda number, frame: requested_signal.__setitem__(0, number))
    report = dict(started_utc=utc(), arguments=vars(args), children={}, watchdog_pid=watcher,
                  supervisor_pid=os.getpid(), run_tag=tag, ros_domain_id=94, display=args.display,
                  hardware_control=False, automatic_exploration_start=False,
                  ground_truth_role='read-only observer evaluation; never a control input',
                  supervisor_sha256=sha(__file__), source_profile_sha256=sha(args.profile), world_sha256=sha(args.world))
    def save(): write_json(output/'run.json', report)
    def start(name, command):
        log = (output/'logs'/f'{name}.log').open('x'); logs.append(log)
        proc = subprocess.Popen(command, env=child_env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = proc
        report['children'][name] = dict(pid=proc.pid, process_group=proc.pid, command=command, log=str(log.name))
        save(); return proc
    failure = None; reason = None
    save()
    try:
        # TARE is loaded by p9explore; prepare_runtime independently proves P9
        # overlay priority over /ws for estimator, terrain and local planning.
        prepare = start('prepare', ['python3', str(SIMULATION/'prepare_runtime.py'),
                        '--output', str(output/'runtime'), '--profile', args.profile, '--world', args.world])
        if prepare.wait(timeout=30.) != 0:
            raise RuntimeError('Runtime/model validation failed; inspect logs/prepare.log')
        report['children']['prepare']['returncode'] = prepare.returncode
        xvfb = start('xvfb', ['Xvfb', args.display, '-screen', '0', '1440x900x24', '-nolisten', 'tcp'])
        socket = Path('/tmp/.X11-unix/X'+args.display[1:]); available = False
        for _ in range(60):
            if xvfb.poll() is not None: raise RuntimeError('Xvfb exited; choose a free display')
            if requested_signal[0] is not None: raise InterruptedError('Interrupted during display setup')
            if shutil.which('xdpyinfo'):
                available = subprocess.run(['xdpyinfo', '-display', args.display], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, timeout=1.).returncode == 0
            else: available = socket.exists()
            if available: break
            time.sleep(.1)
        if not available: raise RuntimeError('Display did not become available')
        start('fluxbox', ['fluxbox'])
        start('simulator', ['ros2', 'launch', str(SIMULATION/'simulator.launch.py'),
                           'world:='+str(output/'runtime/world.sdf'), 'output:='+str(output/'runtime')])
        start('exploration', ['p9explore', 'sim', '--config', str(Path(args.profile).resolve()),
                             '--output', str(output/'navigation'), '--boundary', *map(str, args.boundary),
                             '--sim-command-output', '--sim-auto-resume-inputs', '--rviz'])
        start('observer', ['python3', str(RUNTIME/'observe_exploration.py'), '--output', str(output/'observer'),
                           '--seconds', str(args.seconds+20.), '--stop-file', str(output/'stop-observer')])
        print('Simulation running, commands STOPPED. Output: '+str(output), flush=True)
        print('In the same domain94/loopback environment, inspect p9explore status then explicitly p9explore start.', flush=True)
        print('Actual RViz display '+args.display+'; no hardware controller is launched.', flush=True)
        while requested_signal[0] is None:
            ended = [name for name, child in children.items() if name != 'prepare' and child.poll() is not None]
            if ended: raise RuntimeError('Required process exited: '+', '.join(ended))
            if time.monotonic()-started >= args.seconds:
                reason = 'maximum_wall_time'; break
            time.sleep(.2)
        if requested_signal[0] is not None: reason = 'signal'
    except Exception as exc:
        failure = repr(exc); reason = 'error'; print(failure, file=sys.stderr, flush=True)
    finally:
        if 'exploration' in children:
            try:
                report['stop_request'] = bounded(['p9explore', 'stop', '--timeout', '4'], child_env, 7.)
                report['zero_confirmation'] = confirm_zero(5.)
            except Exception as exc: report['stop_error'] = repr(exc)
        else: report['stop_request'] = dict(skipped='exploration process never started')
        (output/'stop-recording').touch(exist_ok=True)
        (output/'stop-observer').touch(exist_ok=True)
        if 'observer' in children:
            try: children['observer'].wait(timeout=3.)
            except subprocess.TimeoutExpired: pass
        summary_path = output/'observer/summary.json'
        if summary_path.is_file():
            try:
                observed = json.loads(summary_path.read_text())
                report['observed_nonzero_command_samples'] = observed['command_output']['nonzero_samples']
                report['ground_truth_max_displacement_m'] = observed['ground_truth_evaluation_only']['max_displacement_from_first_m']
                report['exploration_flags'] = observed['exploration_flags']
            except (ValueError, KeyError, TypeError) as exc: report['observer_summary_error'] = repr(exc)
        report['cleanup'] = clean_groups(tag)
        for name, child in children.items():
            try: child.wait(timeout=.2)
            except subprocess.TimeoutExpired: pass
            report['children'][name]['returncode'] = child.poll()
        report.update(ended_utc=utc(), elapsed_wall_sec=time.monotonic()-started,
                      signal=requested_signal[0], end_reason=reason, error=failure)
        save()
        for log in logs: log.close()
        # The watchdog exits without intervention only after actual group cleanup.
        if not report['cleanup']['remaining_groups']: os.write(write_fd, b'clean')
        os.close(write_fd)
        os.waitpid(watcher, 0)
    return 1 if failure or report['cleanup']['remaining_groups'] or not report.get('zero_confirmation', {}).get('confirmed') else 0


if __name__ == '__main__': sys.exit(main())
