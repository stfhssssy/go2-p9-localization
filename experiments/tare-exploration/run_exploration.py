#!/usr/bin/env python3
"""Own an isolated Gazebo, P9, TARE and RViz experiment, initially stopped.

Run through /output/integration/inside.sh. This supervisor never submits an
exploration start or fixed goal. A separate explicit /p9/exploration/start call
is required after the guard reports fresh inputs. Ground truth is evaluation
only; no observer or control based on ground truth is launched here.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
SIMULATION = Path('/opt/p9/simulation')


def now_utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('seconds must be finite and positive')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New evidence directory; never reuse an existing run')
    parser.add_argument('--seconds', type=positive_seconds, default=900., help='Maximum wall time, including initialization')
    parser.add_argument('--profile', default=str(HERE/'p9-exploration-sim.yaml'))
    parser.add_argument('--world', default=str(HERE/'exploration.sdf'))
    parser.add_argument('--rviz-config', default=str(HERE/'exploration.rviz'))
    parser.add_argument('--display', default=':94')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--existing-display', action='store_true', help='Use an existing X server without owning it')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '94' or os.environ.get('GZ_PARTITION') != 'ssy-p9-nav-20261110':
        parser.error('Use integration/inside.sh: isolated ROS domain 94 and Gazebo partition required')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp' or 'name="lo"' not in os.environ.get('CYCLONEDDS_URI', ''):
        parser.error('Use integration/inside.sh: explicit CycloneDDS loopback configuration required')
    for path in (Path(args.profile), Path(args.world), HERE/'exploration.launch.py'):
        if not path.is_file():
            parser.error('Missing required input: '+str(path))
    if not args.no_rviz and not Path(args.rviz_config).is_file():
        parser.error('Missing RViz configuration: '+args.rviz_config)

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output/'logs').mkdir()
    os.environ['DISPLAY'] = args.display
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    children, log_handles = {}, []
    interrupted = {'signal': None}
    previous_handlers = {sig: signal.signal(sig, lambda number, frame: interrupted.update(signal=number))
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    started_wall = time.monotonic()
    report = dict(started_utc=now_utc(), arguments=vars(args), children={},
                  ros_domain_id=94, gz_partition=os.environ['GZ_PARTITION'],
                  cyclonedds_uri=os.environ['CYCLONEDDS_URI'], hardware_control=False,
                  automatic_exploration_start=False, ground_truth_role='Evaluation only; never control input',
                  integration_source_sha256={p.name: digest(p) for p in sorted(HERE.iterdir())
                      if p.is_file() and p.suffix in ('.py', '.sh', '.yaml', '.sdf', '.rviz')})

    def save():
        destination = output/'run.json'
        temporary = destination.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        temporary.replace(destination)

    def start(name, command):
        stream = (output/'logs'/f'{name}.log').open('x')
        log_handles.append(stream)
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = child
        report['children'][name] = dict(pid=child.pid, process_group=child.pid, command=command, log=str(stream.name))
        save()
        return child

    failure, end_reason = None, None
    save()
    try:
        sys.path.insert(0, str(SIMULATION))
        from prepare_runtime import prepare
        from ament_index_python.packages import get_package_prefix
        tare_prefix = Path(get_package_prefix('tare_planner')).resolve()
        if not tare_prefix.is_relative_to(Path('/tare_ws/install').resolve()):
            raise RuntimeError('TARE resolved outside new overlay: '+str(tare_prefix))
        report['tare_package_prefix'] = str(tare_prefix)
        prepare(output/'runtime', args.profile, args.world)
        save()
        if not args.existing_display:
            xserver = start('xvfb', ['Xvfb', args.display, '-screen', '0', '1440x900x24', '-nolisten', 'tcp'])
            available = False
            for _ in range(50):
                if xserver.poll() is not None:
                    raise RuntimeError('Xvfb exited; inspect logs/xvfb.log and use a free display')
                if shutil.which('xdpyinfo'):
                    available = subprocess.run(['xdpyinfo', '-display', args.display],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
                else:
                    available = Path('/tmp/.X11-unix/X'+args.display.lstrip(':').split('.')[0]).exists()
                if available:
                    break
                time.sleep(.1)
            if not available:
                raise RuntimeError('X server did not become available')
            if shutil.which('fluxbox'):
                start('window-manager', ['fluxbox'])
        start('simulator', ['ros2', 'launch', str(SIMULATION/'simulator.launch.py'),
                           'world:='+str(output/'runtime/world.sdf'), 'output:='+str(output/'runtime')])
        start('exploration', ['ros2', 'launch', str(HERE/'exploration.launch.py'),
                             'output:='+str(output/'navigation')])
        if not args.no_rviz:
            start('rviz', ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(Path(args.rviz_config).resolve()),
                           '--ros-args', '-p', 'use_sim_time:=true'])
        print('Exploration simulation started with commands STOPPED. Evidence: '+str(output), flush=True)
        print('Wait for fresh guard inputs, then explicitly call /p9/exploration/start in the same inside.sh environment.', flush=True)
        while interrupted['signal'] is None:
            ended = [name for name, child in children.items()
                     if name != 'window-manager' and child.poll() is not None]
            if ended:
                raise RuntimeError('Required process exited: '+', '.join(ended))
            if time.monotonic()-started_wall >= args.seconds:
                end_reason = 'maximum_wall_time'
                break
            time.sleep(.25)
        if interrupted['signal'] is not None:
            end_reason = 'signal'
    except Exception as exc:
        failure, end_reason = repr(exc), 'error'
        print(failure, file=sys.stderr, flush=True)
    finally:
        # This service is in the isolated simulation domain. It does not command
        # a real robot or affect any navigation instance in other DDS domains.
        if 'exploration' in children:
            try:
                stopped = subprocess.run(['ros2', 'service', 'call', '/p9/exploration/stop',
                    'std_srvs/srv/Trigger', '{}'], text=True, capture_output=True, timeout=6)
                report['stop'] = dict(returncode=stopped.returncode, stdout=stopped.stdout, stderr=stopped.stderr)
            except (OSError, subprocess.TimeoutExpired) as exc:
                report['stop'] = dict(error=repr(exc))
        else:
            report['stop'] = dict(not_requested='Exploration launch was not started')
        # Signal only groups created by this supervisor, in reverse launch order.
        # ROS launch receives SIGINT first and owns shutdown of its ROS children.
        for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            for child in reversed(list(children.values())):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic()+grace
            while time.monotonic() < deadline and any(p.poll() is None for p in children.values()):
                time.sleep(.1)
        for name, child in children.items():
            report['children'][name]['returncode'] = child.poll()
        report.update(ended_utc=now_utc(), elapsed_wall_sec=time.monotonic()-started_wall,
                      signal=interrupted['signal'], end_reason=end_reason, error=failure)
        save()
        for stream in log_handles:
            stream.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return 1 if failure else 0


if __name__ == '__main__':
    sys.exit(main())
