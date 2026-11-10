#!/usr/bin/env python3
"""Run isolated Gazebo plus the NEW P9 navigation release, owning only its children."""
import argparse
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from prepare_runtime import prepare

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New run directory; never overwrite an earlier run')
    parser.add_argument('--profile', default='/opt/p9/config/p9nav-sim.yaml')
    parser.add_argument('--world')
    parser.add_argument('--display', default=':94')
    parser.add_argument('--existing-display', action='store_true', help='Use an existing X server without owning it')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--sim-command-output', action='store_true', help='Explicitly permit guarded commands to Gazebo /cmd_vel')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '94' or os.environ.get('GZ_PARTITION') != 'ssy-p9-nav-20261110':
        parser.error('Run through simulation/inside.sh: isolated ROS_DOMAIN_ID=94 / GZ_PARTITION required')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output/'logs').mkdir()
    prepare(output/'runtime', args.profile, args.world)
    os.environ['DISPLAY'] = args.display
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    children, handles = {}, []
    interrupted = {'signal': None}
    old_handlers = {sig: signal.signal(sig, lambda number, frame: interrupted.update(signal=number))
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    report = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'arguments': vars(args), 'ros_domain_id': 94, 'gz_partition': os.environ['GZ_PARTITION'],
              'children': {}, 'hardware_control': False, 'ground_truth_role': 'evaluation only'}

    def save():
        (output/'run.json').write_text(json.dumps(report, indent=2)+'\n')

    def start(name, command):
        log = (output/'logs'/f'{name}.log').open('x')
        handles.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children[name] = process
        report['children'][name] = {'pid': process.pid, 'command': command, 'log': str(log.name)}
        save()
        return process

    failure = None
    try:
        if not args.existing_display:
            xserver = start('xvfb', ['Xvfb', args.display, '-screen', '0', '1440x900x24', '-nolisten', 'tcp'])
            for _ in range(30):
                if xserver.poll() is not None:
                    raise RuntimeError('Xvfb failed; inspect logs/xvfb.log (do not reuse another run display)')
                if shutil.which('xdpyinfo') and subprocess.run(['xdpyinfo', '-display', args.display],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                    break
                time.sleep(.1)
            if shutil.which('fluxbox'):
                start('window-manager', ['fluxbox'])
        start('simulator', ['ros2', 'launch', str(HERE/'simulator.launch.py'),
            'world:='+str(output/'runtime/world.sdf'), 'output:='+str(output/'runtime')])
        nav = ['ros2', 'run', 'go2_p9_navigation', 'p9nav', 'sim', '--config', str(Path(args.profile).resolve()),
               '--output', str(output/'navigation')]
        if args.sim_command_output:
            nav.append('--sim-command-output')
        if not args.no_rviz:
            nav.append('--rviz')
        start('navigation', nav)
        print('Simulation started. Evidence directory: '+str(output), flush=True)
        print('Validate in another terminal through simulation/inside.sh. Ctrl-C stops owned processes.', flush=True)
        while interrupted['signal'] is None:
            ended = [name for name, child in children.items() if name != 'window-manager' and child.poll() is not None]
            if ended:
                raise RuntimeError('Required process exited: '+', '.join(ended))
            time.sleep(.5)
    except Exception as exc:
        failure = repr(exc)
        print(failure, file=sys.stderr)
    finally:
        # Request stop while the guard is still alive. This is only the isolated simulator domain.
        try:
            stop = subprocess.run(['ros2', 'service', 'call', '/p9/navigation/stop', 'std_srvs/srv/Trigger', '{}'],
                                  text=True, capture_output=True, timeout=5)
            report['stop'] = {'returncode': stop.returncode, 'stdout': stop.stdout, 'stderr': stop.stderr}
        except (OSError, subprocess.TimeoutExpired) as exc:
            report['stop'] = {'error': repr(exc)}
        for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            for process in reversed(list(children.values())):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic()+grace
            while time.monotonic() < deadline and any(p.poll() is None for p in children.values()):
                time.sleep(.1)
        for name, child in children.items():
            report['children'][name]['returncode'] = child.poll()
        report.update(ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                      signal=interrupted['signal'], error=failure)
        save()
        for stream in handles:
            stream.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 1 if failure else 0


if __name__ == '__main__':
    sys.exit(main())
