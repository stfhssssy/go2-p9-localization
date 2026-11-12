#!/usr/bin/env python3
"""Standalone P9 + TARE entry. Live/replay never publish hardware commands."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import yaml
from make_config import validate_options, tare_parameters, core_profile, local_parameters


def runtime_home():
    return Path(os.environ.get('P9_EXPLORATION_HOME', Path(__file__).resolve().parent)).resolve()


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='mode', required=True)
    for mode in ('live', 'replay', 'sim'):
        s = sub.add_parser(mode)
        s.add_argument('--config', default='/opt/p9/config/p9nav-'+mode+'.yaml')
        s.add_argument('--navigation-config')
        s.add_argument('--tare-config', default='/tare_ws/src/tare_planner/config/indoor.yaml')
        s.add_argument('--output', required=True)
        s.add_argument('--boundary', nargs=4, type=float, required=True,
                       metavar=('MIN_X', 'MAX_X', 'MIN_Y', 'MAX_Y'),
                       help='Task rectangle in camera_init metres; used by TARE AND command guard')
        s.add_argument('--max-forward', type=float, default=.20)
        s.add_argument('--max-lateral', type=float, default=.10)
        s.add_argument('--max-yaw', type=float, default=.35)
        s.add_argument('--rviz', action='store_true')
        s.add_argument('--validate-only', action='store_true', help='Write effective configuration without starting ROS nodes')
        s.add_argument('--sim-command-output', action='store_true')
        s.add_argument('--sim-auto-resume-inputs', action='store_true')
        s.add_argument('--allow-return-home', action='store_true', help='Unverified return behavior, simulation only')
        if mode == 'replay':
            s.add_argument('--bag', required=True)
            s.add_argument('--rate', type=float, default=.5)
            s.add_argument('--exit-on-finish', action='store_true')
    for name in ('start', 'stop', 'status'):
        s = sub.add_parser(name)
        s.add_argument('--timeout', type=float, default=5.)
    return p


def service(name, timeout):
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('p9_exploration_'+name+'_client')
    try:
        client = node.create_client(Trigger, '/p9/exploration/'+name)
        deadline = time.monotonic()+timeout
        if not client.wait_for_service(timeout_sec=max(0., deadline-time.monotonic())):
            raise RuntimeError('Exploration service unavailable; verify ROS_DOMAIN_ID and network interface')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., deadline-time.monotonic()))
        if not future.done():
            raise RuntimeError('Exploration service timed out')
        result = future.result()
        if name == 'status' and result.success:
            print(json.dumps(json.loads(result.message), indent=2))
        else:
            print(json.dumps(dict(success=result.success, message=result.message)))
        return 0 if result.success else 2
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(a):
    from go2_p9.configuration import load, estimator_config_path
    from go2_p9_navigation.configuration import load_navigation, share
    guard = validate_options(a.mode, a.boundary, a.max_forward, a.max_lateral, a.max_yaw,
                             a.sim_command_output, a.sim_auto_resume_inputs, a.allow_return_home)
    source = Path(a.config).expanduser().resolve()
    nav = Path(a.navigation_config or share()/'config/navigation.yaml').expanduser().resolve()
    tare = Path(a.tare_config).expanduser().resolve()
    output = Path(a.output).expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError('Choose a new empty --output directory')
    core, _, _, _ = load(source, a.mode)
    params, _ = load_navigation(nav, a.mode, a.sim_command_output, core_config=core)
    params = local_parameters(params, guard)
    estimator = estimator_config_path(source, core)
    effective = core_profile(core, estimator)
    tare_document = tare_parameters(tare, a.mode, a.allow_return_home)
    if a.mode == 'replay' and (not Path(a.bag).exists() or not math.isfinite(a.rate) or a.rate <= 0):
        raise ValueError('Replay needs an existing bag and finite positive rate')
    # Validate before creating artifacts and revalidate the copied profile afterwards.
    for key in ('exploration.launch.py', 'guard_node.py', 'boundary.py', 'exploration.rviz'):
        if not (runtime_home()/key).is_file(): raise ValueError('Runtime file missing: '+key)
    output.mkdir(parents=True, exist_ok=True)
    profile = output/'effective-core.yaml'
    profile.write_text(yaml.safe_dump(effective, sort_keys=False))
    load(profile, a.mode)
    tare_out = output/'effective-tare.yaml'
    tare_out.write_text(yaml.safe_dump(tare_document, sort_keys=False))
    record = dict(schema_version=1, created_utc=datetime.now(timezone.utc).isoformat(), mode=a.mode,
                  runtime_root=str(runtime_home()), output=str(output), use_sim_time=a.mode != 'live',
                  source_core_config=str(source), source_core_sha256=sha(source),
                  effective_core_config=str(profile), effective_core_sha256=sha(profile),
                  estimator_config=str(estimator), estimator_sha256=sha(estimator),
                  source_navigation_config=str(nav), source_navigation_sha256=sha(nav),
                  source_tare_config=str(tare), source_tare_sha256=sha(tare),
                  tare_config=str(tare_out), tare_sha256=sha(tare_out), parameters=params, guard=guard,
                  calibration=effective['calibration'], sensor=effective['sensor'],
                  mapping=effective['mapping'], initial_state='STOPPED',
                  task_boundary_frame='camera_init', boundary_configured=True,
                  stop_on_finished=not a.allow_return_home, sim_auto_resume_inputs=a.sim_auto_resume_inputs,
                  sim_command_output=a.sim_command_output, hardware_control=False,
                  command_topic='/p9/exploration/cmd_vel', rviz=a.rviz,
                  bag=str(Path(a.bag).resolve()) if a.mode == 'replay' else None,
                  rate=getattr(a, 'rate', None), exit_on_finish=getattr(a, 'exit_on_finish', False),
                  ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID'),
                  source_revision=os.environ.get('P9_EXPLORATION_REVISION', 'unrecorded'),
                  return_home_validation='failed in prior simulation; disabled outside explicit sim experiment')
    manifest = output/'effective-exploration.json'
    manifest.write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    return manifest


def main(argv=None):
    p = parser(); a = p.parse_args(argv)
    try:
        if a.mode in ('start', 'stop', 'status'):
            if not math.isfinite(a.timeout) or not 0 < a.timeout <= 30:
                raise ValueError('--timeout must be in (0, 30] seconds')
            return service(a.mode, a.timeout)
        manifest = prepare(a)
        print('Effective exploration configuration: '+str(manifest), flush=True)
        print('Startup STOPPED; use p9explore start after checking topics, boundary and calibration.', flush=True)
        if a.validate_only:
            return 0
        command = ['ros2', 'launch', str(runtime_home()/'exploration.launch.py'), 'manifest:='+str(manifest)]
        os.execvp(command[0], command)
    except (ValueError, TypeError, KeyError, OSError, RuntimeError) as error:
        p.error(str(error))


if __name__ == '__main__':
    sys.exit(main())
