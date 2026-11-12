#!/usr/bin/env python3
"""Start the actual TARE binary on loopback and verify true/false ROS parameters.

No sensor, odometry, exploration-start, or robot-control messages are published.
Run the caller in a Docker --network none container for an additional boundary.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

BINARY = '/tare_ws/install/tare_planner/lib/tare_planner/tare_planner_node'
CONFIG = '/tare_ws/src/tare_planner/config/indoor.yaml'
LOOPBACK = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', default=BINARY)
    p.add_argument('--config', default=CONFIG, help='Frozen source/effective TARE ROS YAML')
    p.add_argument('--timeout', type=float, default=30., help='Per-process startup/query deadline, at most 60 seconds')
    p.add_argument('--output', help='Optional JSON result path; logs remain in the temporary directory reported')
    a = p.parse_args()
    if not 0 < a.timeout <= 60: p.error('--timeout must be in (0, 60] seconds')
    for name in ('binary', 'config'):
        path = Path(getattr(a, name)).expanduser().resolve()
        if not path.is_file(): p.error('Missing '+name+': '+str(path))
        setattr(a, name, str(path))
    if not os.access(a.binary, os.X_OK): p.error('--binary is not executable')
    return a


def stop_child(child):
    """Only stop the process group started by this test, with bounded escalation."""
    if child.poll() is None:
        try: os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError: pass
        try:
            child.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            try: os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError: pass
            try:
                child.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                try: os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                child.wait(timeout=2.)
    return child.returncode


def test_value(args, expected, log_directory):
    # These imports occur only after forcing the DDS environment in main().
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient
    from rcl_interfaces.msg import ParameterType
    from rclpy.signals import SignalHandlerOptions

    label = 'true' if expected else 'false'
    command = [args.binary, '--ros-args', '--params-file', args.config,
               '-p', 'kUseFrontier:='+label, '-p', 'kAutoStart:=false',
               '-p', 'use_sim_time:=false']
    log_path = log_directory/('tare-frontier-'+label+'.log')
    result = {'requested_kUseFrontier': expected, 'command': command, 'log': str(log_path)}
    child = None
    node = None
    initialized = False
    started = time.monotonic()
    try:
        with log_path.open('w') as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                     env=os.environ.copy(), start_new_session=True)
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            node = rclpy.create_node('p9_frontier_parameter_check_'+label)
            client = AsyncParameterClient(node, '/tare_planner_node')
            deadline = started + args.timeout
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError('Actual TARE binary exited before parameter service became ready')
                if client.wait_for_services(timeout_sec=.1): break
                rclpy.spin_once(node, timeout_sec=0.)
            else:
                raise TimeoutError('Actual TARE parameter services did not become ready')
            future = client.get_parameters(['kUseFrontier', 'kAutoStart'])
            while not future.done() and time.monotonic() < deadline:
                if child.poll() is not None: raise RuntimeError('Actual TARE binary exited during query')
                rclpy.spin_once(node, timeout_sec=.05)
            if not future.done(): raise TimeoutError('Actual TARE parameter query timed out')
            reply = future.result()
            if reply is None or len(reply.values) != 2: raise AssertionError('Unexpected parameter response')
            frontier, auto_start = reply.values
            result.update(actual_parameter_type=int(frontier.type), actual_kUseFrontier=bool(frontier.bool_value),
                          actual_kAutoStart=bool(auto_start.bool_value))
            if frontier.type != ParameterType.PARAMETER_BOOL:
                raise AssertionError('kUseFrontier is not a declared BOOL parameter')
            if frontier.bool_value is not expected:
                raise AssertionError('kUseFrontier override was not respected')
            if auto_start.type != ParameterType.PARAMETER_BOOL or auto_start.bool_value:
                raise AssertionError('Test must have kAutoStart=false')
            result['passed'] = True
    except (Exception, KeyboardInterrupt) as error:
        result.update(passed=False, error_type=type(error).__name__, error=str(error))
    finally:
        if child is not None: result['process_returncode'] = stop_child(child)
        if node is not None: node.destroy_node()
        if initialized and rclpy.ok(): rclpy.shutdown()
        result['elapsed_seconds'] = round(time.monotonic()-started, 3)
    return result


def main():
    args = parse_args()
    def interrupted(*_):
        raise KeyboardInterrupt('Parameter test interrupted')
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    # Before importing ROS or launching a child: no host/robot NIC is enabled.
    os.environ.update(ROS_DOMAIN_ID='102', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI=LOOPBACK,
                      SAR_ROBOT_IFACE='lo')
    sdk = '/tare_ws/src/tare_planner/or-tools/lib'
    os.environ['LD_LIBRARY_PATH'] = sdk + (':'+os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else '')
    directory = Path(tempfile.mkdtemp(prefix='p9-frontier-parameter-'))
    result = {'test':'actual TARE declared frontier parameter', 'domain':102, 'interface':'lo',
              'hardware_access':False, 'sensor_inputs_published':False, 'automatic_exploration_start':False,
              'binary_sha256':hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(),
              'config_sha256':hashlib.sha256(Path(args.config).read_bytes()).hexdigest(),
              'temporary_log_directory':str(directory), 'cases':[]}
    for expected in (True, False):
        case = test_value(args, expected, directory)
        result['cases'].append(case)
        if not case['passed']: break
    result['passed'] = len(result['cases']) == 2 and all(case['passed'] for case in result['cases'])
    text = json.dumps(result, indent=2, allow_nan=False)+'\n'
    if args.output:
        output = Path(args.output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(text)
    print(text, end='')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
