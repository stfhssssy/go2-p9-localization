"""Explicit live/replay/sim entries. No control or automatic sensor drivers."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from .configuration import home, load, estimator_config_path


def experimental_delta():
    """Preserve release provenance while identifying an explicit local experiment."""
    path = home()/'provenance/experiment-delta.json'
    if not path.exists():
        return None
    contents = path.read_bytes()
    return dict(sha256=hashlib.sha256(contents).hexdigest(), manifest=json.loads(contents))


def main():
    parser = argparse.ArgumentParser(description='P9 LiDAR/camera-IMU localization and map accumulation')
    sub = parser.add_subparsers(dest='command', required=True)
    for mode in ('live', 'replay', 'sim'):
        p = sub.add_parser(mode)
        p.add_argument('--config', required=True)
        p.add_argument('--output', default=None, help='New directory, normally inside a mounted host directory')
        p.add_argument('--rviz', action='store_true')
        if mode == 'replay':
            p.add_argument('--bag', required=True)
            p.add_argument('--rate', type=float, default=.5)
            p.add_argument('--exit-on-finish', action='store_true', help='Save and exit 5 seconds after bag EOF')
    p = sub.add_parser('diagnose')
    p.add_argument('--seconds', type=float, default=10.)
    p.add_argument('--cloud-topic', default='/utlidar/cloud')
    p.add_argument('--imu-topic', default='/camera/camera/imu')
    p.add_argument('--output', default=None)
    sub.add_parser('save')
    sub.add_parser('version')
    a = parser.parse_args()
    if a.command == 'version':
        original = (home()/'provenance/source-manifest.json').read_text()
        delta = experimental_delta()
        if delta is None:
            print(original)
        else:
            print(json.dumps(dict(source_manifest=json.loads(original), experimental_delta=delta), indent=2))
        return
    if a.command == 'diagnose':
        from .diagnose import run
        return run(a.seconds, a.cloud_topic, a.imu_topic, a.output)
    if a.command == 'save':
        os.execvp('ros2', ['ros2', 'service', 'call', '/p9/save', 'std_srvs/srv/Trigger', '{}'])
    config_path = Path(a.config).expanduser().resolve()
    try:
        config, sensor, _, engine = load(config_path, a.command)
        if a.command == 'replay' and (not math.isfinite(a.rate) or a.rate <= 0 or not Path(a.bag).exists()):
            raise ValueError('Replay needs an existing bag and a finite positive rate')
    except (ValueError, OSError, TypeError, KeyError) as error:
        parser.error(str(error))
    if config['calibration']['status'] != 'measured':
        print('WARNING: calibration is '+config['calibration']['status']+'; this run does not establish robot extrinsics or absolute accuracy.', flush=True)
    output = Path(a.output or ('./run-output/'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('Output directory is not empty; choose a new --output')
    output.mkdir(parents=True, exist_ok=True)
    record = dict(config=config, mode=a.command, bag=str(Path(a.bag).resolve()) if a.command == 'replay' else None,
                  rate=getattr(a, 'rate', None), source_revision=os.environ.get('P9_SOURCE_REVISION', 'native-unrecorded'),
                  binary_sha256=(home()/'provenance/binary.sha256').read_text().strip() if (home()/'provenance/binary.sha256').exists() else None,
                  ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                  rmw=os.environ.get('RMW_IMPLEMENTATION'), source_manifest=json.loads((home()/'provenance/source-manifest.json').read_text()))
    engine_path = estimator_config_path(config_path, config)
    record.update(estimator_config_path=str(engine_path),
                  estimator_config_sha256=hashlib.sha256(engine_path.read_bytes()).hexdigest(),
                  effective_sensor_parameters=sensor,
                  effective_pointlio_parameters=engine)
    delta = experimental_delta()
    if delta is not None:
        record['experimental_delta'] = delta
    (output/'effective-config.json').write_text(json.dumps(record, indent=2)+'\n')
    os.environ['ROS_LOG_DIR'] = str(output/'ros-logs')
    command = ['ros2', 'launch', 'go2_p9', 'go2_p9.launch.py',
               'mode:='+a.command, 'config:='+str(config_path), 'output:='+str(output),
               'rviz:='+str(a.rviz).lower()]
    if a.command == 'replay':
        command += ['bag:='+str(Path(a.bag).resolve()), 'rate:='+str(a.rate),
                    'exit_on_finish:='+str(a.exit_on_finish).lower()]
    print('P9 output: '+str(output), flush=True)
    os.execvp(command[0], command)
