"""Validate deployment choices separately from historical estimator defaults."""
import copy
import math
import os
from pathlib import Path
import numpy as np
import yaml


def home():
    return Path(os.environ.get('P9_HOME', str(Path(__file__).resolve().parents[3])))


def load(path, mode):
    if mode not in ('live', 'replay'):
        raise ValueError('mode must be live or replay')
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict) or config.get('version') != 1:
        raise ValueError('Expected a version: 1 P9 configuration')
    unknown = set(config) - {'version', 'calibration', 'sensor', 'mapping'}
    if unknown:
        raise ValueError(f'Unknown config sections: {sorted(unknown)}')
    c = config.get('calibration', {})
    rotation = np.asarray(c.get('rotation'), dtype=float)
    translation = np.asarray(c.get('translation'), dtype=float)
    if rotation.size != 9 or translation.size != 3:
        raise ValueError('Enter explicit LiDAR -> camera IMU rotation[9] and translation[3]; see docs/calibration.md. Historical experimental profile is separately named.')
    rotation = rotation.reshape(3, 3)
    if (not np.isfinite(rotation).all() or not np.isfinite(translation).all()
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)):
        raise ValueError('Calibration must be a finite proper rigid transform (det(R)=+1)')
    if c.get('status') not in ('measured', 'experimental', 'synthetic'):
        raise ValueError('Set calibration.status to measured, experimental or synthetic, accurately')
    sensor = dict(config.get('sensor', {}))
    forbidden = {'mode', 'use_sim_time', 'output_cloud_topic', 'output_imu_topic'} & set(sensor)
    if forbidden:
        raise ValueError(f'Entrypoint owns these sensor parameters: {sorted(forbidden)}')
    if not math.isfinite(float(sensor.get('clock_offset_sec', 0.))):
        raise ValueError('clock_offset_sec must be finite')
    sensor.update(mode=mode, use_sim_time=(mode == 'replay'))
    mapping = dict(config.get('mapping', {}))
    forbidden = {'input_cloud_topic', 'input_odom_topic', 'world_frame', 'native_frame', 'output_dir', 'use_sim_time'} & set(mapping)
    if forbidden:
        raise ValueError(f'Entrypoint owns these mapping parameters: {sorted(forbidden)}')
    mapping.update(use_sim_time=(mode == 'replay'))
    pointlio = copy.deepcopy(yaml.safe_load((home()/'config/p9-pointlio.yaml').read_text())['/**']['ros__parameters'])
    pointlio['use_sim_time'] = mode == 'replay'
    pointlio['mapping']['extrinsic_R'] = [float(v) for v in rotation.flat]
    pointlio['mapping']['extrinsic_T'] = [float(v) for v in translation.flat]
    return config, sensor, mapping, pointlio
