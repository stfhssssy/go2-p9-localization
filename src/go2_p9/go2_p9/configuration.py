"""Validate deployment choices separately from historical estimator defaults."""
import copy
import math
import os
from pathlib import Path
import numpy as np
import yaml


# Keep these contracts in sync with each node's declared defaults. Tests read
# those declarations with AST only, so validation never imports a ROS node.
SENSOR_NODE_PARAMETERS = frozenset({
    'input_cloud_topic', 'input_imu_topic', 'output_cloud_topic', 'output_imu_topic',
    'gyro_topic', 'accel_topic', 'mode', 'imu_mode', 'cloud_frame', 'imu_frame',
    'clock_offset_sec', 'point_time_field', 'point_time_scale', 'min_range', 'max_range',
    'max_scan_duration_sec', 'raw_gyro_frame', 'raw_accel_frame', 'max_accel_gap_sec',
    'raw_queue_size', 'raw_buffer_sec', 'raw_pending_timeout_sec', 'cloud_queue_size',
    'imu_queue_size', 'max_message_age_sec', 'future_tolerance_sec',
    'clock_tolerance_sec', 'input_timeout_sec',
})
MAPPING_NODE_PARAMETERS = frozenset({
    'input_cloud_topic', 'input_odom_topic', 'world_frame', 'native_frame', 'output_dir',
    'voxel_size', 'max_map_voxels', 'max_path_poses', 'max_pending_pairs', 'pair_timeout_sec',
    'max_cloud_points', 'max_cloud_bytes', 'input_queue_depth', 'input_reliability',
    'publish_terrain_inputs', 'publish_map_alias', 'map_alias_frame', 'publish_base_frame',
    'base_frame', 'T_imu_base_rotation', 'T_imu_base_translation', 'map_publish_period_sec',
    'path_publish_period_sec', 'stale_timeout_sec', 'tracking_timeout_sec',
})
SENSOR_ENTRYPOINT_PARAMETERS = frozenset({'mode', 'use_sim_time', 'output_cloud_topic', 'output_imu_topic'})
MAPPING_ENTRYPOINT_PARAMETERS = frozenset({'input_cloud_topic', 'input_odom_topic', 'world_frame',
                                         'native_frame', 'output_dir', 'use_sim_time'})


def _section(config, name, allowed, owned=frozenset()):
    value = config.get(name, {})
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f'{name} must be a mapping with string parameter names')
    forbidden = owned & set(value)
    if forbidden:
        raise ValueError(f'Entrypoint owns these {name} parameters: {sorted(forbidden)}')
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f'Unknown {name} parameters: {sorted(unknown)}')
    return dict(value)


def home():
    return Path(os.environ.get('P9_HOME', str(Path(__file__).resolve().parents[3])))


def load(path, mode):
    if mode not in ('live', 'replay'):
        raise ValueError('mode must be live or replay')
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict) or type(config.get('version')) is not int or config['version'] != 1:
        raise ValueError('Expected a version: 1 P9 configuration')
    if any(not isinstance(key, str) for key in config):
        raise ValueError('Configuration section names must be strings')
    unknown = set(config) - {'version', 'calibration', 'sensor', 'mapping'}
    if unknown:
        raise ValueError(f'Unknown config sections: {sorted(unknown)}')
    c = _section(config, 'calibration', {'status', 'rotation', 'translation', 'note'})
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
    sensor = _section(config, 'sensor', SENSOR_NODE_PARAMETERS, SENSOR_ENTRYPOINT_PARAMETERS)
    if sensor.get('imu_mode', 'merged_linear') not in ('merged_linear', 'raw_linear'):
        raise ValueError('sensor.imu_mode must be merged_linear or raw_linear')
    clock_offset = sensor.get('clock_offset_sec', 0.)
    if isinstance(clock_offset, bool) or not isinstance(clock_offset, (int, float)) or not math.isfinite(clock_offset):
        raise ValueError('clock_offset_sec must be finite')
    if 'clock_offset_sec' in sensor:
        sensor['clock_offset_sec'] = float(clock_offset)
    sensor.update(mode=mode, use_sim_time=(mode == 'replay'))
    mapping = _section(config, 'mapping', MAPPING_NODE_PARAMETERS, MAPPING_ENTRYPOINT_PARAMETERS)
    if mapping.get('input_reliability', 'best_effort') not in ('best_effort', 'reliable'):
        raise ValueError('mapping.input_reliability must be best_effort or reliable')
    for key in ('publish_terrain_inputs', 'publish_map_alias', 'publish_base_frame'):
        if key in mapping and not isinstance(mapping[key], bool):
            raise ValueError(f'mapping.{key} must be a YAML boolean')
    mapping.update(use_sim_time=(mode == 'replay'))
    pointlio = copy.deepcopy(yaml.safe_load((home()/'config/p9-pointlio.yaml').read_text())['/**']['ros__parameters'])
    pointlio['use_sim_time'] = mode == 'replay'
    pointlio['mapping']['extrinsic_R'] = [float(v) for v in rotation.flat]
    pointlio['mapping']['extrinsic_T'] = [float(v) for v in translation.flat]
    return config, sensor, mapping, pointlio
