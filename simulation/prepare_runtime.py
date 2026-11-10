#!/usr/bin/env python3
"""Check expanded Gazebo geometry, frozen model, sim profile and overlay provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE/'sensor_model'))
from model_contract import contract


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_profile(profile, model):
    """No real-machine calibration or bag epoch shift may leak into the simulator."""
    if profile.get('profile_mode') != 'sim':
        raise ValueError('Only the explicit sim profile is allowed')
    sensor = profile['sensor']
    required = {'input_cloud_topic': '/camera_linear/points',
                'input_imu_topic': '/camera_linear/imu', 'imu_mode': 'merged_linear',
                'imu_frame': model['imu_frame'], 'cloud_frame': model['lidar_frame'],
                'clock_offset_sec': 0.0, 'common_clock_offset_sec': 0.0,
                'point_time_field': 'time', 'point_time_scale': 1.0}
    for key, expected in required.items():
        if sensor.get(key) != expected:
            raise ValueError(f'Sim sensor {key} must equal {expected!r}')
    for key in ('gyro_bias', 'accel_offset'):
        if not np.array_equal(sensor.get(key), [0., 0., 0.]):
            raise ValueError('Sim '+key+' must be zero')
    for section, r_key, t_key, model_key in [
        ('calibration', 'rotation', 'translation', 'imu_from_lidar'),
        ('mapping', 'T_imu_base_rotation', 'T_imu_base_translation', 'imu_from_body')]:
        if not np.allclose(np.asarray(profile[section][r_key]).reshape(3, 3),
                           model[model_key]['rotation_matrix'], atol=1e-10, rtol=0):
            raise ValueError(section+' rotation does not match simulator model')
        if not np.allclose(profile[section][t_key], model[model_key]['translation'], atol=1e-10, rtol=0):
            raise ValueError(section+' translation does not match simulator model')
    if profile['mapping'].get('base_frame') != 'base_link':
        raise ValueError('Sim body frame must be base_link')


def prepare(output, profile_path, world_path=None):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output/'runtime-manifest.json').exists():
        raise FileExistsError('Refusing to reuse prepared runtime')
    profile_path = Path(profile_path).resolve()
    profile = yaml.safe_load(profile_path.read_text())
    frozen_path = HERE/'sensor_model/model_manifest.json'
    frozen = json.loads(frozen_path.read_text())
    verify_profile(profile, frozen)
    sim_share = Path(get_package_share_directory('unitree_go2_sim'))
    description_share = Path(get_package_share_directory('unitree_go2_description'))
    wrapper = HERE/'sensor_model/go2_p9_camera_imu.urdf.xacro'
    args = ['xacro', str(wrapper), 'robot_controllers:='+str(sim_share/'config/ros_control/ros_control.yaml'),
            'command_interface:=effort', 'disable_camera:=true', 'disable_d455:=true',
            'disable_lidar_l1:=false', 'disable_velodyne_lidar:=true']
    urdf = output/'expanded.urdf'
    urdf.write_text(subprocess.check_output(args, text=True))
    actual = contract(urdf)
    if not actual['runtime_validation']['expanded_urdf_checked']:
        raise ValueError('Actual expanded URDF was not validated')
    for key in ('body_from_imu', 'body_from_lidar', 'imu_from_lidar', 'imu_from_body'):
        if not np.allclose(actual[key]['homogeneous_matrix'], frozen[key]['homogeneous_matrix'], atol=1e-10, rtol=0):
            raise ValueError('Expanded robot differs from frozen model: '+key)
    verify_profile(profile, actual)
    (output/'model-validated.json').write_text(json.dumps(actual, indent=2)+'\n')
    original_world = Path(world_path).resolve() if world_path else description_share/'worlds/default.sdf'
    world = output/'world.sdf'
    shutil.copyfile(original_world, world)
    packages = {}
    for package in ('go2_p9', 'go2_p9_navigation', 'point_lio_unilidar', 'terrain_analysis',
                    'terrain_analysis_ext', 'local_planner', 'far_planner'):
        prefix = Path(get_package_prefix(package)).resolve()
        # A stale /ws planner must never silently replace the new release.
        if '/opt/p9_ws/install' not in str(prefix):
            raise ValueError(f'{package} resolved outside new overlay: {prefix}')
        packages[package] = str(prefix)
    report = {'schema_version': 1, 'profile': str(profile_path), 'profile_sha256': sha(profile_path),
              'world_source': str(original_world), 'world_sha256': sha(world),
              'urdf_sha256': sha(urdf), 'xacro_command': args,
              'model_manifest_sha256': sha(frozen_path), 'package_prefixes': packages,
              'source_sha256': {str(p.relative_to(HERE)): sha(p) for p in HERE.rglob('*')
                  if p.is_file() and '__pycache__' not in p.parts},
              'ground_truth_role': 'Evaluation only; not an estimator/planner/control input',
              'ros_domain_id': 94, 'gz_partition': 'ssy-p9-nav-20261110'}
    (output/'runtime-manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--profile', default='/opt/p9/config/p9nav-sim.yaml')
    parser.add_argument('--world')
    args = parser.parse_args()
    print(json.dumps(prepare(args.output, args.profile, args.world), indent=2))
