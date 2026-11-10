#!/usr/bin/env python3
"""Derive exact simulation transforms; optionally check a fully expanded URDF.

No ROS, network, simulation process, calibration fit or odometry input. The IMU
mount is read from our wrapper. Native L1 mounting values are the audited source
values and must match the runtime-expanded URDF when --urdf is supplied.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


HERE = Path(__file__).resolve().parent
IMU_FRAME = 'camera_imu_optical_frame'
LIDAR_FRAME = 'lidar_l1_link'
LIDAR_XYZ = [.25, -.038, -.03]
LIDAR_RPY = [2.879, 0., 1.5705]


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def joint_transform(joint):
    origin = joint.find('origin')
    return transform([float(x) for x in origin.get('xyz', '0 0 0').split()],
                     [float(x) for x in origin.get('rpy', '0 0 0').split()])


def describe(matrix):
    return dict(translation=matrix[:3, 3].tolist(),
                quaternion_xyzw=Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
                rotation_matrix=matrix[:3, :3].tolist(),
                homogeneous_matrix=matrix.tolist())


def base_transform(root, child):
    parents = {}
    for joint in root.findall('joint'):
        c = joint.find('child').get('link')
        if c in parents:
            raise ValueError('Duplicate joint child '+c)
        parents[c] = joint
    result = np.eye(4)
    seen = set()
    while child != 'base_link':
        if child in seen or child not in parents:
            raise ValueError('No unique path to base_link for '+child)
        seen.add(child)
        joint = parents[child]
        if joint.get('type') != 'fixed':
            raise ValueError('Sensor/body chain contains a movable joint')
        result = joint_transform(joint) @ result
        child = joint.find('parent').get('link')
    return result


def contract(urdf=None):
    wrapper = HERE/'go2_p9_camera_imu.urdf.xacro'
    root = ET.parse(wrapper).getroot()
    imu_joint = root.find("joint[@name='p9_camera_imu_joint']")
    T_BI = joint_transform(imu_joint)
    T_BL = transform(LIDAR_XYZ, LIDAR_RPY)
    T_IB = np.linalg.inv(T_BI)
    T_IL = T_IB @ T_BL
    # A nonzero lever arm is an essential part of this model.
    assert np.linalg.norm(T_IL[:3, 3]) > .09
    assert np.allclose(T_BI @ T_IL, T_BL, atol=1e-12)
    assert np.allclose(T_BI[:3, :3], [[0,0,1],[-1,0,0],[0,-1,0]], atol=1e-12)
    validation = dict(expanded_urdf_checked=False)
    if urdf is not None:
        expanded = ET.parse(urdf).getroot()
        for name, expected in [(IMU_FRAME,T_BI),(LIDAR_FRAME,T_BL),('imu_link',np.eye(4))]:
            actual = base_transform(expanded, name)
            if not np.allclose(actual, expected, rtol=0, atol=1e-10):
                raise ValueError('Expanded model sensor transform mismatch: '+name)
        sensors = {}
        for gazebo in expanded.findall('gazebo'):
            for sensor in gazebo.findall('sensor'):
                if sensor.get('name', '').startswith('p9_camera_'):
                    sensors[sensor.get('name')] = (gazebo, sensor)
        for name, topic, hz in [('p9_camera_gyro_sensor','/p9_sim/gyro',200),
                                ('p9_camera_accel_sensor','/p9_sim/accel',100)]:
            gazebo, sensor = sensors[name]
            if (gazebo.get('reference') != IMU_FRAME or sensor.get('type') != 'imu' or
                    sensor.findtext('topic') != topic or float(sensor.findtext('update_rate')) != hz or
                    sensor.findtext('gz_frame_id') != IMU_FRAME or
                    any(float(v) != 0 for v in sensor.findtext('pose').split())):
                raise ValueError('Expanded model IMU sensor contract mismatch: '+name)
        validation = dict(expanded_urdf_checked=True, file=str(urdf),
                          sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
                          camera_sensors=list(sensors))
    specific_force = T_BI[:3, :3].T @ np.array([0.,0.,9.81])
    gravity = -specific_force
    return dict(schema_version=1, mode='P9-camera-linear-Gazebo-sensor-model',
        transform_convention='parent_from_child: p_parent = R * p_child + translation; quaternions xyzw',
        body_frame='base_link', imu_frame=IMU_FRAME, lidar_frame=LIDAR_FRAME,
        raw_gyro_topic='/p9_sim/gyro', raw_accel_topic='/p9_sim/accel',
        raw_cloud_topic='/unitree_lidar/points', gyro_rate_hz=200, accel_rate_hz=100,
        body_from_imu=describe(T_BI), body_from_lidar=describe(T_BL),
        imu_from_lidar=describe(T_IL), imu_from_body=describe(T_IB),
        mounting_definition='Virtual camera IMU at existing D455 mounting-link origin on trunk; exact simulation definition, not internal D435i/D455 chip calibration',
        camera_mount_source='unitree_go2_description/urdf/realsense_d455.xacro:11-13; [0.28,0,0.062] with no mount rotation',
        lidar_mount_source='unitree_go2_description/urdf/lidar_4D_lidar.xacro:33-38',
        body_from_lidar_rpy_xyz=LIDAR_RPY,
        wrapper_sha256=hashlib.sha256(wrapper.read_bytes()).hexdigest(),
        gyro_noise_stddev_rad_s=.0002, accel_noise_stddev_m_s2=.017,
        gyro_noise_covariance_diagonal_rad2_s2=[.0002**2]*3,
        accel_noise_covariance_diagonal_m2_s4=[.017**2]*3,
        imu_noise_scope='Independent zero-mean Gaussian per sample; no fitted bias, drift, clipping, timing jitter or vendor preprocessing. Merged interpolation introduces correlation.',
        initial_level_body_specific_force_in_imu_m_s2=specific_force.tolist(),
        initial_level_body_gravity_in_imu_m_s2=gravity.tolist(),
        pointlio_world_gravity_after_gravity_alignment_m_s2=[0.,0.,-9.81],
        initialization='Use measured stationary camera specific-force mean and gyro mean; do not subtract gravity in adapter; gravity_align=true rotates estimator world to Z-up. Do not use simulated quaternion as estimator orientation input.',
        camera_pose_to_body_equation='T_map_body = T_map_imu * T_imu_body',
        truth_body_to_camera_equation='T_world_imu = T_world_body * T_body_imu',
        lidar=dict(kind='GPU instantaneous frame; not Unitree L1 scanning mechanics',
                   rate_hz=10, horizontal_samples=600, vertical_samples=30,
                   range_m=[.8,30.], noise_stddev_m=.008,
                   timestamp_offset_ns=0, relative_point_time_sec=0.,
                   point_time_explanation='All rendered points share header time. Do not invent rolling-scan times or apply real-bag epoch offsets.'),
        bodybox_filter=dict(frame='base_link', x_open_m=[-.42,.42], y_open_m=[-.24,.24], z_open_m=[-.5,.2],
            apply='Compute p_body=R_BL*p_lidar+t_BL for the boolean mask only; publish surviving original native lidar coordinates. Do not mutate point coordinates and then reuse native imu_from_lidar.'),
        runtime_validation=validation,
        limits=['Physical quadruped simulation differs from the real camera hardware and L1 scan pattern.',
                'No inference about competition or real-robot accuracy follows solely from a successful simulated navigation run.',
                'Gazebo ground truth is evaluation-only and must never feed SLAM, planner pose, or command gating.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,help='Fully expanded URDF to check before spawning')
    parser.add_argument('--output',type=Path,help='New JSON file; refuses overwrite, otherwise print')
    args = parser.parse_args()
    result = contract(args.urdf)
    text = json.dumps(result,indent=2,allow_nan=False)+'\n'
    if args.output:
        with args.output.open('x') as out: out.write(text)
        print(args.output)
    else:
        print(text,end='')


if __name__ == '__main__':
    main()
