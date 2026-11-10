#!/usr/bin/env python3
"""Gazebo native lidar + independent optical IMUs -> P9 Point-LIO inputs.

No ground-truth or body-IMU subscription. All times are Gazebo simulation time.
GPU lidar snapshots intentionally have zero per-point relative time.
"""
import bisect
import copy
import json
import math
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

DTYPE = np.dtype({'names': ['x','y','z','intensity','ring','time'],
    'formats': ['<f4','<f4','<f4','<f4','<u2','<f4'],
    'offsets': [0,4,8,16,20,24], 'itemsize': 32})


def stamp_ns(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec


def interpolate_acceleration(stamps, values, target, max_gap=30000000):
    """Return None without a bounded bracket; never extrapolate."""
    i = bisect.bisect_left(stamps, target)
    if i < len(stamps) and stamps[i] == target:
        return np.asarray(values[i], dtype=float)
    if i == 0 or i == len(stamps) or stamps[i]-stamps[i-1] > max_gap:
        return None
    w = (target-stamps[i-1]) / (stamps[i]-stamps[i-1])
    return (1-w)*np.asarray(values[i-1])+w*np.asarray(values[i])


def keep_native_cloud(raw, width, R_BL, t_BL, min_range=.2, max_range=25.):
    """Filter ranges and robot geometry; output XYZ stays in native lidar axes."""
    xyz = np.column_stack([raw[k] for k in ('x','y','z')])
    ranges = np.linalg.norm(xyz, axis=1)
    body = xyz @ R_BL.T + t_BL
    self_hit = ((np.abs(body[:,0]) < .42) & (np.abs(body[:,1]) < .24)
                & (body[:,2] > -.5) & (body[:,2] < .2))
    valid = np.isfinite(xyz).all(axis=1) & (ranges >= min_range) & (ranges <= max_range) & ~self_hit
    out = np.zeros(int(valid.sum()), dtype=DTYPE)
    for i,key in enumerate(('x','y','z')):
        out[key] = xyz[valid,i]
    if 'intensity' in raw.dtype.names:
        out['intensity'] = np.nan_to_num(raw['intensity'][valid])
    if 'ring' in raw.dtype.names:
        out['ring'] = raw['ring'][valid]
    else:
        out['ring'] = (np.arange(len(raw)) // max(width,1))[valid]
    return out


class P9Sensors(Node):
    def __init__(self):
        super().__init__('p9_gazebo_sensor_normalizer')
        self.declare_parameter('sensor_start_delay',40.)
        self.declare_parameter('manifest',str(Path(__file__).resolve().parent/'sensor_model/model_manifest.json'))
        self.declare_parameter('summary_path','/output/runtime/sensor_summary.json')
        self.model = json.loads(Path(self.get_parameter('manifest').value).read_text())
        self.start = time.monotonic()
        self.counts = Counter()
        self.acc = deque(maxlen=1000)
        self.gyro = deque(maxlen=1000)
        self.pending_clouds = deque(maxlen=100)
        self.last_acc = self.last_gyro = self.last_cloud = self.last_imu_out = -1
        self.first_imu_out = None
        tf = self.model['body_from_lidar']
        self.R_BL = Rotation.from_quat(tf['quaternion_xyzw']).as_matrix()
        self.t_BL = np.array(tf['translation'])
        self.imu_pub = self.create_publisher(Imu,'/camera_linear/imu',100)
        self.cloud_pub = self.create_publisher(PointCloud2,'/camera_linear/points',10)
        self.status_pub = self.create_publisher(String,'/p9/sensor_status',1)
        self.create_subscription(Imu,self.model['raw_accel_topic'],self.acc_cb,qos_profile_sensor_data)
        self.create_subscription(Imu,self.model['raw_gyro_topic'],self.gyro_cb,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/unitree_lidar/points',self.cloud_cb,qos_profile_sensor_data)
        self.create_timer(2.,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ready(self):
        return time.monotonic()-self.start >= self.get_parameter('sensor_start_delay').value

    def check_imu(self,msg):
        if msg.header.frame_id != self.model['imu_frame']:
            self.counts['imu_bad_frame'] += 1
            return False
        return True

    def acc_cb(self,msg):
        self.counts['acc_received'] += 1
        if not self.ready() or not self.check_imu(msg): return
        ns = stamp_ns(msg)
        if ns <= self.last_acc:
            self.counts['acc_nonincreasing'] += 1
            return
        v = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        if not all(math.isfinite(x) for x in v):
            self.counts['acc_nonfinite'] += 1
            return
        self.last_acc = ns
        self.acc.append((ns,v))
        self.flush()

    def gyro_cb(self,msg):
        self.counts['gyro_received'] += 1
        if not self.ready() or not self.check_imu(msg): return
        ns = stamp_ns(msg)
        if ns <= self.last_gyro:
            self.counts['gyro_nonincreasing'] += 1
            return
        if not all(math.isfinite(x) for x in (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)):
            self.counts['gyro_nonfinite'] += 1
            return
        self.last_gyro = ns
        if len(self.gyro) == self.gyro.maxlen: self.counts['gyro_queue_overflow'] += 1
        self.gyro.append(msg)
        self.flush()

    def flush(self):
        if not self.acc: return
        stamps,values = zip(*self.acc)
        while self.gyro and stamp_ns(self.gyro[0]) <= stamps[-1]:
            msg = self.gyro.popleft()
            ns = stamp_ns(msg)
            a = interpolate_acceleration(stamps,values,ns)
            if a is None:
                self.counts['imu_without_bracket'] += 1
                continue
            out = copy.deepcopy(msg)
            out.linear_acceleration.x,out.linear_acceleration.y,out.linear_acceleration.z = map(float,a)
            # The Gazebo IMU publishes physical vectors; orientation is not a
            # separate absolute-attitude input to Point-LIO. No gravity removal.
            self.imu_pub.publish(out)
            self.last_imu_out = ns
            if self.first_imu_out is None:self.first_imu_out=ns
            self.counts['imu_published'] += 1
            if 'first_acceleration' not in self.__dict__:
                self.first_acceleration = a.tolist()
        while self.pending_clouds and stamp_ns(self.pending_clouds[0]) <= self.last_imu_out:
            cloud=self.pending_clouds.popleft()
            if stamp_ns(cloud)<self.first_imu_out:
                self.counts['cloud_before_imu_coverage']+=1
                continue
            self.cloud_pub.publish(cloud)
            self.counts['cloud_published'] += 1

    def cloud_cb(self,msg):
        self.counts['cloud_received'] += 1
        if not self.ready(): return
        if msg.header.frame_id != self.model['lidar_frame']:
            self.counts['cloud_bad_frame'] += 1
            self.get_logger().error('Unexpected lidar frame: '+msg.header.frame_id,throttle_duration_sec=5.)
            return
        ns = stamp_ns(msg)
        if ns <= self.last_cloud:
            self.counts['cloud_nonincreasing'] += 1
            return
        self.last_cloud = ns
        raw = point_cloud2.read_points(msg).reshape(-1)
        out = keep_native_cloud(raw,msg.width,self.R_BL,self.t_BL)
        self.counts['raw_points'] += len(raw)
        self.counts['kept_points'] += len(out)
        if len(out) < 10:
            self.counts['cloud_too_small'] += 1
            return
        cloud = PointCloud2()
        cloud.header = copy.deepcopy(msg.header)
        cloud.height,cloud.width = 1,len(out)
        cloud.fields = [PointField(name=k,offset=DTYPE.fields[k][1],
            datatype=PointField.UINT16 if k=='ring' else PointField.FLOAT32,count=1) for k in DTYPE.names]
        cloud.point_step,cloud.row_step,cloud.is_dense = 32,32*len(out),True
        cloud.data = out.tobytes()
        if len(self.pending_clouds) == self.pending_clouds.maxlen: self.counts['cloud_queue_overflow'] += 1
        self.pending_clouds.append(cloud)
        self.flush()

    def report(self):
        d = dict(counts=dict(self.counts),pending_gyro=len(self.gyro),pending_clouds=len(self.pending_clouds),
            first_acceleration=getattr(self,'first_acceleration',None),
            sensor_source='Independent Gazebo optical IMUs; no body IMU or truth',
            cloud_geometry='native lidar, robot box filtered in body coordinates',
            point_times='all zero: instantaneous GPU lidar snapshot',
            clock_offset_ns=0,acceleration='linear interpolation at gyro stamps, no extrapolation')
        summary_path = Path(self.get_parameter('summary_path').value)
        summary_path.parent.mkdir(parents=True,exist_ok=True)
        summary_path.write_text(json.dumps(d,indent=2)+'\n')
        self.status_pub.publish(String(data=json.dumps(d)))


def main():
    rclpy.init()
    node=P9Sensors()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
