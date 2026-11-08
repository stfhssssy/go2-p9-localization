"""Standalone ROS 2 P9 registered-scan bridge; no external project imports."""
import copy
import csv
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float64MultiArray, Header, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from .mapping_core import (MappingError, CapacityError, ExactPairBuffer, MappingCore,
                           change_reference_point, covariance, decode_cloud,
                           matrix_quaternion, positive_float, positive_integer,
                           quaternion_matrix, rigid_transform, save_pcd, vector)


def stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise MappingError('Invalid nonnegative ROS timestamp')
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value


def atomic_json(path, value):
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(json_safe(value), indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def cloud_message(header, points):
    points = np.asarray(points, dtype='<f4').reshape(-1, 4)
    if not np.isfinite(points).all():
        raise MappingError('Refusing to publish a nonfinite map/cloud')
    message = PointCloud2(header=copy.deepcopy(header), height=1, width=len(points),
                          is_bigendian=False, point_step=16, row_step=16*len(points), is_dense=True)
    message.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    message.data = points.tobytes()
    return message


class P9Mapping(Node):
    def __init__(self):
        super().__init__('p9_mapping')
        defaults = dict(input_cloud_topic='/pointlio/registered_scan', input_odom_topic='/pointlio/odom',
            world_frame='camera_init', native_frame='aft_mapped', output_dir='./p9-output',
            voxel_size=.05, max_map_voxels=1_000_000, max_path_poses=200_000,
            max_pending_pairs=128, pair_timeout_sec=5., max_cloud_points=250_000,
            max_cloud_bytes=32*1024*1024, input_queue_depth=128, input_reliability='best_effort',
            publish_terrain_inputs=True, publish_map_alias=False, map_alias_frame='map',
            publish_base_frame=False, base_frame='base_link',
            T_imu_base_rotation=[float('nan')]*9, T_imu_base_translation=[float('nan')]*3,
            map_publish_period_sec=.5, path_publish_period_sec=.2,
            stale_timeout_sec=2., tracking_timeout_sec=2.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key in ('map_publish_period_sec', 'path_publish_period_sec', 'stale_timeout_sec', 'tracking_timeout_sec'):
            self.cfg[key] = positive_float(self.cfg[key], key)
        for key in ('max_cloud_points', 'max_cloud_bytes', 'input_queue_depth'):
            self.cfg[key] = positive_integer(self.cfg[key], key)
        frames = [self.cfg[key] for key in ('world_frame', 'native_frame', 'base_frame', 'map_alias_frame')]
        if any(not isinstance(f, str) or not f or f.startswith('/') or any(c.isspace() for c in f) for f in frames):
            raise MappingError('TF frames must be nonempty and contain no whitespace or leading slash')
        if len(set(frames)) != len(frames):
            raise MappingError('world/native/base/map-alias frame names must be distinct')
        if self.cfg['native_frame'] == 'base_link':
            raise MappingError('Native Point-LIO pose is an IMU origin, not base_link')
        if self.cfg['input_cloud_topic'] == '/registered_scan' or self.cfg['input_odom_topic'] == '/state_estimation':
            raise MappingError('Input and terrain output topics must not create a feedback loop')
        reliability = {'best_effort': ReliabilityPolicy.BEST_EFFORT, 'reliable': ReliabilityPolicy.RELIABLE}
        if self.cfg['input_reliability'] not in reliability:
            raise MappingError('input_reliability must be best_effort or reliable')
        self.extrinsic = None
        if self.cfg['publish_base_frame']:
            self.extrinsic = rigid_transform(self.cfg['T_imu_base_rotation'], self.cfg['T_imu_base_translation'])
        self.pairs = ExactPairBuffer(self.cfg['max_pending_pairs'], self.cfg['pair_timeout_sec'])
        self.core = MappingCore(self.cfg['voxel_size'], self.cfg['max_map_voxels'], self.cfg['max_path_poses'])
        self.fault = None
        self.last_input_wall = {'odom': None, 'cloud': None}
        self.last_pair_wall = self.last_header = None
        self.tracking_valid = self.last_tracking_wall = self.latest_filter_diagnostics = None
        self.last_diagnostic_wall = None
        self.map_dirty = self.path_dirty = False
        self.last_map_publish = self.last_path_publish = 0.
        self.ignored_after_fault = self.intensity_missing_scans = self.sanitized_terrain_scans = 0
        self.last_save_reason = None
        self.started_wall = time.monotonic()
        self.path_message = RosPath(header=Header(frame_id=self.cfg['world_frame']))
        self.output = Path(self.cfg['output_dir']).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        for name in ('trajectory.csv', 'mapping_summary.json', 'mapping_status.json', 'PCD/map.pcd'):
            if (self.output/name).exists():
                raise FileExistsError(f'Refusing to overwrite an earlier mapping run: {self.output/name}')
        self.csv_handle = (self.output/'trajectory.csv').open('x', newline='')
        self.csv_writer = csv.writer(self.csv_handle)
        self.csv_writer.writerow(['sensor_timestamp_sec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                                  'stamp_ns', 'frame_id', 'child_frame_id'])
        persistent = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        incoming = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.cfg['input_queue_depth'],
                              reliability=reliability[self.cfg['input_reliability']],
                              durability=DurabilityPolicy.VOLATILE)
        outgoing = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=32,
                              reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.map_pub = self.create_publisher(PointCloud2, '/p9/map', persistent)
        self.path_pub = self.create_publisher(RosPath, '/p9/path', persistent)
        self.status_pub = self.create_publisher(String, '/p9/mapping_status', persistent)
        self.state_pub = self.scan_pub = None
        if self.cfg['publish_terrain_inputs']:
            self.state_pub = self.create_publisher(Odometry, '/state_estimation', outgoing)
            self.scan_pub = self.create_publisher(PointCloud2, '/registered_scan', outgoing)
        self.create_subscription(PointCloud2, self.cfg['input_cloud_topic'], lambda m: self.receive('cloud', m), incoming)
        self.create_subscription(Odometry, self.cfg['input_odom_topic'], lambda m: self.receive('odom', m), incoming)
        self.create_subscription(Bool, '/pointlio/tracking_valid', self.tracking, incoming)
        self.create_subscription(Float64MultiArray, '/pointlio/filter_diagnostics', self.diagnostic, incoming)
        self.create_service(Trigger, '/p9/save', self.save_service)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        if self.cfg['publish_map_alias']:
            alias = TransformStamped()
            alias.header.frame_id, alias.child_frame_id = self.cfg['map_alias_frame'], self.cfg['world_frame']
            alias.transform.rotation.w = 1.
            transforms.append(alias)
            self.get_logger().warn('Identity map alias enabled: this is not a separate optimized map. Do not run another TF authority for this edge.')
        if self.extrinsic is not None:
            body = TransformStamped()
            body.header.frame_id, body.child_frame_id = self.cfg['native_frame'], self.cfg['base_frame']
            for axis, value in zip('xyz', self.extrinsic[:3, 3]):
                setattr(body.transform.translation, axis, float(value))
            for axis, value in zip('xyzw', matrix_quaternion(self.extrinsic[:3, :3])):
                setattr(body.transform.rotation, axis, float(value))
            transforms.append(body)
        if transforms:
            self.static_tf.sendTransform(transforms)
        # Wall/steady timers flush the final outputs even when bag /clock stops.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.maintenance, clock=self.steady_clock)
        self.create_timer(1., self.write_status, clock=self.steady_clock)
        self.write_status()
        self.get_logger().info('P9 map/path retain camera IMU origin and world; registered XYZ is never pose-transformed.')

    def tracking(self, message):
        self.tracking_valid = bool(message.data)
        self.last_tracking_wall = time.monotonic()

    def diagnostic(self, message):
        self.latest_filter_diagnostics = [float(v) if np.isfinite(v) else None for v in message.data]
        self.last_diagnostic_wall = time.monotonic()

    def latch_fault(self, error):
        if self.fault is None:
            self.fault = dict(type=type(error).__name__, message=str(error),
                              last_accepted_stamp_ns=self.core.path[-1].stamp_ns if self.core.path else None)
            self.get_logger().error('Mapping stopped accepting data: '+str(error))
            self.write_status()

    def receive(self, kind, message):
        if self.fault is not None:
            self.ignored_after_fault += 1
            return
        try:
            now = time.monotonic()
            stamp = stamp_ns(message)
            if message.header.frame_id != self.cfg['world_frame']:
                raise MappingError(f'{kind} frame {message.header.frame_id!r} is not {self.cfg["world_frame"]!r}')
            if kind == 'odom':
                if message.child_frame_id != self.cfg['native_frame']:
                    raise MappingError('Point-LIO odometry child is not the configured native IMU origin')
                p, q = message.pose.pose.position, message.pose.pose.orientation
                vector([p.x, p.y, p.z], 3, 'position')
                quaternion_matrix([q.x, q.y, q.z, q.w])
                covariance(message.pose.covariance, 'pose covariance')
                covariance(message.twist.covariance, 'twist covariance')
                v, w = message.twist.twist.linear, message.twist.twist.angular
                vector([v.x, v.y, v.z, w.x, w.y, w.z], 6, 'twist')
            else:
                if message.width*message.height > self.cfg['max_cloud_points'] or len(message.data) > self.cfg['max_cloud_bytes']:
                    raise CapacityError('Input cloud exceeds max_cloud_points/max_cloud_bytes; not queued')
            self.last_input_wall[kind] = now
            for stamp, odom, cloud in self.pairs.add(kind, stamp, message, now):
                self.accept_pair(stamp, odom, cloud)
        except (MappingError, ValueError, TypeError, OverflowError, OSError) as error:
            self.latch_fault(error)

    def accept_pair(self, stamp, odom, cloud):
        points = decode_cloud(cloud.data, [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields],
                              cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.is_bigendian)
        if not any(f.name == 'intensity' for f in cloud.fields):
            self.intensity_missing_scans += 1
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        position, quaternion = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
        state = odom
        if self.extrinsic is not None:
            v, w = odom.twist.twist.linear, odom.twist.twist.angular
            converted = change_reference_point(position, quaternion, odom.pose.covariance,
                [v.x, v.y, v.z], [w.x, w.y, w.z], odom.twist.covariance, self.extrinsic)
            state = copy.deepcopy(odom)
            state.child_frame_id = self.cfg['base_frame']
            for field, key, axes in ((state.pose.pose.position, 'position', 'xyz'),
                                     (state.pose.pose.orientation, 'quaternion', 'xyzw'),
                                     (state.twist.twist.linear, 'linear_velocity', 'xyz'),
                                     (state.twist.twist.angular, 'angular_velocity', 'xyz')):
                for axis, value in zip(axes, converted[key]):
                    setattr(field, axis, float(value))
            state.pose.covariance = converted['pose_covariance'].ravel().tolist()
            state.twist.covariance = converted['twist_covariance'].ravel().tolist()
        clean = self.core.accept(stamp, points, position, quaternion)
        # Path/CSV are always the native estimator pose, even when terrain needs B.
        pose = PoseStamped(header=copy.deepcopy(odom.header), pose=copy.deepcopy(odom.pose.pose))
        self.path_message.poses.append(pose)
        self.path_message.header = copy.deepcopy(odom.header)
        self.csv_writer.writerow([f'{stamp//10**9}.{stamp%10**9:09d}', *position, *quaternion,
                                  stamp, self.cfg['world_frame'], self.cfg['native_frame']])
        self.last_header = copy.deepcopy(cloud.header)
        self.last_pair_wall = time.monotonic()
        self.map_dirty = self.path_dirty = True
        if self.state_pub is not None:
            self.state_pub.publish(state)
            if len(clean) != len(points):
                # Same finite XYZ, same stamp/frame; remove invalid rows for terrain.
                cloud = cloud_message(cloud.header, clean)
                self.sanitized_terrain_scans += 1
            self.scan_pub.publish(cloud)

    def publish_updates(self, force=False):
        now = time.monotonic()
        if self.path_dirty and (force or now-self.last_path_publish >= self.cfg['path_publish_period_sec']):
            self.path_pub.publish(self.path_message)
            self.last_path_publish, self.path_dirty = now, False
        if self.map_dirty and (force or (self.map_pub.get_subscription_count() and
                                        now-self.last_map_publish >= self.cfg['map_publish_period_sec'])):
            self.map_pub.publish(cloud_message(self.last_header, self.core.voxels.points()))
            self.last_map_publish, self.map_dirty = now, False

    def maintenance(self):
        try:
            if self.fault is None:
                self.pairs.check_timeout(time.monotonic())
            self.publish_updates()
        except (MappingError, ValueError, OSError) as error:
            self.latch_fault(error)

    def report(self):
        now = time.monotonic()
        age = lambda stamp: None if stamp is None else max(0., now-stamp)
        pair_age, tracking_age = age(self.last_pair_wall), age(self.last_tracking_wall)
        if self.fault:
            status = 'FAULT'
        elif pair_age is None:
            status = 'WAITING_FOR_EXACT_PAIR'
        elif pair_age > self.cfg['stale_timeout_sec']:
            status = 'STALE_INPUT'
        elif self.tracking_valid is False:
            status = 'TRACKING_INVALID'
        elif tracking_age is None or tracking_age > self.cfg['tracking_timeout_sec']:
            status = 'TRACKING_UNKNOWN_OR_STALE'
        else:
            status = 'OK'
        return dict(status=status, fault=self.fault, configuration=self.cfg,
            world_frame=self.cfg['world_frame'], trajectory_child_frame=self.cfg['native_frame'],
            state_child_frame=self.cfg['base_frame'] if self.extrinsic is not None else self.cfg['native_frame'],
            state_reference_point='configured base origin' if self.extrinsic is not None else 'native Point-LIO IMU origin',
            T_imu_base=self.extrinsic.tolist() if self.extrinsic is not None else None,
            pose_equation='T_WB = T_WI * T_IB' if self.extrinsic is not None else 'native T_WI unchanged',
            map_alias_identity_of=self.cfg['world_frame'] if self.cfg['publish_map_alias'] else None,
            exact_pairing=self.pairs.report(), accumulation=self.core.report(),
            last_input_wall_age_sec={k: age(v) for k, v in self.last_input_wall.items()},
            last_pair_wall_age_sec=pair_age, tracking_valid=self.tracking_valid,
            tracking_wall_age_sec=tracking_age, latest_filter_diagnostics=self.latest_filter_diagnostics,
            filter_diagnostics_wall_age_sec=age(self.last_diagnostic_wall),
            tracking_diagnostic_has_no_stamp=True, tracking_diagnostic_does_not_gate_mapping=True,
            ignored_inputs_after_fault=self.ignored_after_fault, intensity_missing_scans=self.intensity_missing_scans,
            sanitized_terrain_scans=self.sanitized_terrain_scans, original_sensor_stamps_preserved=True,
            registered_xyz_never_pose_transformed=True, dynamic_tf_published=False,
            unitree_odom_used=False, alignment_applied=False, absolute_accuracy_validated=False,
            zero_covariance_means='unspecified upstream covariance, not zero uncertainty',
            upstream_twist_semantics='not estimated here; zero/unavailable remains zero/unavailable',
            trajectory_sampling='every accepted exact scan/odom pair; no interpolation or decimation',
            middleware_loss_detection='pair mismatch detected; simultaneous loss of both inputs is not observable',
            terrain_delivery='same-stamp pair; DDS cross-topic callback ordering is not an acknowledgement',
            last_save_reason=self.last_save_reason, map_file=str(self.output/'PCD/map.pcd'),
            trajectory_file=str(self.output/'trajectory.csv'))

    def write_status(self):
        self.csv_handle.flush()
        report = self.report()
        atomic_json(self.output/'mapping_status.json', report)
        if rclpy.ok():
            self.status_pub.publish(String(data=json.dumps(json_safe(report), allow_nan=False)))

    def save(self, reason='service'):
        self.last_save_reason = reason
        self.csv_handle.flush()
        path = self.output/'PCD/map.pcd'
        save_pcd(path, self.core.voxels.points(), frame=self.cfg['world_frame'])
        if rclpy.ok():
            self.publish_updates(force=True)
        atomic_json(self.output/'mapping_summary.json', self.report())
        self.write_status()
        return path

    def save_service(self, _request, response):
        try:
            path = self.save()
            response.success = True
            response.message = str(path)+(f'; PARTIAL RUN: {self.fault["message"]}' if self.fault else '')
        except (OSError, ValueError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = P9Mapping()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.save(reason='shutdown')
            finally:
                node.csv_handle.close()
                node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
