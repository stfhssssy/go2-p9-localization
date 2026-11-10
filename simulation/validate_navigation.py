#!/usr/bin/env python3
"""Record one P9/Gazebo navigation experiment; ground truth is evaluation only.

Example (inside the ROS environment):
  python3 validate_navigation.py --seconds 240 --goal-forward 6.5 --goal-left 0 \
      --output /output/validation-forward.json
  python3 validate_navigation.py --seconds 240 --goal-map 0.2 -0.1 \
      --output /output/validation-return.json

``--seconds`` is the maximum wall observation time AFTER the single goal is
published; startup has its own bounded timeout. One goal request is published,
its matching acknowledgement is required, and the navigation stop service is
called on exit. No velocity, reset, ground-truth feedback or estimator parameter
is published. Ground truth never determines goal, readiness, or early stopping.

Raw CSVs are placed beside the report in <output-stem>.data/. Every received
odometry, IMU, command, waypoint, FAR event, and Path pose is recorded. Clouds
have per-message metadata only, not binary point payloads. DDS delivery is not
certified by this observer. Run --self-test without ROS for geometry checks.
"""
import argparse
import csv
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


POSE_TOPICS = ('/pointlio/odom', '/state_estimation', '/odom/ground_truth')
CLOUD_TOPICS = ('/registered_scan', '/terrain_map', '/terrain_map_ext',
                '/camera_linear/points')
REQUIRED_BEFORE_GOAL = ('/state_estimation', '/pointlio/odom',
                        '/camera_linear/imu') + CLOUD_TOPICS
NS = 1_000_000_000


def pose_matrix(position, quaternion):
    p, q = np.asarray(position, float), np.asarray(quaternion, float)
    if (p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all()
            or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01):
        raise ValueError('Invalid pose or non-unit xyzw quaternion')
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(q).as_matrix()
    out[:3, 3] = p
    return out


def inverse_pose(pose):
    out = np.eye(4)
    out[:3, :3] = pose[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ pose[:3, 3]
    return out


def interpolate_poses(samples, query_ns, max_gap_ns=200_000_000):
    """Native positions linear, quaternion shortest-arc SLERP; no extrapolation.

    Bad, duplicated, or reversed stamps fail closed; exact sample hits do not
    require a neighboring bracket. Returns matrices and per-query valid mask.
    Raw samples remain untouched even when interpolation is disallowed.
    """
    stamps = np.array([x['stamp_ns'] for x in samples], dtype=np.int64)
    query = np.asarray(query_ns, dtype=np.int64)
    if not len(stamps) or query.ndim != 1:
        raise ValueError('Empty trajectory or invalid query')
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('Nonmonotonic/duplicate pose stamps; no silent repair')
    matrices = np.array([pose_matrix(x['position'], x['quaternion']) for x in samples])
    output = np.full((len(query), 4, 4), np.nan)
    right = np.searchsorted(stamps, query, side='left')
    exact = (right < len(stamps)) & (stamps[np.minimum(right, len(stamps)-1)] == query)
    output[exact] = matrices[right[exact]]
    left = np.maximum(right-1, 0)
    inside = (query >= stamps[0]) & (query <= stamps[-1])
    bracket = (inside & ~exact & (right < len(stamps))
               & ((stamps[np.minimum(right, len(stamps)-1)]-stamps[left]) <= max_gap_ns))
    if bracket.any():
        # Integer subtraction before floating point conversion preserves epochs.
        origin = stamps[0]
        secs = (stamps-origin).astype(float)/NS
        targets = (query[bracket]-origin).astype(float)/NS
        rotations = Slerp(secs, Rotation.from_matrix(matrices[:, :3, :3]))(targets)
        output[bracket] = np.eye(4)
        output[bracket, :3, :3] = rotations.as_matrix()
        for axis in range(3):
            output[bracket, axis, 3] = np.interp(targets, secs, matrices[:, axis, 3])
    return output, exact | bracket


def relative_horizontal_goal(pose, forward, left):
    """Forward/left in initial body's yaw plane; keep initial body height."""
    yaw = math.atan2(pose[1, 0], pose[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return pose[:3, 3] + [c*forward-s*left, s*forward+c*left, 0.0]


def map_goal_in_truth(map_goal, truth_from_estimate, map_from_estimate):
    """Evaluation only: p_T = T_TE inverse(T_ME) p_M (no goal generation)."""
    point = np.asarray(map_goal, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('Map goal must be a finite XYZ point')
    for transform in (truth_from_estimate, map_from_estimate):
        transform = np.asarray(transform)
        if (transform.shape != (4, 4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-10)
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-8)):
            raise ValueError('Expected proper SE(3) goal-frame transform')
    return (np.asarray(truth_from_estimate) @ inverse_pose(np.asarray(map_from_estimate))
            @ np.append(point, 1.0))[:3]


def stationary_window(samples, duration_ns, position_limit, angle_limit):
    if len(samples) < 2:
        return None
    end = samples[-1]['stamp_ns']
    selected = [x for x in samples if x['stamp_ns'] >= end-duration_ns]
    if len(selected) < 3 or end-selected[0]['stamp_ns'] < duration_ns*.9:
        return None
    if np.any(np.diff([x['stamp_ns'] for x in selected]) <= 0):
        return None
    matrices = np.array([pose_matrix(x['position'], x['quaternion']) for x in selected])
    distance = np.linalg.norm(matrices[:, :3, 3]-matrices[0, :3, 3], axis=1)
    angle = Rotation.from_matrix(matrices[0, :3, :3].T @ matrices[:, :3, :3]).magnitude()
    if distance.max() > position_limit or angle.max() > angle_limit:
        return None
    return dict(sample=selected[0], max_position_excursion_m=float(distance.max()),
                max_orientation_excursion_rad=float(angle.max()), samples=len(selected),
                duration_sim_sec=(end-selected[0]['stamp_ns'])/NS)


def select_evaluation_anchor(estimate, truth, original_anchor_ns, goal_stamp_ns,
                             stationary_end_ns, period_ns=100_000_000,
                             max_gap_ns=200_000_000, position_limit=.05, angle_limit=.12):
    """Earliest covered pre-goal point on the original 10 Hz evaluation grid.

    This changes only evaluation, never the published goal or its original
    stationary anchor. No extrapolation or trajectory-dependent alignment fit
    is allowed. Both streams must remain stationary in the remaining original
    pre-goal window; do not search later moving intervals for a better match.
    """
    stop = min(int(stationary_end_ns), int(goal_stamp_ns)-1)
    if stop < original_anchor_ns:
        raise ValueError('No pre-goal stationary interval for evaluation')
    grid = np.arange(original_anchor_ns, stop+1, period_ns, dtype=np.int64)
    _, ev = interpolate_poses(estimate, grid, max_gap_ns)
    _, tv = interpolate_poses(truth, grid, max_gap_ns)
    candidates = np.flatnonzero(ev & tv)
    if not len(candidates):
        raise ValueError('No same-time estimate/truth coverage in the pre-goal stationary window')
    stamp = int(grid[candidates[0]])
    evidence = {}
    for name, source in [('estimate', estimate), ('truth', truth)]:
        anchor, valid = interpolate_poses(source, [stamp], max_gap_ns)
        assert valid[0]
        samples = [x for x in source if stamp <= x['stamp_ns'] <= stop]
        if len(samples) < 2:
            raise ValueError('Insufficient '+name+' samples to verify pre-goal stationarity')
        matrices = np.array([pose_matrix(x['position'], x['quaternion']) for x in samples])
        distance = np.linalg.norm(matrices[:, :3, 3]-anchor[0, :3, 3], axis=1)
        angle = Rotation.from_matrix(anchor[0, :3, :3].T @ matrices[:, :3, :3]).magnitude()
        if distance.max() > position_limit or angle.max() > angle_limit:
            raise ValueError(name+' is not stationary in the original pre-goal window')
        evidence[name] = dict(samples=len(samples), max_position_excursion_m=float(distance.max()),
                              max_orientation_excursion_rad=float(angle.max()))
    return dict(original_anchor_ns=int(original_anchor_ns), evaluation_anchor_ns=stamp,
                shift_ns=stamp-int(original_anchor_ns), goal_stamp_ns=int(goal_stamp_ns),
                stationary_window_end_ns=stop, period_ns=period_ns,
                original_anchor_estimate_covered=bool(ev[0]), original_anchor_truth_covered=bool(tv[0]),
                reason=('Original stationary anchor has same-time coverage' if stamp == original_anchor_ns else
                        'Original anchor lacks common coverage; earliest valid 10 Hz point in original pre-goal stationary window'),
                stationarity=evidence, position_limit_m=position_limit, angle_limit_rad=angle_limit,
                alignment_fitted_to_trajectory=False, goal_modified=False, extrapolation=False)


def position_metrics(estimate, truth, anchor_ns, period_ns=100_000_000,
                     max_gap_ns=200_000_000):
    """Time-matched, first-pose FULL SE(3) alignment at the declared anchor."""
    if not estimate or not truth:
        raise ValueError('Missing estimate or truth poses')
    if len({(x['frame'], x['child']) for x in estimate}) != 1:
        raise ValueError('Estimate frames changed')
    if len({(x['frame'], x['child']) for x in truth}) != 1:
        raise ValueError('Truth frames changed')
    if any(x['child'] != 'base_link' for x in estimate+truth):
        raise ValueError('Both evaluated poses must describe base_link, not IMU origins')
    end = min(estimate[-1]['stamp_ns'], truth[-1]['stamp_ns'])
    if end < anchor_ns:
        raise ValueError('No common post-anchor interval')
    grid = np.arange(anchor_ns, end+1, period_ns, dtype=np.int64)
    e, ev = interpolate_poses(estimate, grid, max_gap_ns)
    t, tv = interpolate_poses(truth, grid, max_gap_ns)
    valid = ev & tv
    if not len(grid) or not valid[0] or valid.sum() < 2:
        raise ValueError('Anchor/trajectory lacks same-time truth coverage')
    alignment = t[0] @ inverse_pose(e[0])
    aligned = alignment @ e[valid]
    delta = aligned[:, :3, 3]-t[valid, :3, 3]
    error = np.linalg.norm(delta[:, :2], axis=1)
    angle = Rotation.from_matrix(np.swapaxes(t[valid, :3, :3], 1, 2)
                                 @ aligned[:, :3, :3]).magnitude()
    consecutive = np.diff(np.flatnonzero(valid)) == 1
    truth_steps = np.linalg.norm(np.diff(t[valid, :3, 3], axis=0)[:, :2], axis=1)
    result = dict(alignment='first common pose full SE(3), fixed scale 1, no path fit',
                  anchor_ns=int(anchor_ns), first_pose_transform=alignment.tolist(),
                  grid_period_ns=period_ns, grid_samples=len(grid), matched_samples=int(valid.sum()),
                  skipped_for_missing_bracket=int((~valid).sum()), extrapolation=False,
                  xy_rmse_m=float(np.sqrt(np.mean(error**2))), xy_max_m=float(error.max()),
                  xy_final_m=float(error[-1]), orientation_rmse_deg=float(
                      np.degrees(np.sqrt(np.mean(angle**2)))),
                  last_matched_stamp_ns=int(grid[valid][-1]),
                  truth_xy_path_m=float(truth_steps[consecutive].sum()),
                  truth_xy_net_m=float(np.linalg.norm(t[valid][-1, :2, 3]-t[valid][0, :2, 3])),
                  truth_max_xy_distance_from_anchor_m=float(np.linalg.norm(
                      t[valid, :2, 3]-t[valid][0, :2, 3], axis=1).max()),
                  note='Gazebo truth is used only for evaluation. Path length is sampled, not a collision test.')
    return result, grid[valid], aligned, t[valid], error


def compact_json(value):
    return json.dumps(value, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


class CsvFiles:
    def __init__(self, directory):
        self.directory, self.files, self.writers = directory, {}, {}

    def write(self, filename, row):
        if filename not in self.files:
            stream = (self.directory/filename).open('x', newline='')
            writer = csv.DictWriter(stream, fieldnames=list(row))
            writer.writeheader()
            self.files[filename], self.writers[filename] = stream, writer
        self.writers[filename].writerow(row)

    def flush(self):
        for stream in self.files.values():
            stream.flush()

    def close(self):
        for stream in self.files.values():
            stream.close()


def run(args):
    # Keeping ROS imports here makes the numerical checks runnable without ROS.
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from geometry_msgs.msg import Twist, TwistStamped, PointStamped
    from nav_msgs.msg import Odometry, Path as NavPath
    from sensor_msgs.msg import PointCloud2, Imu
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener, TransformException

    if os.environ.get('ROS_DOMAIN_ID') != '94' or os.environ.get('GZ_PARTITION') != 'ssy-p9-nav-20261110':
        raise RuntimeError('Use simulation/inside.sh; isolated simulation domain required')
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    directory = output.with_suffix('.data')
    if output.exists() or directory.exists():
        raise FileExistsError('Refusing to overwrite output or raw-record directory')
    directory.mkdir()
    csvs = CsvFiles(directory)
    status_file = (directory/'status.jsonl').open('x')
    rclpy.init(args=[])
    node = rclpy.create_node('p9_navigation_validation')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    goal_pub = node.create_publisher(PointStamped, '/p9/navigation/goal_request', 1)
    stop_client = node.create_client(Trigger, '/p9/navigation/stop')
    started = time.monotonic()
    counts, latest, received, invalid = Counter(), {}, {}, Counter()
    poses = {key: [] for key in POSE_TOPICS}
    commands, far_events, path_sizes = [], [], []
    goal, anchor, goal_ack = None, None, None
    reason, exception = None, None
    interrupt = {'signal': None}
    old_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, lambda number, frame: interrupt.update(signal=number))
    clock_last, clock_advanced_wall, first_clock = None, started, None
    max_clock_stall, clock_backwards = 0.0, 0
    last_flush, last_wait_reason = started, 'Waiting for sensors'

    def stamp_ns(stamp):
        return int(stamp.sec)*NS+int(stamp.nanosec)

    def common(topic, msg):
        wall = time.monotonic()
        now = node.get_clock().now().nanoseconds
        header = getattr(msg, 'header', None)
        result = dict(topic=topic, sequence=counts[topic], received_wall_sec=wall-started,
                      received_ros_ns=now, stamp_ns=stamp_ns(header.stamp) if header else '',
                      frame=header.frame_id if header else '')
        return result, wall

    def callback(topic, msg):
        nonlocal goal_ack
        counts[topic] += 1
        base, wall = common(topic, msg)
        received[topic] = wall
        try:
            if isinstance(msg, Odometry):
                p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
                pos, quat = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
                motion = [v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z]
                row = dict(base, child=msg.child_frame_id, x=p.x, y=p.y, z=p.z,
                           qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                           vx=motion[0], vy=motion[1], vz=motion[2], wx=motion[3], wy=motion[4], wz=motion[5],
                           pose_covariance=json.dumps(list(msg.pose.covariance)),
                           twist_covariance=json.dumps(list(msg.twist.covariance)))
                csvs.write('odometry.csv', row)
                sample = dict(stamp_ns=base['stamp_ns'], position=pos, quaternion=quat,
                              frame=base['frame'], child=msg.child_frame_id,
                              received_wall_sec=wall-started, twist=motion)
                # Preserve invalid raw values in CSV, but do not allow goals/metrics from them.
                pose_matrix(pos, quat)
                if poses[topic] and sample['stamp_ns'] <= poses[topic][-1]['stamp_ns']:
                    invalid[topic+' nonincreasing_stamp'] += 1
                poses[topic].append(sample)
                latest[topic] = sample
            elif isinstance(msg, Imu):
                a, w, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
                csvs.write('imu.csv', dict(base, ax=a.x, ay=a.y, az=a.z, wx=w.x, wy=w.y, wz=w.z,
                    qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                    acceleration_covariance=json.dumps(list(msg.linear_acceleration_covariance)),
                    angular_velocity_covariance=json.dumps(list(msg.angular_velocity_covariance)),
                    orientation_covariance=json.dumps(list(msg.orientation_covariance))))
                latest[topic] = dict(stamp_ns=base['stamp_ns'], frame=base['frame'],
                                     acceleration=[a.x, a.y, a.z], angular_velocity=[w.x, w.y, w.z])
                if not np.isfinite([a.x, a.y, a.z, w.x, w.y, w.z]).all():
                    invalid[topic+' nonfinite_measurement'] += 1
            elif isinstance(msg, PointCloud2):
                row = dict(base, width=msg.width, height=msg.height, points=msg.width*msg.height,
                           point_step=msg.point_step, row_step=msg.row_step, data_bytes=len(msg.data),
                           is_dense=msg.is_dense, is_bigendian=msg.is_bigendian,
                           fields=compact_json([dict(name=f.name, offset=f.offset,
                                                    datatype=f.datatype, count=f.count) for f in msg.fields]))
                csvs.write('cloud_messages.csv', row)
                latest[topic] = row
            elif isinstance(msg, (Twist, TwistStamped)):
                velocity = msg.twist if isinstance(msg, TwistStamped) else msg
                v, w = velocity.linear, velocity.angular
                row = dict(base, vx=v.x, vy=v.y, vz=v.z, wx=w.x, wy=w.y, wz=w.z)
                csvs.write('commands.csv', row)
                latest[topic] = row
                commands.append(row)
            elif isinstance(msg, NavPath):
                csvs.write('paths.csv', dict(base, pose_count=len(msg.poses)))
                for i, item in enumerate(msg.poses):
                    p, q = item.pose.position, item.pose.orientation
                    csvs.write('path_poses.csv', dict(base, pose_index=i, pose_stamp_ns=stamp_ns(item.header.stamp),
                        pose_frame=item.header.frame_id, x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w))
                path_sizes.append(len(msg.poses))
                latest[topic] = dict(base, pose_count=len(msg.poses))
            elif isinstance(msg, PointStamped):
                row = dict(base, x=msg.point.x, y=msg.point.y, z=msg.point.z)
                csvs.write('waypoints.csv', row)
                latest[topic] = row
            elif isinstance(msg, Bool):
                row = dict(base, value=bool(msg.data))
                csvs.write('far_status.csv', row)
                far_events.append(row)
                latest[topic] = row
            elif isinstance(msg, String):
                row = dict(base, text=msg.data)
                try:
                    row['parsed_json'] = json.loads(msg.data)
                except (ValueError, TypeError):
                    row['parsed_json'] = None
                status_file.write(json.dumps(row, ensure_ascii=False)+'\n')
                latest[topic] = row
                if topic == '/p9/navigation/goal_result' and goal and isinstance(row['parsed_json'], dict):
                    if row['parsed_json'].get('request_id') == goal['stamp_ns']:
                        goal_ack = row['parsed_json']
        except (ValueError, TypeError, OverflowError) as exc:
            invalid[topic+' invalid_callback_data'] += 1
            node.get_logger().warning(f'{topic}: {exc}', throttle_duration_sec=5.)

    types = {**{t: Odometry for t in POSE_TOPICS}, **{t: PointCloud2 for t in CLOUD_TOPICS},
             '/camera_linear/imu': Imu, '/p9_sim/gyro': Imu, '/p9_sim/accel': Imu, '/way_point': PointStamped, '/path': NavPath,
             '/p9/navigation/raw_cmd_vel': TwistStamped, '/cmd_vel': Twist,
             '/p9/navigation/cmd_vel': Twist, '/p9/navigation/status': String,
             '/p9/navigation/goal_result': String,
             '/far_reach_goal_status': Bool, '/p9/status': String, '/p9/sensor_status': String}
    subscriptions = [node.create_subscription(typ, topic, lambda msg, t=topic: callback(t, msg), qos)
                     for topic, typ in types.items()]

    def maybe_goal():
        nonlocal anchor, goal, last_wait_reason
        now = time.monotonic()
        missing = [t for t in REQUIRED_BEFORE_GOAL if t not in latest or now-received[t] > 2]
        if missing:
            last_wait_reason = 'Missing/stale required topics: '+', '.join(missing)
            return
        status = latest.get('/p9/navigation/status', {}).get('parsed_json')
        if (not isinstance(status, dict) or now-received.get('/p9/navigation/status', 0) > 2
                or not status.get('ready') or not status.get('sim_command_output')
                or status.get('hardware_control') is not False):
            last_wait_reason = 'Waiting for ready simulator guard with explicit sim_command_output'
            return
        if node.get_clock().now().nanoseconds <= 0:
            last_wait_reason = 'No simulated clock'
            return
        if any(v for k, v in invalid.items() if not k.startswith('/odom/ground_truth ')):
            last_wait_reason = 'Invalid input has been recorded; refusing automatic goal'
            return
        window = stationary_window(poses['/state_estimation'], int(args.stationary_seconds*NS),
                                   args.stationary_position, args.stationary_angle)
        if window is None:
            last_wait_reason = 'Waiting for stationary estimate window'
            return
        sample = window['sample']
        if sample['child'] != 'base_link':
            last_wait_reason = 'state_estimation does not describe base_link'
            return
        # The guarded /cmd_vel is the executed command. pathFollower may already
        # suggest motion for its default waypoint on /p9/navigation/raw_cmd_vel; the P9
        # guard deliberately blocks that suggestion until an explicit goal.
        # Keep recording both streams, but do not gate startup on the suggestion.
        command = latest.get('/cmd_vel')
        if command and (math.hypot(command['vx'], command['vy']) > .02 or abs(command['wz']) > .05):
            last_wait_reason = 'Nonzero pre-goal executed /cmd_vel'
            return
        if goal_pub.get_subscription_count() < 1:
            last_wait_reason = 'No /p9/navigation/goal_request subscriber discovered'
            return
        try:
            if sample['frame'] == args.map_frame:
                map_from_estimate = np.eye(4)
            else:
                tf = buffer.lookup_transform(args.map_frame, sample['frame'],
                                             Time(nanoseconds=sample['stamp_ns']),
                                             timeout=Duration(seconds=0.0)).transform
                map_from_estimate = pose_matrix([tf.translation.x, tf.translation.y, tf.translation.z],
                    [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
        except (TransformException, ValueError) as exc:
            last_wait_reason = 'Waiting for map<-estimate TF: '+str(exc)
            return
        estimate_pose = pose_matrix(sample['position'], sample['quaternion'])
        map_pose = map_from_estimate @ estimate_pose
        if args.goal_map is None:
            target = relative_horizontal_goal(map_pose, args.goal_forward, args.goal_left)
            goal_mode = 'relative_initial_body_yaw'
            goal_convention = 'Horizontal initial body yaw; z remains initial body height'
        else:
            target = np.array([args.goal_map[0], args.goal_map[1], map_pose[2, 3]], dtype=float)
            goal_mode = 'absolute_map'
            goal_convention = 'Specified absolute map X/Y; z remains initial estimated body height'
        point = PointStamped()
        point.header.frame_id = args.map_frame
        point.header.stamp = node.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = target.tolist()
        goal_pub.publish(point)
        goal = dict(published_once=True, topic='/p9/navigation/goal_request', frame=args.map_frame, mode=goal_mode,
                    stamp_ns=stamp_ns(point.header.stamp), received_wall_sec=now-started,
                    position=target.tolist(), forward_m=args.goal_forward if args.goal_map is None else None,
                    left_m=args.goal_left if args.goal_map is None else None,
                    requested_map_xy=args.goal_map, convention=goal_convention,
                    truth_used_to_generate_goal=False, map_from_estimate=map_from_estimate.tolist(),
                    pre_goal_command_gate_topic='/cmd_vel',
                    pre_goal_commands={topic: latest.get(topic)
                                       for topic in ('/cmd_vel', '/p9/navigation/cmd_vel', '/p9/navigation/raw_cmd_vel')},
                    pre_goal_command_note='Guarded executed command gates startup; navigation suggestion is recorded without blocking the goal')
        anchor = dict(stamp_ns=sample['stamp_ns'], estimate_pose=estimate_pose.tolist(),
                      estimate_frame=sample['frame'], body_frame=sample['child'], stationary_window=window)
        csvs.write('sent_goal.csv', dict(stamp_ns=goal['stamp_ns'], received_wall_sec=now-started,
                                       frame=args.map_frame, x=target[0], y=target[1], z=target[2]))
        node.get_logger().info(f'Published ONE goal {target.tolist()} in {args.map_frame}; ground truth is not a control input')

    def final_motion(include_truth=True):
        report = {}
        now = time.monotonic()
        for topic in ('/cmd_vel', '/p9/navigation/cmd_vel', '/p9/navigation/raw_cmd_vel'):
            sample = latest.get(topic)
            report[topic] = None if sample is None else dict(
                final_velocity=[sample[k] for k in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')],
                age_wall_sec=now-received[topic],
                stopped=math.hypot(sample['vx'], sample['vy']) <= args.stop_speed
                        and abs(sample['wz']) <= args.stop_yaw_rate)
        for topic in (('/state_estimation', '/odom/ground_truth') if include_truth else ('/state_estimation',)):
            values = poses[topic]
            if len(values) < 2:
                report[topic] = None
                continue
            end = values[-1]['stamp_ns']
            tail = [x for x in values if x['stamp_ns'] >= end-NS]
            if len(tail) < 2:
                report[topic] = None
                continue
            dt = np.diff([x['stamp_ns'] for x in tail])/NS
            ds = np.linalg.norm(np.diff([x['position'][:2] for x in tail], axis=0), axis=1)
            valid = dt > 0
            speed = ds[valid]/dt[valid]
            report[topic] = dict(final_twist=values[-1]['twist'], age_wall_sec=now-received[topic],
                                last_second_xy_speed_median=float(np.median(speed)) if len(speed) else None,
                                last_second_xy_speed_p95=float(np.percentile(speed, 95)) if len(speed) else None,
                                stopped=bool(len(speed) and np.percentile(speed, 95) <= args.stop_speed))
        return report

    arrival_since = None
    try:
        while rclpy.ok() and interrupt['signal'] is None:
            rclpy.spin_once(node, timeout_sec=.05)
            wall, ros = time.monotonic(), node.get_clock().now().nanoseconds
            if clock_last is not None and ros < clock_last:
                clock_backwards += 1
                reason = 'simulation_clock_moved_backwards'
                break
            if ros > 0 and (clock_last is None or ros > clock_last):
                if first_clock is None:
                    first_clock = ros
                max_clock_stall = max(max_clock_stall, wall-clock_advanced_wall)
                clock_advanced_wall = wall
            clock_last = ros
            if first_clock is not None and wall-clock_advanced_wall > args.clock_stall_timeout:
                reason = 'simulation_clock_stalled'
                break
            if goal is None:
                maybe_goal()
                if wall-started > args.startup_timeout:
                    reason = 'startup_timeout'
                    break
            elif wall-started-goal['received_wall_sec'] >= args.seconds:
                reason = 'post_goal_wall_budget_complete'
                break
            elif goal_ack is None:
                if wall-started-goal['received_wall_sec'] > 5:
                    reason = 'goal_ack_timeout'
                    break
            elif not goal_ack.get('accepted', False):
                reason = 'goal_rejected'
                break
            elif args.stop_on_arrival:
                reached = any(x['value'] and x['received_wall_sec'] >= goal['received_wall_sec'] for x in far_events)
                motion = final_motion(include_truth=False)
                estimate = poses['/state_estimation']
                moved = bool(len(estimate) > 1 and np.linalg.norm(np.array(estimate[-1]['position'])[:2]
                                   -np.array(anchor['estimate_pose'])[:2, 3]) > .1)
                stopped = all(motion.get(k) and motion[k]['stopped'] and motion[k]['age_wall_sec'] < 2
                              for k in ('/cmd_vel', '/p9/navigation/cmd_vel', '/state_estimation'))
                if reached and moved and stopped:
                    arrival_since = wall if arrival_since is None else arrival_since
                    if wall-arrival_since >= 2:
                        reason = 'estimated_motion_far_arrival_and_stopped'
                        break
                else:
                    arrival_since = None
            if wall-last_flush > 1:
                csvs.flush()
                status_file.flush()
                last_flush = wall
        if reason is None:
            reason = 'signal_'+str(interrupt['signal']) if interrupt['signal'] else 'ros_shutdown'
    except Exception as exc:
        reason, exception = 'recorder_exception', repr(exc)
    finally:
        # Freeze observed arrival before requesting stop; do not manufacture arrival from our stop.
        pre_stop_motion = final_motion()
        evaluation_poses = {topic: list(samples) for topic, samples in poses.items()}
        stop_result = {'requested': False, 'success': False}
        try:
            if stop_client.wait_for_service(timeout_sec=2.):
                stop_result['requested'] = True
                future = stop_client.call_async(Trigger.Request())
                deadline = time.monotonic()+3.
                while not future.done() and time.monotonic() < deadline and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=.05)
                if future.done() and future.exception() is None:
                    response = future.result()
                    stop_result.update(success=bool(response.success), message=response.message)
                else:
                    stop_result['error'] = 'Stop response timeout or exception'
                # Observe a short post-stop tail; this is recorded separately from arrival.
                deadline = time.monotonic()+1.
                while time.monotonic() < deadline and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=.05)
            else:
                stop_result['error'] = 'Stop service unavailable'
        except Exception as exc:
            stop_result['error'] = repr(exc)
        duration = time.monotonic()-started
        max_clock_stall = max(max_clock_stall, time.monotonic()-clock_advanced_wall)
        report = dict(recording_stop_reason=reason, exception=exception, duration_wall_sec=duration,
                      requested_post_goal_wall_sec=args.seconds, startup_timeout_wall_sec=args.startup_timeout,
                      goal=goal, goal_ack=goal_ack, stop_service=stop_result, anchor=anchor, startup_wait_reason=last_wait_reason,
                      counts=dict(counts), invalid=dict(invalid), latest=latest,
                      topic_type={k: v.__name__ for k, v in types.items()},
                      raw_directory=str(directory), cloud_payloads_saved=False,
                      raw_timestamps_modified=False, simulated_clock=dict(first_ns=first_clock,
                          last_ns=clock_last, backwards_events=clock_backwards,
                          max_no_advance_wall_sec=max_clock_stall,
                          elapsed_sec=None if first_clock is None else (clock_last-first_clock)/NS),
                      path_size_range=[min(path_sizes), max(path_sizes)] if path_sizes else None,
                      final_motion=pre_stop_motion, post_stop_motion=final_motion(), collision=dict(validated=False,
                          reason='No contact topic recorded. No collision-free claim or world-clearance calculation.'),
                      controls_published=(['/p9/navigation/goal_request'] if goal else [])
                          + (['/p9/navigation/stop'] if stop_result['requested'] else []),
                      no_velocity_command_published=True,
                      ground_truth_role='Evaluation only; never fed to estimator, planner, command bridge or goal generation')
        report['far_after_goal'] = dict(true_count=sum(x['value'] for x in far_events
            if goal and x['received_wall_sec'] >= goal['received_wall_sec']),
            last_after_goal=next((x for x in reversed(far_events)
                if goal and x['received_wall_sec'] >= goal['received_wall_sec']), None))
        try:
            if anchor is None:
                raise ValueError('No accepted stationary anchor / goal was not published')
            evaluation = select_evaluation_anchor(
                evaluation_poses['/state_estimation'], evaluation_poses['/odom/ground_truth'], anchor['stamp_ns'], goal['stamp_ns'],
                anchor['stamp_ns']+round(anchor['stationary_window']['duration_sim_sec']*NS),
                max_gap_ns=int(args.max_pose_gap*NS), position_limit=args.stationary_position,
                angle_limit=args.stationary_angle)
            report['evaluation_anchor'] = evaluation
            evaluation_ns = evaluation['evaluation_anchor_ns']
            m, stamps, aligned, truth_m, errors = position_metrics(
                evaluation_poses['/state_estimation'], evaluation_poses['/odom/ground_truth'], evaluation_ns,
                max_gap_ns=int(args.max_pose_gap*NS))
            report['localization'] = m
            truth_anchor, ok = interpolate_poses(evaluation_poses['/odom/ground_truth'], [evaluation_ns],
                                                int(args.max_pose_gap*NS))
            assert ok[0]
            if goal['mode'] == 'absolute_map' or evaluation_ns != anchor['stamp_ns']:
                truth_goal = map_goal_in_truth(goal['position'], np.array(m['first_pose_transform']),
                                               np.array(goal['map_from_estimate']))
                description = 'truth_from_estimate(evaluation_anchor) @ inverse(map_from_estimate) @ unchanged published map goal; evaluation only'
            else:
                truth_goal = relative_horizontal_goal(truth_anchor[0], args.goal_forward, args.goal_left)
                description = 'Same forward/left offset at synchronized truth anchor; evaluation only'
            report['truth_goal'] = dict(frame=evaluation_poses['/odom/ground_truth'][0]['frame'], mode=goal['mode'],
                                       position=truth_goal.tolist(), generated_from=description)
            if goal['mode'] == 'relative_initial_body_yaw':
                # Preserve the existing forward-run report key and semantics.
                report['truth_relative_goal'] = report['truth_goal']
            report['final_truth_goal_distance_xy_m'] = float(np.linalg.norm(truth_m[-1, :2, 3]-truth_goal[:2]))
            estimate_last, valid_last = interpolate_poses(evaluation_poses['/state_estimation'], [int(stamps[-1])], int(args.max_pose_gap*NS))
            assert valid_last[0]
            map_last = np.array(goal['map_from_estimate']) @ estimate_last[0]
            report['final_estimated_goal_distance_xy_m'] = float(np.linalg.norm(map_last[:2, 3]-np.array(goal['position'])[:2]))
            report['goal_distance_evaluation_stamp_ns'] = int(stamps[-1])
            report['truth_within_goal_tolerance'] = report['final_truth_goal_distance_xy_m'] <= args.goal_tolerance
            for i, stamp in enumerate(stamps):
                csvs.write('aligned_comparison.csv', dict(stamp_ns=int(stamp),
                    elapsed_sim_sec=(int(stamp)-evaluation_ns)/NS,
                    estimate_x=aligned[i, 0, 3], estimate_y=aligned[i, 1, 3], estimate_z=aligned[i, 2, 3],
                    truth_x=truth_m[i, 0, 3], truth_y=truth_m[i, 1, 3], truth_z=truth_m[i, 2, 3], xy_error_m=errors[i]))
        except (ValueError, AssertionError) as exc:
            report['localization'] = dict(available=False, reason=str(exc))
        motion = report['final_motion']
        stop_topics = ('/cmd_vel', '/p9/navigation/cmd_vel', '/state_estimation', '/odom/ground_truth')
        stopped_fresh = all(motion.get(topic) and motion[topic]['stopped']
                            and motion[topic]['age_wall_sec'] < 2 for topic in stop_topics)
        moved = report['localization'].get('truth_max_xy_distance_from_anchor_m', 0) > .1
        arrival = bool(moved and report['far_after_goal']['true_count'] > 0
                       and stopped_fresh and report.get('truth_within_goal_tolerance', False))
        report['navigation_observations'] = dict(
            actual_truth_movement_over_10cm=bool(moved), far_true_after_goal=report['far_after_goal']['true_count'] > 0,
            final_commands_and_measured_motion_stopped_and_fresh=bool(stopped_fresh),
            truth_goal_tolerance_m=args.goal_tolerance,
            arrival_observed=arrival,
            attempt_status='arrival_observed' if arrival else 'arrival_not_confirmed_in_recorded_interval',
            note='This combines separate observations, not a collision or absolute-accuracy PASS. Exit code reports recording usability.')
        report['recording_usable'] = bool(goal and goal_ack and goal_ack.get('accepted')
                                           and stop_result['success']
                                           and report['localization'].get('matched_samples', 0) >= 2
                                           and not invalid and exception is None and clock_backwards == 0)
        report['accuracy_pass'] = None
        report['notes'] = ['No accuracy threshold is assumed; inspect localization and raw series.',
                           'FAR true, final commanded stop, actual motion, truth goal distance and collision are separate observations.',
                           'CSV counts are received messages, not proof that middleware dropped none.',
                           'IMU and odometry CSVs contain every received message; cloud CSV contains metadata, not points.',
                           'Exit requests navigation stop; pre-stop arrival observations and post-stop motion are kept separate.']
        # JSON summaries cannot represent nonfinite input; raw CSV retains it.
        def clean(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {k: clean(v) for k, v in value.items()}
            if isinstance(value, (list, tuple)):
                return [clean(v) for v in value]
            return value
        report = clean(report)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
        csvs.close()
        status_file.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        node.destroy_node()
        rclpy.try_shutdown()
        print(json.dumps({k: report.get(k) for k in ('recording_stop_reason', 'recording_usable',
              'duration_wall_sec', 'counts', 'localization', 'final_truth_goal_distance_xy_m',
              'final_estimated_goal_distance_xy_m', 'far_after_goal', 'navigation_observations')}, indent=2))
    return 0 if report['recording_usable'] else 2


def self_test():
    import unittest

    class GeometryTests(unittest.TestCase):
        def sample(self, ns, p=(0, 0, 0), q=(0, 0, 0, 1)):
            return dict(stamp_ns=ns, position=list(p), quaternion=list(q), frame='world', child='base_link')

        def test_nanoseconds_slerp_and_sign(self):
            epoch = 1_793_255_293_084_837_436
            q = Rotation.from_euler('z', 90, degrees=True).as_quat()
            s = [self.sample(epoch), self.sample(epoch+100_000_000, (2, 0, 0), -q)]
            p, valid = interpolate_poses(s, [epoch+50_000_000])
            self.assertTrue(valid[0])
            np.testing.assert_allclose(p[0, :3, 3], [1, 0, 0])
            np.testing.assert_allclose(p[0, :3, :3], Rotation.from_euler('z', 45, degrees=True).as_matrix())

        def test_no_extrapolation_and_long_gap(self):
            s = [self.sample(100), self.sample(500_000_100)]
            _, valid = interpolate_poses(s, [99, 100, 1000, 500_000_100, 500_000_101])
            self.assertEqual(valid.tolist(), [False, True, False, True, False])

        def test_duplicates_rejected(self):
            with self.assertRaises(ValueError):
                interpolate_poses([self.sample(10), self.sample(10)], [10])

        def test_same_time_full_se3_alignment(self):
            A = pose_matrix([2, -3, .5], Rotation.from_euler('xyz', [.1, .2, .7]).as_quat())
            e, t = [], []
            for i in range(5):
                q = Rotation.from_euler('z', .1*i).as_quat()
                e.append(self.sample(i*100_000_000+1, (i*.2, i*.1, 0), q))
                T = A @ pose_matrix(e[-1]['position'], q)
                t.append(self.sample(i*100_000_000+1, T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_quat()))
            m, *_ = position_metrics(e, t, 1)
            self.assertLess(m['xy_rmse_m'], 1e-12)

        def test_goal_uses_yaw(self):
            T = pose_matrix([1, 2, .3], Rotation.from_euler('z', 90, degrees=True).as_quat())
            np.testing.assert_allclose(relative_horizontal_goal(T, 6.5, 1), [0, 8.5, .3], atol=1e-12)

        def test_absolute_map_goal_uses_inverse_map_transform(self):
            map_from_estimate = pose_matrix([4, 5, .2],
                Rotation.from_euler('z', 90, degrees=True).as_quat())
            truth_from_estimate = pose_matrix([-2, 3, 1.],
                Rotation.from_euler('z', -90, degrees=True).as_quat())
            # Estimate-frame point [1,2,.5] -> map [2,6,.7] -> truth [0,2,1.5].
            point = map_goal_in_truth([2, 6, .7], truth_from_estimate, map_from_estimate)
            np.testing.assert_allclose(point, [0, 2, 1.5], atol=1e-12)

        def test_absolute_map_goal_identity_and_invalid_transform(self):
            np.testing.assert_allclose(map_goal_in_truth([1, -2, .3], np.eye(4), np.eye(4)), [1, -2, .3])
            reflected = np.diag([-1., 1., 1., 1.])
            with self.assertRaises(ValueError):
                map_goal_in_truth([1, -2, .3], reflected, np.eye(4))

        def test_reject_imu_origin(self):
            s = [self.sample(1), self.sample(100_000_001)]
            s[0]['child'] = 'camera_imu'
            with self.assertRaises(ValueError):
                position_metrics(s, s, 1)

        def test_stationary_gate(self):
            s = [self.sample(i*100_000_000+1, (0.001*(i % 2), 0, 0)) for i in range(21)]
            self.assertIsNotNone(stationary_window(s, 2*NS, .05, .12))
            s[-1]['position'][0] = 1
            self.assertIsNone(stationary_window(s, 2*NS, .05, .12))

        def test_late_truth_40ms_uses_first_covered_pregoal_grid_point(self):
            original = 205_900_000_000
            estimate = [self.sample(original+i*100_000_000) for i in range(40)]
            truth = [self.sample(original+40_000_000+i*20_000_000, (2, 3, 0)) for i in range(195)]
            chosen = select_evaluation_anchor(estimate, truth, original, 207_825_000_000,
                                               207_800_000_000)
            self.assertEqual(chosen['evaluation_anchor_ns'], 206_000_000_000)
            self.assertFalse(chosen['original_anchor_truth_covered'])
            m, *_ = position_metrics(estimate, truth, chosen['evaluation_anchor_ns'])
            self.assertLess(m['xy_rmse_m'], 1e-12)

        def test_evaluation_anchor_cannot_move_past_goal_or_into_motion(self):
            estimate = [self.sample(i*100_000_000) for i in range(40)]
            truth_late = [self.sample(2*NS+i*100_000_000) for i in range(20)]
            with self.assertRaises(ValueError):
                select_evaluation_anchor(estimate, truth_late, 0, 2*NS, 1900_000_000)
            truth_moving = [self.sample(i*100_000_000, (i*.1, 0, 0)) for i in range(40)]
            with self.assertRaises(ValueError):
                select_evaluation_anchor(estimate, truth_moving, 0, 2*NS, 1900_000_000)

    return 0 if unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(GeometryTests)).wasSuccessful() else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=180, help='Maximum wall seconds AFTER goal publication')
    parser.add_argument('--startup-timeout', type=float, default=240, help='Maximum wall seconds waiting to send goal')
    parser.add_argument('--goal-forward', type=float, default=6.5)
    parser.add_argument('--goal-left', type=float, default=0.)
    parser.add_argument('--goal-map', type=float, nargs=2, metavar=('X', 'Y'),
                        help='Absolute X/Y in map-frame; overrides the default relative forward/left goal')
    parser.add_argument('--output', default='/output/validation-forward.json')
    parser.add_argument('--map-frame', default='camera_init')
    parser.add_argument('--stationary-seconds', type=float, default=2., help='Initial simulated-time estimate window')
    parser.add_argument('--stationary-position', type=float, default=.05)
    parser.add_argument('--stationary-angle', type=float, default=.12, help='Radians of initial orientation excursion')
    parser.add_argument('--clock-stall-timeout', type=float, default=10., help='Wall seconds without simulation-clock advance')
    parser.add_argument('--max-pose-gap', type=float, default=.2)
    parser.add_argument('--goal-tolerance', type=float, default=.5, help='Evaluation only; does not change FAR')
    parser.add_argument('--stop-speed', type=float, default=.05)
    parser.add_argument('--stop-yaw-rate', type=float, default=.1)
    parser.add_argument('--stop-on-arrival', action='store_true', help='Finish early from estimated motion, FAR true and fresh stopped commands/estimate; never GT')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    for key in ('seconds', 'startup_timeout', 'stationary_seconds', 'stationary_position',
                'stationary_angle', 'clock_stall_timeout', 'max_pose_gap', 'goal_tolerance',
                'stop_speed', 'stop_yaw_rate'):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(key+' must be finite and positive')
    if not np.isfinite([args.goal_forward, args.goal_left]).all():
        parser.error('goal offsets must be finite')
    if args.goal_map is not None and not np.isfinite(args.goal_map).all():
        parser.error('absolute map goal must be finite')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
