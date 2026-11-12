#!/usr/bin/env python3
"""Passive P9/TARE simulation recorder. No publishers, services or control clients.

The observed 0.2 m voxel union is sensor data growth, NOT GT coverage. Ground-truth
odometry is subscribed for independent motion evaluation only, never fed to TARE.
"""
import argparse
import ast
import csv
from collections import defaultdict, deque
import json
import math
from pathlib import Path
import time


def stamp_seconds(stamp):
    return int(stamp.sec) + int(stamp.nanosec) / 1e9


def finite_xyz(point):
    values = (float(point.x), float(point.y), float(point.z))
    return values if all(math.isfinite(x) for x in values) else None


def atomic_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


class Trajectory:
    def __init__(self):
        self.first = self.last = None
        self.count = 0
        self.path_length = self.max_displacement = 0.0
        self.recent = deque()

    def add(self, xyz, wall):
        if xyz is None or len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            return False
        xyz = tuple(xyz)
        if self.first is None:
            self.first = xyz
        if self.last is not None:
            self.path_length += math.dist(self.last, xyz)
        self.last = xyz
        self.count += 1
        self.max_displacement = max(self.max_displacement, math.dist(self.first, xyz))
        self.recent.append((wall, xyz))
        while self.recent and wall - self.recent[0][0] > 3.0:
            self.recent.popleft()
        return True

    def summary(self):
        duration = self.recent[-1][0] - self.recent[0][0] if len(self.recent) > 1 else 0.0
        distance = sum(math.dist(a[1], b[1]) for a, b in zip(self.recent, list(self.recent)[1:]))
        return dict(samples=self.count, first_xyz=self.first, last_xyz=self.last,
                    raw_path_length_m=self.path_length, max_displacement_from_first_m=self.max_displacement,
                    final_displacement_from_first_m=math.dist(self.first, self.last) if self.first else None,
                    recent_motion_window_wall_sec=duration,
                    recent_mean_speed_m_per_wall_sec=distance / duration if duration > 0 else None)


class Waypoints:
    def __init__(self):
        self.count = self.invalid = 0
        self.cells = set()
        self.first = self.last = None
        self.max_distance_from_first = 0.0

    def add(self, xyz):
        if xyz is None or not all(math.isfinite(v) for v in xyz):
            self.invalid += 1
            return
        self.count += 1
        self.cells.add(tuple(math.floor(v / .25) for v in xyz))
        if self.first is None:
            self.first = xyz
        self.last = xyz
        self.max_distance_from_first = max(self.max_distance_from_first, math.dist(self.first, xyz))

    def summary(self):
        return dict(count=self.count, invalid=self.invalid, distinct_025m_cells=len(self.cells),
                    first_xyz=self.first, last_xyz=self.last,
                    max_distance_from_first_m=self.max_distance_from_first)


class CommandTail:
    def __init__(self):
        self.count = self.nonzero = self.invalid = 0
        self.last_wall = self.zero_since = self.last_nonzero_wall = None
        self.max_abs_linear = self.max_abs_yaw_rate = 0.0
        self.last = None

    def add(self, values, wall):
        self.count += 1
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            self.invalid += 1
            self.zero_since = None
            return
        self.last_wall, self.last = wall, values
        self.max_abs_linear = max(self.max_abs_linear, *(abs(v) for v in values[:3]))
        self.max_abs_yaw_rate = max(self.max_abs_yaw_rate, abs(values[5]))
        if any(abs(v) > 1e-4 for v in values):
            self.nonzero += 1
            self.last_nonzero_wall = wall
            self.zero_since = None
        elif self.zero_since is None:
            self.zero_since = wall

    def summary(self, wall):
        tail = self.last_wall - self.zero_since if self.zero_since is not None else 0.0
        age = wall - self.last_wall if self.last_wall is not None else None
        return dict(samples=self.count, nonzero_samples=self.nonzero, invalid_samples=self.invalid,
                    max_abs_linear_m_s=self.max_abs_linear, max_abs_yaw_rate_rad_s=self.max_abs_yaw_rate,
                    last_command=self.last, last_message_wall_age_sec=age,
                    consecutive_observed_zero_tail_wall_sec=tail,
                    recent_zero_commands_for_at_least_2_sec=bool(tail >= 2 and age is not None and age < 1))


class ObservedVoxels:
    def __init__(self, size=.2, baseline_seconds=30.):
        self.size, self.baseline_seconds = size, baseline_seconds
        self.seen = set()
        self.baseline = None
        self.first_stamp = self.last_stamp = None
        self.last_valid_wall = None
        self.valid_clouds = self.finite_points = self.clock_regressions = 0

    def add(self, xyz, stamp):
        import numpy as np
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) == 0 or not math.isfinite(stamp):
            return 0
        if self.first_stamp is None:
            self.first_stamp = stamp
        if self.last_stamp is not None and stamp < self.last_stamp - .001:
            self.clock_regressions += 1
        # Snapshot exactly the union preceding the first scan at/after 30 s.
        if self.baseline is None and stamp - self.first_stamp >= self.baseline_seconds:
            self.baseline = set(self.seen)
        before = len(self.seen)
        quantized = np.unique(np.floor(xyz / self.size).astype('int64'), axis=0)
        self.seen.update(map(tuple, quantized.tolist()))
        self.valid_clouds += 1
        self.finite_points += len(xyz)
        self.last_stamp = stamp
        return len(self.seen) - before

    def summary(self):
        return dict(metric='union of observed world-frame lidar voxels; NOT ground-truth coverage',
                    voxel_size_m=self.size, initial_baseline_scan_time_sec=self.baseline_seconds,
                    baseline_complete=self.baseline is not None,
                    baseline_observed_voxels=len(self.baseline) if self.baseline is not None else None,
                    observed_voxels=len(self.seen),
                    added_voxels_after_baseline=len(self.seen - self.baseline) if self.baseline is not None else None,
                    valid_cloud_count=self.valid_clouds, finite_points_seen=self.finite_points,
                    first_scan_stamp=self.first_stamp, last_scan_stamp=self.last_stamp,
                    scan_stamp_regressions=self.clock_regressions)


def cloud_xyz(message):
    """Respect PointCloud2 endian, point stride and row padding; retain finite xyz."""
    import numpy as np
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 has no x/y/z fields')
    endian = '>' if message.is_bigendian else '<'
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('XYZ fields must be scalar FLOAT32 or FLOAT64')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            raise ValueError('XYZ offset outside point stride')
        formats.append(endian + ('f4' if size == 4 else 'f8'))
        offsets.append(field.offset)
    if not message.width or not message.height:
        return np.empty((0, 3))
    required = (message.height - 1) * message.row_step + message.width * message.point_step
    if message.row_step < message.width * message.point_step or len(message.data) < required:
        raise ValueError('PointCloud2 truncated data or invalid row stride')
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats, offsets=offsets,
                         itemsize=message.point_step))
    array = np.ndarray((message.height, message.width), dtype=dtype,
                       buffer=memoryview(message.data), strides=(message.row_step, message.point_step))
    return np.stack([array[name].reshape(-1) for name in ('x', 'y', 'z')], axis=-1)


def assert_read_only(source):
    tree = ast.parse(source)
    forbidden = {'create_publisher', 'create_client', 'create_service', 'publish', 'call_async'}
    violations = [node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute) and node.attr in forbidden]
    if violations:
        raise AssertionError('Observer contains control API calls: ' + ', '.join(violations))


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped, Twist
    from nav_msgs.msg import Odometry, Path as PathMsg
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    stop_file = Path(args.stop_file).resolve() if args.stop_file else output / 'stop'
    start_wall = time.monotonic()
    streams = defaultdict(lambda: dict(count=0, last_wall=None, last_stamp=None,
                                       max_sim_age_sec=None, invalid=0, future_stamp_count=0))
    estimates, truth = Trajectory(), Trajectory()
    tare_waypoints, forwarded_waypoints = Waypoints(), Waypoints()
    commands, voxels = CommandTail(), ObservedVoxels(args.voxel_size, 30.)
    latest = {'status': None, 'tare_diagnostics': None}
    flags = dict(tare_finished_seen=False, tare_returned_home_seen=False, tare_stopped_seen=False,
                 guard_finished_seen=False, guard_returned_home_seen=False)
    path_stats = dict(count=0, nonempty_count=0, max_poses=0, last_poses=0, max_path_length_m=0.)
    files = {}
    writers = {}
    headers = {
        'poses': ['wall_sec','sim_sec','source','stamp','frame','x','y','z'],
        'commands': ['wall_sec','sim_sec','vx','vy','vz','wx','wy','wz'],
        'waypoints': ['wall_sec','sim_sec','source','stamp','frame','x','y','z'],
        'paths': ['wall_sec','sim_sec','stamp','frame','poses','length_m'],
        'clouds': ['wall_sec','sim_sec','stamp','frame','finite_points','new_voxels','total_voxels','added_after_30s'],
    }
    for name, header in headers.items():
        files[name] = (output / (name + '.csv')).open('w', newline='')
        writers[name] = csv.writer(files[name]); writers[name].writerow(header)
    events = (output / 'events.jsonl').open('w')
    sampled = {}
    previous_state = {}
    rclpy.init(args=[])
    node = Node('p9_exploration_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    sim = lambda: node.get_clock().now().nanoseconds / 1e9
    elapsed = lambda: time.monotonic() - start_wall

    def event(kind, data):
        events.write(json.dumps(dict(wall_sec=elapsed(), sim_sec=sim(), kind=kind, data=data),
                                ensure_ascii=False, allow_nan=False) + '\n')

    def mark(name, stamp=None):
        item = streams[name]
        item['count'] += 1; item['last_wall'] = elapsed()
        if stamp is not None:
            item['last_stamp'] = stamp
            if stamp > sim() + .1:
                item['future_stamp_count'] += 1

    def write_sample(name, source, row, interval=.2):
        wall = elapsed()
        key = (name, source)
        if wall - sampled.get(key, -1e12) >= interval:
            writers[name].writerow(row)
            sampled[key] = wall

    def parse_json(name, message):
        mark(name)
        try:
            data = json.loads(message.data, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            if not isinstance(data, dict):
                raise ValueError('expected JSON object')
            latest[name] = data
            keys = ('state','active','reason','tare_finished','returned_home','starts') if name == 'status' else (
                'exploration_finished','returned_home','stopped')
            state = tuple(data.get(k) for k in keys)
            if previous_state.get(name) != state or elapsed() - sampled.get(('event', name), -1e12) >= 1:
                event(name, data); sampled[('event', name)] = elapsed()
            previous_state[name] = state
            if name == 'status':
                flags['guard_finished_seen'] |= bool(data.get('tare_finished', False))
                flags['guard_returned_home_seen'] |= bool(data.get('returned_home', False))
            else:
                flags['tare_finished_seen'] |= bool(data.get('exploration_finished', False))
                flags['tare_returned_home_seen'] |= bool(data.get('returned_home', False))
                flags['tare_stopped_seen'] |= bool(data.get('stopped', False))
        except (ValueError, TypeError) as error:
            streams[name]['invalid'] += 1
            event('invalid_' + name, {'error': str(error)})

    def pose(name, trajectory, message):
        stamp = stamp_seconds(message.header.stamp); mark(name, stamp)
        xyz = finite_xyz(message.pose.pose.position)
        if not trajectory.add(xyz, elapsed()):
            streams[name]['invalid'] += 1; return
        write_sample('poses', name, [elapsed(), sim(), name, stamp, message.header.frame_id, *xyz])

    def waypoint(name, metrics, message):
        stamp = stamp_seconds(message.header.stamp); mark(name, stamp)
        xyz = finite_xyz(message.point); metrics.add(xyz)
        if xyz is None:
            streams[name]['invalid'] += 1; return
        writers['waypoints'].writerow([elapsed(), sim(), name, stamp, message.header.frame_id, *xyz])

    def path(message):
        stamp = stamp_seconds(message.header.stamp); mark('tare_path', stamp)
        points = [finite_xyz(p.pose.position) for p in message.poses]
        if any(p is None for p in points):
            streams['tare_path']['invalid'] += 1; return
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        path_stats['count'] += 1; path_stats['nonempty_count'] += bool(points)
        path_stats['last_poses'] = len(points)
        path_stats['max_poses'] = max(path_stats['max_poses'], len(points))
        path_stats['max_path_length_m'] = max(path_stats['max_path_length_m'], length)
        writers['paths'].writerow([elapsed(), sim(), stamp, message.header.frame_id, len(points), length])

    def command(message):
        mark('cmd_vel')
        values = tuple(float(v) for v in (message.linear.x, message.linear.y, message.linear.z,
                                          message.angular.x, message.angular.y, message.angular.z))
        commands.add(values, elapsed())
        if all(math.isfinite(v) for v in values):
            write_sample('commands', 'cmd_vel', [elapsed(), sim(), *values])

    def cloud(message):
        stamp = stamp_seconds(message.header.stamp); mark('registered_scan', stamp)
        if message.header.frame_id != args.world_frame:
            streams['registered_scan']['invalid'] += 1
            if streams['registered_scan']['invalid'] <= 3:
                event('rejected_cloud_frame', {'actual': message.header.frame_id, 'expected': args.world_frame})
            return
        try:
            xyz = cloud_xyz(message)
            before_points = voxels.finite_points
            added = voxels.add(xyz, stamp)
            if before_points == voxels.finite_points:
                streams['registered_scan']['invalid'] += 1
            else:
                voxels.last_valid_wall = elapsed()
            writers['clouds'].writerow([elapsed(), sim(), stamp, message.header.frame_id,
                                       voxels.finite_points - before_points, added, len(voxels.seen),
                                       len(voxels.seen - voxels.baseline) if voxels.baseline is not None else ''])
        except (ValueError, TypeError, BufferError, OverflowError) as error:
            streams['registered_scan']['invalid'] += 1
            if streams['registered_scan']['invalid'] <= 3:
                event('invalid_cloud', {'error': str(error)})

    subscriptions = [
        (String, '/p9/exploration/status', lambda m: parse_json('status', m)),
        (String, '/p9/exploration/tare_diagnostics', lambda m: parse_json('tare_diagnostics', m)),
        (PointStamped, '/p9/exploration/tare_waypoint', lambda m: waypoint('tare_waypoint', tare_waypoints, m)),
        (PointStamped, '/way_point', lambda m: waypoint('way_point', forwarded_waypoints, m)),
        (PathMsg, '/p9/exploration/tare_path', path),
        (Twist, '/cmd_vel', command),
        (Odometry, '/state_estimation', lambda m: pose('state_estimation', estimates, m)),
        (Odometry, '/odom/ground_truth', lambda m: pose('ground_truth', truth, m)),
        (PointCloud2, '/registered_scan', cloud),
    ]
    handles = [node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
               for msg_type, topic, callback in subscriptions]
    event('observer_started', dict(read_only=True, use_sim_time=True,
                                 qos='BEST_EFFORT depth 5', topics=[t for _, t, _ in subscriptions],
                                 seconds=args.seconds, stop_file=str(stop_file)))
    previous_sim, clock_backwards = None, 0
    reason = 'wall_time_limit'

    def report(final=False):
        wall, now = elapsed(), sim()
        stream_report = {}
        for name, item in streams.items():
            stream_report[name] = dict(item, last_message_wall_age_sec=wall - item['last_wall'] if item['last_wall'] is not None else None,
                                       latest_header_age_sim_sec=now - item['last_stamp'] if item['last_stamp'] is not None else None)
        return dict(schema_version=1, final=final, stop_reason=reason if final else None,
                    wall_duration_sec=wall, final_sim_stamp=now, use_sim_time=True, read_only=True,
                    simulation_clock_regressions=clock_backwards,
                    streams=stream_report, autonomous_tare_waypoints=tare_waypoints.summary(),
                    forwarded_waypoints=forwarded_waypoints.summary(), tare_path=path_stats,
                    command_output=commands.summary(wall), estimated_trajectory=estimates.summary(),
                    ground_truth_evaluation_only=truth.summary(),
                    observed_cloud_growth=dict(voxels.summary(),
                        last_valid_cloud_wall_age_sec=wall - voxels.last_valid_wall if voxels.last_valid_wall is not None else None,
                        last_valid_cloud_header_age_sim_sec=now - voxels.last_stamp if voxels.last_stamp is not None else None),
                    exploration_flags=flags, last_guard_status=latest['status'],
                    last_tare_diagnostics=latest['tare_diagnostics'],
                    max_odom_age_sim_sec=streams['state_estimation']['max_sim_age_sec'],
                    limitations=[
                        'Observed voxel growth is not ground-truth explored-area coverage and may include pose drift.',
                        'TARE finished means exploration completion/return phase, not returned home or physically stopped.',
                        'TARE stopped is a latched planner state; verify zero command tail and independent GT motion.',
                        'CSV pose/command samples are limited to 5 Hz; counters and metrics use every received message.',
                        'All sensor subscriptions are BEST_EFFORT; the recorder cannot establish lossless acquisition.',
                    ])

    next_progress = 0.
    try:
        while rclpy.ok() and elapsed() < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            current_sim = sim()
            if previous_sim is not None and current_sim < previous_sim - .001:
                clock_backwards += 1
                event('simulation_clock_regression', dict(before=previous_sim, after=current_sim))
            previous_sim = current_sim
            for item in streams.values():
                if item['last_stamp'] is not None and current_sim >= item['last_stamp']:
                    age = current_sim - item['last_stamp']
                    item['max_sim_age_sec'] = max(item['max_sim_age_sec'] or 0., age)
            if stop_file.exists():
                reason = 'stop_file'; break
            if elapsed() >= next_progress:
                atomic_json(output / 'progress.json', report())
                for handle in files.values(): handle.flush()
                events.flush(); next_progress = elapsed() + 1.
    except KeyboardInterrupt:
        reason = 'keyboard_interrupt'
    finally:
        event('observer_finished', {'reason': reason})
        summary = report(final=True)
        atomic_json(output / 'summary.json', summary)
        atomic_json(output / 'progress.json', summary)
        for handle in files.values(): handle.close()
        events.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(json.dumps(dict(output=str(output), stop_reason=reason,
                          nonzero_commands=commands.nonzero,
                          gt_max_displacement_m=truth.max_displacement,
                          tare_finished=flags['tare_finished_seen'],
                          returned_home=flags['guard_returned_home_seen']), allow_nan=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New output directory (must not already exist)')
    parser.add_argument('--seconds', type=float, default=480., help='Wall-time duration, 1 to 1200 seconds')
    parser.add_argument('--stop-file', help='Optional file whose appearance ends recording')
    parser.add_argument('--world-frame', default='camera_init')
    parser.add_argument('--voxel-size', type=float, default=.2)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 1200:
        parser.error('--seconds must be between 1 and 1200')
    if not math.isfinite(args.voxel_size) or args.voxel_size <= 0:
        parser.error('--voxel-size must be finite and positive')
    assert_read_only(Path(__file__).read_text())
    run(args)


if __name__ == '__main__':
    main()
