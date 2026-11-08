#!/usr/bin/env python3
"""Exercise a running P9 pipeline with stationary synthetic room sensors.

Start P9 live with the dedicated smoke config: R=identity, T=0, gravity=-Z,
cloud offset=0, and no physical sensor drivers. Use a fresh isolated ROS domain.
Then run this script in the same domain. --raw requires imu_mode=raw_linear.
No estimator launch/stop, TF pose publisher, robot odometry or motion command is
sent. The only TF messages describe co-located synthetic IMU measurement axes.
This is a wiring/geometry/finite-output smoke test, not an accuracy benchmark.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--raw', action='store_true')
    parser.add_argument('--cloud-hz', type=float, default=10.)
    parser.add_argument('--imu-hz', type=float, default=200.)
    parser.add_argument('--point-stride', type=int, default=1)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 5:
        parser.error('--seconds must be at least five wall seconds')
    if any(not math.isfinite(v) or v <= 0 for v in (args.cloud_hz, args.imu_hz)):
        parser.error('sensor rates must be finite and positive')
    if args.point_stride < 1 or args.point_stride > 16:
        parser.error('--point-stride must be between 1 and 16')
    if args.raw and args.imu_hz < 100:
        parser.error('raw mode needs --imu-hz >= 100 for <= 20 ms acceleration brackets')
    if args.output.exists():
        parser.error('output exists; refusing overwrite')
    try:
        domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    except ValueError:
        parser.error('ROS_DOMAIN_ID must be an integer')
    if not 1 <= domain <= 232:
        parser.error('set a dedicated nonzero ROS_DOMAIN_ID shared only with the smoke P9 launch')
    if not (os.environ.get('ROS_LOCALHOST_ONLY') == '1'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'):
        parser.error('require ROS_LOCALHOST_ONLY=1 or ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header, String
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from nav_msgs.msg import Odometry, Path as PathMsg
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from go2_p9.sensor_core import POINT, POINT_STEP, FIELD_SPECS, stamp_nanoseconds
    from go2_p9.mapping_core import decode_cloud

    def qos(depth=100, transient=False):
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)

    # Five nonparallel room surfaces; fixed physical points, no synthetic motion.
    points = []
    for side in (-4., 4.):
        points.extend((side, y, z) for y in np.linspace(-3, 3, 49) for z in np.linspace(-1, 2.5, 29))
    for side in (-3., 3.):
        points.extend((x, side, z) for x in np.linspace(-4, 4, 65) for z in np.linspace(-1, 2.5, 29))
    points.extend((x, y, -1.) for x in np.linspace(-4, 4, 65) for y in np.linspace(-3, 3, 49))
    points = points[::args.point_stride]
    encoded = bytearray(len(points)*POINT_STEP)
    for index, (x, y, z) in enumerate(points):
        POINT.pack_into(encoded, index*POINT_STEP, x, y, z, 50., index % 16,
                        .095 * index / (len(points)-1))
    encoded = bytes(encoded)
    payload_hash = hashlib.sha256(encoded).hexdigest()
    fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in FIELD_SPECS]

    rclpy.init()
    node = Node('p9_synthetic_smoke')
    counts = Counter()
    failures = []
    stamps = {'points': set(), 'imu': set(), 'odom': set(), 'registered': set()}
    sent_cloud, sent_imu = set(), set()
    last_stamp = {}
    mapping_statuses, input_levels = [], []
    latest_map, latest_path = {}, {}
    starting = time.monotonic()

    def fail(message):
        if message not in failures:
            failures.append(message)

    def ordered(kind, ns):
        if kind in last_stamp and ns <= last_stamp[kind]:
            fail(kind + ' duplicate/backward timestamp')
        last_stamp[kind] = ns

    def cloud_array(msg):
        return decode_cloud(msg.data, [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                            msg.width, msg.height, msg.point_step, msg.row_step, msg.is_bigendian)

    def receive_points(msg):
        counts['points'] += 1
        ns = stamp_nanoseconds(msg.header.stamp)
        ordered('points', ns)
        stamps['points'].add(ns)
        if msg.header.frame_id != 'utlidar_lidar' or ns not in sent_cloud:
            fail('adapter cloud frame or original timestamp differs')
        if msg.width*msg.height != len(points) or hashlib.sha256(bytes(msg.data)).hexdigest() != payload_hash:
            fail('adapter changed static room XYZ/intensity/ring/point times or layout')

    def receive_imu(msg):
        counts['imu'] += 1
        ns = stamp_nanoseconds(msg.header.stamp)
        ordered('imu', ns)
        stamps['imu'].add(ns)
        if msg.header.frame_id != 'camera_imu_optical_frame' or ns not in sent_imu:
            fail('adapter IMU frame or source gyro timestamp differs')
        expected = [0., 0., 0., 0., 0., 9.81]
        actual = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        if not np.allclose(actual, expected, rtol=0., atol=1e-12):
            fail('adapter changed gyro or stationary acceleration')

    def receive_odom(msg):
        counts['odom'] += 1
        ns = stamp_nanoseconds(msg.header.stamp)
        ordered('odom', ns)
        stamps['odom'].add(ns)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'aft_mapped':
            fail('native odometry frame/origin mismatch')
        if not np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]).all():
            fail('native odometry is nonfinite')
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
            fail('native odometry quaternion is not unit')

    def receive_registered(msg):
        counts['registered'] += 1
        ns = stamp_nanoseconds(msg.header.stamp)
        ordered('registered', ns)
        stamps['registered'].add(ns)
        if msg.header.frame_id != 'camera_init':
            fail('registered scan is not in native world')
        try:
            xyz = cloud_array(msg)
            if not len(xyz) or not np.isfinite(xyz).all():
                fail('registered scan empty/nonfinite')
        except Exception as exc:
            fail('registered scan decode: ' + str(exc))

    def receive_map(msg):
        counts['map'] += 1
        try:
            array = cloud_array(msg)
            latest_map.update(points=len(array), stamp_ns=stamp_nanoseconds(msg.header.stamp), frame=msg.header.frame_id)
            if msg.header.frame_id != 'camera_init' or not len(array) or not np.isfinite(array).all():
                fail('accumulated map wrong frame/empty/nonfinite')
        except Exception as exc:
            fail('map decode: ' + str(exc))

    def receive_path(msg):
        counts['path'] += 1
        latest_path.update(poses=len(msg.poses), frame=msg.header.frame_id,
                           stamp_ns=stamp_nanoseconds(msg.header.stamp))
        if msg.header.frame_id != 'camera_init' or not msg.poses:
            fail('path wrong frame or empty')
        values = [stamp_nanoseconds(p.header.stamp) for p in msg.poses]
        if any(b <= a for a, b in zip(values, values[1:])):
            fail('path contains duplicate/backward timestamps')
        for pose in msg.poses:
            p, q = pose.pose.position, pose.pose.orientation
            if pose.header.frame_id != 'camera_init' or not np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]).all():
                fail('path pose wrong frame/nonfinite')

    def receive_mapping_status(msg):
        try:
            value = json.loads(msg.data)
            mapping_statuses.append(value.get('status'))
            if value.get('fault') is not None:
                fail('mapping latched fault: ' + str(value['fault']))
        except (ValueError, TypeError) as exc:
            fail('mapping status invalid JSON: ' + str(exc))

    def receive_diagnostics(msg):
        for status in msg.status:
            if status.name == 'p9/input':
                level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
                input_levels.append(dict(elapsed=time.monotonic()-starting, level=level, message=status.message))

    for typ, topic, callback, profile in [
        (PointCloud2, '/p9/points', receive_points, qos()), (Imu, '/p9/imu', receive_imu, qos(4000)),
        (Odometry, '/pointlio/odom', receive_odom, qos()),
        (PointCloud2, '/pointlio/registered_scan', receive_registered, qos()),
        (PointCloud2, '/p9/map', receive_map, qos(1, True)),
        (PathMsg, '/p9/path', receive_path, qos(1, True)),
        (String, '/p9/mapping_status', receive_mapping_status, qos(1, True)),
        (DiagnosticArray, '/p9/input_diagnostics', receive_diagnostics, qos(10))]:
        node.create_subscription(typ, topic, callback, profile)
    cloud_pub = node.create_publisher(PointCloud2, '/utlidar/cloud', qos())
    if args.raw:
        gyro_pub = node.create_publisher(Imu, '/camera/camera/gyro/sample', qos(4000))
        accel_pub = node.create_publisher(Imu, '/camera/camera/accel/sample', qos(4000))
        tf_pub = node.create_publisher(TFMessage, '/tf_static', qos(1, True))
        transforms = []
        for child in ('camera_gyro_optical_frame', 'camera_accel_optical_frame', 'camera_imu_optical_frame'):
            tr = TransformStamped()
            tr.header.frame_id, tr.child_frame_id = 'smoke_camera_axes', child
            tr.transform.rotation.w = 1.
            transforms.append(tr)
        tf_pub.publish(TFMessage(transforms=transforms))
    else:
        imu_pub = node.create_publisher(Imu, '/camera/camera/imu', qos(4000))

    def message(ns, frame):
        sec, nano = divmod(ns, 10**9)
        msg = Imu(header=Header(stamp=Time(sec=sec, nanosec=nano), frame_id=frame))
        msg.orientation_covariance[0] = -1.
        msg.angular_velocity_covariance = [.01, 0., 0., 0., .01, 0., 0., 0., .01]
        msg.linear_acceleration_covariance = [.01, 0., 0., 0., .01, 0., 0., 0., .01]
        msg.linear_acceleration.z = 9.81
        return msg

    send_started = None
    next_imu = next_cloud = time.monotonic() + 2.0
    deadline = next_imu + args.seconds
    tick, last_input_ns = 0, None
    try:
        while time.monotonic() < deadline + 1.0:
            now = time.monotonic()
            if now >= next_imu and now < deadline:
                if send_started is None:
                    send_started = now
                ns = node.get_clock().now().nanoseconds
                if last_input_ns is not None and ns <= last_input_ns:
                    fail('smoke source ROS time is not advancing; use live smoke config')
                    break
                last_input_ns = ns
                sent_imu.add(ns)
                if args.raw:
                    g = message(ns, 'camera_gyro_optical_frame')
                    g.linear_acceleration_covariance[0] = -1.
                    gyro_pub.publish(g)
                    if tick % 2 == 0:
                        a = message(ns, 'camera_accel_optical_frame')
                        a.angular_velocity_covariance[0] = -1.
                        accel_pub.publish(a)
                else:
                    imu_pub.publish(message(ns, 'camera_imu_optical_frame'))
                counts['input_gyro_or_merged'] += 1
                tick += 1
                next_imu = now + 1. / args.imu_hz  # no catchup bursts or invented times
            if now >= next_cloud and now < deadline:
                # Scan acquisition started 100 ms ago and ended 5 ms ago.
                # IMU samples already cover it; no future stamped scan is needed.
                ns = node.get_clock().now().nanoseconds - 100_000_000
                sec, nano = divmod(ns, 10**9)
                sent_cloud.add(ns)
                cloud_pub.publish(PointCloud2(header=Header(stamp=Time(sec=sec, nanosec=nano), frame_id='utlidar_lidar'),
                    height=1, width=len(points), fields=fields, is_bigendian=False,
                    point_step=POINT_STEP, row_step=len(points)*POINT_STEP, data=encoded, is_dense=True))
                counts['input_cloud'] += 1
                next_cloud = now + 1. / args.cloud_hz
            rclpy.spin_once(node, timeout_sec=.001)
    except (KeyboardInterrupt, Exception) as exc:
        fail(type(exc).__name__ + ': ' + str(exc))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    for kind in ('points', 'imu', 'odom', 'registered', 'map', 'path'):
        if not counts[kind]:
            fail('no ' + kind + ' received')
    if counts['points'] < 10 or counts['imu'] < 100:
        fail('too few adapter outputs to establish sustained streaming')
    only_odom, only_scan = stamps['odom']-stamps['registered'], stamps['registered']-stamps['odom']
    if only_odom or only_scan:
        fail('native odometry/registered scans are not exact timestamp pairs')
    if latest_path.get('poses') != counts['odom']:
        fail('final native path count differs from observed exact odometry pairs')
    if stamps['odom'] and (latest_map.get('stamp_ns') != max(stamps['odom'])
                          or latest_path.get('stamp_ns') != max(stamps['odom'])):
        fail('final map/path stamps do not cover the last observed exact pair')
    if 'OK' not in mapping_statuses:
        fail('mapping never reported OK')
    if not any(v['level'] == 0 and v['elapsed'] >= 3 for v in input_levels):
        fail('adapter never reported valid input contracts after warmup')
    # Stop/drain itself can legitimately produce a stale-input diagnostic.
    active_errors = [v for v in input_levels if 4 <= v['elapsed'] <= args.seconds+1 and v['level'] >= 2]
    if active_errors:
        fail('adapter reported ERROR during steady synthetic input')
    report = dict(passed=not failures, failures=failures, kind='synthetic_stationary_pipeline_smoke',
                  absolute_accuracy_test=False, no_real_robot_motion=True, raw=args.raw,
                  domain=domain, requested_input_seconds=args.seconds, counts=dict(counts),
                  requested_cloud_hz=args.cloud_hz, requested_imu_hz=args.imu_hz,
                  point_stride=args.point_stride,
                  exact_pairs=len(stamps['odom'] & stamps['registered']),
                  unmatched_odom_ns=sorted(only_odom), unmatched_registered_ns=sorted(only_scan),
                  latest_map=latest_map, latest_path=latest_path,
                  mapping_statuses=mapping_statuses, input_diagnostics=input_levels,
                  room_input_points=len(points), input_payload_sha256=payload_hash,
                  sensor_contract='R=identity,T=0,cloud_offset=0,gravity=-Z,acceleration=+Z 9.81; native IMU origin')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'], counts=report['counts'], failures=failures, output=str(args.output)), indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
