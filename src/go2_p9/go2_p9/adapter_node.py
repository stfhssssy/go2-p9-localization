"""ROS 2 P9 LiDAR/camera-IMU adapter; no TF or motion publishers."""
from collections import Counter
import json
import math
from pathlib import Path
import struct
import time

from .sensor_core import (AdapterOptions, FIELD_SPECS, POINT_STEP, NS_PER_SEC,
                          IdentityFrameGraph, InvalidSensorData, RawImuInterpolator,
                          check_live_age, clock_relation, prepare_cloud, prepare_imu,
                          stamp_nanoseconds, validate_merged_imu)


def write_diagnostic_report(path, details, level, message, stamp_ns):
    """Persist the same status/values as DiagnosticArray without partial JSON."""
    if path is None:
        return
    numeric_level = level[0] if isinstance(level, (bytes, bytearray)) else int(level)
    document = dict(details, status_level=numeric_level, status_message=message,
                    diagnostic_stamp_ns=int(stamp_ns))
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(document, indent=2, allow_nan=False, ensure_ascii=False) + '\n')
    temporary.replace(path)


def main(args=None):
    import rclpy
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage

    class SensorAdapter(Node):
        def __init__(self):
            super().__init__('p9_sensor_adapter')
            defaults = dict(
                input_cloud_topic='/utlidar/cloud', input_imu_topic='/camera/camera/imu',
                output_cloud_topic='/p9/points', output_imu_topic='/p9/imu',
                gyro_topic='/camera/camera/gyro/sample', accel_topic='/camera/camera/accel/sample',
                mode='live', imu_mode='merged_linear', cloud_frame='utlidar_lidar',
                imu_frame='camera_imu_optical_frame', clock_offset_sec=0.0, common_clock_offset_sec=0.0,
                accel_offset=[0.0, 0.0, 0.0], gyro_bias=[0.0, 0.0, 0.0],
                point_time_field='time', point_time_scale=1.0, min_range=0.2, max_range=30.0,
                max_scan_duration_sec=0.2, raw_gyro_frame='camera_gyro_optical_frame',
                raw_accel_frame='camera_accel_optical_frame', max_accel_gap_sec=0.03,
                raw_queue_size=4000, raw_buffer_sec=2.0, raw_pending_timeout_sec=2.0,
                cloud_queue_size=100, imu_queue_size=4000, max_message_age_sec=1.0,
                future_tolerance_sec=0.1, clock_tolerance_sec=0.5,
                input_timeout_sec=2.0, diagnostic_output_path='')
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.cfg = {key: self.get_parameter(key).value for key in defaults}
            c = self.cfg
            if c['mode'] not in ('live', 'replay'):
                raise ValueError('mode must be live or replay')
            if c['imu_mode'] not in ('merged_linear', 'raw_linear', 'native'):
                raise ValueError('imu_mode must be merged_linear, raw_linear or native')
            for name in ('max_message_age_sec', 'future_tolerance_sec',
                         'clock_tolerance_sec', 'input_timeout_sec'):
                if not math.isfinite(c[name]) or c[name] <= 0:
                    raise ValueError(name + ' must be finite and positive')
            for name in ('cloud_queue_size', 'imu_queue_size', 'raw_queue_size'):
                if not isinstance(c[name], int) or isinstance(c[name], bool) or c[name] < 2:
                    raise ValueError(name + ' must be an integer >= 2')
            topics = [c['input_cloud_topic']]
            topics += ([c['input_imu_topic']] if c['imu_mode'] != 'raw_linear'
                       else [c['gyro_topic'], c['accel_topic']])
            outputs = [c['output_cloud_topic'], c['output_imu_topic'], '/p9/input_diagnostics']
            if any(not topic for topic in topics+outputs) or len(set(topics+outputs)) != len(topics+outputs):
                raise ValueError('active sensor input and output topics must be nonempty and distinct')
            self.options = AdapterOptions(**{key: c[key] for key in AdapterOptions.__dataclass_fields__})
            self.diagnostic_output = None
            if not isinstance(c['diagnostic_output_path'], str):
                raise ValueError('diagnostic_output_path must be a path string or empty')
            if c['diagnostic_output_path']:
                self.diagnostic_output = Path(c['diagnostic_output_path']).expanduser().resolve()
                if self.diagnostic_output.exists():
                    raise FileExistsError('Refusing to overwrite earlier input diagnostics: ' + str(self.diagnostic_output))
                self.diagnostic_output.parent.mkdir(parents=True, exist_ok=True)
            self.counts = Counter()
            self.first_stamp, self.last_stamp, self.received_steady = {}, {}, {}
            self.last_cloud_raw = self.last_imu = self.cloud_end_ns = None
            self.last_imu_raw = None
            self.last_error = None
            self.started_steady = time.monotonic()
            self.graph = IdentityFrameGraph()
            self.merger = RawImuInterpolator(
                imu_frame=c['imu_frame'], gyro_frame=c['raw_gyro_frame'],
                accel_frame=c['raw_accel_frame'], frame_graph=self.graph,
                max_accel_gap_sec=c['max_accel_gap_sec'], queue_size=c['raw_queue_size'],
                buffer_sec=c['raw_buffer_sec'], pending_timeout_sec=c['raw_pending_timeout_sec'])
            self.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                           for name, offset, kind in FIELD_SPECS]

            def qos(depth, reliable=False, transient=False):
                return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                    reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)

            self.cloud_pub = self.create_publisher(PointCloud2, c['output_cloud_topic'], qos(c['cloud_queue_size'], True))
            self.imu_pub = self.create_publisher(Imu, c['output_imu_topic'], qos(c['imu_queue_size'], True))
            self.diag_pub = self.create_publisher(DiagnosticArray, '/p9/input_diagnostics', qos(10, True))
            self.create_subscription(PointCloud2, c['input_cloud_topic'], self.cloud, qos(c['cloud_queue_size']))
            if c['imu_mode'] != 'raw_linear':
                self.create_subscription(Imu, c['input_imu_topic'], self.merged, qos(c['imu_queue_size']))
            else:
                self.create_subscription(TFMessage, '/tf_static', self.static_tf, qos(100, True, True))
                self.create_subscription(Imu, c['gyro_topic'], self.gyro, qos(c['imu_queue_size']))
                self.create_subscription(Imu, c['accel_topic'], self.accel, qos(c['imu_queue_size']))
            self.create_timer(1.0, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(
                f"P9 {c['mode']}/{c['imu_mode']}: cloud offset {self.options.cloud_offset_ns} ns; "
                f"IMU offset {self.options.imu_offset_ns} ns; explicit accel offset {c['accel_offset']}, gyro bias {c['gyro_bias']}. "
                'No inferred time offset, TF, or motion output.')
            if c['imu_mode'] == 'merged_linear':
                self.get_logger().warning('merged_linear requires RealSense unite_imu_method=2 (or an explicitly prepared linear IMU source); this node cannot infer driver interpolation from Imu messages.')
            self.report()

        def received(self, kind):
            self.counts[kind + '_received'] += 1
            self.received_steady[kind] = time.monotonic()

        def accepted(self, kind, stamp):
            self.first_stamp.setdefault(kind, stamp)
            self.last_stamp[kind] = stamp
            self.counts[kind + '_accepted'] += 1

        def rejected(self, kind, error):
            self.counts[kind + '_rejected'] += 1
            if 'stale' in str(error):
                self.counts[kind + '_stale_rejected'] += 1
            self.last_error = dict(stream=kind, reason=str(error),
                                   receipt_steady_sec=time.monotonic())
            count = self.counts[kind + '_rejected']
            if count & (count-1) == 0:
                self.get_logger().warning(f'{kind} rejected ({count}): {error}')

        def age_check(self, stamp):
            if self.cfg['mode'] == 'live':
                check_live_age(stamp, self.get_clock().now().nanoseconds,
                               self.cfg['max_message_age_sec'], self.cfg['future_tolerance_sec'])

        def cloud(self, msg):
            self.received('cloud')
            try:
                result = prepare_cloud(msg, self.last_cloud_raw, self.options)
                self.age_check(result.output_stamp_ns)
            except (InvalidSensorData, ValueError, TypeError, OverflowError, struct.error) as error:
                self.rejected('cloud', error)
                return
            output = PointCloud2(header=result.header, height=1, width=result.width,
                fields=self.fields, is_bigendian=False, point_step=POINT_STEP,
                row_step=result.width*POINT_STEP, data=result.data, is_dense=True)
            self.cloud_pub.publish(output)
            self.last_cloud_raw = result.input_stamp_ns
            self.accepted('cloud', result.output_stamp_ns)
            self.first_stamp.setdefault('raw_cloud', result.input_stamp_ns)
            self.last_stamp['raw_cloud'] = result.input_stamp_ns
            self.cloud_end_ns = result.output_stamp_ns + round(result.last_point_time_sec*NS_PER_SEC)
            self.counts['cloud_published'] += 1
            self.counts['points_input_accepted_clouds'] += result.input_points
            self.counts['points_published'] += result.width
            self.counts['points_nonfinite_dropped'] += result.dropped_coordinates
            self.counts['points_range_dropped'] += result.dropped_range
            self.counts['clouds_time_sorted'] += int(result.sorted_by_time)

        def publish_imu(self, msg):
            msg, input_stamp, stamp = prepare_imu(msg, self.last_imu_raw, self.options)
            # A raw gyro can spend time awaiting its right acceleration sample.
            # Recheck freshness at publication, not only at original receipt.
            self.age_check(stamp)
            self.imu_pub.publish(msg)
            self.last_imu = stamp
            self.last_imu_raw = input_stamp
            self.accepted('imu', stamp)
            self.counts['imu_published'] += 1

        def merged(self, msg):
            self.received('merged_imu')
            try:
                self.publish_imu(msg)
            except (InvalidSensorData, ValueError, TypeError, OverflowError) as error:
                self.rejected('merged_imu', error)

        def raw(self, kind, msg):
            self.received(kind)
            try:
                self.age_check(stamp_nanoseconds(msg.header.stamp) + self.options.imu_offset_ns)
                values = (self.merger.push_gyro if kind == 'gyro' else self.merger.push_accel)(msg, time.monotonic())
                self.accepted(kind, stamp_nanoseconds(msg.header.stamp))
            except (InvalidSensorData, ValueError, TypeError, OverflowError) as error:
                self.rejected(kind, error)
                return
            for output in values:
                try:
                    self.publish_imu(output)
                except (InvalidSensorData, ValueError, TypeError, OverflowError) as error:
                    # One stale pending item must not hide newer fresh items.
                    self.rejected('imu_output', error)

        def gyro(self, msg):
            self.raw('gyro', msg)

        def accel(self, msg):
            self.raw('accel', msg)

        def static_tf(self, msg):
            for item in msg.transforms:
                tr, q = item.transform.translation, item.transform.rotation
                try:
                    self.graph.add(item.header.frame_id, item.child_frame_id,
                                   (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
                except InvalidSensorData as error:
                    self.rejected('static_tf', error)

        def report(self):
            now = time.monotonic()
            self.merger.expire(now)
            level, reasons = DiagnosticStatus.OK, []
            required = ['cloud', 'merged_imu'] if self.cfg['imu_mode'] != 'raw_linear' else ['cloud', 'gyro', 'accel']
            missing = [k for k in required if k not in self.received_steady]
            stale = [k for k in required if now-self.received_steady.get(k, self.started_steady) > self.cfg['input_timeout_sec']]
            if missing:
                level = DiagnosticStatus.WARN
                reasons.append('waiting for: ' + ', '.join(missing))
            if stale:
                level = DiagnosticStatus.ERROR
                reasons.append('input receipt timeout: ' + ', '.join(stale))
            relation = clock_relation(self.cloud_end_ns, self.last_imu, self.cfg['clock_tolerance_sec'])
            if relation['mismatch']:
                level = DiagnosticStatus.ERROR
                reasons.append('cloud/IMU header timelines are separated; check source clocks, explicit cloud offset, or stalled stream')
            if self.cfg['imu_mode'] == 'raw_linear':
                try:
                    self.merger.frames_ready()
                except InvalidSensorData as error:
                    level = DiagnosticStatus.ERROR
                    reasons.append(str(error))
            if not self.counts['cloud_published'] or not self.counts['imu_published']:
                level = max(level, DiagnosticStatus.WARN)
                reasons.append('both outputs have not yet produced valid data')
            if self.last_error and now-self.last_error['receipt_steady_sec'] <= self.cfg['input_timeout_sec']:
                level = DiagnosticStatus.ERROR
                reasons.append(self.last_error['stream'] + ': ' + self.last_error['reason'])
            # Invalid messages may continue to arrive while no accepted output is
            # produced. Diagnose this separately from transport receipt staleness.
            spans = {key: dict(first_ns=self.first_stamp[key], last_ns=value,
                               span_sec=(value-self.first_stamp[key])/NS_PER_SEC)
                     for key, value in self.last_stamp.items()}
            details = dict(mode=self.cfg['mode'], imu_mode=self.cfg['imu_mode'],
                counts=dict(self.counts), raw_counts=dict(self.merger.counts),
                time_spans=spans, clock_relation=relation, last_error=self.last_error,
                input_receipt_age_sec={k: now-v for k, v in self.received_steady.items()},
                pending_gyro=len(self.merger.gyros), buffered_accel=len(self.merger.accels),
                max_accel_bracket_sec=self.merger.max_used_bracket_ns/NS_PER_SEC,
                cloud_header_offset_ns=self.options.cloud_offset_ns, imu_header_offset_ns=self.options.imu_offset_ns,
                native_coordinates_preserved=True, per_point_time_unit='seconds',
                acceleration_offset_native=self.cfg['accel_offset'], gyro_bias_native=self.cfg['gyro_bias'],
                correction_uncertainty_modelled=False,
                required_pointlio_timestamp_unit=0, tf_published=False, motion_published=False,
                raw_covariance_policy='preceding/exact measured accel covariance; interpolation correlation not estimated',
                merged_driver_linear_interpolation_verified_by_adapter=False,
                configured_frames=dict(cloud=self.cfg['cloud_frame'], imu=self.cfg['imu_frame']),
                time_policy='explicit common epoch shift plus cloud-only shift; no inferred delay or drift')
            status = DiagnosticStatus(level=level, name='p9/input', hardware_id='go2_lidar_camera_imu',
                message='; '.join(reasons) if reasons else 'valid sensor input contracts; not a localization accuracy verdict',
                values=[KeyValue(key=k, value=json.dumps(v, allow_nan=False, ensure_ascii=False)) for k, v in details.items()])
            diagnostic = DiagnosticArray()
            report_time = self.get_clock().now()
            diagnostic.header.stamp = report_time.to_msg()
            diagnostic.status = [status]
            write_diagnostic_report(self.diagnostic_output, details, level, status.message,
                                    report_time.nanoseconds)
            if rclpy.ok():
                self.diag_pub.publish(diagnostic)

    rclpy.init(args=args)
    node = None
    try:
        node = SensorAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.report()
            finally:
                node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
