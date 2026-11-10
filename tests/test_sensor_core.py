"""Sensor contracts without ROS, a network, or access to recorded data."""
import copy
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/go2_p9'))
from go2_p9.sensor_core import (  # noqa: E402
    AccelSample, AdapterOptions, FIELD_SPECS, IdentityFrameGraph, InvalidSensorData,
    NS_PER_SEC, POINT, POINT_STEP, RawImuInterpolator, check_live_age,
    clock_relation, interpolate_acceleration, prepare_cloud, prepare_imu, validate_merged_imu)
from go2_p9.adapter_node import write_diagnostic_report  # noqa: E402
from go2_p9.cloud_codec import CloudOptions, normalize_cloud, normalize_cloud_numpy  # noqa: E402


EPOCH = 1_793_245_181_931_124_536


def stamp(ns):
    sec, nanosec = divmod(ns, NS_PER_SEC)
    return NS(sec=sec, nanosec=nanosec)


def imu(ns=EPOCH, frame='camera_imu_optical_frame', acceleration=(.3, -9.65, .7)):
    return NS(header=NS(frame_id=frame, stamp=stamp(ns)),
              angular_velocity=NS(x=.11, y=-.22, z=.33),
              linear_acceleration=NS(**dict(zip('xyz', acceleration))),
              orientation=NS(x=0., y=0., z=0., w=0.),
              orientation_covariance=[-1.] + [0.]*8,
              angular_velocity_covariance=[.02, 0., 0., 0., .02, 0., 0., 0., .02],
              linear_acceleration_covariance=[.01, 0., 0., 0., .01, 0., 0., 0., .01])


def cloud(points=None, endian='<', padding=11, ns=EPOCH, time_field='time'):
    points = points or [(1., 2., -3., 42., 7, .02), (2., 0., 0., 43., 8, .01)]
    fields = [(time_field, 0, 8, 'd'), ('ring', 8, 2, 'B'), ('x', 16, 8, 'd'),
              ('z', 24, 8, 'd'), ('intensity', 32, 7, 'f'), ('y', 40, 8, 'd')]
    stride = 48 + padding
    data = bytearray(stride*len(points))
    for index, point in enumerate(points):
        values = dict(zip(('x', 'y', 'z', 'intensity', 'ring', time_field), point))
        for name, offset, _, fmt in fields:
            struct.pack_into(endian+fmt, data, index*stride+offset, values[name])
    return NS(header=NS(frame_id='utlidar_lidar', stamp=stamp(ns)), width=1,
              height=len(points), fields=[NS(name=n, offset=o, datatype=d, count=1)
                  for n, o, d, _ in fields], point_step=48, row_step=stride,
              is_bigendian=endian == '>', data=bytes(data))


def graph():
    result = IdentityFrameGraph()
    # Common optical transform may be nonidentity in the body, but relative
    # accel/gyro/merged transformations must all be identity.
    result.add('camera_link', 'optical', (.1, .2, .3), (-.5, .5, -.5, .5))
    for frame in ('camera_accel_optical_frame', 'camera_gyro_optical_frame', 'camera_imu_optical_frame'):
        result.add('optical', frame, (0., 0., 0.), (0., 0., 0., 1.))
    return result


def gyro(ns, **kwargs):
    result = imu(ns, 'camera_gyro_optical_frame', **kwargs)
    result.linear_acceleration_covariance[0] = -1.
    return result


def accel(ns, values=(0., 0., 9.81)):
    result = imu(ns, 'camera_accel_optical_frame', acceleration=values)
    result.angular_velocity_covariance[0] = -1.
    return result


class SensorCoreTests(unittest.TestCase):
    def assert_codec_matches_oracle(self, msg, options=None):
        options = options or CloudOptions()
        scalar = normalize_cloud(msg, options)
        vector = normalize_cloud_numpy(msg, options)
        self.assertEqual(vector.dropped_coordinates, scalar.dropped_coordinates)
        self.assertEqual(vector.sorted_by_time, scalar.sorted_by_time)
        self.assertEqual(vector.points.tolist(), [list(p) for p in scalar.points])
        scalar_bytes = b''.join(POINT.pack(*p) for p in scalar.points)
        vector_bytes = b''.join(POINT.pack(*p[:4], int(p[4]), p[5]) for p in vector.points)
        self.assertEqual(vector_bytes, scalar_bytes)

    def test_numpy_oracle_multicolumn_padded_endian_stable_optional(self):
        rows = [(2., -0., 3., 100., 2, .03), (1., 1., 1., 50., 1, .01),
                (4., 4., 4., 80., 3, .01), (float('nan'), 2., 3., float('nan'), 5, .02),
                (5., 5., 5., 20., 7, .04), (6., 6., 6., 30., 8, 0.)]
        for endian in ('<', '>'):
            msg = cloud(rows, endian=endian, padding=0)
            packed = msg.data
            msg.width, msg.height, msg.row_step = 3, 2, 3*48+17
            msg.data = packed[:3*48] + b'\xff'*17 + packed[3*48:]
            self.assert_codec_matches_oracle(msg)
            actual = prepare_cloud(msg)
            oracle = normalize_cloud(msg)
            self.assertEqual(actual.data, b''.join(POINT.pack(*p) for p in oracle.points))
            for remove in [('ring',), ('intensity',), ('ring', 'intensity')]:
                subset = copy.deepcopy(msg)
                subset.fields = [f for f in subset.fields if f.name not in remove]
                self.assert_codec_matches_oracle(subset)

    def test_numpy_oracle_all_scalar_datatypes_and_time_scaling(self):
        formats = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}
        for datatype, fmt in formats.items():
            for endian in ('<', '>'):
                fields = [NS(name=n, offset=i*8, datatype=datatype, count=1)
                          for i, n in enumerate(('x', 'y', 'z', 'intensity', 'ring', 'ticks'))]
                values = [(2, 3, 4, 5, 6, 20), (7, 8, 9, 10, 11, 10)]
                data = bytearray(96)
                for i, row in enumerate(values):
                    for j, value in enumerate(row):
                        struct.pack_into(endian+fmt, data, i*48+j*8, value)
                msg = NS(header=cloud().header, width=2, height=1, point_step=48,
                         row_step=96, fields=fields, data=data, is_bigendian=endian == '>')
                self.assert_codec_matches_oracle(msg, CloudOptions(time_field='ticks', point_time_scale=.001))
        absolute = cloud([(1., 2., 3., 5., 1, 1.+.01), (2., 3., 4., 6., 2, 1.+.02)], ns=NS_PER_SEC)
        self.assert_codec_matches_oracle(absolute, CloudOptions(point_time_is_absolute=True))

    def test_numpy_oracle_rejection_on_invalid_coordinates_times_and_metadata(self):
        invalid = []
        for t in (-.01, .21, float('nan'), float('inf')):
            invalid.append(cloud([(1., 2., 3., 4., 1, .01), (float('nan'), 2., 3., float('nan'), 2, t)]))
        for intensity in (float('nan'), float('inf')):
            invalid.append(cloud([(1., 2., 3., intensity, 1, .01), (2., 3., 4., 5., 2, .02)]))
        # Noninteger or out-of-range ring in a floating-point field.
        for ring in (.5, -1., 65536., float('inf')):
            msg = cloud()
            next(f for f in msg.fields if f.name == 'ring').datatype = 8
            msg.data = bytearray(msg.data)
            struct.pack_into('<d', msg.data, 8, ring)
            invalid.append(msg)
        for mutation in ('duplicate', 'missing', 'count', 'offset', 'short', 'stride'):
            msg = cloud()
            if mutation == 'duplicate': msg.fields.append(copy.deepcopy(msg.fields[0]))
            if mutation == 'missing': msg.fields = [f for f in msg.fields if f.name != 'time']
            if mutation == 'count': msg.fields[0].count = 2
            if mutation == 'offset': msg.fields[0].offset = 47
            if mutation == 'short': msg.data = msg.data[:10]
            if mutation == 'stride': msg.row_step = 1
            invalid.append(msg)
        for msg in invalid:
            for implementation in (normalize_cloud, normalize_cloud_numpy):
                with self.assertRaises(InvalidSensorData):
                    implementation(msg)

    def test_numpy_prepare_matches_scalar_bytes_range_boundaries(self):
        rng = np.random.default_rng(11)
        points = []
        for limit in (.2, 30.):
            for _ in range(100):
                xyz = rng.normal(size=3)
                xyz *= limit / math.hypot(*xyz)
                for scale in (np.nextafter(1., 0.), 1., np.nextafter(1., 2.)):
                    points.append((*tuple(xyz * scale), 50., 3, .1 * len(points) / 1000))
        points += [(float('nan'), 1., 2., float('nan'), 0, .15)]
        msg = cloud(points)
        scalar = normalize_cloud(msg)
        keep = [p for p in scalar.points if .2 <= math.hypot(*p[:3]) <= 30.]
        result = prepare_cloud(msg)
        self.assertEqual(result.data, b''.join(POINT.pack(*p) for p in keep))
        self.assertEqual(result.dropped_range, len(scalar.points)-len(keep))
        self.assertEqual(result.dropped_coordinates, 1)

    def test_cloud_padding_endian_named_fields_and_canonical_layout(self):
        for endian in ('<', '>'):
            msg = cloud(endian=endian)
            before = copy.deepcopy(msg)
            out = prepare_cloud(msg)
            a, b = [POINT.unpack_from(out.data, i*POINT_STEP) for i in range(2)]
            self.assertEqual(a[:5], (2., 0., 0., 43., 8))
            self.assertEqual(b[:5], (1., 2., -3., 42., 7))
            self.assertAlmostEqual(a[5], .01)
            self.assertTrue(out.sorted_by_time)
            self.assertEqual(msg, before)
            self.assertEqual(out.output_stamp_ns, EPOCH)
            self.assertEqual(out.header.frame_id, msg.header.frame_id)
        self.assertEqual(POINT_STEP, 32)
        self.assertEqual(POINT.size, 32)
        self.assertIn(('time', 24, 7), FIELD_SPECS)

    def test_only_explicit_cloud_offset_and_time_unit_conversion(self):
        msg = cloud([(1., 0., 0., 1., 2, 20.), (2., 0., 0., 3., 4, 10.)], time_field='t_ms')
        options = AdapterOptions(clock_offset_sec=-1.123456789,
                                 point_time_field='t_ms', point_time_scale=.001)
        out = prepare_cloud(msg, options=options)
        self.assertEqual(out.output_stamp_ns, EPOCH-1_123_456_789)
        self.assertAlmostEqual(POINT.unpack_from(out.data)[5], .01)
        self.assertEqual(msg.header.stamp, stamp(EPOCH))
        for offset in (-3., 3.):
            edge = cloud(ns=0 if offset < 0 else 2_147_483_647*NS_PER_SEC)
            with self.assertRaises(InvalidSensorData):
                prepare_cloud(edge, options=AdapterOptions(clock_offset_sec=offset))
        with self.assertRaises(ValueError):
            AdapterOptions(clock_offset_sec=1e308)

    def test_cloud_timing_and_monotonicity_fail_closed(self):
        msg = cloud()
        msg.fields = [f for f in msg.fields if f.name != 'time']
        with self.assertRaisesRegex(InvalidSensorData, 'missing required'):
            prepare_cloud(msg)
        for pair in [(0., 0.), (-.01, .02), (.01, float('nan')), (.01, .3)]:
            with self.assertRaises(InvalidSensorData):
                prepare_cloud(cloud([(1., 0., 0., 1., 0, pair[0]), (1., 0., 0., 1., 0, pair[1])]))
        for previous in (EPOCH, EPOCH+1):
            with self.assertRaisesRegex(InvalidSensorData, 'duplicate or backward'):
                prepare_cloud(cloud(), previous)
        self.assertEqual(prepare_cloud(cloud(), EPOCH-1).input_stamp_ns, EPOCH)

    def test_range_counts_and_bad_coordinates(self):
        points = [(0., 0., 0., 1., 0, 0.), (.2, 0., 0., 1., 0, .01),
                  (30., 0., 0., 1., 0, .02), (31., 0., 0., 1., 0, .03),
                  (float('nan'), 0., 0., 1., 0, .04)]
        out = prepare_cloud(cloud(points))
        self.assertEqual((out.input_points, out.width, out.dropped_range, out.dropped_coordinates), (5, 2, 2, 1))
        # A no-return point still cannot hide invalid timing.
        points[-1] = (float('nan'), 0., 0., 1., 0, -.01)
        with self.assertRaises(InvalidSensorData):
            prepare_cloud(cloud(points))

    def test_merged_payload_stays_exact_and_frames_cannot_be_relabelled(self):
        msg, original = imu(), imu()
        self.assertEqual(validate_merged_imu(msg), EPOCH)
        self.assertEqual(msg, original)
        for previous in (EPOCH, EPOCH+1):
            with self.assertRaises(InvalidSensorData):
                validate_merged_imu(msg, previous)
        msg.header.frame_id = 'camera_imu_frame'
        with self.assertRaises(InvalidSensorData):
            validate_merged_imu(msg)
        msg = cloud()
        msg.header.frame_id = 'odom'
        with self.assertRaises(InvalidSensorData):
            prepare_cloud(msg)
        for attribute in ('angular_velocity', 'linear_acceleration', 'orientation'):
            msg = imu()
            getattr(msg, attribute).x = float('nan')
            with self.assertRaises(InvalidSensorData):
                validate_merged_imu(msg)

    def test_native_corrections_use_declared_sign_and_preserve_other_fields(self):
        original = imu(frame='utlidar_imu', acceleration=(1., -2., 9.81))
        before = copy.deepcopy(original)
        options = AdapterOptions(imu_frame='utlidar_imu', accel_offset=(.5, -1., -19.),
                                 gyro_bias=(.01, -.02, .03))
        output, raw_stamp, output_stamp = prepare_imu(original, options=options)
        self.assertIsNot(output, original)
        self.assertEqual(original, before)
        self.assertEqual((raw_stamp, output_stamp), (EPOCH, EPOCH))
        np.testing.assert_allclose([getattr(output.linear_acceleration, a) for a in 'xyz'],
                                   [1.5, -3., -9.19], rtol=0., atol=1e-14)
        np.testing.assert_allclose([getattr(output.angular_velocity, a) for a in 'xyz'],
                                   [.10, -.20, .30], rtol=0., atol=1e-14)
        for field in ('header', 'orientation', 'orientation_covariance',
                      'linear_acceleration_covariance', 'angular_velocity_covariance'):
            self.assertEqual(getattr(output, field), getattr(before, field))
        # Reusing the original message cannot apply corrections twice.
        self.assertEqual(prepare_imu(original, options=options)[0], output)
        unchanged = prepare_imu(original, options=AdapterOptions(imu_frame='utlidar_imu'))
        self.assertIs(unchanged[0], original)

    def test_common_clock_preserves_relative_time_and_cloud_point_offsets(self):
        raw_imu = imu(EPOCH+7, frame='utlidar_imu')
        raw_cloud = cloud(ns=EPOCH)
        before = copy.deepcopy((raw_cloud, raw_imu))
        for common in (2245.232644025, -2.123456789):
            options = AdapterOptions(imu_frame='utlidar_imu', common_clock_offset_sec=common)
            prepared_cloud = prepare_cloud(raw_cloud, options=options)
            prepared_imu, received, sent = prepare_imu(raw_imu, options=options)
            shift = round(common*NS_PER_SEC)
            self.assertEqual(prepared_cloud.output_stamp_ns, EPOCH+shift)
            self.assertEqual((received, sent), (EPOCH+7, EPOCH+7+shift))
            self.assertEqual(sent-prepared_cloud.output_stamp_ns, 7)
            self.assertEqual(prepared_cloud.data, prepare_cloud(raw_cloud).data)
            self.assertEqual(prepared_imu.header.stamp, stamp(sent))
            self.assertEqual(prepared_imu.angular_velocity, raw_imu.angular_velocity)
            self.assertEqual(prepared_imu.linear_acceleration, raw_imu.linear_acceleration)
            # Monotonicity uses native input stamps, not the shifted output epoch.
            prepare_imu(imu(EPOCH+8, frame='utlidar_imu'), received, options)
            with self.assertRaisesRegex(InvalidSensorData, 'duplicate or backward'):
                prepare_imu(raw_imu, received, options)
        self.assertEqual((raw_cloud, raw_imu), before)

    def test_common_and_cloud_only_clock_corrections_are_added_once(self):
        options = AdapterOptions(common_clock_offset_sec=2.123456789,
                                 clock_offset_sec=-.065432101)
        prepared_cloud = prepare_cloud(cloud(), options=options)
        _, raw_stamp, imu_stamp = prepare_imu(imu(), options=options)
        self.assertEqual(raw_stamp, EPOCH)
        self.assertEqual(imu_stamp, EPOCH+2_123_456_789)
        self.assertEqual(prepared_cloud.output_stamp_ns, EPOCH+2_123_456_789-65_432_101)
        self.assertEqual(imu_stamp-prepared_cloud.output_stamp_ns, 65_432_101)

    def test_corrections_reject_invalid_input_and_output_without_mutation(self):
        options = AdapterOptions(imu_frame='utlidar_imu', common_clock_offset_sec=2.)
        for ns in (2_147_483_647*NS_PER_SEC,):
            message = imu(ns, frame='utlidar_imu')
            before = copy.deepcopy(message)
            with self.assertRaisesRegex(InvalidSensorData, 'shifted IMU'):
                prepare_imu(message, options=options)
            self.assertEqual(message, before)
        with self.assertRaisesRegex(InvalidSensorData, 'shifted IMU'):
            prepare_imu(imu(1), options=AdapterOptions(common_clock_offset_sec=-1.))
        with self.assertRaisesRegex(InvalidSensorData, 'expected native'):
            prepare_imu(imu(), options=options)
        for attribute, correction in (('linear_acceleration', 'accel_offset'),
                                       ('angular_velocity', 'gyro_bias')):
            message = imu(frame='utlidar_imu')
            getattr(message, attribute).x = 1.7e308
            before = copy.deepcopy(message)
            amount = 1.7e308 if correction == 'accel_offset' else -1.7e308
            with self.assertRaisesRegex(InvalidSensorData, 'non-finite'):
                prepare_imu(message, options=AdapterOptions(imu_frame='utlidar_imu',
                                                            **{correction: (amount, 0., 0.)}))
            self.assertEqual(message, before)
        unavailable = imu(frame='utlidar_imu')
        unavailable.linear_acceleration_covariance[0] = -1.
        with self.assertRaisesRegex(InvalidSensorData, 'unavailable'):
            prepare_imu(unavailable, options=options)

    def test_correction_options_reject_invalid_values_and_freeze_vectors(self):
        for name in ('clock_offset_sec', 'common_clock_offset_sec'):
            for value in (True, '1', None, float('nan'), float('inf'), 1e308):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    AdapterOptions(**{name: value})
        for name in ('accel_offset', 'gyro_bias'):
            for value in (None, 1., '123', [], [1., 2.], [0., 0., 0., 0.],
                          [True, 0., 0.], ['1', 0., 0.], [float('nan'), 0., 0.],
                          [0., float('inf'), 0.]):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    AdapterOptions(**{name: value})
            values = [1., 2., 3.]
            options = AdapterOptions(**{name: values})
            values[0] = float('nan')
            self.assertEqual(getattr(options, name), (1., 2., 3.))

    def test_raw_camera_interpolation_happens_before_common_epoch_shift(self):
        merger = RawImuInterpolator(frame_graph=graph())
        merger.push_accel(accel(EPOCH, (0., 1., 2.)), 0.)
        merger.push_gyro(gyro(EPOCH+5_000_000), .001)
        raw = merger.push_accel(accel(EPOCH+10_000_000, (10., 11., 12.)), .002)[0]
        before = copy.deepcopy(raw)
        options = AdapterOptions(common_clock_offset_sec=3., accel_offset=(1., 0., -1.))
        output, received, sent = prepare_imu(raw, options=options)
        self.assertEqual(received, EPOCH+5_000_000)
        self.assertEqual(sent, EPOCH+3_005_000_000)
        self.assertEqual([getattr(output.linear_acceleration, a) for a in 'xyz'], [6., 6., 6.])
        self.assertEqual(raw, before)
        self.assertEqual(merger.max_used_bracket_ns, 10_000_000)

    def test_ns_interpolation_exact_hits_long_gap_and_no_extrapolation(self):
        cov = (.01, 0., 0., 0., .01, 0., 0., 0., .01)
        samples = [AccelSample(EPOCH, (0., 2., 9.), cov),
                   AccelSample(EPOCH+7, (7., 9., 16.), cov)]
        value, returned_cov, gap = interpolate_acceleration(samples, EPOCH+3, 10)
        self.assertEqual(value, (3., 5., 12.))
        self.assertEqual(returned_cov, cov)
        self.assertEqual(gap, 7)
        # A measured exact endpoint does not need an adjacent short bracket.
        self.assertEqual(interpolate_acceleration(samples, EPOCH+7, 1)[0], (7., 9., 16.))
        for target, limit in [(EPOCH-1, 10), (EPOCH+8, 10), (EPOCH+3, 6)]:
            with self.assertRaises(InvalidSensorData):
                interpolate_acceleration(samples, target, limit)
        with self.assertRaises(InvalidSensorData):
            interpolate_acceleration(samples[::-1], EPOCH+3, 10)
        with self.assertRaises(InvalidSensorData):
            AccelSample(EPOCH, (0., float('nan'), 9.), cov)
        with self.assertRaises(InvalidSensorData):
            AccelSample(EPOCH, (0., 0., 9.), (-1.,) + cov[1:])

    def test_raw_merger_waits_for_right_sample_and_preserves_gyro(self):
        merger = RawImuInterpolator(frame_graph=graph())
        start = EPOCH
        self.assertEqual(merger.push_accel(accel(start, (0., 1., 2.)), 0.), [])
        original = gyro(start+5_000_000)
        before = copy.deepcopy(original)
        self.assertEqual(merger.push_gyro(original, .001), [])
        result = merger.push_accel(accel(start+10_000_000, (10., 11., 12.)), .002)
        self.assertEqual(len(result), 1)
        out = result[0]
        self.assertEqual((out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z), (5., 6., 7.))
        self.assertEqual(out.angular_velocity, before.angular_velocity)
        self.assertEqual(out.angular_velocity_covariance, before.angular_velocity_covariance)
        self.assertEqual(out.orientation, before.orientation)
        self.assertEqual(out.orientation_covariance, before.orientation_covariance)
        self.assertEqual(out.header.stamp, before.header.stamp)
        self.assertEqual(out.header.frame_id, 'camera_imu_optical_frame')
        self.assertEqual(original, before)
        self.assertEqual(merger.max_used_bracket_ns, 10_000_000)

    def test_raw_boundaries_drop_gap_and_precoverage_without_extrapolation(self):
        merger = RawImuInterpolator(frame_graph=graph())
        merger.push_gyro(gyro(EPOCH-1), 0.)
        self.assertEqual(merger.push_accel(accel(EPOCH), .001), [])
        self.assertEqual(merger.counts['gyro_dropped_no_left_sample'], 1)
        exact = merger.push_gyro(gyro(EPOCH), .002)
        self.assertEqual(len(exact), 1)
        merger.push_gyro(gyro(EPOCH+20_000_000), .003)
        self.assertEqual(merger.push_accel(accel(EPOCH+40_000_000), .004), [])
        self.assertEqual(merger.counts['gyro_dropped_accel_gap'], 1)
        # Still accept the measured right endpoint after a too-long gap.
        self.assertEqual(len(merger.push_gyro(gyro(EPOCH+40_000_000), .005)), 1)
        for msg in (gyro(EPOCH+40_000_000), gyro(EPOCH+30_000_000)):
            with self.assertRaises(InvalidSensorData):
                merger.push_gyro(msg, .006)
        with self.assertRaises(InvalidSensorData):
            merger.push_accel(accel(EPOCH), .007)

    def test_raw_queues_bounded_and_pending_gyro_expires(self):
        merger = RawImuInterpolator(frame_graph=graph(), queue_size=2, pending_timeout_sec=.1)
        for i in range(5):
            merger.push_gyro(gyro(EPOCH+i), .001*i)
        self.assertEqual(len(merger.gyros), 2)
        self.assertEqual(merger.counts['gyro_dropped_queue'], 3)
        merger.expire(.2)
        self.assertEqual(len(merger.gyros), 0)
        self.assertEqual(merger.counts['gyro_dropped_stale'], 2)
        for i in range(5):
            merger.push_accel(accel(EPOCH+i), .21+i*.001)
        self.assertEqual(len(merger.accels), 2)
        self.assertEqual(merger.counts['accel_evicted_queue'], 3)

    def test_raw_component_validation_does_not_require_unmeasured_component(self):
        merger = RawImuInterpolator(frame_graph=graph())
        message = accel(EPOCH)
        message.angular_velocity.x = float('nan')
        merger.push_accel(message, 0.)
        message = gyro(EPOCH)
        message.linear_acceleration.x = float('nan')
        output = merger.push_gyro(message, .001)[0]
        self.assertEqual(output.linear_acceleration.z, 9.81)
        self.assertEqual(validate_merged_imu(output), EPOCH)
        message = accel(EPOCH+1)
        message.linear_acceleration_covariance[0] = -1.
        with self.assertRaises(InvalidSensorData):
            merger.push_accel(message, .002)

    def test_static_frame_identity_required_nonidentity_conflicts_fail_closed(self):
        with self.assertRaisesRegex(InvalidSensorData, 'unavailable'):
            RawImuInterpolator().push_gyro(gyro(EPOCH), 0.)
        g = graph()
        g.require_identity('camera_gyro_optical_frame', 'camera_imu_optical_frame')
        # Quaternion signs are equivalent; duplicate static replay is harmless.
        g.add('optical', 'camera_imu_optical_frame', (0., 0., 0.), (0., 0., 0., -1.))
        for translation, rotation in [((.01, 0., 0.), (0., 0., 0., 1.)), ((0., 0., 0.), (1., 0., 0., 0.))]:
            other = IdentityFrameGraph()
            other.add('root', 'a', translation, rotation)
            other.add('root', 'b', (0., 0., 0.), (0., 0., 0., 1.))
            with self.assertRaisesRegex(InvalidSensorData, 'not co-located'):
                other.require_identity('a', 'b')
        with self.assertRaisesRegex(InvalidSensorData, 'conflicting'):
            g.add('optical', 'camera_imu_optical_frame', (.01, 0., 0.), (0., 0., 0., 1.))
        with self.assertRaises(InvalidSensorData):
            g.require_identity('camera_gyro_optical_frame', 'camera_imu_optical_frame')

    def test_live_age_guard_and_cross_sensor_clock_diagnostics(self):
        check_live_age(EPOCH, EPOCH+100_000_000, 1., .1)
        for now in (0, EPOCH+2*NS_PER_SEC, EPOCH-NS_PER_SEC):
            with self.assertRaises(InvalidSensorData):
                check_live_age(EPOCH, now, 1., .1)
        self.assertFalse(clock_relation(None, EPOCH, .5)['available'])
        self.assertFalse(clock_relation(EPOCH+100_000_000, EPOCH, .5)['mismatch'])
        self.assertTrue(clock_relation(EPOCH+2*NS_PER_SEC, EPOCH, .5)['mismatch'])

    def test_diagnostic_json_atomic_counts_and_byte_level(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'input_status.json'
            details = dict(counts={'cloud_received': 3, 'cloud_published': 2,
                                   'cloud_rejected': 1}, raw_counts={'gyro_dropped_stale': 1})
            write_diagnostic_report(path, details, b'\x02', 'clock mismatch', EPOCH)
            value = json.loads(path.read_text())
            self.assertEqual(value['counts'], details['counts'])
            self.assertEqual(value['diagnostic_stamp_ns'], EPOCH)
            self.assertEqual(value['status_level'], 2)
            self.assertFalse(path.with_name('input_status.json.tmp').exists())
            write_diagnostic_report(path, details, 0, 'valid', EPOCH+1)
            self.assertEqual(json.loads(path.read_text())['status_level'], 0)
            # A serialization failure cannot corrupt the preceding complete file.
            with self.assertRaises(ValueError):
                write_diagnostic_report(path, {'bad': float('nan')}, 0, 'bad', EPOCH+2)
            self.assertEqual(json.loads(path.read_text())['diagnostic_stamp_ns'], EPOCH+1)


if __name__ == '__main__':
    unittest.main()
