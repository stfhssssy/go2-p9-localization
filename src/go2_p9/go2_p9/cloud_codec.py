"""ROS-independent, datatype-aware PointCloud2 normalization.

The decoder deliberately handles row padding and endianness itself: the
sensor_msgs_py read_points return type differs across ROS releases. ROS output
is constructed by sensor_msgs_py in sensor_adapter.py.
"""

from dataclasses import dataclass
import math
import struct

import numpy as np


# sensor_msgs/msg/PointField datatype values.
FIELD_FORMATS = {
    1: 'b', 2: 'B', 3: 'h', 4: 'H',
    5: 'i', 6: 'I', 7: 'f', 8: 'd',
}
FLOAT32_MAX = 3.4028234663852886e38
NANOSECONDS_PER_SECOND = 1_000_000_000


class InvalidSensorData(ValueError):
    """A sensor message cannot safely be passed to the estimator."""


@dataclass(frozen=True)
class CloudOptions:
    time_field: str = 'time'
    point_time_scale: float = 1.0
    point_time_is_absolute: bool = False
    max_scan_duration_sec: float = 0.2
    mode: str = 'live'
    instantaneous_cloud: bool = False

    def __post_init__(self):
        if self.mode not in ('live', 'replay', 'sim'):
            raise ValueError('cloud mode must be live, replay or sim')
        if type(self.instantaneous_cloud) is not bool:
            raise ValueError('instantaneous_cloud must be a boolean')
        if self.instantaneous_cloud and self.mode != 'sim':
            raise ValueError('instantaneous_cloud is allowed only in sim mode')
        if self.instantaneous_cloud and self.point_time_is_absolute:
            raise ValueError('instantaneous_cloud requires explicit zero relative point times')
        if not self.time_field or self.time_field in ('x', 'y', 'z', 'intensity', 'ring'):
            raise ValueError('time_field must be a distinct, nonempty field name')
        if not math.isfinite(self.point_time_scale) or self.point_time_scale <= 0.0:
            raise ValueError('point_time_scale must be finite and positive')
        if (not math.isfinite(self.max_scan_duration_sec)
                or self.max_scan_duration_sec <= 0.0):
            raise ValueError('max_scan_duration_sec must be finite and positive')


@dataclass
class NormalizedCloud:
    # Canonical order: x, y, z, intensity, ring, relative time in seconds.
    points: list
    dropped_coordinates: int
    sorted_by_time: bool


def stamp_nanoseconds(stamp):
    """Convert a ROS stamp without floating-point epoch precision loss."""
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < NANOSECONDS_PER_SECOND:
        raise InvalidSensorData('invalid or negative header timestamp')
    return sec * NANOSECONDS_PER_SECOND + nanosec


def shifted_stamp_parts(stamp, offset_sec):
    """Apply only an explicitly configured fixed offset, preserving precision."""
    if not math.isfinite(offset_sec):
        raise ValueError('timestamp offset must be finite')
    value = stamp_nanoseconds(stamp) + round(offset_sec * NANOSECONDS_PER_SECOND)
    if value < 0 or value // NANOSECONDS_PER_SECOND > 2_147_483_647:
        raise InvalidSensorData('shifted header timestamp is outside ROS Time range')
    return divmod(value, NANOSECONDS_PER_SECOND)


def normalize_cloud(msg, options=CloudOptions()):
    """Decode a PointCloud2-like object, keeping original sensor coordinates.

    Invalid coordinates (for example, no-return NaNs) are filtered. Invalid
    timing rejects the entire scan, even if attached to an invalid coordinate;
    timestamps are never synthesized. Optional intensity/ring default to zero.
    """
    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    if width <= 0 or height <= 0:
        raise InvalidSensorData('empty point cloud')
    if point_step <= 0 or row_step < width * point_step:
        raise InvalidSensorData('invalid PointCloud2 point_step/row_step')
    data = memoryview(msg.data)
    required_size = (height - 1) * row_step + width * point_step
    if len(data) < required_size:
        raise InvalidSensorData('PointCloud2 data is shorter than declared dimensions')

    fields = {}
    for field in msg.fields:
        if field.name in fields:
            raise InvalidSensorData(f'duplicate field {field.name!r}')
        fields[field.name] = field
    for name in ('x', 'y', 'z', options.time_field):
        if name not in fields:
            raise InvalidSensorData(
                f'missing required field {name!r}; genuine per-point timing is required')

    decoders = {}
    endian = '>' if msg.is_bigendian else '<'
    for name in ('x', 'y', 'z', 'intensity', 'ring', options.time_field):
        field = fields.get(name)
        if field is None:
            continue
        if field.datatype not in FIELD_FORMATS or field.count != 1:
            raise InvalidSensorData(f'field {name!r} must have a supported scalar datatype')
        decoder = struct.Struct(endian + FIELD_FORMATS[field.datatype])
        if field.offset < 0 or field.offset + decoder.size > point_step:
            raise InvalidSensorData(f'field {name!r} lies outside point_step')
        decoders[name] = (decoder, field.offset)

    header_sec = stamp_nanoseconds(msg.header.stamp) / NANOSECONDS_PER_SECOND
    points = []
    dropped_coordinates = 0
    previous_time = -math.inf
    ordered = True
    first_time = math.inf
    last_time = -math.inf
    for row in range(height):
        for column in range(width):
            offset = row * row_step + column * point_step
            values = {
                name: decoder.unpack_from(data, offset + field_offset)[0]
                for name, (decoder, field_offset) in decoders.items()
            }
            point_time = values[options.time_field] * options.point_time_scale
            if options.point_time_is_absolute:
                point_time -= header_sec
            if not math.isfinite(point_time) or point_time < 0.0:
                raise InvalidSensorData('per-point time must be finite and nonnegative')
            if point_time > options.max_scan_duration_sec:
                raise InvalidSensorData(
                    f'point offset {point_time:.6g}s exceeds max_scan_duration_sec; '
                    'check time field, units and header reference')
            if options.instantaneous_cloud and values[options.time_field] != 0.0:
                raise InvalidSensorData('instantaneous simulator cloud requires every relative point time to be zero')
            first_time = min(first_time, point_time)
            last_time = max(last_time, point_time)
            xyz = tuple(float(values[name]) for name in ('x', 'y', 'z'))
            if not all(math.isfinite(v) and abs(v) <= FLOAT32_MAX for v in xyz):
                dropped_coordinates += 1
                continue
            intensity = float(values.get('intensity', 0.0))
            ring = values.get('ring', 0)
            if not math.isfinite(intensity) or abs(intensity) > FLOAT32_MAX:
                raise InvalidSensorData('intensity cannot be represented as finite float32')
            if not math.isfinite(ring) or ring != int(ring) or not 0 <= ring <= 65535:
                raise InvalidSensorData('ring must be an integer in [0, 65535]')
            ordered = ordered and point_time >= previous_time
            previous_time = point_time
            points.append((*xyz, intensity, int(ring), point_time))
    if not points:
        raise InvalidSensorData('point cloud has no finite coordinates')
    if not options.instantaneous_cloud and width * height > 1 and last_time <= first_time:
        raise InvalidSensorData(
            'all point timestamps are identical; a scan needs actual acquisition times')
    if not ordered:
        points.sort(key=lambda point: point[-1])
    return NormalizedCloud(points, dropped_coordinates, not ordered)


def normalize_cloud_numpy(msg, options=CloudOptions()):
    """Vectorized equivalent of :func:`normalize_cloud`, retained as its oracle.

    ``points`` is an N-by-6 float64 array in the same canonical column order.
    Float64 intermediate calculations match the scalar decoder's Python floats;
    only the downstream canonical serialization rounds to float32. All timing
    is checked before removing invalid coordinates. Padding is never a point.
    """
    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    if width <= 0 or height <= 0:
        raise InvalidSensorData('empty point cloud')
    if point_step <= 0 or row_step < width * point_step:
        raise InvalidSensorData('invalid PointCloud2 point_step/row_step')
    data = memoryview(msg.data)
    required_size = (height - 1) * row_step + width * point_step
    if len(data) < required_size:
        raise InvalidSensorData('PointCloud2 data is shorter than declared dimensions')
    fields = {}
    for field in msg.fields:
        if field.name in fields:
            raise InvalidSensorData(f'duplicate field {field.name!r}')
        fields[field.name] = field
    for name in ('x', 'y', 'z', options.time_field):
        if name not in fields:
            raise InvalidSensorData(
                f'missing required field {name!r}; genuine per-point timing is required')
    numpy_formats = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2',
                     5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    endian = '>' if msg.is_bigendian else '<'
    names, formats, offsets = [], [], []
    for name in ('x', 'y', 'z', 'intensity', 'ring', options.time_field):
        field = fields.get(name)
        if field is None:
            continue
        if field.datatype not in FIELD_FORMATS or field.count != 1:
            raise InvalidSensorData(f'field {name!r} must have a supported scalar datatype')
        dtype = np.dtype(endian + numpy_formats[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > point_step:
            raise InvalidSensorData(f'field {name!r} lies outside point_step')
        names.append(name)
        formats.append(dtype)
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=point_step))
    records = np.ndarray((height, width), dtype=dtype, buffer=data,
                         strides=(row_step, point_step))
    # C-order flattening preserves row/column order even for padded rows.
    values = {name: records[name].astype(np.float64).reshape(-1) for name in names}
    header_sec = stamp_nanoseconds(msg.header.stamp) / NANOSECONDS_PER_SECOND
    with np.errstate(over='ignore', invalid='ignore'):
        times = values[options.time_field] * options.point_time_scale
        if options.point_time_is_absolute:
            times -= header_sec
    if np.any(~np.isfinite(times) | (times < 0.0)):
        raise InvalidSensorData('per-point time must be finite and nonnegative')
    if np.any(times > options.max_scan_duration_sec):
        invalid_time = times[times > options.max_scan_duration_sec][0]
        raise InvalidSensorData(
            f'point offset {invalid_time:.6g}s exceeds max_scan_duration_sec; '
            'check time field, units and header reference')
    if options.instantaneous_cloud and np.any(values[options.time_field] != 0.0):
        raise InvalidSensorData('instantaneous simulator cloud requires every relative point time to be zero')
    xyz = np.column_stack([values[name] for name in ('x', 'y', 'z')])
    valid = np.all(np.isfinite(xyz) & (np.abs(xyz) <= FLOAT32_MAX), axis=1)
    dropped_coordinates = int(np.count_nonzero(~valid))
    if not np.any(valid):
        raise InvalidSensorData('point cloud has no finite coordinates')
    # Match the oracle: intensity/ring on a discarded coordinate are ignored.
    intensity = values.get('intensity', np.zeros(len(times)))[valid]
    ring = values.get('ring', np.zeros(len(times)))[valid]
    if np.any(~np.isfinite(intensity) | (np.abs(intensity) > FLOAT32_MAX)):
        raise InvalidSensorData('intensity cannot be represented as finite float32')
    if np.any(~np.isfinite(ring) | (ring != np.trunc(ring)) | (ring < 0) | (ring > 65535)):
        raise InvalidSensorData('ring must be an integer in [0, 65535]')
    if not options.instantaneous_cloud and width * height > 1 and np.max(times) <= np.min(times):
        raise InvalidSensorData(
            'all point timestamps are identical; a scan needs actual acquisition times')
    points = np.column_stack((xyz[valid], intensity, ring, times[valid]))
    ordered = bool(np.all(points[1:, 5] >= points[:-1, 5]))
    if not ordered:
        points = points[np.argsort(points[:, 5], kind='stable')]
    return NormalizedCloud(points, dropped_coordinates, not ordered)


def validate_imu(msg):
    """Validate IMU numbers without changing orientation, gravity or biases."""
    stamp_nanoseconds(msg.header.stamp)
    for name in ('linear_acceleration', 'angular_velocity', 'orientation'):
        vector = getattr(msg, name)
        axes = ('x', 'y', 'z', 'w') if name == 'orientation' else ('x', 'y', 'z')
        if not all(math.isfinite(getattr(vector, axis)) for axis in axes):
            raise InvalidSensorData(f'IMU {name} contains non-finite values')
    for name in ('linear_acceleration_covariance', 'angular_velocity_covariance',
                 'orientation_covariance'):
        covariance = getattr(msg, name)
        if len(covariance) != 9 or not all(math.isfinite(value) for value in covariance):
            raise InvalidSensorData(f'IMU {name} must contain nine finite values')
    # sensor_msgs defines covariance[0] == -1 as "this measurement is absent".
    for name in ('linear_acceleration_covariance', 'angular_velocity_covariance'):
        if getattr(msg, name)[0] < 0.0:
            raise InvalidSensorData(f'IMU {name} marks a required measurement unavailable')
