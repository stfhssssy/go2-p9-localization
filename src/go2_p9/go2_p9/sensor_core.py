"""ROS-independent P9 sensor contracts. No inferred clock or spatial offsets."""
from bisect import bisect_left
from collections import Counter, deque
import copy
from dataclasses import dataclass
import math
import struct

from .cloud_codec import (CloudOptions, InvalidSensorData, normalize_cloud,
                          stamp_nanoseconds, validate_imu)

NS_PER_SEC = 1_000_000_000
POINT_STEP = 32
POINT = struct.Struct('<fff4xfH2xf4x')
FIELD_SPECS = (('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
               ('intensity', 16, 7), ('ring', 20, 4), ('time', 24, 7))


@dataclass(frozen=True)
class AdapterOptions:
    cloud_frame: str = 'utlidar_lidar'
    imu_frame: str = 'camera_imu_optical_frame'
    clock_offset_sec: float = 0.0  # cloud header only; never per-point offsets
    point_time_field: str = 'time'
    point_time_scale: float = 1.0
    min_range: float = 0.2
    max_range: float = 30.0
    max_scan_duration_sec: float = 0.2

    def __post_init__(self):
        if not self.cloud_frame or not self.imu_frame:
            raise ValueError('sensor frame IDs must be explicit and nonempty')
        if not math.isfinite(self.clock_offset_sec) or abs(self.clock_offset_sec) > 2_147_483_648:
            raise ValueError('clock_offset_sec must be finite and within ROS Time range')
        if (not math.isfinite(self.min_range) or not math.isfinite(self.max_range)
                or not 0 <= self.min_range < self.max_range):
            raise ValueError('require finite 0 <= min_range < max_range')
        CloudOptions(time_field=self.point_time_field,
                     point_time_scale=self.point_time_scale,
                     max_scan_duration_sec=self.max_scan_duration_sec)

    @property
    def cloud_offset_ns(self):
        return round(self.clock_offset_sec * NS_PER_SEC)


@dataclass
class PreparedCloud:
    header: object
    data: bytes
    width: int
    input_stamp_ns: int
    output_stamp_ns: int
    input_points: int
    dropped_coordinates: int
    dropped_range: int
    sorted_by_time: bool
    first_point_time_sec: float
    last_point_time_sec: float


def checked_stamp(msg, previous_stamp_ns=None):
    stamp = stamp_nanoseconds(msg.header.stamp)
    if stamp // NS_PER_SEC > 2_147_483_647:
        raise InvalidSensorData('header timestamp is outside ROS Time range')
    if previous_stamp_ns is not None and stamp <= previous_stamp_ns:
        raise InvalidSensorData('duplicate or backward header timestamp')
    return stamp


def prepare_cloud(msg, previous_stamp_ns=None, options=None):
    options = options or AdapterOptions()
    if msg.header.frame_id != options.cloud_frame:
        raise InvalidSensorData(f'expected native LiDAR frame {options.cloud_frame!r}')
    stamp = checked_stamp(msg, previous_stamp_ns)
    shifted = stamp + options.cloud_offset_ns
    if not 0 <= shifted < 2_147_483_648 * NS_PER_SEC:
        raise InvalidSensorData('shifted cloud header is outside ROS Time range')
    normalized = normalize_cloud(msg, CloudOptions(
        time_field=options.point_time_field, point_time_scale=options.point_time_scale,
        max_scan_duration_sec=options.max_scan_duration_sec))
    points = [p for p in normalized.points
              if options.min_range <= math.hypot(p[0], p[1], p[2]) <= options.max_range]
    if not points:
        raise InvalidSensorData('no points remain in the configured range')
    data = bytearray(len(points) * POINT_STEP)
    for index, point in enumerate(points):
        POINT.pack_into(data, index * POINT_STEP, *point)
    header = copy.deepcopy(msg.header)
    header.stamp.sec, header.stamp.nanosec = divmod(shifted, NS_PER_SEC)
    return PreparedCloud(header, bytes(data), len(points), stamp, shifted,
                         int(msg.width) * int(msg.height), normalized.dropped_coordinates,
                         len(normalized.points) - len(points), normalized.sorted_by_time,
                         points[0][-1], points[-1][-1])


def validate_merged_imu(msg, previous_stamp_ns=None, frame='camera_imu_optical_frame'):
    if msg.header.frame_id != frame:
        raise InvalidSensorData(f'expected native merged IMU frame {frame!r}')
    stamp = checked_stamp(msg, previous_stamp_ns)
    validate_imu(msg)
    return stamp


def validate_component(msg, component, frame, previous_stamp_ns=None):
    if msg.header.frame_id != frame:
        raise InvalidSensorData(f'expected raw {component} frame {frame!r}')
    stamp = checked_stamp(msg, previous_stamp_ns)
    vector = getattr(msg, component)
    values = tuple(float(getattr(vector, k)) for k in 'xyz')
    covariance = tuple(float(v) for v in getattr(msg, component + '_covariance'))
    if not all(math.isfinite(v) for v in values):
        raise InvalidSensorData(f'{component} contains non-finite values')
    if len(covariance) != 9 or not all(math.isfinite(v) for v in covariance):
        raise InvalidSensorData(f'{component} covariance must have nine finite values')
    if covariance[0] < 0:
        raise InvalidSensorData(f'{component} is unavailable')
    # Gyro's orientation is copied without pretending it is measured.
    if component == 'angular_velocity':
        orientation = tuple(float(getattr(msg.orientation, k)) for k in 'xyzw')
        cov = tuple(float(v) for v in msg.orientation_covariance)
        if len(cov) != 9 or not all(math.isfinite(v) for v in orientation + cov):
            raise InvalidSensorData('gyro orientation/covariance contains invalid values')
    return stamp, values, covariance


def _rotation(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise InvalidSensorData('static TF quaternion is invalid')
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm - 1) > 1e-3:
        raise InvalidSensorData('static TF quaternion is not normalized')
    x, y, z, w = (v / norm for v in q)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


IDENTITY_R = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def _compose(a, b):
    ar, at = a
    br, bt = b
    r = tuple(tuple(sum(ar[i][k]*br[k][j] for k in range(3)) for j in range(3))
              for i in range(3))
    t = tuple(at[i] + sum(ar[i][k]*bt[k] for k in range(3)) for i in range(3))
    return r, t


class IdentityFrameGraph:
    """Only /tf_static edges; verifies co-located, aligned raw/output frames.

    A nonidentity raw-to-output transform is rejected, never applied. Conflicting
    static transforms invalidate this graph until restart; no last-writer wins.
    """
    def __init__(self):
        self.edges = {}
        self.error = None

    def add(self, parent, child, translation, quaternion_xyzw):
        try:
            if not parent or not child or parent == child:
                raise InvalidSensorData('invalid static TF frame IDs')
            if len(translation) != 3 or not all(math.isfinite(v) for v in translation):
                raise InvalidSensorData('invalid static TF translation')
            transform = (_rotation(quaternion_xyzw), tuple(translation))
            old = self.edges.get(child)
            if old is not None:
                same = old[0] == parent
                same = same and max(abs(old[1][0][i][j]-transform[0][i][j])
                                    for i in range(3) for j in range(3)) <= 1e-9
                same = same and max(abs(old[1][1][i]-transform[1][i]) for i in range(3)) <= 1e-9
                if not same:
                    raise InvalidSensorData('conflicting static TF for ' + child)
            self.edges[child] = (parent, transform)
            self._to_root(child)  # detects cycles immediately
        except InvalidSensorData as exc:
            self.error = str(exc)
            raise

    def _to_root(self, frame):
        transform = (IDENTITY_R, (0., 0., 0.))
        visited = set()
        while frame in self.edges:
            if frame in visited:
                raise InvalidSensorData('cycle in static TF graph')
            visited.add(frame)
            parent, edge = self.edges[frame]
            transform = _compose(edge, transform)
            frame = parent
        return frame, transform

    def require_identity(self, source, target, tolerance=1e-6):
        if self.error:
            raise InvalidSensorData(self.error)
        if source == target:
            return
        root_a, (ra, ta) = self._to_root(source)
        root_b, (rb, tb) = self._to_root(target)
        if root_a != root_b:
            raise InvalidSensorData(f'static TF unavailable for {source} -> {target}')
        if (max(abs(ra[i][j]-rb[i][j]) for i in range(3) for j in range(3)) > tolerance
                or math.sqrt(sum((ta[i]-tb[i])**2 for i in range(3))) > tolerance):
            raise InvalidSensorData(f'raw IMU frames are not co-located and aligned: {source} -> {target}; spatial correction is not implemented')


@dataclass(frozen=True)
class AccelSample:
    stamp_ns: int
    acceleration: tuple
    covariance: tuple

    def __post_init__(self):
        if not isinstance(self.stamp_ns, int) or isinstance(self.stamp_ns, bool) or self.stamp_ns < 0:
            raise InvalidSensorData('acceleration stamp must be nonnegative integer nanoseconds')
        if len(self.acceleration) != 3 or not all(math.isfinite(v) for v in self.acceleration):
            raise InvalidSensorData('acceleration sample must contain three finite values')
        if (len(self.covariance) != 9 or not all(math.isfinite(v) for v in self.covariance)
                or self.covariance[0] < 0):
            raise InvalidSensorData('acceleration covariance is invalid or unavailable')


def interpolate_acceleration(samples, target_ns, max_gap_ns):
    """Integer-ns brackets, exact hits allowed even beside a long gap.

    Covariance is retained from the preceding (or exact) measured sample. It is
    not divided by interpolation weights: inter-sample correlation is unknown.
    """
    if not samples:
        raise InvalidSensorData('no acceleration samples')
    stamps = [sample.stamp_ns for sample in samples]
    if not isinstance(target_ns, int) or any(not isinstance(t, int) for t in stamps):
        raise InvalidSensorData('interpolation timestamps must be integer nanoseconds')
    if any(b <= a for a, b in zip(stamps, stamps[1:])) or max_gap_ns <= 0:
        raise InvalidSensorData('acceleration timestamps must be strictly increasing with positive max gap')
    index = bisect_left(stamps, target_ns)
    if index < len(samples) and stamps[index] == target_ns:
        return samples[index].acceleration, samples[index].covariance, 0
    if index == 0 or index == len(samples):
        raise InvalidSensorData('acceleration extrapolation is forbidden')
    left, right = samples[index-1], samples[index]
    gap = right.stamp_ns - left.stamp_ns
    if not 0 < gap <= max_gap_ns:
        raise InvalidSensorData('acceleration bracket exceeds max_accel_gap_sec')
    alpha = (target_ns - left.stamp_ns) / gap
    values = tuple(a + alpha * (b-a) for a, b in zip(left.acceleration, right.acceleration))
    if not all(math.isfinite(v) for v in values):
        raise InvalidSensorData('interpolated acceleration is nonfinite')
    return values, left.covariance, gap


class RawImuInterpolator:
    """Bounded causal stream merger; push methods return ordered Imu-like values."""
    def __init__(self, *, imu_frame='camera_imu_optical_frame',
                 gyro_frame='camera_gyro_optical_frame',
                 accel_frame='camera_accel_optical_frame', frame_graph=None,
                 max_accel_gap_sec=0.03, queue_size=4000, buffer_sec=2.0,
                 pending_timeout_sec=2.0):
        if not isinstance(queue_size, int) or queue_size < 2 or isinstance(queue_size, bool):
            raise ValueError('raw queue_size must be an integer >= 2')
        for name, value in [('max_accel_gap_sec', max_accel_gap_sec),
                            ('buffer_sec', buffer_sec), ('pending_timeout_sec', pending_timeout_sec)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if buffer_sec < max_accel_gap_sec:
            raise ValueError('buffer_sec must cover max_accel_gap_sec')
        if not all((imu_frame, gyro_frame, accel_frame)):
            raise ValueError('raw/output frames must be nonempty')
        self.imu_frame, self.gyro_frame, self.accel_frame = imu_frame, gyro_frame, accel_frame
        self.frame_graph = frame_graph or IdentityFrameGraph()
        self.max_gap_ns = round(max_accel_gap_sec * NS_PER_SEC)
        if self.max_gap_ns < 1:
            raise ValueError('max_accel_gap_sec must be at least one nanosecond')
        self.buffer_ns = round(buffer_sec * NS_PER_SEC)
        self.pending_timeout_sec = pending_timeout_sec
        self.queue_size = int(queue_size)
        self.accels, self.gyros = deque(), deque()
        self.last_gyro = self.last_accel = self.last_output = None
        self.counts = Counter()
        self.max_used_bracket_ns = 0

    def frames_ready(self):
        self.frame_graph.require_identity(self.gyro_frame, self.imu_frame)
        self.frame_graph.require_identity(self.accel_frame, self.imu_frame)

    def push_gyro(self, msg, received_steady_sec):
        self.frames_ready()
        stamp, _, _ = validate_component(msg, 'angular_velocity', self.gyro_frame, self.last_gyro)
        self.last_gyro = stamp
        self.gyros.append((stamp, copy.deepcopy(msg), received_steady_sec))
        self.counts['gyro_accepted'] += 1
        if len(self.gyros) > self.queue_size:
            self.gyros.popleft()
            self.counts['gyro_dropped_queue'] += 1
        return self._drain(received_steady_sec)

    def push_accel(self, msg, received_steady_sec):
        self.frames_ready()
        stamp, values, covariance = validate_component(msg, 'linear_acceleration', self.accel_frame, self.last_accel)
        self.last_accel = stamp
        self.accels.append(AccelSample(stamp, values, covariance))
        self.counts['accel_accepted'] += 1
        while len(self.accels) > self.queue_size:
            self.accels.popleft()
            self.counts['accel_evicted_queue'] += 1
        # Retain one sample before the history cutoff to preserve a bracket.
        while len(self.accels) > 2 and self.accels[1].stamp_ns < stamp-self.buffer_ns:
            self.accels.popleft()
            self.counts['accel_evicted_history'] += 1
        return self._drain(received_steady_sec)

    def expire(self, now_steady_sec):
        while self.gyros and now_steady_sec-self.gyros[0][2] > self.pending_timeout_sec:
            self.gyros.popleft()
            self.counts['gyro_dropped_stale'] += 1

    def _drain(self, now_steady_sec):
        self.expire(now_steady_sec)
        result = []
        while self.gyros and self.accels:
            stamp, gyro, _ = self.gyros[0]
            if stamp > self.accels[-1].stamp_ns:
                break  # wait for a future acceleration message; never extrapolate
            self.gyros.popleft()
            if stamp < self.accels[0].stamp_ns:
                self.counts['gyro_dropped_no_left_sample'] += 1
                continue
            try:
                values, covariance, gap = interpolate_acceleration(self.accels, stamp, self.max_gap_ns)
            except InvalidSensorData:
                self.counts['gyro_dropped_accel_gap'] += 1
                continue
            if self.last_output is not None and stamp <= self.last_output:
                raise InvalidSensorData('internal nonmonotonic merged output')
            gyro.header.frame_id = self.imu_frame  # static identity verified, never a rotation
            for axis, value in zip('xyz', values):
                setattr(gyro.linear_acceleration, axis, value)
            gyro.linear_acceleration_covariance = list(covariance)
            validate_imu(gyro)
            self.last_output = stamp
            self.max_used_bracket_ns = max(self.max_used_bracket_ns, gap)
            self.counts['imu_produced'] += 1
            result.append(gyro)
        return result


def clock_relation(cloud_end_ns, imu_stamp_ns, tolerance_sec):
    if cloud_end_ns is None or imu_stamp_ns is None:
        return dict(available=False, mismatch=False, cloud_minus_imu_sec=None)
    difference = (cloud_end_ns - imu_stamp_ns) / NS_PER_SEC
    return dict(available=True, mismatch=abs(difference) > tolerance_sec,
                cloud_minus_imu_sec=difference)


def check_live_age(stamp_ns, now_ns, max_age_sec, future_tolerance_sec):
    """Explicit live-mode guard; a zero/unstarted ROS clock is not usable."""
    if now_ns <= 0:
        raise InvalidSensorData('live ROS clock is zero/unstarted; no timestamp rewrite attempted')
    age = (now_ns-stamp_ns) / NS_PER_SEC
    if age > max_age_sec:
        raise InvalidSensorData(f'live message stale by {age:.6f}s; check clocks/explicit cloud offset')
    if age < -future_tolerance_sec:
        raise InvalidSensorData(f'live message is {-age:.6f}s in the future; check clocks/explicit cloud offset')
