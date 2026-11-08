"""ROS-independent P9 map accumulation and exact timestamp pairing.

Registered scans are already in the estimator world.  No pose is ever applied
to their XYZ.  The path retains the estimator's IMU reference point; an optional
T_imu_base changes only the downstream state reference point, not the map/path.
"""
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import math
import numbers

import numpy as np


class MappingError(ValueError):
    """Invalid or inconsistent input; the ROS wrapper latches this condition."""


class CapacityError(MappingError):
    """A declared capacity would be exceeded; nothing is silently evicted."""


def positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < 1:
        raise MappingError(f'{name} must be a positive integer')
    return int(value)


def positive_float(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise MappingError(f'{name} must be positive and finite')
    return value


def timestamp_ns(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < 0:
        raise MappingError('Timestamp must be a nonnegative integer in nanoseconds')
    return int(value)


def vector(value, length, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise MappingError(f'{name} must contain {length} finite values')
    return result


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion_matrix(value):
    q = vector(value, 4, 'quaternion xyzw')
    norm = np.linalg.norm(q)
    if abs(norm - 1.) > 1e-3:
        raise MappingError('Quaternion must be normalized (within 1e-3)')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rigid_transform(rotation, translation):
    r = vector(rotation, 9, 'T_imu_base_rotation').reshape(3, 3)
    t = vector(translation, 3, 'T_imu_base_translation')
    if not np.allclose(r.T @ r, np.eye(3), rtol=0., atol=1e-6) or abs(np.linalg.det(r)-1.) > 1e-6:
        raise MappingError('Extrinsic rotation must be proper SO(3), not a reflection')
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = r, t
    return result


def inverse_transform(transform):
    t = np.asarray(transform, dtype=np.float64)
    if t.shape != (4, 4) or not np.allclose(t[3], [0., 0., 0., 1.]):
        raise MappingError('Expected a homogeneous rigid transform')
    checked = rigid_transform(t[:3, :3].ravel(), t[:3, 3])
    result = np.eye(4)
    result[:3, :3] = checked[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ checked[:3, 3]
    return result


def matrix_quaternion(rotation):
    """Unit xyzw quaternion, including stable handling near 180 degrees."""
    r = np.asarray(rotation, dtype=np.float64)
    rigid_transform(r.ravel(), np.zeros(3))
    trace = np.trace(r)
    if trace > 0:
        s = 2 * math.sqrt(trace + 1.)
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s, (r[1, 0]-r[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * math.sqrt(max(0., 1.+r[i, i]-r[j, j]-r[k, k]))
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = s/4, (r[j, i]+r[i, j])/s, (r[k, i]+r[i, k])/s, (r[k, j]-r[j, k])/s
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)


def covariance(values, name):
    result = vector(values, 36, name).reshape(6, 6)
    scale = max(1., float(np.abs(result).max()))
    if not np.allclose(result, result.T, rtol=0., atol=1e-8*scale):
        raise MappingError(f'{name} must be symmetric')
    result = (result+result.T)/2
    if np.linalg.eigvalsh(result).min() < -1e-8*scale:
        raise MappingError(f'{name} must be positive semidefinite')
    return result


def change_reference_point(position, quaternion, pose_covariance,
                           linear_velocity, angular_velocity, twist_covariance,
                           imu_from_base):
    """T_WB = T_WI T_IB. Twist input axes are I, output axes are B.

    ROS pose covariance is ordered [world position, world fixed-axis rotation].
    Zero covariance is passed through as unspecified, never called certainty.
    A zero/unavailable upstream twist remains zero/unavailable.
    """
    p = vector(position, 3, 'position')
    r_wi = quaternion_matrix(quaternion)
    t = np.asarray(imu_from_base, dtype=np.float64)
    inverse_transform(t)  # Validate without silently accepting a nonrigid matrix.
    r_ib, t_ib = t[:3, :3], t[:3, 3]
    r_bi = r_ib.T
    lever_world = r_wi @ t_ib
    jp = np.eye(6)
    jp[:3, 3:] = -skew(lever_world)
    jt = np.zeros((6, 6))
    jt[:3, :3] = jt[3:, 3:] = r_bi
    jt[:3, 3:] = -r_bi @ skew(t_ib)
    cp = jp @ covariance(pose_covariance, 'pose covariance') @ jp.T
    ct = jt @ covariance(twist_covariance, 'twist covariance') @ jt.T
    twist = jt @ np.r_[vector(linear_velocity, 3, 'linear velocity'),
                      vector(angular_velocity, 3, 'angular velocity')]
    return dict(position=p+lever_world, quaternion=matrix_quaternion(r_wi @ r_ib),
                pose_covariance=(cp+cp.T)/2, linear_velocity=twist[:3],
                angular_velocity=twist[3:], twist_covariance=(ct+ct.T)/2)


class ExactPairBuffer:
    """Emit strictly ordered exact pairs, without interpolation or eviction.

    A newer complete pair waits behind an older pending stamp. Missing inputs
    eventually cause timeout, preventing a successful-looking partial replay.
    The caller supplies monotonic wall time, independent of ROS /clock pauses.
    """
    def __init__(self, max_pending=128, timeout_sec=5.):
        self.max_pending = positive_integer(max_pending, 'max_pending_pairs')
        self.timeout_sec = positive_float(timeout_sec, 'pair_timeout_sec')
        self.pending = {'odom': {}, 'cloud': {}}
        self.counts = Counter()
        self.last_stamp = None
        self.max_wait_sec = 0.

    def add(self, kind, stamp, payload, now):
        if kind not in self.pending:
            raise MappingError('Pair input kind must be odom or cloud')
        stamp = timestamp_ns(stamp)
        if not math.isfinite(now):
            raise MappingError('Monotonic arrival time must be finite')
        queue = self.pending[kind]
        self.counts[kind+'_received'] += 1
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.counts['backward_or_duplicate'] += 1
            raise MappingError('Duplicate/backward stamp after emitted pair; restart after clock reset')
        if stamp in queue:
            self.counts['duplicate_pending'] += 1
            raise MappingError(f'Duplicate pending {kind} stamp {stamp}')
        if len(queue) >= self.max_pending:
            self.counts['capacity_errors'] += 1
            raise CapacityError(f'Pending {kind} limit {self.max_pending} reached; no silent eviction')
        queue[stamp] = (payload, float(now))
        self.check_timeout(now)
        pairs = []
        while self.pending['odom'] or self.pending['cloud']:
            oldest = min(self.pending['odom'].keys() | self.pending['cloud'].keys())
            if any(oldest not in q for q in self.pending.values()):
                break
            odom, oa = self.pending['odom'].pop(oldest)
            cloud, ca = self.pending['cloud'].pop(oldest)
            self.max_wait_sec = max(self.max_wait_sec, now-min(oa, ca))
            self.last_stamp = oldest
            self.counts['paired'] += 1
            pairs.append((oldest, odom, cloud))
        return pairs

    def check_timeout(self, now):
        for kind, queue in self.pending.items():
            for stamp, (_, arrival) in queue.items():
                if now-arrival > self.timeout_sec:
                    self.counts['timeout_errors'] += 1
                    raise MappingError(f'Unpaired {kind} stamp {stamp} exceeded {self.timeout_sec:g}s wall timeout')

    def report(self):
        return dict(counts=dict(self.counts), pending={k: len(v) for k, v in self.pending.items()},
                    max_pending_per_stream=self.max_pending, timeout_sec=self.timeout_sec,
                    maximum_pair_wait_sec=self.max_wait_sec, last_paired_stamp_ns=self.last_stamp,
                    exact_timestamp_pairing=True, silent_evictions=0)


def decode_cloud(data, fields, width, height, point_step, row_step, bigendian=False):
    """Decode XYZI using explicit row strides; accept float32/64 and padding.

    fields is an iterable of (name, offset, PointField datatype, count).
    Intensity may be absent (zero-filled, reported by the ROS wrapper).
    """
    for value, name in ((width, 'width'), (height, 'height'), (point_step, 'point_step'), (row_step, 'row_step')):
        if not isinstance(value, numbers.Integral) or value < 0:
            raise MappingError(f'Invalid cloud {name}')
    if height < 1 or point_step < 1 or row_step < width*point_step or len(data) != height*row_step:
        raise MappingError('Invalid PointCloud2 shape, stride, or payload length')
    lookup = {}
    for name, offset, datatype, count in fields:
        if name in lookup:
            raise MappingError('Duplicate PointCloud2 field name')
        lookup[name] = (offset, datatype, count)
    names, formats, offsets = [], [], []
    for name in ('x', 'y', 'z', 'intensity'):
        if name not in lookup:
            if name == 'intensity':
                continue
            raise MappingError('Missing cloud field '+name)
        offset, datatype, count = lookup[name]
        if datatype not in (7, 8) or count != 1 or offset < 0:
            raise MappingError(f'{name} must be one FLOAT32/FLOAT64 value')
        size = 4 if datatype == 7 else 8
        if offset+size > point_step:
            raise MappingError('Cloud field extends beyond point_step')
        names.append(name); offsets.append(offset)
        formats.append(('>' if bigendian else '<')+f'f{size}')
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=point_step))
    values = np.ndarray((height, width), dtype=dtype, buffer=data, strides=(row_step, point_step))
    result = np.zeros((width*height, 4), dtype=np.float64)
    for column, name in enumerate(('x', 'y', 'z', 'intensity')):
        if name in names:
            result[:, column] = values[name].reshape(-1)
    return result


class VoxelMap:
    """Observation-weighted XYZI centroids, matching the previous P9 mapper."""
    def __init__(self, size=.05, max_voxels=1_000_000):
        self.size = positive_float(size, 'voxel_size')
        self.max_voxels = positive_integer(max_voxels, 'max_map_voxels')
        self.cells = {}
        self.counts = Counter()

    def add(self, points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 4:
            raise MappingError('Registered points must have shape Nx4')
        finite = np.isfinite(points).all(axis=1)
        clean = points[finite]
        if len(clean) and (np.abs(clean).max() > np.finfo(np.float32).max or
                           np.abs(clean[:, :3]/self.size).max() >= 2**62):
            raise MappingError('Registered coordinates/intensity exceed safe float32 or voxel index range')
        if len(clean):
            keys, inverse, counts = np.unique(np.floor(clean[:, :3]/self.size).astype(np.int64),
                                              axis=0, return_inverse=True, return_counts=True)
            keys = [tuple(int(v) for v in row) for row in keys]
            new_cells = sum(key not in self.cells for key in keys)
            if len(self.cells)+new_cells > self.max_voxels:
                self.counts['capacity_errors'] += 1
                raise CapacityError(f'Map requires {len(self.cells)+new_cells} voxels, limit={self.max_voxels}; scan not partially added')
            sums = np.column_stack([np.bincount(inverse, weights=clean[:, i]) for i in range(4)])
            for key, total, count in zip(keys, sums, counts):
                if key in self.cells:
                    previous, n = self.cells[key]
                    self.cells[key] = (previous+total, n+int(count))
                else:
                    self.cells[key] = (total, int(count))
        self.counts['scans'] += 1
        self.counts['input_points'] += len(points)
        self.counts['accepted_points'] += len(clean)
        self.counts['nonfinite_points_rejected'] += int((~finite).sum())
        return clean

    def points(self):
        return np.asarray([total/n for total, n in self.cells.values()], dtype=np.float32).reshape(-1, 4)

    def report(self):
        return dict(counts=dict(self.counts), voxels=len(self.cells), max_voxels=self.max_voxels,
                    voxel_size_m=self.size, policy='observation-weighted XYZI centroid; no expiry or silent truncation',
                    second_pose_transform_applied=False)


@dataclass
class PoseSample:
    stamp_ns: int
    position: np.ndarray
    quaternion: np.ndarray


class MappingCore:
    def __init__(self, voxel_size=.05, max_map_voxels=1_000_000, max_path_poses=200_000):
        self.voxels = VoxelMap(voxel_size, max_map_voxels)
        self.max_path_poses = positive_integer(max_path_poses, 'max_path_poses')
        self.path = []

    def accept(self, stamp, world_xyzi, position, quaternion):
        stamp = timestamp_ns(stamp)
        p = vector(position, 3, 'position')
        q = vector(quaternion, 4, 'quaternion')
        quaternion_matrix(q)
        if self.path and stamp <= self.path[-1].stamp_ns:
            raise MappingError('Map/path pair stamps must be strictly increasing')
        if len(self.path) >= self.max_path_poses:
            raise CapacityError(f'Path limit {self.max_path_poses} reached; no silent truncation')
        clean = self.voxels.add(world_xyzi)  # Points already world; pose intentionally absent.
        self.path.append(PoseSample(stamp, p.copy(), q.copy()))
        return clean

    def report(self):
        return dict(map=self.voxels.report(), path_poses=len(self.path), max_path_poses=self.max_path_poses,
                    first_stamp_ns=self.path[0].stamp_ns if self.path else None,
                    last_stamp_ns=self.path[-1].stamp_ns if self.path else None,
                    path_reference_point='native Point-LIO IMU origin', unitree_odom_used=False,
                    pose_alignment_applied=False)


def save_pcd(path, points, frame='camera_init'):
    path = Path(path)
    if not frame or '\n' in frame or '\r' in frame:
        raise MappingError('Invalid PCD frame label')
    points = np.asarray(points, dtype='<f4')
    if points.ndim != 2 or points.shape[1] != 4 or not np.isfinite(points).all():
        raise MappingError('PCD must contain finite Nx4 XYZI')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.tmp')
    header = (f'# P9 registered geometry; frame={frame}; no second pose transform\n'
              f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    with temporary.open('wb') as handle:
        handle.write(header.encode('ascii'))
        handle.write(points.tobytes())
    temporary.replace(path)
