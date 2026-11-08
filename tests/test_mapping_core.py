"""Run without ROS: python3 -m unittest discover -s tests -p test_mapping_core.py."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SOURCE = Path(__file__).resolve().parents[1]/'src/go2_p9/go2_p9/mapping_core.py'
SPEC = importlib.util.spec_from_file_location('p9_mapping_core_tested', SOURCE)
core = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = core
SPEC.loader.exec_module(core)


class PairTests(unittest.TestCase):
    def test_nanosecond_exact_pair_not_nearest(self):
        buffer = core.ExactPairBuffer()
        epoch = 1_793_255_293_084_837_436
        self.assertEqual(buffer.add('cloud', epoch, 'scan', 0.), [])
        self.assertEqual(buffer.add('odom', epoch+1, 'wrong', .1), [])
        self.assertEqual(buffer.add('odom', epoch, 'correct', .2), [(epoch, 'correct', 'scan')])
        self.assertEqual(buffer.report()['pending'], {'odom': 1, 'cloud': 0})

    def test_newer_pair_waits_for_older(self):
        buffer = core.ExactPairBuffer()
        buffer.add('cloud', 10, 'c10', 0.)
        buffer.add('cloud', 20, 'c20', .1)
        self.assertEqual(buffer.add('odom', 20, 'o20', .2), [])
        self.assertEqual(buffer.add('odom', 10, 'o10', .3), [(10, 'o10', 'c10'), (20, 'o20', 'c20')])
        with self.assertRaises(core.MappingError):
            buffer.add('cloud', 20, 'duplicate', .4)
        with self.assertRaises(core.MappingError):
            buffer.add('cloud', 1, 'clock reset', .5)

    def test_queue_bound_timeout_and_duplicate_are_explicit(self):
        buffer = core.ExactPairBuffer(max_pending=2, timeout_sec=1.)
        buffer.add('cloud', 1, 'c1', 0.)
        with self.assertRaises(core.MappingError):
            buffer.add('cloud', 1, 'duplicate', .1)
        buffer.add('cloud', 2, 'c2', .2)
        with self.assertRaises(core.CapacityError):
            buffer.add('cloud', 3, 'c3', .3)
        self.assertEqual(set(buffer.pending['cloud']), {1, 2})
        self.assertEqual(buffer.report()['silent_evictions'], 0)
        with self.assertRaises(core.MappingError):
            buffer.check_timeout(1.01)


class GeometryTests(unittest.TestCase):
    def test_registered_world_cloud_never_transformed_by_pose(self):
        mapper = core.MappingCore()
        q = [0., 0., np.sqrt(.5), np.sqrt(.5)]
        cloud = np.array([[1.2, -2.3, .7, 12.], [2.5, -1., 3., 13.]])
        mapper.accept(10, cloud, [100., -30., 20.], q)
        np.testing.assert_allclose(mapper.voxels.points(), cloud, atol=1e-6)
        np.testing.assert_array_equal(mapper.path[0].position, [100., -30., 20.])
        self.assertFalse(mapper.report()['map']['second_pose_transform_applied'])

    def test_inverse_and_reference_origin_not_left_multiplication(self):
        # T_BI from a model must be inverted before passing the bridge T_IB.
        r_bi = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        t_bi = core.rigid_transform(r_bi.ravel(), [.28, 0., .062])
        t_ib = core.inverse_transform(t_bi)
        np.testing.assert_allclose(t_ib @ t_bi, np.eye(4), atol=1e-14)
        q = [0., 0., np.sqrt(.5), np.sqrt(.5)]
        t_wi = core.rigid_transform(core.quaternion_matrix(q).ravel(), [2., 3., 4.])
        expected = t_wi @ t_ib
        result = core.change_reference_point([2., 3., 4.], q, np.zeros(36),
                                            [0., 0., 0.], [0., 0., 0.], np.zeros(36), t_ib)
        np.testing.assert_allclose(result['position'], expected[:3, 3], atol=1e-14)
        np.testing.assert_allclose(core.quaternion_matrix(result['quaternion']), expected[:3, :3], atol=1e-14)
        np.testing.assert_array_equal(result['pose_covariance'], np.zeros((6, 6)))

    def test_lever_arm_twist_and_covariance(self):
        t_ib = core.rigid_transform(np.eye(3).ravel(), [1., 0., 0.])
        cov = np.diag([1., 2., 3., .1, .2, .3])
        result = core.change_reference_point([0., 0., 0.], [0., 0., 0., 1.], cov.ravel(),
                                            [0., 0., 0.], [0., 0., 2.], cov.ravel(), t_ib)
        np.testing.assert_allclose(result['linear_velocity'], [0., 2., 0.])
        self.assertAlmostEqual(result['pose_covariance'][1, 1], 2.3)
        self.assertAlmostEqual(result['pose_covariance'][1, 5], .3)
        self.assertAlmostEqual(result['twist_covariance'][1, 5], .3)
        self.assertGreaterEqual(np.linalg.eigvalsh(result['pose_covariance']).min(), 0.)
        # Independent finite difference: world rotation perturbs the offset.
        angle = 1e-7
        q = [0., 0., np.sin(angle/2), np.cos(angle/2)]
        offset = core.quaternion_matrix(q) @ t_ib[:3, 3]
        np.testing.assert_allclose((offset-t_ib[:3, 3])/angle, [0., 1., 0.], atol=1e-7)

    def test_proper_rotation_and_finite_values_required(self):
        for rotation in (np.diag([-1., 1., 1.]), np.eye(3)*2):
            with self.assertRaises(core.MappingError):
                core.rigid_transform(rotation.ravel(), [0., 0., 0.])
        with self.assertRaises(core.MappingError):
            core.rigid_transform(np.eye(3).ravel(), [np.nan, 0., 0.])
        with self.assertRaises(core.MappingError):
            core.quaternion_matrix([0., 0., 0., 0.])
        for axis in range(3):
            r = -np.eye(3); r[axis, axis] = 1.
            np.testing.assert_allclose(core.quaternion_matrix(core.matrix_quaternion(r)), r, atol=1e-14)


class MapTests(unittest.TestCase):
    def test_voxel_centroid_weights_and_nonfinite_report(self):
        voxels = core.VoxelMap(.05)
        voxels.add([[.01, .01, .01, 1.], [.02, .01, .01, 3.], [np.nan, 0., 0., 0.]])
        voxels.add([[.03, .01, .01, 5.]])
        np.testing.assert_allclose(voxels.points(), [[.02, .01, .01, 3.]], atol=1e-8)
        self.assertEqual(voxels.report()['counts']['nonfinite_points_rejected'], 1)
        self.assertEqual(voxels.report()['counts']['accepted_points'], 3)
        # A registered origin can be valid; a finite point is not a raw placeholder.
        voxels.add([[0., 0., 0., 0.], [-.01, 0., 0., 2.]])
        self.assertEqual(len(voxels.cells), 2)

    def test_capacity_is_transactional_and_path_bound_explicit(self):
        mapper = core.MappingCore(max_map_voxels=1, max_path_poses=2)
        mapper.accept(1, [[0., 0., 0., 1.]], [0., 0., 0.], [0., 0., 0., 1.])
        before = mapper.voxels.points().copy()
        with self.assertRaises(core.CapacityError):
            mapper.accept(2, [[.01, 0., 0., 9.], [1., 0., 0., 2.]], [0., 0., 0.], [0., 0., 0., 1.])
        np.testing.assert_array_equal(mapper.voxels.points(), before)
        self.assertEqual(len(mapper.path), 1)
        mapper.accept(2, [[.01, 0., 0., 3.]], [0., 0., 0.], [0., 0., 0., 1.])
        before = mapper.voxels.points().copy()
        with self.assertRaises(core.CapacityError):
            mapper.accept(3, [[.02, 0., 0., 8.]], [0., 0., 0.], [0., 0., 0., 1.])
        np.testing.assert_array_equal(mapper.voxels.points(), before)
        self.assertEqual(mapper.report()['max_path_poses'], 2)

    def test_padded_organized_cloud_both_byte_orders(self):
        fields = [(n, i*4, 7, 1) for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        expected = np.arange(24, dtype=np.float32).reshape(6, 4)
        for endian in ('<', '>'):
            data = bytearray(2*68)  # three 20-byte points plus eight row padding bytes
            dtype = np.dtype(dict(names=['x', 'y', 'z', 'intensity'],
                formats=[endian+'f4']*4, offsets=[0, 4, 8, 12], itemsize=20))
            array = np.ndarray((2, 3), dtype=dtype, buffer=data, strides=(68, 20))
            for col, name in enumerate(dtype.names):
                array[name] = expected[:, col].reshape(2, 3)
            decoded = core.decode_cloud(data, fields, 3, 2, 20, 68, endian == '>')
            np.testing.assert_array_equal(decoded, expected)
        with self.assertRaises(core.MappingError):
            core.decode_cloud(data[:-1], fields, 3, 2, 20, 68)

    def test_binary_pcd_is_the_saved_world_map(self):
        points = np.array([[1., 2., 3., 4.]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'PCD/map.pcd'
            core.save_pcd(path, points)
            header, payload = path.read_bytes().split(b'DATA binary\n', 1)
            self.assertIn(b'frame=camera_init', header)
            np.testing.assert_array_equal(np.frombuffer(payload, dtype='<f4').reshape(-1, 4), points)


if __name__ == '__main__':
    unittest.main()
