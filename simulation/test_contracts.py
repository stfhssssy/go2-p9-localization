#!/usr/bin/env python3
"""Offline checks for real-bag leakage, model drift and native point semantics."""
import ast
import bisect
import copy
import json
from pathlib import Path
import unittest
import numpy as np
import yaml
from prepare_runtime import verify_profile
from sensor_model.model_contract import contract

HERE = Path(__file__).resolve().parent
source = ast.parse((HERE/'p9_sensor_adapter.py').read_text())
# Test the actual pure functions without importing ROS-dependent node code.
selected = [node for node in source.body if
            (isinstance(node, ast.FunctionDef) and node.name in ('interpolate_acceleration', 'keep_native_cloud'))
            or (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'DTYPE' for t in node.targets))]
scope = {'np': np, 'bisect': bisect}
exec(compile(ast.Module(body=selected, type_ignores=[]), str(HERE/'p9_sensor_adapter.py'), 'exec'), scope)


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((HERE/'sensor_model/model_manifest.json').read_text())
        self.profile = yaml.safe_load((HERE.parent/'config/p9nav-sim.yaml').read_text())

    def test_exact_model_profile_and_wrapper(self):
        verify_profile(self.profile, self.model)
        actual = contract()
        self.assertEqual(actual['wrapper_sha256'], self.model['wrapper_sha256'])
        for key in ('body_from_imu', 'body_from_lidar', 'imu_from_lidar', 'imu_from_body'):
            np.testing.assert_allclose(actual[key]['homogeneous_matrix'], self.model[key]['homogeneous_matrix'], atol=1e-12)

    def test_reject_real_clock_and_bias(self):
        for key, bad in [('clock_offset_sec', 2167.645092767), ('common_clock_offset_sec', 1.),
                         ('gyro_bias', [0., .001, 0.]), ('accel_offset', [0., 0., .1])]:
            p = copy.deepcopy(self.profile)
            p['sensor'][key] = bad
            with self.subTest(key=key), self.assertRaises(ValueError):
                verify_profile(p, self.model)

    def test_reject_real_mounting(self):
        p = copy.deepcopy(self.profile)
        p['calibration']['translation'] = [.0412, .1343, -.0708]
        with self.assertRaises(ValueError):
            verify_profile(p, self.model)

    def test_no_accel_extrapolation(self):
        f = scope['interpolate_acceleration']
        self.assertIsNone(f([10, 20], [[1, 2, 3], [3, 4, 5]], 9))
        self.assertIsNone(f([10, 20], [[1, 2, 3], [3, 4, 5]], 21))
        self.assertIsNone(f([10, 40_000_010], [[1, 2, 3], [3, 4, 5]], 20))
        np.testing.assert_array_equal(f([10, 20], [[1, 2, 3], [3, 4, 5]], 15), [2, 3, 4])

    def test_filter_preserves_native_xyz_and_zero_point_time(self):
        raw = np.array([(1., 2., 3.), (2., 3., 4.)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        R = np.array(self.model['body_from_lidar']['rotation_matrix'])
        t = np.array(self.model['body_from_lidar']['translation'])
        out = scope['keep_native_cloud'](raw, 1, R, t)
        for name in ('x', 'y', 'z'):
            np.testing.assert_array_equal(out[name], raw[name])
        np.testing.assert_array_equal(out['time'], [0., 0.])
        np.testing.assert_array_equal(out['ring'], [0, 1])


if __name__ == '__main__':
    unittest.main(verbosity=2)
