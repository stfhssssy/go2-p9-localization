#!/usr/bin/env python3
"""Compare the frozen P9 mapper against its vectorized centroid-publication patch.

No source repository is modified. The numerical operation is still float64
observation-weighted division followed by float32 output, in dict insertion order.
The benchmark tests map export only, not end-to-end ROS latency or navigation.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
import types
import unittest

import numpy as np

OLD = '''    def points(self):
        return np.asarray([total/n for total, n in self.cells.values()], dtype=np.float32).reshape(-1, 4)
'''
NEW = '''    def points(self):
        # Batch the same float64 weighted-centroid calculation. Avoid one NumPy
        # division allocation per voxel on the ROS input-processing thread.
        # Counts are converted to float64, as in the previous float64 division.
        rows = np.fromiter(self.cells.values(),
                           dtype=[('sum', np.float64, (4,)), ('count', np.float64)],
                           count=len(self.cells))
        return np.asarray(rows['sum']/rows['count'][:, None], dtype=np.float32).reshape(-1, 4)
'''
PATCH_PATH = 'src/go2_p9/go2_p9/mapping_core.py'
OldMap = NewMap = None


def default_source():
    root = Path(__file__).resolve().parents[2]
    candidates = (root/PATCH_PATH,
                  root/'p9-nav-release-20261110'/PATCH_PATH)
    return next((path for path in candidates if path.is_file()), candidates[0])


def load_module(name, source):
    module = types.ModuleType(name)
    sys.modules[name] = module  # dataclasses resolves the declaring module
    exec(compile(source, '<'+name+'>', 'exec'), module.__dict__)
    return module


def expected_patch(source):
    if source.count(OLD) != 1:
        raise ValueError('Expected exactly one original VoxelMap.points implementation')
    patched = source.replace(OLD, NEW, 1)
    diff = ''.join(difflib.unified_diff(source.splitlines(keepends=True), patched.splitlines(keepends=True),
                                      fromfile='a/'+PATCH_PATH, tofile='b/'+PATCH_PATH))
    return patched, diff


def initialize_maps(source_path, missing_is_skip=False):
    source_path = Path(source_path)
    if not source_path.is_file():
        message = ('Mapper source not found: '+str(source_path)+
                   '. Run test_mapping_points.py --source /path/to/mapping_core.py explicitly.')
        if missing_is_skip:
            raise unittest.SkipTest(message)
        raise FileNotFoundError(message)
    source = source_path.read_text()
    reconstructed = source.count(OLD) == 0 and source.count(NEW) == 1
    if reconstructed:
        original = source.replace(NEW, OLD, 1)
        print('Mapper source is already patched; reconstructing the original OLD implementation '
              'in memory for comparison: '+str(source_path), file=sys.stderr)
    else:
        original = source
    patched, patch = expected_patch(original)
    actual_patch = Path(__file__).with_name('mapping-points.patch').read_text()
    if patch != actual_patch:
        raise ValueError('Patch file does not exactly match the tested source transformation')
    global OldMap, NewMap
    OldMap = load_module('p9_original_mapping_core', original).VoxelMap
    NewMap = load_module('p9_batched_mapping_core', patched).VoxelMap
    return dict(source=str(source_path), input_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                source_sha256=hashlib.sha256(original.encode()).hexdigest(),
                original_reconstructed_from_patched_source=reconstructed,
                patch_sha256=hashlib.sha256(patch.encode()).hexdigest())


class EquivalenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if OldMap is None or NewMap is None:
            initialize_maps(default_source(), missing_is_skip=True)

    def assert_identical(self, old, new):
        a, b = old.points(), new.points()
        self.assertEqual(a.dtype, np.dtype(np.float32))
        self.assertEqual(a.shape, b.shape)
        self.assertEqual(a.dtype, b.dtype)
        self.assertEqual(a.tobytes(), b.tobytes(), 'Centroid float32 bytes changed')

    def pair(self):
        return OldMap(.05), NewMap(.05)

    def test_empty_shape_and_dtype(self):
        a, b = self.pair()
        self.assert_identical(a, b)
        self.assertEqual(a.points().shape, (0, 4))

    def test_repeated_scans_negative_cells_and_intensity(self):
        a, b = self.pair()
        cloud = np.array([[-.011, -.002, -.049, 2.], [-.013, -.004, -.045, 8.],
                          [1.01, 2.02, 3.01, -30.], [0., 0., 0., 0.]])
        for count in (1, 7, 29):
            for _ in range(count):
                a.add(cloud)
                b.add(cloud)
            self.assert_identical(a, b)
        self.assertEqual(a.report(), b.report())

    def test_nonfinite_filtering_unchanged(self):
        a, b = self.pair()
        points = np.array([[1., 2., 3., 4.], [np.nan, 2., 3., 4.], [1., 2., np.inf, 3.]])
        self.assertEqual(a.add(points).tobytes(), b.add(points).tobytes())
        self.assert_identical(a, b)
        self.assertEqual(a.report(), b.report())

    def test_many_random_scans_and_existing_voxel_updates(self):
        a, b = self.pair()
        rng = np.random.default_rng(71813)
        for _ in range(8):
            xyz = rng.uniform(-2., 2., (10000, 3))
            cloud = np.column_stack((xyz, rng.uniform(-500., 500., 10000)))
            a.add(cloud)
            b.add(cloud)
            self.assert_identical(a, b)
        self.assertEqual(a.report(), b.report())

    def test_extreme_finite_sums_and_large_positive_counts(self):
        a, b = self.pair()
        totals = np.array([[0., -0., 1e-30, -1e-30], [1e30, -1e30, 3.141592653589793, -2.718281828459045],
                           [2.**53+2., 2.**-100, 4.5, 12.]], dtype=np.float64)
        counts = [1, 10**12, 2**53+1]
        cells = {(i, -i, i*2): (total.copy(), count) for i, (total, count) in enumerate(zip(totals, counts))}
        a.cells = b.cells = cells
        before = [(total.tobytes(), count) for total, count in cells.values()]
        self.assert_identical(a, b)
        self.assertEqual(before, [(total.tobytes(), count) for total, count in cells.values()])

    def test_insertion_order_and_no_map_mutation(self):
        a, b = self.pair()
        cells = {(8, 0, 0): (np.array([8., 16., 24., 32.]), 2),
                 (-1, 0, 0): (np.array([3., 6., 9., 12.]), 3),
                 (2, 0, 0): (np.array([7., 14., 21., 28.]), 7)}
        a.cells = b.cells = cells
        self.assert_identical(a, b)
        self.assertEqual(a.points()[:, 0].tolist(), [4., 1., 1.])
        self.assertEqual(list(cells), [(8, 0, 0), (-1, 0, 0), (2, 0, 0)])


def benchmark(voxels, repeats):
    rng = np.random.default_rng(811117)
    totals = rng.uniform(-2000., 2000., (voxels, 4))
    counts = rng.integers(1, 1000000, size=voxels)
    cells = {(i, 0, 0): (totals[i].copy(), int(counts[i])) for i in range(voxels)}
    old, new = OldMap(), NewMap()
    old.cells = new.cells = cells
    expected, actual = old.points(), new.points()
    if expected.tobytes() != actual.tobytes():
        raise AssertionError('Benchmark map outputs differ')
    timings = {'original': [], 'batched': []}
    for _ in range(repeats):
        for label, instance in (('original', old), ('batched', new)):
            start = time.perf_counter()
            result = instance.points()
            timings[label].append(time.perf_counter()-start)
            if result.tobytes() != expected.tobytes():
                raise AssertionError('Repeated export changed its output')
    medians = {key: statistics.median(values) for key, values in timings.items()}
    return dict(voxels=voxels, repeats=repeats, byte_identical=True, elapsed_sec=timings,
                median_sec=medians, median_speedup=medians['original']/medians['batched'],
                scope='Map centroid export only; not ROS or end-to-end throughput')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=default_source())
    parser.add_argument('--benchmark-voxels', type=int, default=200000)
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if args.benchmark_voxels < 1 or args.repeats < 1:
        parser.error('benchmark sizes and repeats must be positive')
    try:
        provenance = initialize_maps(args.source)
    except FileNotFoundError as error:
        parser.error(str(error))
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(EquivalenceTests))
    if not result.wasSuccessful():
        return 1
    print(json.dumps(dict(**provenance, tests=result.testsRun,
                          benchmark=benchmark(args.benchmark_voxels, args.repeats)), indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
