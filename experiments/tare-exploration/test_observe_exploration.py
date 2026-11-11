#!/usr/bin/env python3
"""Offline checks for observer measurement semantics; no ROS dependency."""
import math
from pathlib import Path
import struct
from types import SimpleNamespace as Object
import unittest

from observe_exploration import (Trajectory, Waypoints, CommandTail, ObservedVoxels,
                                cloud_xyz, assert_read_only)


class MeasurementTests(unittest.TestCase):
    def test_actual_motion_and_roundtrip_are_distinct(self):
        track = Trajectory()
        for wall, point in enumerate([(0,0,0), (3,4,0), (0,0,0)]):
            track.add(point, wall)
        report = track.summary()
        self.assertEqual(report['raw_path_length_m'], 10)
        self.assertEqual(report['max_displacement_from_first_m'], 5)
        self.assertEqual(report['final_displacement_from_first_m'], 0)
        self.assertFalse(track.add((math.nan,0,0), 4))

    def test_repeated_waypoints_are_not_diversity(self):
        points = Waypoints()
        for _ in range(30): points.add((1.,1.,0.))
        points.add((1.1,1.1,0.))
        self.assertEqual(points.summary()['distinct_025m_cells'], 1)
        points.add((-1.,1.,0.))
        self.assertEqual(points.summary()['distinct_025m_cells'], 2)

    def test_stale_zero_command_is_not_a_stop_confirmation(self):
        commands = CommandTail()
        commands.add((.2,0,0,0,0,0), 0)
        for wall in (1.,2.,3.1): commands.add((0,0,0,0,0,0), wall)
        self.assertTrue(commands.summary(3.2)['recent_zero_commands_for_at_least_2_sec'])
        self.assertFalse(commands.summary(6.)['recent_zero_commands_for_at_least_2_sec'])
        commands.add((0,0,0,0,0,math.nan), 6.1)
        self.assertFalse(commands.summary(6.2)['recent_zero_commands_for_at_least_2_sec'])

    def test_30_second_voxel_baseline_is_frozen_before_later_scan(self):
        voxels = ObservedVoxels(.2)
        voxels.add([(0.01,0,0), (math.nan,0,0)], 100)
        voxels.add([(.19,0,0), (-.01,0,0)], 129.99)
        self.assertIsNone(voxels.summary()['added_voxels_after_baseline'])
        voxels.add([(.4,0,0)], 130)
        self.assertEqual(voxels.summary()['baseline_observed_voxels'], 2)
        self.assertEqual(voxels.summary()['added_voxels_after_baseline'], 1)
        voxels.add([(0,0,0)], 131)
        self.assertEqual(voxels.summary()['added_voxels_after_baseline'], 1)
        self.assertEqual(voxels.summary()['valid_cloud_count'], 4)

    def test_pointcloud_big_endian_and_row_padding(self):
        fields = [Object(name=n, datatype=7, count=1, offset=i*4) for i,n in enumerate('xyz')]
        cloud = Object(fields=fields, is_bigendian=True, width=1, height=2,
                       point_step=16, row_step=24,
                       data=struct.pack('>fff',1,2,3)+bytes(12)+struct.pack('>fff',4,5,6)+bytes(4))
        self.assertEqual(cloud_xyz(cloud).tolist(), [[1,2,3], [4,5,6]])
        cloud.data = cloud.data[:-1]
        with self.assertRaises(ValueError): cloud_xyz(cloud)

    def test_passive_ast_contract(self):
        source = Path(__file__).with_name('observe_exploration.py').read_text()
        assert_read_only(source)
        with self.assertRaises(AssertionError):
            assert_read_only('node.create_publisher(Twist, "/cmd_vel", 10)')
        with self.assertRaises(AssertionError):
            assert_read_only('node.create_client(Trigger, "/start")')


if __name__ == '__main__':
    unittest.main()
