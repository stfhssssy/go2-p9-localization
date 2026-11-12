"""Mode contracts and calibration preservation without ROS or robot access."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import yaml
from cli_exploration import parser
from make_config import validate_options, core_profile, tare_parameters, local_parameters


class ExplorationConfigTests(unittest.TestCase):
    def test_live_default_calibrated_profile_and_limits(self):
        args = parser().parse_args(['live', '--output', '/tmp/example', '--boundary', '-2', '3', '-4', '5'])
        self.assertEqual(args.config, '/opt/p9/config/p9nav-live.yaml')
        guard = validate_options(args.mode, args.boundary, args.max_forward, args.max_lateral, args.max_yaw)
        self.assertEqual((guard['max_forward_m_s'], guard['max_lateral_m_s'], guard['max_yaw_rad_s']), (.2, .1, .35))
        self.assertEqual((guard['min_x'], guard['max_x'], guard['min_y'], guard['max_y']), (-2., 3., -4., 5.))

    def test_required_boundary_rejects_missing_nonfinite_reversed_or_degenerate(self):
        for value in (None, [], [-1, 1, -1], [2, 1, -1, 1], [-1, 1, 0, 0], [-1, float('nan'), -1, 1]):
            with self.subTest(value=value), self.assertRaises(ValueError): validate_options('live', value)

    def test_live_and_replay_cannot_enable_simulation_behavior(self):
        for mode in ('live', 'replay'):
            for name in ('sim_command_output', 'sim_auto_resume_inputs', 'allow_return_home'):
                with self.subTest(mode=mode, name=name), self.assertRaises(ValueError):
                    validate_options(mode, [-2, 2, -2, 2], **{name: True})

    def test_sim_recovery_requires_sim_command_output(self):
        with self.assertRaises(ValueError):
            validate_options('sim', [-2, 2, -2, 2], sim_auto_resume_inputs=True)
        validate_options('sim', [-2, 2, -2, 2], sim_command_output=True,
                         sim_auto_resume_inputs=True, allow_return_home=True)

    def test_invalid_speed_limits_rejected(self):
        for options in ({'forward': 0.}, {'lateral': float('inf')}, {'yaw': 2.01}, {'forward': -1.}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                validate_options('live', [-2, 2, -2, 2], **options)

    def test_calibration_offsets_and_body_transform_preserved_exactly(self):
        core = dict(calibration=dict(rotation=[.1]*9, translation=[.03, .02, -.1], status='experimental'),
                    sensor=dict(clock_offset_sec=.123, common_clock_offset_sec=.456,
                                gyro_bias=[.001, -.002, .003], accel_offset=[0., 0., 0.]),
                    mapping=dict(voxel_size=.05, T_imu_base_rotation=[.2]*9,
                                 T_imu_base_translation=[.03, .02, -.28]),
                    estimator=dict(config='relative-engine.yaml'))
        original = copy.deepcopy(core)
        output = core_profile(core, '/tmp/pointlio.yaml')
        self.assertEqual(core, original)
        self.assertEqual(output['calibration'], core['calibration'])
        self.assertEqual(output['sensor'], core['sensor'])
        self.assertEqual(output['mapping']['T_imu_base_rotation'], core['mapping']['T_imu_base_rotation'])
        self.assertEqual(output['mapping']['T_imu_base_translation'], core['mapping']['T_imu_base_translation'])
        self.assertEqual(output['mapping']['voxel_size'], .05)
        self.assertEqual(output['estimator']['config'], str(Path('/tmp/pointlio.yaml').resolve()))

    def test_tare_live_wallclock_no_autostart_no_return(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)/'tare.yaml'
            source.write_text(yaml.safe_dump({'tare_planner_node': {'ros__parameters': {'upstream_value': 7}}}))
            for mode in ('live', 'replay', 'sim'):
                out = tare_parameters(source, mode)['tare_planner_node']['ros__parameters']
                self.assertEqual(out['use_sim_time'], mode != 'live')
                self.assertFalse(out['kAutoStart'])
                self.assertFalse(out['kRushHome'])
                self.assertFalse(out['kExtendWayPoint'])
                self.assertEqual(out['upstream_value'], 7)
                self.assertEqual(out['pub_waypoint_topic_'], '/p9/exploration/tare_waypoint')
            self.assertTrue(tare_parameters(source, 'sim', True)['tare_planner_node']['ros__parameters']['kRushHome'])
            with self.assertRaises(ValueError): tare_parameters(source, 'live', True)

    def test_local_controller_limits_match_guard_and_far_is_removed(self):
        params = dict(local_planner=dict(maxSpeed=.5, autonomySpeed=.4),
                      path_follower=dict(maxSpeed=.5, autonomySpeed=.4, maxYawRate=90., stopDisThre=.3),
                      terrain_analysis=dict(use_sim_time=False), terrain_analysis_ext=dict(use_sim_time=False),
                      far_planner=dict(use_sim_time=False))
        original = copy.deepcopy(params)
        out = local_parameters(params, validate_options('live', [-2, 2, -2, 2]))
        self.assertEqual(params, original)
        self.assertNotIn('far_planner', out)
        self.assertEqual(out['local_planner']['autonomySpeed'], .2)
        self.assertEqual(out['path_follower']['maxSpeed'], .2)
        self.assertAlmostEqual(out['path_follower']['maxYawRate'], 20.0535228296)
        self.assertEqual(out['path_follower']['stopDisThre'], .1)

    def test_replay_requires_explicit_bag_and_accepts_output(self):
        args = parser().parse_args(['replay', '--bag', '/tmp/bag', '--output', '/tmp/example',
                                   '--boundary', '-2', '2', '-2', '2'])
        self.assertEqual(args.config, '/opt/p9/config/p9nav-replay.yaml')
        self.assertFalse(args.sim_command_output)
        self.assertEqual(args.rate, .5)


if __name__ == '__main__': unittest.main()
