"""Check environment separation and launch behavior without starting ROS."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/go2_p9'))
from go2_p9.configuration import load, estimator_config_path


BIAS = [0.00028568964388210946, -0.0018381138448739292, 0.0005906477368247819]
C_R = [[0.8719407787767535, 0.48938821000005656, -0.01478033217243356],
       [-0.12099380744880735, 0.2446292590717415, 0.9620379535990522],
       [0.4744257337721345, -0.8370517938090508, 0.27251516951505783]]
C_T = [0.04121579449303409, 0.13430446475755778, -0.07078958736127818]
NOMINAL_T_LB = np.array([
    [0.5261136974654234, -0.8386683061659721, 0.1408540009086018, -0.14844810401939176],
    [-0.810135958720698, -0.5446425178319859, -0.21689687908619218, 0.2285879561136748],
    [0.25861961589747207, 1.5279590110544368e-06, -0.9659792411178969, -0.0729721763927847],
    [0., 0., 0., 1.],
])


def transform(rotation, translation):
    value = np.eye(4)
    value[:3, :3] = np.asarray(rotation).reshape(3, 3)
    value[:3, 3] = translation
    return value


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def launch_module_without_ros():
    """Use minimal launch records to exercise setup branches, not ROS itself."""
    class Record:
        def __init__(self, kind, *args, **kwargs):
            self.kind, self.args, self.kwargs = kind, args, kwargs

    class Configuration:
        def __init__(self, key):
            self.key = key

        def perform(self, context):
            return context[self.key]

    declarations = {
        'launch': {'LaunchDescription': list},
        'launch.actions': {name: (lambda *a, _name=name, **kw: Record(_name, *a, **kw))
                           for name in ('DeclareLaunchArgument', 'OpaqueFunction', 'ExecuteProcess',
                                        'TimerAction', 'RegisterEventHandler', 'EmitEvent', 'LogInfo')},
        'launch.event_handlers': {'OnProcessExit': lambda *a, **kw: Record('OnProcessExit', *a, **kw)},
        'launch.events': {'Shutdown': lambda *a, **kw: Record('Shutdown', *a, **kw)},
        'launch.substitutions': {'LaunchConfiguration': Configuration},
        'launch_ros': {},
        'launch_ros.actions': {'Node': lambda *a, **kw: Record('Node', *a, **kw)},
    }
    modules = {}
    for name, attrs in declarations.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        modules[name] = module
    path = ROOT/'src/go2_p9/launch/go2_p9.launch.py'
    spec = importlib.util.spec_from_file_location('p9_profile_test_launch', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class NavigationProfiles(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.temp = Path(self.temporary.name)
        self.environment = patch.dict(os.environ, {'P9_HOME': str(ROOT)})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def profile(self, mode):
        return ROOT/'config'/f'p9nav-{mode}.yaml'

    def test_replay_keeps_frozen_B1_C_and_uses_nominal_body_chain_only(self):
        raw, sensor, mapping, engine = load(self.profile('replay'), 'replay')
        self.assertEqual(sensor['gyro_bias'], BIAS)
        self.assertEqual(sensor['accel_offset'], [0., 0., 0.])
        self.assertEqual(sensor['clock_offset_sec'], 2167.645092767)
        self.assertEqual(sensor['common_clock_offset_sec'], 0.)
        self.assertEqual(sensor['imu_mode'], 'merged_linear')
        np.testing.assert_array_equal(np.array(raw['calibration']['rotation']).reshape(3, 3), C_R)
        np.testing.assert_array_equal(raw['calibration']['translation'], C_T)
        expected_ib = transform(C_R, C_T) @ NOMINAL_T_LB
        np.testing.assert_allclose(transform(mapping['T_imu_base_rotation'], mapping['T_imu_base_translation']),
                                   expected_ib, rtol=0, atol=2e-15)
        self.assertTrue(mapping['publish_base_frame'])
        self.assertTrue(mapping['publish_terrain_inputs'])
        self.assertFalse(mapping['publish_map_alias'])
        self.assertEqual(mapping['base_frame'], 'base_link')
        self.assertEqual(raw['calibration']['status'], 'experimental')
        self.assertIn('not measured', raw['calibration']['note'])
        self.assertEqual(engine['preprocess']['scan_line'], 18)

    def test_live_uses_raw_linear_and_does_not_copy_public_epoch(self):
        raw, sensor, mapping, engine = load(self.profile('live'), 'live')
        self.assertEqual(sensor['imu_mode'], 'raw_linear')
        self.assertEqual(sensor['gyro_topic'], '/camera/camera/gyro/sample')
        self.assertEqual(sensor['accel_topic'], '/camera/camera/accel/sample')
        self.assertEqual(sensor['raw_gyro_frame'], 'camera_gyro_optical_frame')
        self.assertEqual(sensor['raw_accel_frame'], 'camera_accel_optical_frame')
        self.assertEqual(sensor['gyro_bias'], BIAS)
        self.assertEqual(sensor['clock_offset_sec'], 0.)
        self.assertEqual(sensor['common_clock_offset_sec'], 0.)
        self.assertEqual(sensor['max_accel_gap_sec'], .03)
        self.assertIn('UNVERIFIED ZERO INITIAL', raw['calibration']['note'])
        self.assertFalse(sensor['use_sim_time'])
        self.assertFalse(mapping['use_sim_time'])
        self.assertFalse(engine['use_sim_time'])
        self.assertNotIn('max_message_age_sec', raw['sensor'])
        self.assertNotIn('clock_tolerance_sec', raw['sensor'])

    def test_sim_uses_model_installation_and_no_real_hardware_corrections(self):
        raw, sensor, mapping, engine = load(self.profile('sim'), 'sim')
        # Independently compose the published URDF installation, not the real C fit.
        t_bi = transform([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [.28, 0., .062])
        t_bl = transform(rpy_matrix(2.879, 0., 1.5705), [.25, -.038, -.03])
        np.testing.assert_allclose(transform(raw['calibration']['rotation'], raw['calibration']['translation']),
                                   np.linalg.inv(t_bi) @ t_bl, rtol=0, atol=2e-15)
        np.testing.assert_allclose(transform(mapping['T_imu_base_rotation'], mapping['T_imu_base_translation']),
                                   np.linalg.inv(t_bi), rtol=0, atol=2e-15)
        self.assertEqual(raw['calibration']['status'], 'synthetic')
        self.assertEqual(sensor['gyro_bias'], [0., 0., 0.])
        self.assertEqual(sensor['accel_offset'], [0., 0., 0.])
        self.assertEqual(sensor['clock_offset_sec'], 0.)
        self.assertEqual(sensor['common_clock_offset_sec'], 0.)
        self.assertEqual(sensor['input_cloud_topic'], '/camera_linear/points')
        self.assertEqual(sensor['input_imu_topic'], '/camera_linear/imu')
        self.assertEqual(sensor['cloud_frame'], 'lidar_l1_link')
        self.assertEqual(sensor['point_time_field'], 'time')
        self.assertEqual(sensor['imu_mode'], 'merged_linear')
        self.assertEqual(sensor['mode'], 'sim')
        self.assertIs(sensor['instantaneous_cloud'], True)
        self.assertTrue(sensor['use_sim_time'])
        self.assertTrue(mapping['use_sim_time'])
        self.assertTrue(engine['use_sim_time'])
        self.assertEqual(engine['preprocess']['scan_line'], 30)
        self.assertEqual(engine['preprocess']['scan_rate'], 10)
        self.assertEqual(engine['common']['lid_topic'], '/p9/points')
        self.assertEqual(engine['common']['imu_topic'], '/p9/imu')

    def test_new_profiles_reject_cross_environment_reuse(self):
        for intended in ('live', 'replay', 'sim'):
            for requested in ('live', 'replay', 'sim'):
                if intended != requested:
                    with self.subTest(intended=intended, requested=requested), \
                            self.assertRaisesRegex(ValueError, 'profile_mode'):
                        load(self.profile(intended), requested)

    def test_legacy_profile_has_same_engine_default_and_no_new_bias(self):
        path = ROOT/'config/replay.example.yaml'
        raw, sensor, mapping, engine = load(path, 'replay')
        self.assertEqual(estimator_config_path(path, raw), (ROOT/'config/p9-pointlio.yaml').resolve())
        self.assertNotIn('gyro_bias', sensor)
        self.assertNotIn('accel_offset', sensor)
        self.assertFalse(mapping['publish_base_frame'])
        self.assertFalse(mapping['publish_terrain_inputs'])
        self.assertEqual(engine['preprocess']['scan_line'], 18)
        self.assertEqual(engine['mapping']['extrinsic_T'], [0., 0., 0.])

    def test_explicit_engine_relative_path_resolves_at_profile_not_cwd(self):
        raw = yaml.safe_load((ROOT/'config/smoke.yaml').read_text())
        config = self.temp/'profile.yaml'
        engine_path = self.temp/'engine.yaml'
        engine_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'mapping': {'extrinsic_T': [99, 99, 99]}, 'preprocess': {'scan_line': 7}, 'use_sim_time': False}}}))
        raw['estimator'] = {'config': 'engine.yaml'}
        config.write_text(yaml.safe_dump(raw))
        _, _, _, engine = load(config, 'sim')
        self.assertEqual(estimator_config_path(config, raw), engine_path.resolve())
        self.assertEqual(engine['preprocess']['scan_line'], 7)
        self.assertEqual(engine['mapping']['extrinsic_T'], raw['calibration']['translation'])
        self.assertTrue(engine['use_sim_time'])
        self.assertFalse(yaml.safe_load(engine_path.read_text())['/**']['ros__parameters']['use_sim_time'])

    def test_bad_engine_selection_or_document_is_rejected(self):
        raw = yaml.safe_load((ROOT/'config/smoke.yaml').read_text())
        config = self.temp/'profile.yaml'
        for section in ({'config': ''}, {'config': 42}, {'config': 'missing.yaml'},
                        {'file': 'engine.yaml'}, None):
            variant = copy.deepcopy(raw)
            variant['estimator'] = section
            config.write_text(yaml.safe_dump(variant))
            with self.subTest(section=section), self.assertRaises(ValueError):
                load(config, 'sim')
        (self.temp/'bad.yaml').write_text('not_an_engine: true\n')
        raw['estimator'] = {'config': 'bad.yaml'}
        config.write_text(yaml.safe_dump(raw))
        with self.assertRaisesRegex(ValueError, 'ros__parameters'):
            load(config, 'sim')

    def test_body_reference_is_not_just_a_frame_rename(self):
        raw = yaml.safe_load((ROOT/'config/smoke.yaml').read_text())
        raw['mapping']['publish_base_frame'] = True
        config = self.temp/'profile.yaml'
        config.write_text(yaml.safe_dump(raw))
        with self.assertRaisesRegex(ValueError, 'explicit T_imu_base'):
            load(config, 'sim')
        raw['mapping'].update(T_imu_base_rotation=np.diag([1, 1, -1]).ravel().tolist(),
                              T_imu_base_translation=[0., 0., 0.])
        config.write_text(yaml.safe_dump(raw))
        with self.assertRaisesRegex(ValueError, 'proper rigid'):
            load(config, 'sim')

    def test_sim_cli_records_engine_and_has_no_bag_argument(self):
        from go2_p9 import cli
        output = self.temp/'sim-run'
        argv = ['p9', 'sim', '--config', str(self.profile('sim')), '--output', str(output)]
        with patch.object(sys, 'argv', argv), patch.object(cli.os, 'execvp') as execute, \
                contextlib.redirect_stdout(io.StringIO()):
            cli.main()
        command = execute.call_args.args[1]
        self.assertIn('mode:=sim', command)
        self.assertFalse(any(arg.startswith('bag:=') or arg.startswith('rate:=') for arg in command))
        record = json.loads((output/'effective-config.json').read_text())
        self.assertEqual(record['mode'], 'sim')
        self.assertIs(record['effective_sensor_parameters']['instantaneous_cloud'], True)
        self.assertEqual(record['effective_sensor_parameters']['mode'], 'sim')
        self.assertIsNone(record['bag'])
        self.assertIsNone(record['rate'])
        self.assertTrue(record['effective_pointlio_parameters']['use_sim_time'])
        self.assertEqual(record['effective_pointlio_parameters']['preprocess']['scan_line'], 30)
        self.assertEqual(record['estimator_config_sha256'],
                         hashlib.sha256((ROOT/'config/pointlio-sim.yaml').read_bytes()).hexdigest())

    def test_sim_launch_uses_clock_but_does_not_create_player(self):
        module = launch_module_without_ros()
        context = dict(mode='sim', config=str(self.profile('sim')), output=str(self.temp/'run'),
                       rviz='true', bag='', rate='0.5', exit_on_finish='false')
        actions = module.setup(context)
        self.assertFalse(any(a.kind in ('ExecuteProcess', 'TimerAction') for a in actions))
        nodes = [a for a in actions if a.kind == 'Node']
        self.assertEqual(len(nodes), 4)
        self.assertTrue(all(n.kwargs['parameters'][0]['use_sim_time'] for n in nodes))
        adapter = next(node for node in nodes if node.kwargs.get('executable') == 'p9_adapter')
        self.assertEqual(adapter.kwargs['parameters'][0]['mode'], 'sim')
        self.assertIs(adapter.kwargs['parameters'][0]['instantaneous_cloud'], True)

    def test_replay_player_and_live_wall_clock_remain_separate(self):
        module = launch_module_without_ros()
        common = dict(output=str(self.temp/'run'), rviz='false', bag='/input/bag', rate='0.5', exit_on_finish='false')
        actions = module.setup(dict(common, mode='replay', config=str(self.profile('replay'))))
        timers = [a for a in actions if a.kind == 'TimerAction']
        self.assertEqual(len(timers), 1)
        player = timers[0].kwargs['actions'][0]
        self.assertEqual(player.kind, 'ExecuteProcess')
        self.assertIn('--clock', player.kwargs['cmd'])
        self.assertIn('/input/bag', player.kwargs['cmd'])
        actions = module.setup(dict(common, mode='live', config=str(self.profile('live'))))
        self.assertFalse(any(a.kind in ('ExecuteProcess', 'TimerAction') for a in actions))
        nodes = [a for a in actions if a.kind == 'Node']
        self.assertTrue(all(not n.kwargs['parameters'][0]['use_sim_time'] for n in nodes))


if __name__ == '__main__':
    unittest.main()
