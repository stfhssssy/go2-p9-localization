"""Configuration rejection/clock contracts, runnable without ROS."""
import copy
import ast
import contextlib
import io
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/go2_p9"))
from go2_p9.configuration import load, SENSOR_NODE_PARAMETERS, MAPPING_NODE_PARAMETERS


class ConfigurationContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "config.yaml"
        self.example = yaml.safe_load((ROOT / "config/live-p9-experimental.yaml").read_text())
        self.environment = patch.dict(os.environ, {"P9_HOME": str(ROOT)})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def read_config(self, config, mode="live"):
        self.path.write_text(yaml.safe_dump(config))
        return load(self.path, mode)

    def test_unfilled_live_template_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "explicit LiDAR"):
            load(ROOT / "config/live.example.yaml", "live")

    def test_explicit_experimental_profile_stays_experimental(self):
        raw, sensor, mapping, engine = self.read_config(self.example)
        self.assertEqual(raw["calibration"]["status"], "experimental")
        self.assertEqual(sensor["clock_offset_sec"], 0.0)
        self.assertEqual(sensor["imu_mode"], "merged_linear")
        self.assertFalse(mapping["publish_base_frame"])
        self.assertEqual(engine["mapping"]["extrinsic_R"], self.example["calibration"]["rotation"])
        self.assertEqual(engine["mapping"]["extrinsic_T"], [0.0, 0.0, 0.0])

    def test_mode_sets_every_processing_clock_without_mutating_profile(self):
        for mode, expected in (("live", False), ("replay", True), ("sim", True), ("live", False)):
            with self.subTest(mode=mode):
                raw, sensor, mapping, engine = self.read_config(self.example, mode)
                self.assertEqual(sensor["mode"], "replay" if mode == "sim" else mode)
                self.assertIs(sensor["use_sim_time"], expected)
                self.assertIs(mapping["use_sim_time"], expected)
                self.assertIs(engine["use_sim_time"], expected)
                self.assertNotIn("use_sim_time", raw["sensor"])

    def test_non_rigid_or_nonfinite_calibration_is_rejected(self):
        variants = [
            ("rotation", np.diag([1.0, 1.0, -1.0]).ravel().tolist()),
            ("rotation", (np.eye(3) * 1.01).ravel().tolist()),
            ("rotation", [float("nan")] + np.eye(3).ravel().tolist()[1:]),
            ("rotation", [1.0, 0.0, 0.0]),
            ("translation", [0.0, float("inf"), 0.0]),
        ]
        for key, value in variants:
            with self.subTest(key=key, value=value):
                config = copy.deepcopy(self.example)
                config["calibration"][key] = value
                with self.assertRaises(ValueError):
                    self.read_config(config)

    def test_measured_transform_has_declared_direction_and_is_not_inverted(self):
        config = copy.deepcopy(self.example)
        rotation = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        translation = [0.2, -0.1, 0.05]
        config["calibration"] = dict(status="measured", rotation=rotation, translation=translation)
        _, _, _, engine = self.read_config(config)
        self.assertEqual(engine["mapping"]["extrinsic_R"], rotation)
        self.assertEqual(engine["mapping"]["extrinsic_T"], translation)

    def test_user_cannot_override_entrypoint_clock_or_output_ownership(self):
        cases = [("sensor", "mode", "replay"), ("sensor", "use_sim_time", True),
                 ("sensor", "output_cloud_topic", "/other"),
                 ("mapping", "use_sim_time", True), ("mapping", "output_dir", "/other"),
                 ("mapping", "world_frame", "map")]
        for section, key, value in cases:
            with self.subTest(section=section, key=key):
                config = copy.deepcopy(self.example)
                config[section][key] = value
                with self.assertRaisesRegex(ValueError, "Entrypoint owns"):
                    self.read_config(config)

    def test_nonfinite_clock_offsets_are_rejected(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                config = copy.deepcopy(self.example)
                config["sensor"]["clock_offset_sec"] = value
                with self.assertRaisesRegex(ValueError, "finite"):
                    self.read_config(config)

    def test_unknown_sections_are_rejected(self):
        config = copy.deepcopy(self.example)
        config["sensors"] = {}
        with self.assertRaisesRegex(ValueError, "Unknown config sections"):
            self.read_config(config)

    def test_unknown_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "mode must be"):
            self.read_config(self.example, "Replay")

    def test_unknown_node_parameters_are_rejected_before_ros_can_ignore_them(self):
        for section, key, value in (("sensor", "clock_ofset_sec", 2167.),
                                    ("mapping", "voxel_szie", .1),
                                    ("calibration", "staus", "measured")):
            with self.subTest(section=section, key=key):
                config = copy.deepcopy(self.example)
                config[section][key] = value
                with self.assertRaisesRegex(ValueError, "Unknown " + section + " parameters"):
                    self.read_config(config)

    def test_sections_require_mappings(self):
        for section in ("sensor", "mapping", "calibration"):
            for value in (None, [], "not a mapping"):
                with self.subTest(section=section, value=value):
                    config = copy.deepcopy(self.example)
                    config[section] = value
                    with self.assertRaisesRegex(ValueError, "must be a mapping"):
                        self.read_config(config)

    def test_mode_qos_and_boolean_values_are_not_silently_coerced(self):
        variants = [("sensor", "imu_mode", "linear"),
                    ("sensor", "clock_offset_sec", True),
                    ("sensor", "clock_offset_sec", "0.0"),
                    ("mapping", "input_reliability", "best-effort"),
                    ("mapping", "publish_base_frame", "false")]
        for section, key, value in variants:
            with self.subTest(section=section, key=key, value=value):
                config = copy.deepcopy(self.example)
                config[section][key] = value
                with self.assertRaises(ValueError):
                    self.read_config(config)

    def test_raw_mode_and_its_options_are_accepted(self):
        config = copy.deepcopy(self.example)
        config['sensor'].update(imu_mode='raw_linear', gyro_topic='/raw/gyro', accel_topic='/raw/accel',
                                raw_gyro_frame='gyro', raw_accel_frame='accel', max_accel_gap_sec=.03,
                                raw_queue_size=4000, raw_buffer_sec=2., raw_pending_timeout_sec=2.)
        _, sensor, _, _ = self.read_config(config, 'replay')
        self.assertEqual(sensor['imu_mode'], 'raw_linear')
        self.assertEqual(sensor['gyro_topic'], '/raw/gyro')
        self.assertTrue(sensor['use_sim_time'])

    def test_native_mode_keeps_sensor_choice_calibration_and_correction_signs(self):
        config = copy.deepcopy(self.example)
        config['sensor'].update(imu_mode='native', input_imu_topic='/utlidar/imu',
                                imu_frame='utlidar_imu', common_clock_offset_sec=2245.232644025,
                                clock_offset_sec=-.001, accel_offset=[1, -2, -19.1],
                                gyro_bias=[.01, -.02, .03])
        config['calibration'].update(rotation=np.eye(3).ravel().tolist(),
                                     translation=[.007698, .014655, -.00667])
        before = copy.deepcopy(config)
        for mode in ('replay', 'live'):
            _, sensor, _, engine = self.read_config(config, mode)
            self.assertEqual(sensor['imu_mode'], 'native')
            self.assertEqual(sensor['input_imu_topic'], '/utlidar/imu')
            self.assertEqual(sensor['imu_frame'], 'utlidar_imu')
            self.assertEqual(sensor['common_clock_offset_sec'], 2245.232644025)
            self.assertEqual(sensor['clock_offset_sec'], -.001)
            self.assertEqual(sensor['accel_offset'], [1., -2., -19.1])
            self.assertEqual(sensor['gyro_bias'], [.01, -.02, .03])
            self.assertEqual(engine['mapping']['extrinsic_R'], config['calibration']['rotation'])
            self.assertEqual(engine['mapping']['extrinsic_T'], config['calibration']['translation'])
            self.assertEqual(config, before)

    def test_native_correction_fields_reject_nonfinite_or_ambiguous_types(self):
        for name in ('accel_offset', 'gyro_bias'):
            for value in (None, '123', [], [1, 2], [0, 0, 0, 0],
                          [True, 0, 0], ['1', 0, 0], [0, float('nan'), 0], [0, 0, float('inf')]):
                config = copy.deepcopy(self.example)
                config['sensor'].update(imu_mode='native', **{name: value})
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.read_config(config)
        for name in ('clock_offset_sec', 'common_clock_offset_sec'):
            for value in (True, None, '1', float('nan'), float('inf'), 1e308):
                config = copy.deepcopy(self.example)
                config['sensor'][name] = value
                with self.subTest(name=name, value=value), self.assertRaisesRegex(ValueError, 'finite'):
                    self.read_config(config)

    def test_allowed_parameter_names_match_node_declarations_without_importing_ros(self):
        for filename, expected in (('adapter_node.py', SENSOR_NODE_PARAMETERS),
                                   ('mapping_node.py', MAPPING_NODE_PARAMETERS)):
            with self.subTest(file=filename):
                tree = ast.parse((ROOT/'src/go2_p9/go2_p9'/filename).read_text())
                declarations = [node.value for node in ast.walk(tree)
                                if isinstance(node, ast.Assign)
                                and any(isinstance(t, ast.Name) and t.id == 'defaults' for t in node.targets)]
                self.assertEqual(len(declarations), 1)
                call = declarations[0]
                self.assertIsInstance(call, ast.Call)
                self.assertIsInstance(call.func, ast.Name)
                self.assertEqual(call.func.id, 'dict')
                self.assertEqual({keyword.arg for keyword in call.keywords}, expected)

    def test_cli_refuses_any_nonempty_output_before_recording_or_launching(self):
        from go2_p9 import cli
        output = Path(self.temp.name) / "prior-output"
        output.mkdir()
        prior = output / "trajectory.csv"
        prior.write_text("previous run\n")
        argv = ["p9", "live", "--config", str(ROOT / "config/live-p9-experimental.yaml"),
                "--output", str(output)]
        with patch.object(sys, "argv", argv), patch.object(cli.os, "execvp") as execute, \
                contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                cli.main()
        self.assertEqual(raised.exception.code, 2)
        execute.assert_not_called()
        self.assertEqual(prior.read_text(), "previous run\n")
        self.assertFalse((output / "effective-config.json").exists())

    def test_cli_records_experiment_delta_and_original_provenance_before_launch(self):
        from go2_p9 import cli
        output = Path(self.temp.name)/'new-experiment'
        argv = ['p9', 'live', '--config', str(ROOT/'config/live-p9-experimental.yaml'),
                '--output', str(output)]
        with patch.object(sys, 'argv', argv), patch.object(cli.os, 'execvp') as execute, \
                contextlib.redirect_stdout(io.StringIO()):
            cli.main()
        execute.assert_called_once()
        record = json.loads((output/'effective-config.json').read_text())
        delta_bytes = (ROOT/'provenance/experiment-delta.json').read_bytes()
        self.assertEqual(record['experimental_delta']['sha256'], hashlib.sha256(delta_bytes).hexdigest())
        self.assertEqual(record['experimental_delta']['manifest'], json.loads(delta_bytes))
        self.assertEqual(record['source_manifest'], json.loads((ROOT/'provenance/source-manifest.json').read_text()))
        self.assertIn('go2-native-imu-repair', record['experimental_delta']['manifest']['experiment'])

    def test_cli_version_exposes_delta_without_rewriting_original_manifest(self):
        from go2_p9 import cli
        original_path = ROOT/'provenance/source-manifest.json'
        original_bytes = original_path.read_bytes()
        stream = io.StringIO()
        with patch.object(sys, 'argv', ['p9', 'version']), contextlib.redirect_stdout(stream):
            cli.main()
        version = json.loads(stream.getvalue())
        self.assertEqual(version['source_manifest'], json.loads(original_bytes))
        self.assertEqual(version['experimental_delta'], cli.experimental_delta())
        self.assertEqual(original_path.read_bytes(), original_bytes)
        # An original release with no delta keeps its previous version format.
        old_home = Path(self.temp.name)/'original-release'
        (old_home/'provenance').mkdir(parents=True)
        (old_home/'provenance/source-manifest.json').write_bytes(original_bytes)
        stream = io.StringIO()
        with patch.object(cli, 'home', return_value=old_home), \
                patch.object(sys, 'argv', ['p9', 'version']), contextlib.redirect_stdout(stream):
            cli.main()
        self.assertEqual(json.loads(stream.getvalue()), json.loads(original_bytes))


if __name__ == "__main__":
    unittest.main()
