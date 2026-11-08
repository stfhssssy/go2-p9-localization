"""Configuration rejection/clock contracts, runnable without ROS."""
import copy
import ast
import contextlib
import io
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
        for mode, expected in (("live", False), ("replay", True), ("live", False)):
            with self.subTest(mode=mode):
                raw, sensor, mapping, engine = self.read_config(self.example, mode)
                self.assertEqual(sensor["mode"], mode)
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


if __name__ == "__main__":
    unittest.main()
