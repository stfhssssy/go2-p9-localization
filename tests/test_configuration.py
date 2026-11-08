"""Configuration rejection/clock contracts, runnable without ROS."""
import copy
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
from go2_p9.configuration import load


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


if __name__ == "__main__":
    unittest.main()
