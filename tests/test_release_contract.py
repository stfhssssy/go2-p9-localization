"""Static provenance checks; these do not certify ARM64 or robot execution.

Run with: python3 -m unittest discover -s tests -p test_release_contract.py -v
The hashes deliberately identify the source used by the P9 target workspace,
not the separate local development branch. Any algorithm change needs an
explicit new provenance record and a consciously updated contract.
"""
import hashlib
import json
from pathlib import Path
import re
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
POINTLIO = ROOT / "src" / "point_lio_unilidar"
P9_SOURCE_SHA256 = {
    "src/laserMapping.cpp": "aaa6fdc595a0d162df92f87e9a226cebe6ac79d9bfcc790dce7b6a06945b2ffe",
    "src/preprocess.cpp": "885db240dd192c1c9c574afa98dbdb4ff8e7e88b6b602804359cd3e8779cabee",
    "src/parameters.cpp": "39a8ef3f75da3e3a79bd2a068134e2c2796fddbd391d9aede2724ceb82470bda",
    "src/IMU_Processing.hpp": "1ae25097bc55e669061be2f9597c3e3e62d9d241902eca68c2b00b7c620c972c",
    "src/Estimator.cpp": "fc5615c59591f44416e3062957bfd748ce5218212f26d68ac4a2c1ef5d1a9ce5",
}


class ReleaseContract(unittest.TestCase):
    def test_estimator_sources_are_the_audited_p9_snapshot(self):
        for relative, expected in P9_SOURCE_SHA256.items():
            with self.subTest(file=relative):
                actual = hashlib.sha256((POINTLIO / relative).read_bytes()).hexdigest()
                self.assertEqual(actual, expected, "Unrecorded estimator source change or wrong worktree")

    def test_source_manifest_identifies_audited_originals(self):
        manifest = json.loads((ROOT / "provenance/source-manifest.json").read_text())
        self.assertFalse(manifest["checkout_clean_certified"])
        self.assertEqual(manifest["observed_checkout_head"], "43d5f54b389b251713f0097893c30fa76c870d54")
        self.assertEqual(manifest["base_archive_sha256"], "f2ec60225d0d07f9d66663a93005aeafca7e56979ed6a04809cf178578a2e99d")
        for relative, expected in P9_SOURCE_SHA256.items():
            with self.subTest(file=relative):
                self.assertEqual(manifest["original_files"]["src/point_lio_unilidar/" + relative], expected)

    def test_estimator_profile_keeps_frozen_p9_parameters(self):
        p = yaml.safe_load((ROOT / "config/p9-pointlio.yaml").read_text())["/**"]["ros__parameters"]
        expected = {
            "use_imu_as_input": False, "prop_at_freq_of_imu": True, "check_satu": True,
            "init_map_size": 10, "point_filter_num": 1, "space_down_sample": True,
            "filter_size_surf": .1, "filter_size_map": .1, "cube_side_length": 200.,
            "common": {"lid_topic": "/p9/points", "imu_topic": "/p9/imu", "con_frame": False,
                       "cut_frame": False, "time_lag_imu_to_lidar": 0.},
            "preprocess": {"lidar_type": 5, "timestamp_unit": 0, "scan_line": 18, "blind": .2},
            "mapping": {
                "imu_en": True, "start_in_aggressive_motion": False, "extrinsic_est_en": False,
                "imu_time_inte": .005, "acc_cov_output": 500., "gyr_cov_output": 1000.,
                "imu_meas_omg_cov": .01, "acc_norm": 9.81, "satu_acc": 30., "satu_gyro": 35.,
                "lidar_meas_cov": .01, "acc_cov_input": .01, "gyr_cov_input": .001,
                "b_acc_cov": .0001, "b_gyr_cov": .0001, "plane_thr": .1, "match_s": 81.,
                "fov_degree": 180., "det_range": 30., "gravity_align": True,
                "gravity": [0., 0., -9.81], "gravity_init": [0., 0., -9.81],
                "extrinsic_T": [0., 0., 0.],
                "extrinsic_R": [0.8355866054446094, 0.5483605551895937, -0.03310175695815267,
                    -0.11474335096813629, 0.23313628860067767, 0.9656507828123501,
                    0.5372420201456313, -0.8030866531387898, 0.25772628763899647],
            },
            "odometry": {"publish_odometry_without_downsample": False},
            "publish": {"path_en": False, "scan_publish_en": True, "scan_bodyframe_pub_en": False},
            "runtime_pos_log_enable": False, "pcd_save": {"pcd_save_en": False, "interval": -1},
            "use_sim_time": False,
        }
        self.assertEqual(p, expected)

    def test_live_templates_do_not_claim_historical_calibration_or_bag_clock(self):
        live = yaml.safe_load((ROOT / "config/live.example.yaml").read_text())
        experimental = yaml.safe_load((ROOT / "config/live-p9-experimental.yaml").read_text())
        replay = yaml.safe_load((ROOT / "config/replay.example.yaml").read_text())
        self.assertEqual(live["calibration"]["status"], "uncalibrated")
        self.assertIsNone(live["calibration"]["rotation"])
        self.assertIsNone(live["calibration"]["translation"])
        self.assertEqual(experimental["calibration"]["status"], "experimental")
        self.assertEqual(replay["calibration"]["status"], "experimental")
        self.assertEqual(experimental["calibration"], replay["calibration"])
        for config in (live, experimental):
            self.assertEqual(config["sensor"]["clock_offset_sec"], 0.)
            self.assertFalse(config["mapping"]["publish_base_frame"])
        self.assertEqual(replay["sensor"]["clock_offset_sec"], 2167.645092767)
        self.assertEqual(replay["sensor"]["imu_mode"], "merged_linear")

    def test_image_contract_and_runtime_entrypoint_keep_offline_arm64_scope(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        entrypoint = (ROOT / "docker/entrypoint.sh").read_text()
        self.assertRegex(dockerfile, r"(?m)^ARG BASE_IMAGE=go2-control:arm64-v4$")
        self.assertIn("dpkg --print-architecture", dockerfile)
        self.assertIn("= arm64", dockerfile)
        self.assertIn("ros-jazzy-rosbag2-storage-mcap", dockerfile)
        self.assertIn("ros-jazzy-rosbag2-storage-default-plugins", dockerfile)
        self.assertIn("python3-yaml", dockerfile)
        self.assertIn("python3-numpy", dockerfile)
        self.assertIn("SAR_ROBOT_IFACE", entrypoint)
        self.assertIn("ROS_DOMAIN_ID", entrypoint)
        # Installation is allowed at build time, not during offline startup.
        self.assertNotRegex(entrypoint, r"\b(?:apt(?:-get)?\s+install|pip\s+install|git\s+clone|curl\s|wget\s)")
        self.assertNotIn("-march=native", (POINTLIO / "CMakeLists.txt").read_text())

    def test_replay_static_tf_is_durable_for_raw_imu_frame_validation(self):
        qos = yaml.safe_load((ROOT / "config/replay-qos.yaml").read_text())
        self.assertEqual(qos['/tf_static']['durability'], 'transient_local')
        self.assertEqual(qos['/tf_static']['reliability'], 'reliable')
        for topic in ('/utlidar/cloud', '/camera/camera/imu',
                      '/camera/camera/gyro/sample', '/camera/camera/accel/sample'):
            self.assertEqual(qos[topic]['reliability'], 'reliable')

    def test_upstream_license_texts_and_embedded_notices_are_preserved(self):
        for relative in ("LICENSE", "include/IKFoM/LICENSE"):
            with self.subTest(file=relative):
                contents = (POINTLIO / relative).read_text()
                self.assertIn("GNU GENERAL PUBLIC LICENSE", contents)
                self.assertIn("Version 2, June 1991", contents)
                self.assertIn("NO WARRANTY", contents)
        header = (POINTLIO / "include/IKFoM/IKFoM_toolkit/mtk/build_manifold.hpp").read_text()
        self.assertIn("The University of Hong Kong", header)
        self.assertIn("Universitaet Bremen", header)
        self.assertIn("Redistribution and use in source and binary forms", header)
        tree_source = (POINTLIO / "include/ikd-Tree/ikd_Tree.cpp").read_text()
        self.assertIn("Author: Yixi Cai", tree_source)

    def test_no_architecture_specific_prebuilt_estimator_is_vendored(self):
        # Recompilation is mandatory: historical P9 was x86-64, this release
        # targets ARM64. Read only the magic bytes of vendored source files.
        for path in POINTLIO.rglob("*"):
            if not path.is_file():
                continue
            with path.open("rb") as handle:
                magic = handle.read(4)
            with self.subTest(file=str(path.relative_to(POINTLIO))):
                self.assertNotEqual(magic, b"\x7fELF", "Do not ship the old host binary as ARM64 source")

    def test_release_tree_contains_no_recognizable_access_credentials(self):
        # This limited static scan supplements review; it is not a general
        # proof that arbitrary private data cannot exist in a release.
        credential = re.compile(
            r"(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}"
            r"|hf_[A-Za-z0-9]{20,}|tskey-auth-[A-Za-z0-9-]{12,}"
            r"|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----)"
        )
        forbidden_names = {"id_rsa", "id_ed25519", ".git-credentials", ".netrc", ".env"}
        ignored_parts = {".git", "__pycache__", "build", "install", "log", ".pytest_cache"}
        for path in ROOT.rglob("*"):
            relative = path.relative_to(ROOT)
            if any(part in ignored_parts for part in relative.parts) or not path.is_file():
                continue
            with self.subTest(file=str(relative)):
                self.assertNotIn(path.name, forbidden_names, "Credential file must not enter release")
                try:
                    contents = path.read_text(encoding="utf-8")
                except UnicodeDecodeError:
                    continue
                self.assertIsNone(credential.search(contents), "Recognizable access secret in release")

    def test_release_documents_preserve_provenance_and_calibration_limits(self):
        provenance = (ROOT / "docs/provenance.md").read_text()
        calibration = (ROOT / "docs/calibration.md").read_text()
        deployment = (ROOT / "docs/deployment.md").read_text()
        notices = (ROOT / "THIRD_PARTY_NOTICES.md").read_text()
        self.assertIn("b7513098067ccf0d44b5c835dc34c69c4be94e0ced923c6115f09d8f81900ea7", provenance)
        self.assertIn("x86-64", provenance)
        self.assertIn("ARM64", provenance)
        self.assertIn("未标定", calibration)
        self.assertIn("merged_linear", calibration)
        self.assertIn("raw_linear", calibration)
        self.assertIn("SAR_ROBOT_IFACE", deployment)
        self.assertIn("ROS_DOMAIN_ID=0", deployment)
        self.assertIn("FORCE_RSUSB_BACKEND=ON", deployment)
        self.assertIn("go2-control:arm64-v4", deployment)
        self.assertIn("metadata discrepancy", notices)


if __name__ == "__main__":
    unittest.main()
