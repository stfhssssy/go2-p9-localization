"""Static provenance checks; these do not certify ARM64 or robot execution.

Run with: python3 -m unittest discover -s tests -p test_release_contract.py -v
The hashes deliberately identify the source used by the P9 target workspace,
not the separate local development branch. Any algorithm change needs an
explicit new provenance record and a consciously updated contract.
"""
import hashlib
import json
import copy
from pathlib import Path
import re
import subprocess
import sys
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
BASE_INVENTORY_SHA256 = "43eeb4740a4a0c2a3a4f78f15546de932b28b0f48ee7c7db0007fd9a02ddef1e"
ORIGINAL_MANIFEST_SHA256 = "5d3345a330e94542a0399b9e8d3311d24438d763bcee575058c2b12c44838b8d"
COMMON_FIX_DELTA_SHA256 = "1b0a93225c8df2774e98ddff3c46367b19bff346b1f464adee9bc23cde868c9f"
# Updated only by an explicit freeze of the complete reviewed release tree.
NAVIGATION_MANIFEST_SHA256 = "07309109e5ffe4b2729dc20860a095c9a2a86994b6818428beefdc0888bf27ae"
EXPERIMENT_FILES = frozenset({
    "config/replay-qos.yaml",
    "src/go2_p9/go2_p9/adapter_node.py",
    "src/go2_p9/go2_p9/cli.py",
    "src/go2_p9/go2_p9/configuration.py",
    "src/go2_p9/go2_p9/sensor_core.py",
    "src/point_lio_unilidar/include/gravity_alignment.h",
    "src/point_lio_unilidar/src/IMU_Processing.hpp",
    "src/point_lio_unilidar/src/laserMapping.cpp",
    "src/point_lio_unilidar/test/CMakeLists.txt",
    "src/point_lio_unilidar/test/gravity_alignment_test.cpp",
})


def runtime_inventory():
    files = {}
    for directory in ('src', 'config', 'scripts', 'docker', 'simulation'):
        for path in (ROOT/directory).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                files[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    files['Dockerfile'] = hashlib.sha256((ROOT/'Dockerfile').read_bytes()).hexdigest()
    files['Dockerfile.navigation'] = hashlib.sha256((ROOT/'Dockerfile.navigation').read_bytes()).hexdigest()
    return files


def verify_experiment_delta(delta, current):
    """Reject unlisted changes/additions/deletions, including config and launch."""
    def require(condition, message):
        if not condition:
            raise ValueError(message)
    require(delta['schema_version'] == 1, 'unknown delta schema')
    require(delta['base_release_revision'] == '42364157dd5ca0350bbbf4a749a852ad3e8ebc25',
            'wrong base release')
    require(delta['scope'] == dict(directories=['src', 'config', 'scripts', 'docker'],
                                   files=['Dockerfile'], excluded_directory_names=['__pycache__']),
            'runtime scope cannot be relaxed')
    base = delta['base_files_sha256']
    base_hash = hashlib.sha256(json.dumps(base, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    require(base_hash == BASE_INVENTORY_SHA256 == delta['base_inventory_sha256'], 'baseline inventory changed')
    require(delta['original_source_manifest_sha256'] == ORIGINAL_MANIFEST_SHA256, 'original provenance changed')
    require(set(delta['changes']) == EXPERIMENT_FILES, 'unapproved delta file set')
    expected = dict(base)
    for relative, change in delta['changes'].items():
        require(change['original_sha256'] == base.get(relative), 'wrong original source hash: '+relative)
        require(isinstance(change['new_sha256'], str)
                and re.fullmatch('[0-9a-f]{64}', change['new_sha256']) is not None,
                'invalid replacement source hash: '+relative)
        require(bool(change['reason']), 'missing change rationale')
        expected[relative] = change['new_sha256']
    require(current == expected, 'unrecorded runtime file change/addition/deletion')


def common_fix_inventory(delta):
    """Reconstruct the immutable prior experiment, before navigation additions."""
    expected = dict(delta['base_files_sha256'])
    for relative, change in delta['changes'].items():
        expected[relative] = change['new_sha256']
    verify_experiment_delta(delta, expected)
    return expected


def verify_navigation_inventory(manifest, common, current):
    """Require complete exact final inventory and explicitly declared ancestry."""
    def require(condition, message):
        if not condition:
            raise ValueError(message)
    require(manifest['schema_version'] == 1, 'unknown navigation schema')
    require(manifest['release'] == 'p9-navigation-20261110', 'wrong navigation release')
    require(manifest['original_source_manifest_sha256'] == ORIGINAL_MANIFEST_SHA256,
            'original navigation ancestry changed')
    require(manifest['common_fix_delta_sha256'] == COMMON_FIX_DELTA_SHA256,
            'common-fix delta changed')
    common_hash = hashlib.sha256(json.dumps(common, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    require(manifest['common_fix_runtime_inventory_sha256'] == common_hash,
            'common-fix inventory changed')
    require(manifest['runtime_scope'] == dict(directories=['src', 'config', 'scripts', 'docker', 'simulation'],
                                             files=['Dockerfile', 'Dockerfile.navigation'], excluded_directory_names=['__pycache__']),
            'navigation runtime scope cannot be relaxed')
    frozen = manifest['runtime_files_sha256']
    require(not (common.keys() - frozen.keys()), 'original/common-fix file deleted')
    expected_changes = {key for key in frozen if frozen[key] != common.get(key)}
    require(set(manifest['changes_from_common_fix']) == expected_changes,
            'incomplete navigation change inventory')
    for relative in expected_changes:
        change = manifest['changes_from_common_fix'][relative]
        require(change['common_fix_sha256'] == common.get(relative), 'wrong navigation ancestor: ' + relative)
        require(change['release_sha256'] == frozen[relative], 'wrong navigation replacement: ' + relative)
        require(bool(change['reason']), 'missing navigation rationale: ' + relative)
    for relative, sha in frozen.items():
        require(re.fullmatch('[0-9a-f]{64}', sha) is not None, 'invalid final source hash: ' + relative)
    require(current == frozen, 'unrecorded navigation runtime change/addition/deletion')


class ReleaseContract(unittest.TestCase):
    def test_estimator_sources_are_the_audited_p9_snapshot_plus_declared_delta(self):
        delta = json.loads((ROOT/'provenance/experiment-delta.json').read_text())
        common = common_fix_inventory(delta)
        navigation = json.loads((ROOT/'provenance/planning-upstream.json').read_text())
        verify_navigation_inventory(navigation, common, runtime_inventory())
        for relative, expected in P9_SOURCE_SHA256.items():
            with self.subTest(file=relative):
                change = delta['changes'].get('src/point_lio_unilidar/'+relative)
                if change:
                    self.assertEqual(change['original_sha256'], expected)
                    expected = change['new_sha256']
                actual = hashlib.sha256((POINTLIO / relative).read_bytes()).hexdigest()
                self.assertEqual(actual, expected, "Unrecorded estimator source change or wrong worktree")

    def test_experiment_delta_cannot_hide_other_runtime_changes(self):
        delta = json.loads((ROOT/'provenance/experiment-delta.json').read_text())
        current = common_fix_inventory(delta)
        verify_experiment_delta(delta, current)
        variants = []
        altered = dict(current)
        altered['src/point_lio_unilidar/src/Estimator.cpp'] = '0'*64
        variants.append(altered)
        added = dict(current)
        added['src/go2_p9/go2_p9/unreviewed.py'] = '0'*64
        variants.append(added)
        deleted = dict(current)
        del deleted['config/p9-pointlio.yaml']
        variants.append(deleted)
        for value in variants:
            with self.assertRaisesRegex(ValueError, 'unrecorded runtime'):
                verify_experiment_delta(delta, value)
        expanded = copy.deepcopy(delta)
        expanded['changes']['src/point_lio_unilidar/src/Estimator.cpp'] = dict(
            original_sha256=P9_SOURCE_SHA256['src/Estimator.cpp'], new_sha256='0'*64, reason='not authorized')
        with self.assertRaisesRegex(ValueError, 'unapproved delta'):
            verify_experiment_delta(expanded, current)

    def test_navigation_manifest_and_original_common_fix_are_pinned(self):
        self.assertEqual(hashlib.sha256((ROOT/'provenance/experiment-delta.json').read_bytes()).hexdigest(),
                         COMMON_FIX_DELTA_SHA256)
        self.assertEqual(hashlib.sha256((ROOT/'provenance/planning-upstream.json').read_bytes()).hexdigest(),
                         NAVIGATION_MANIFEST_SHA256,
                         'Freeze the reviewed complete release; do not edit the manifest without updating its contract')
        # The generator is included in the runtime inventory. Its read-only
        # check also covers all user documentation and license inventories.
        subprocess.run([sys.executable, str(ROOT/'scripts/freeze-navigation-provenance.py'), '--check'],
                       check=True, capture_output=True, text=True)

    def test_navigation_inventory_rejects_unlisted_changes_additions_and_deletions(self):
        delta = json.loads((ROOT/'provenance/experiment-delta.json').read_text())
        common = common_fix_inventory(delta)
        navigation = json.loads((ROOT/'provenance/planning-upstream.json').read_text())
        current = runtime_inventory()
        verify_navigation_inventory(navigation, common, current)
        changed = dict(current)
        changed['src/point_lio_unilidar/src/Estimator.cpp'] = '0'*64
        added = dict(current)
        added['src/go2_p9_navigation/unrecorded.py'] = '0'*64
        deleted = dict(current)
        del deleted['config/p9-pointlio.yaml']
        for value in (changed, added, deleted):
            with self.assertRaisesRegex(ValueError, 'unrecorded navigation runtime'):
                verify_navigation_inventory(navigation, common, value)
        hidden = copy.deepcopy(navigation)
        hidden['changes_from_common_fix'].pop(next(iter(hidden['changes_from_common_fix'])))
        with self.assertRaisesRegex(ValueError, 'incomplete navigation change inventory'):
            verify_navigation_inventory(hidden, common, current)

    def test_planning_package_sources_and_local_deltas_are_complete(self):
        navigation = json.loads((ROOT/'provenance/planning-upstream.json').read_text())
        expected = {'terrain_analysis', 'terrain_analysis_ext', 'local_planner', 'far_planner', 'visibility_graph_msg'}
        self.assertEqual(set(navigation['packages']), expected)
        self.assertEqual(navigation['observed_checkout_head'], '43d5f54b389b251713f0097893c30fa76c870d54')
        current = runtime_inventory()
        for name, package in navigation['packages'].items():
            with self.subTest(package=name):
                self.assertEqual(package['revision'], navigation['observed_checkout_head'])
                deployed = {k: v for k, v in current.items() if k.startswith('src/'+name+'/')}
                self.assertEqual(package['release_files_sha256'], deployed)
                original = package['upstream_files_sha256']
                self.assertIn('src/'+name+'/package.xml', original)
                differences = {k: {'upstream_sha256': original.get(k), 'release_sha256': deployed.get(k)}
                               for k in original.keys() | deployed.keys() if original.get(k) != deployed.get(k)}
                self.assertEqual(package['differences_from_upstream'], differences)
                server = package['server_snapshot_files_sha256']
                self.assertIn('src/'+name+'/package.xml', server)
                server_differences = {k: {'server_sha256': server.get(k), 'release_sha256': deployed.get(k)}
                                      for k in server.keys() | deployed.keys() if server.get(k) != deployed.get(k)}
                self.assertEqual(package['differences_from_server_snapshot'], server_differences)
                self.assertTrue(package['local_change_description'])
        self.assertEqual(navigation['server_snapshot']['archive_sha256'],
                         '885a46f0a1c1e558fe99a5c3d85c738b21847fe054ea3bf10ee6ac3fc14efac0')
        self.assertFalse(navigation['server_snapshot']['git_metadata_available'])
        self.assertFalse(navigation['server_snapshot']['clean_upstream_checkout_certified'])

    def test_source_manifest_identifies_audited_originals(self):
        self.assertEqual(hashlib.sha256((ROOT/'provenance/source-manifest.json').read_bytes()).hexdigest(),
                         ORIGINAL_MANIFEST_SHA256, 'Preserve the original source manifest byte-for-byte')
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
        for topic in ('/utlidar/cloud', '/utlidar/imu', '/camera/camera/imu',
                      '/camera/camera/gyro/sample', '/camera/camera/accel/sample'):
            self.assertEqual(qos[topic]['reliability'], 'reliable')
        self.assertEqual(qos['/utlidar/imu']['depth'], 4000)

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
