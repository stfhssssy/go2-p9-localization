#!/usr/bin/env python3
"""Freeze or verify the navigation release inventory; no network or credentials.

Freeze only after review of the complete source tree. This records bytes, not
algorithm correctness, hardware performance, or calibration acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tarfile


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "provenance/planning-upstream.json"
CONTRACT = ROOT / "tests/test_release_contract.py"
UPSTREAM_REVISION = "43d5f54b389b251713f0097893c30fa76c870d54"
ORIGINAL_MANIFEST_SHA256 = "5d3345a330e94542a0399b9e8d3311d24438d763bcee575058c2b12c44838b8d"
SERVER_ARCHIVE_SHA256 = "885a46f0a1c1e558fe99a5c3d85c738b21847fe054ea3bf10ee6ac3fc14efac0"
RUNTIME_SCOPE = {
    "directories": ["src", "config", "scripts", "docker", "simulation"],
    "files": ["Dockerfile", "Dockerfile.navigation"],
    "excluded_directory_names": ["__pycache__"],
}
DOCUMENT_SCOPE = {
    "directories": ["docs"],
    "files": ["README.md", "LICENSE", "THIRD_PARTY_NOTICES.md", ".dockerignore", ".gitignore"],
    "excluded_directory_names": ["__pycache__"],
}
PACKAGES = {
    "terrain_analysis": "src/base_autonomy/terrain_analysis",
    "terrain_analysis_ext": "src/base_autonomy/terrain_analysis_ext",
    "local_planner": "src/base_autonomy/local_planner",
    "far_planner": "src/route_planner/far_planner",
    "visibility_graph_msg": "src/route_planner/visibility_graph_msg",
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def inventory(scope):
    result = {}
    paths = [ROOT / p for p in scope["files"]]
    for name in scope["directories"]:
        directory = ROOT / name
        if not directory.is_dir():
            raise ValueError("Missing inventory directory: " + name)
        paths.extend(directory.rglob("*"))
    for path in paths:
        if any(p in scope["excluded_directory_names"] for p in path.relative_to(ROOT).parts):
            continue
        if path.is_symlink():
            raise ValueError("Release inventory refuses symlinks: " + str(path.relative_to(ROOT)))
        if path.is_file():
            result[path.relative_to(ROOT).as_posix()] = digest(path.read_bytes())
    for name in scope["files"]:
        if name not in result:
            raise ValueError("Missing required inventory file: " + name)
    return dict(sorted(result.items()))


def canonical_digest(value):
    return digest(json.dumps(value, sort_keys=True, separators=(",", ":")).encode())


def common_fix_inventory():
    original = ROOT / "provenance/source-manifest.json"
    if digest(original.read_bytes()) != ORIGINAL_MANIFEST_SHA256:
        raise ValueError("Original P9 source manifest must remain byte-for-byte unchanged")
    delta = json.loads((ROOT / "provenance/experiment-delta.json").read_text())
    if delta["base_release_revision"] != "42364157dd5ca0350bbbf4a749a852ad3e8ebc25":
        raise ValueError("Unexpected P9 base release")
    base = dict(delta["base_files_sha256"])
    if canonical_digest(base) != delta["base_inventory_sha256"]:
        raise ValueError("Corrupt original runtime inventory")
    for relative, change in delta["changes"].items():
        if change["original_sha256"] != base.get(relative):
            raise ValueError("Corrupt common-fix ancestry: " + relative)
        base[relative] = change["new_sha256"]
    return dict(sorted(base.items()))


def change_reason(relative):
    if relative.startswith("src/go2_p9_navigation/"):
        return "Navigation integration, body-frame bridge, isolated planner commands, and optional explicit simulation command output"
    if any(relative.startswith("src/" + name + "/") for name in PACKAGES):
        return "Vendored planning package; exact upstream and local differences are recorded in packages"
    if relative.startswith("src/go2_p9/"):
        return "P9 navigation/simulation integration and effective profile support; preserve original profile semantics"
    if relative.startswith("config/"):
        return "Explicit navigation, replay/live/simulation, B1 compensation, and visualization configuration"
    if relative.startswith("scripts/"):
        return "Offline build, navigation entrypoint, provenance, or release verification tooling"
    if relative.startswith("simulation/"):
        return "Explicit Gazebo sensor model, sensor bridge and adapter, independent of real-robot calibration"
    if relative.startswith("docker/") or relative in ("Dockerfile", "Dockerfile.navigation"):
        return "ARM64 navigation build and offline runtime entrypoints"
    raise ValueError("Unclassified runtime modification requires review: " + relative)


def package_snapshot(upstream_root, server_root, server_archive, runtime):
    def git(*args):
        return subprocess.check_output(["git", "-C", str(upstream_root), *args])
    observed = git("rev-parse", "HEAD").decode().strip()
    if observed != UPSTREAM_REVISION:
        raise ValueError("Upstream HEAD differs from the reviewed planning source")
    if digest(server_archive.read_bytes()) != SERVER_ARCHIVE_SHA256:
        raise ValueError("Server source archive differs from the reviewed snapshot")
    archived = {}
    with tarfile.open(server_archive) as archive:
        for member in archive:
            name = Path(member.name).as_posix().removeprefix("./")
            if any(name.startswith(source_path + "/") for source_path in PACKAGES.values()):
                if member.isfile():
                    archived[name] = digest(archive.extractfile(member).read())
                elif not member.isdir():
                    raise ValueError("Unexpected non-file member in server package snapshot: " + name)
    result = {}
    for package, source_path in PACKAGES.items():
        tracked = git("ls-tree", "-r", "--name-only", "HEAD", "--", source_path).decode().splitlines()
        if not tracked:
            raise ValueError("No upstream tracked files for " + package + ": " + source_path)
        upstream = {}
        for source_name in tracked:
            relative = "src/" + package + "/" + str(Path(source_name).relative_to(source_path))
            upstream[relative] = digest(git("show", "HEAD:" + source_name))
        deployed = {k: v for k, v in runtime.items() if k.startswith("src/" + package + "/")}
        if not deployed or "src/" + package + "/package.xml" not in deployed:
            raise ValueError("Vendored package is incomplete: " + package)
        differences = {}
        for relative in sorted(upstream.keys() | deployed.keys()):
            if upstream.get(relative) != deployed.get(relative):
                differences[relative] = {
                    "upstream_sha256": upstream.get(relative),
                    "release_sha256": deployed.get(relative),
                }
        server = {}
        for path in (server_root / source_path).rglob("*"):
            if path.is_symlink():
                raise ValueError("Symlink in extracted server package snapshot")
            if path.is_file():
                source_name = path.relative_to(server_root).as_posix()
                value = digest(path.read_bytes())
                if archived.get(source_name) != value:
                    raise ValueError("Extracted server file differs from archive: " + source_name)
                relative = "src/" + package + "/" + path.relative_to(server_root / source_path).as_posix()
                server[relative] = value
        archived_package = {k: v for k, v in archived.items() if k.startswith(source_path + "/")}
        if len(server) != len(archived_package) or not server:
            raise ValueError("Missing extracted server source files for " + package)
        server_differences = {
            k: {"server_sha256": server.get(k), "release_sha256": deployed.get(k)}
            for k in sorted(server.keys() | deployed.keys()) if server.get(k) != deployed.get(k)
        }
        result[package] = {
            "repository_path": source_path,
            "revision": UPSTREAM_REVISION,
            "upstream_files_sha256": dict(sorted(upstream.items())),
            "server_snapshot_files_sha256": dict(sorted(server.items())),
            "release_files_sha256": dict(sorted(deployed.items())),
            "differences_from_upstream": differences,
            "differences_from_server_snapshot": server_differences,
            "local_change_description": {
                "terrain_analysis": "Preserve audited world-frame/original-stamp, optional acknowledgement, and Jazzy compatibility changes",
                "terrain_analysis_ext": "Preserve audited world-frame/original-stamp, optional acknowledgement, and Jazzy compatibility changes",
                "local_planner": "Remove direct Unitree SportClient/API actuation and dependencies; retain path-following TwistStamped output and Jazzy compatibility",
                "far_planner": "Jazzy compatibility, corrected std_srvs metadata, empty terrain KD-tree query protection with CTest regression, and explicit far_ready graph-initialization state for guarded goal acceptance",
                "visibility_graph_msg": "Preserve FAR visibility graph message definitions and required build compatibility",
            }[package],
        }
    return result


def make_manifest(upstream_root, server_root, server_archive, server_workspace):
    runtime = inventory(RUNTIME_SCOPE)
    common = common_fix_inventory()
    deleted = common.keys() - runtime.keys()
    if deleted:
        raise ValueError("Deletion from original/common-fix runtime requires separate review: " + ", ".join(sorted(deleted)))
    changes = {}
    for relative in sorted(runtime):
        if common.get(relative) != runtime[relative]:
            changes[relative] = {
                "common_fix_sha256": common.get(relative),
                "release_sha256": runtime[relative],
                "reason": change_reason(relative),
            }
    return {
        "schema_version": 1,
        "release": "p9-navigation-20261110",
        "status": "experimental; runtime and calibration validation recorded separately",
        "upstream_repository": "https://github.com/jizhang-cmu/autonomy_stack_go2",
        "observed_checkout_head": UPSTREAM_REVISION,
        "upstream_checkout_clean_certified": False,
        "upstream_authority": "Git HEAD is only the base reference. Actual vendor input is the separately hashed server archive, which has no Git metadata.",
        "server_snapshot": {
            "host": "10.96.45.13",
            "workspace": server_workspace,
            "git_metadata_available": False,
            "clean_upstream_checkout_certified": False,
            "archive_name": server_archive.name,
            "archive_sha256": SERVER_ARCHIVE_SHA256,
            "archive_size_bytes": server_archive.stat().st_size,
            "authority": "Exact archived source bytes and per-package original hashes; server source is not represented as clean Git HEAD",
        },
        "original_source_manifest_sha256": ORIGINAL_MANIFEST_SHA256,
        "common_fix_delta_sha256": digest((ROOT / "provenance/experiment-delta.json").read_bytes()),
        "common_fix_runtime_inventory_sha256": canonical_digest(common),
        "runtime_scope": RUNTIME_SCOPE,
        "runtime_files_sha256": runtime,
        "changes_from_common_fix": changes,
        "document_scope": DOCUMENT_SCOPE,
        "document_files_sha256": inventory(DOCUMENT_SCOPE),
        "packages": package_snapshot(upstream_root, server_root, server_archive, runtime),
        "limitations": [
            "A source hash inventory is not proof of algorithm correctness or calibration accuracy.",
            "B1 is a frozen static gyro compensation candidate, not complete IMU intrinsic calibration.",
            "No real robot command output is enabled by the default navigation entrypoint.",
            "Bag replay cannot establish closed-loop navigation; target-robot validation remains separate.",
        ],
    }


def check_manifest():
    manifest = json.loads(MANIFEST.read_text())
    expected = re.search(r'^NAVIGATION_MANIFEST_SHA256 = "([0-9a-f]{64})"$', CONTRACT.read_text(), re.M)
    if expected is None or digest(MANIFEST.read_bytes()) != expected.group(1):
        raise ValueError("Navigation manifest differs from the frozen release contract")
    if manifest["runtime_scope"] != RUNTIME_SCOPE or manifest["document_scope"] != DOCUMENT_SCOPE:
        raise ValueError("Inventory scope changed")
    if manifest["runtime_files_sha256"] != inventory(RUNTIME_SCOPE):
        raise ValueError("Runtime bytes changed after freeze")
    if manifest["document_files_sha256"] != inventory(DOCUMENT_SCOPE):
        raise ValueError("Documentation/license bytes changed after freeze")
    if manifest["common_fix_runtime_inventory_sha256"] != canonical_digest(common_fix_inventory()):
        raise ValueError("Common-fix ancestry changed")
    if manifest["common_fix_delta_sha256"] != digest((ROOT / "provenance/experiment-delta.json").read_bytes()):
        raise ValueError("Common-fix delta changed")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--freeze", action="store_true", help="Explicitly record reviewed source and update the test pin")
    action.add_argument("--check", action="store_true", help="Read-only verification of the frozen source tree")
    parser.add_argument("--upstream-root", type=Path, help="Local reviewed autonomy_stack_go2 checkout; required for --freeze")
    parser.add_argument("--server-source-root", type=Path, help="Read-only extracted server archive root, containing src; required for --freeze")
    parser.add_argument("--server-archive", type=Path, help="Reviewed planner-source-reference.tar; required for --freeze")
    parser.add_argument("--server-workspace", help="Original absolute server workspace path, stored as provenance; required for --freeze")
    args = parser.parse_args()
    if args.freeze:
        if any(value is None for value in (args.upstream_root, args.server_source_root, args.server_archive, args.server_workspace)):
            parser.error("--freeze requires --upstream-root, --server-source-root, --server-archive, and --server-workspace")
        if not args.server_workspace.startswith("/"):
            parser.error("--server-workspace must be an absolute server path")
        manifest = make_manifest(args.upstream_root.resolve(), args.server_source_root.resolve(),
                                 args.server_archive.resolve(), args.server_workspace)
        payload = (json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()
        contract, count = re.subn(r'^NAVIGATION_MANIFEST_SHA256 = "[^"\n]*"$',
                                 'NAVIGATION_MANIFEST_SHA256 = "' + digest(payload) + '"',
                                 CONTRACT.read_text(), flags=re.M)
        if count != 1:
            raise ValueError("Expected exactly one navigation manifest pin in test contract")
        MANIFEST.write_bytes(payload)
        CONTRACT.write_text(contract)
    manifest = check_manifest()
    print(json.dumps({"release": manifest["release"], "verified": True,
                      "runtime_files": len(manifest["runtime_files_sha256"]),
                      "documents": len(manifest["document_files_sha256"]),
                      "manifest_sha256": digest(MANIFEST.read_bytes())}))


if __name__ == "__main__":
    main()
