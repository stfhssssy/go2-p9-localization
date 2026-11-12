#!/usr/bin/env python3
"""Package verified P9/TARE release artifacts; no export, ROS, auth or upload."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile

from release_common import (DEFAULT_ROOT, DEFAULT_REPO, DEFAULT_PREFIX, files, local_inventory,
                            reject_recognizable_secret, run_safely, safe_prefix, safe_relative, sha, write_json)

GITHUB_BRANCH = 'p9-exploration-live-20261112'
REQUIRED_SOURCE_FILES = (
    'provenance/source-manifest.json',
    'provenance/experiment-delta.json',
    'provenance/planning-upstream.json',
    'exploration/README.md',
    'exploration/runtime/p9explore',
    'exploration/runtime/cli_exploration.py',
    'exploration/control/README.md',
    'exploration/build/Dockerfile.tare',
    'exploration/build/sources.json',
    'config/p9nav-live.yaml',
)


def docker_archive_metadata(path, image_id, tag, source_commit):
    """Stream-check Docker-save metadata and image config ID, plus zstd integrity."""
    subprocess.run(['zstd', '-t', '--', str(path)], check=True, capture_output=True)
    expected = image_id.removeprefix('sha256:')
    process = subprocess.Popen(['zstd', '-dc', '--', str(path)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    manifests = []
    config_hashes = {}
    configs = {}
    try:
        with tarfile.open(fileobj=process.stdout, mode='r|') as archive:
            for member in archive:
                name = PurePosixPath(member.name).as_posix().removeprefix('./')
                safe_relative(name)
                if member.isfile() and name == 'manifest.json':
                    if member.size > 8*1024*1024:
                        raise ValueError('Unexpectedly large Docker manifest')
                    manifests = json.loads(archive.extractfile(member).read())
                elif member.isfile() and (name == expected+'.json' or name == 'blobs/sha256/'+expected):
                    if member.size > 8*1024*1024:
                        raise ValueError('Unexpectedly large Docker image config')
                    content = archive.extractfile(member).read()
                    config_hashes[name] = hashlib.sha256(content).hexdigest()
                    configs[name] = json.loads(content)
        # Tar iteration stops at end-of-archive markers; drain any trailing
        # uncompressed padding so zstd can exit normally.
        while process.stdout.read(8*1024*1024):
            pass
        if process.wait() != 0:
            raise ValueError('Docker archive decompression failed')
    finally:
        process.stdout.close()
        if process.poll() is None:
            process.kill()
            process.wait()
    matches = [item for item in manifests if config_hashes.get(item.get('Config')) == expected
               and tag in (item.get('RepoTags') or [])]
    if len(matches) != 1 or len(manifests) != 1:
        raise ValueError('Archive must contain exactly the requested image and tag')
    config = configs[matches[0]['Config']]
    if config.get('architecture') != 'arm64' or config.get('os') != 'linux':
        raise ValueError('Archived image config is not linux/arm64')
    label = (config.get('config', {}).get('Labels') or {}).get('org.opencontainers.image.revision')
    if label != source_commit:
        raise ValueError('Archived image config source revision differs from release commit')
    return {'format': 'docker-save tar compressed with zstd', 'zstd_integrity_passed': True,
            'archived_image_id_verified': True, 'archived_tag_verified': tag,
            'archived_platform_verified': 'linux/arm64', 'archived_source_label_verified': source_commit,
            'layer_count': len(matches[0].get('Layers', []))}


def source_archive_metadata(path, commit, extra_config_names=()):
    with Path(path).open('rb') as handle:
        if handle.read(2) != b'\x1f\x8b':
            raise ValueError('Source archive must be gzip-compressed git archive output')
    required = set(REQUIRED_SOURCE_FILES) | {'config/'+name for name in extra_config_names}
    found = {}
    source_prefixes = set()
    names = set()
    with tarfile.open(path, mode='r:*') as archive:
        for member in archive:
            name = PurePosixPath(member.name).as_posix().removeprefix('./')
            if name == '.':
                continue
            safe_relative(name)
            if name in names:
                raise ValueError('Duplicate source archive path')
            names.add(name)
            if not (member.isdir() or member.isfile()):
                raise ValueError('Source archive may contain only regular files/directories')
            if member.isfile():
                if member.size > 128 * 1024 * 1024:
                    raise ValueError('Source archive contains an unexpectedly large individual file')
                data = archive.extractfile(member).read()
                reject_recognizable_secret(data)
                for target in required:
                    if name == target or name.endswith('/'+target):
                        if target in found:
                            raise ValueError('Duplicate required source artifact')
                        found[target] = hashlib.sha256(data).hexdigest()
                        source_prefixes.add(name[:-len(target)])
        archived_commit = archive.pax_headers.get('comment')
    if archived_commit != commit:
        raise ValueError('Source archive must be produced by git archive of the exact release commit')
    if set(found) != required or len(source_prefixes) != 1:
        raise ValueError('Source archive is missing required release/provenance files or has mixed roots')
    return {'git_archive_commit_verified': archived_commit, 'required_source_sha256': found,
            'recognizable_access_credential_scan_passed': True}


def named_inputs(items, reserved=()):
    """Parse explicit NAME=PATH inputs without accepting path traversal/symlinks."""
    result = {}
    for item in items:
        name, path = item.split('=', 1)
        safe_relative(name)
        if name in result or name in reserved:
            raise ValueError('Duplicate/reserved release input name')
        source = Path(path)
        if source.is_symlink() or not source.is_file():
            raise ValueError('Release inputs must be regular, non-symlink files')
        if source.stat().st_size > 128 * 1024 * 1024:
            raise ValueError('Individual evidence/config file exceeds the packaging limit')
        reject_recognizable_secret(source.read_bytes())
        result[name] = source
    return result


def validate_checks(validation, image_id, source_commit):
    build = validation.get('build', {})
    if (build.get('passed') is not True or build.get('platform') != 'linux/arm64'
            or build.get('image_id') != image_id or build.get('source_commit') != source_commit):
        raise ValueError('validation.build must identify this passed ARM64 build, exact image ID and source commit')
    if not isinstance(validation.get('checks'), list) or not validation['checks']:
        raise ValueError('validation must list performed checks and their actual outcomes')
    names = set()
    for check in validation['checks']:
        if (not isinstance(check, dict) or not isinstance(check.get('name'), str) or not check['name']
                or check['name'] in names or check.get('status') not in {'passed', 'failed', 'not_run', 'partial'}):
            raise ValueError('Every validation check needs a unique name and actual outcome status')
        names.add(check['name'])
    if validation.get('hardware_validated') is not False:
        raise ValueError('This experimental release must explicitly state hardware_validated=false')
    if validation.get('automatic_return_accepted') is not False:
        raise ValueError('This release must explicitly state automatic_return_accepted=false')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dist', type=Path, default=DEFAULT_ROOT/'dist')
    parser.add_argument('--source-commit', required=True)
    parser.add_argument('--image-id', required=True)
    parser.add_argument('--image-tag', default='go2-p9-explore:arm64-20261112')
    parser.add_argument('--archive', type=Path, required=True, help='docker save | zstd output')
    parser.add_argument('--source-archive', type=Path, required=True, help='git archive --format=tar.gz COMMIT output')
    parser.add_argument('--image-inspect', type=Path, required=True, help='docker image inspect JSON')
    parser.add_argument('--validation-json', type=Path, required=True)
    parser.add_argument('--deployment-readme', type=Path, required=True,
                        help='Exact exploration/README.md from the source commit')
    parser.add_argument('--calibration-config', action='append', required=True, metavar='NAME=PATH',
                        help='Tracked config/NAME copied to configs/NAME; p9nav-live.yaml is mandatory')
    parser.add_argument('--evidence', action='append', default=[], metavar='NAME=PATH')
    parser.add_argument('--repo', default=DEFAULT_REPO)
    parser.add_argument('--prefix', default=DEFAULT_PREFIX)
    args = parser.parse_args()
    safe_prefix(args.prefix)
    if not re.fullmatch('[0-9a-f]{40}', args.source_commit) or not re.fullmatch('sha256:[0-9a-f]{64}', args.image_id):
        parser.error('Expected full Git commit and sha256 image ID')
    if args.dist.exists() and (not args.dist.is_dir() or any(args.dist.iterdir())):
        raise ValueError('dist must be absent or empty; old release files will not be overwritten')
    for path in (args.archive, args.source_archive, args.image_inspect, args.validation_json, args.deployment_readme):
        if path.is_symlink() or not path.is_file():
            raise ValueError('Release inputs must be regular non-symlink files')
    image = json.loads(args.image_inspect.read_text())
    if isinstance(image, list):
        if len(image) != 1:
            raise ValueError('Expected exactly one inspected image')
        image = image[0]
    if image.get('Id') != args.image_id or image.get('Architecture') != 'arm64' or image.get('Os') != 'linux':
        raise ValueError('Inspected image ID/platform differs from requested ARM64 release')
    if args.image_tag not in (image.get('RepoTags') or []):
        raise ValueError('Inspected image is missing the requested tag')
    label = (image.get('Config', {}).get('Labels') or {}).get('org.opencontainers.image.revision')
    if label != args.source_commit:
        raise ValueError('Image source-revision label differs from release commit')
    validation = json.loads(args.validation_json.read_text())
    validate_checks(validation, args.image_id, args.source_commit)
    reject_recognizable_secret(args.image_inspect.read_bytes())
    reject_recognizable_secret(args.validation_json.read_bytes())
    reject_recognizable_secret(args.deployment_readme.read_bytes())
    extra = named_inputs(args.evidence, reserved={'image-inspect.json'})
    if not extra:
        raise ValueError('Provide at least one build/test evidence file')
    configs = named_inputs(args.calibration_config)
    if 'p9nav-live.yaml' not in configs:
        raise ValueError('The exact calibrated p9nav-live.yaml must be packaged')
    archive_meta = docker_archive_metadata(args.archive, args.image_id, args.image_tag, args.source_commit)
    source_meta = source_archive_metadata(args.source_archive, args.source_commit, configs)
    tracked_hashes = source_meta['required_source_sha256']
    if sha(args.deployment_readme) != tracked_hashes['exploration/README.md']:
        raise ValueError('Deployment README differs from the exact committed source')
    for name, path in configs.items():
        if sha(path) != tracked_hashes['config/'+name]:
            raise ValueError('Calibration configuration differs from the exact committed source: '+name)
    args.dist.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.p9-explore-dist-', dir=args.dist.parent))
    try:
        archive_name = 'go2-p9-explore-arm64-20261112.tar.zst'
        source_name = 'go2-p9-explore-source-'+args.source_commit[:12]+'.tar.gz'
        shutil.copyfile(args.archive, stage/archive_name)
        shutil.copyfile(args.source_archive, stage/source_name)
        shutil.copyfile(args.validation_json, stage/'validation.json')
        shutil.copyfile(args.deployment_readme, stage/'DEPLOYMENT.md')
        config_manifest = {}
        for name, path in configs.items():
            destination = stage/'configs'/name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, destination)
            config_manifest[name] = {'path': 'configs/'+name, 'sha256': sha(destination),
                                     'source_path': 'config/'+name,
                                     'matches_git_source': True,
                                     'default_image_path': '/opt/p9/config/'+name}
        write_json(stage/'calibration-settings.json', {
            'source_commit': args.source_commit,
            'configs': config_manifest,
            'selection': 'C LiDAR-to-D435i IMU R/t plus B1 static gyroscope correction; inherited P9 profile',
            'application': 'Adapter correction applies once; no full six-position intrinsic calibration claim',
            'live_clock_offset': 'Current-boot verification is required; public-bag offset is not a live setting',
            'hardware_validated_in_this_release': False,
        })
        (stage/'evidence').mkdir()
        shutil.copyfile(args.image_inspect, stage/'evidence/image-inspect.json')
        for name, path in extra.items():
            destination = stage/'evidence'/name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, destination)
        release = {
            'format_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'experimental-attended-exploration-release',
            'github': {'url': 'https://github.com/stfhssssy/go2-p9-localization',
                       'branch': GITHUB_BRANCH, 'commit': args.source_commit},
            'huggingface': {'repo_type': 'dataset', 'repo_id': args.repo, 'prefix': args.prefix,
                            'visibility_policy': 'preserve the existing repository visibility'},
            'image': {'tag': args.image_tag, 'id': args.image_id, 'platform': 'linux/arm64',
                      'image_size_bytes': image.get('Size'), 'archive': archive_name,
                      'archive_sha256': sha(stage/archive_name), 'archive_size_bytes': (stage/archive_name).stat().st_size,
                      **archive_meta},
            'source_archive': {'path': source_name, 'sha256': sha(stage/source_name),
                               'size_bytes': (stage/source_name).stat().st_size, **source_meta},
            'entrypoints': ['p9explore live --config PATH --output NEW_PATH --boundary XMIN XMAX YMIN YMAX [--rviz]',
                            'p9explore replay --bag PATH --config PATH --output NEW_PATH --boundary XMIN XMAX YMIN YMAX [--rviz]',
                            'p9explore sim --config PATH --output NEW_PATH --boundary XMIN XMAX YMIN YMAX [--rviz] [--sim-command-output]',
                            'p9explore status', 'p9explore start', 'p9explore stop',
                            'p9explore-control', 'p9explore-control --enable-motion'],
            'default_robot_actuation_enabled': False,
            'hardware_validated': False,
            'automatic_return_accepted': False,
            'live_completion_policy': 'Stop and latch on exploration completion; autonomous return is disabled in live',
            'inherited_commands': ['p9', 'p9nav'],
            'calibration_settings': 'calibration-settings.json',
            'deployment_instructions': 'DEPLOYMENT.md',
            'previous_release_preservation_policy': {
                'github_old_branches': 'No edits or force push to original P9/navigation branches',
                'old_image_tags': ['go2-p9:arm64', 'go2-p9-nav:arm64-20261110'],
                'huggingface': 'Upload only the new release prefix; upload helper verifies outside-prefix file identities',
            },
            'validation': 'validation.json', 'checksums': 'SHA256SUMS',
            'limitations': [
                'B1 is a frozen static gyro-bias candidate, not full D435i intrinsic calibration.',
                'C R/t and nominal body extrinsics remain experimental; no ground-truth accuracy claim.',
                'Live clocks and sensor availability require current-device verification.',
                'ARM64 execution under QEMU is not real-device performance validation.',
                'Replay, protocol mocks and simulation do not establish real-robot closed-loop safety or performance.',
                'TARE return-home was not accepted in simulation; live stops on completion rather than returning home.',
                'An external working RealSense driver is required; live deployment does not bundle Gazebo/CHAMP.',
                'Optional motion bridge requires explicit enablement, acknowledgment, operator deadman input and independent stopping means.',
            ],
        }
        write_json(stage/'release.json', release)
        readme = f'''# P9 calibrated localization, mapping and TARE exploration

这是用于现场有人监护测试的独立实验版本。源码：[{GITHUB_BRANCH}](https://github.com/stfhssssy/go2-p9-localization/tree/{args.source_commit}/exploration)。

- 源码提交：`{args.source_commit}`
- 镜像：`{args.image_tag}`，`linux/arm64`
- Image ID：`{args.image_id}`
- 部署范围：已选 C 外参＋B1 陀螺仪补偿、Point-LIO 定位、地图累积、地形分析、TARE 探索与局部规划。
- 默认不发送真狗动作；可选运动桥需要单独显式启用和操作员持续许可。
- **未做本版本真机闭环验收，自动返航未通过。Live 探索完成后停车锁定，不自动返航。**

下载本目录全部文件，在本目录核验并导入：

```bash
sha256sum -c SHA256SUMS
zstd -dc {archive_name} | docker load
docker image inspect {args.image_tag} --format '{{{{.Id}}}} {{{{.Os}}}}/{{{{.Architecture}}}}'
```

核对 Image ID 和平台与 `release.json` 相同。按本目录的 **DEPLOYMENT.md** 分阶段进行静止检查、仅规划检查及有监护运动测试。不要同时运行 FAR 和 TARE。Live、replay、sim 的时钟与外参不能混用；相机驱动由外部提供。

`configs/` 提供与此提交完全一致的标定配置，哈希见 `calibration-settings.json`。原 P9/FAR 命令与配置保留。实时钟差、话题、外参安装关系仍需当场检查，不能套用公开包约 2167 秒的钟差。

`validation.json` 逐项记录本版本实际执行的检查、失败和未执行项；`evidence/` 保存证据。旧版本检查不代表本版本验收。B1 不是完整六面 IMU 内参标定，构建、QEMU、协议模拟和 Gazebo 验证都不能替代真机测试。

源码归档 `{source_name}` 来自准确的 Git 提交，包含原 P9 与新 exploration 目录。镜像及第三方源码保留上游声明和现存许可证文件，详见源码 `exploration/build/BUILDING.md`。发布工具只写新的版本目录并验证其它目录不变。
'''
        (stage/'README.md').write_text(readme)
        entries = files(stage)
        (stage/'SHA256SUMS').write_text(''.join(f'{sha(path)}  {name}\n' for name, path in entries.items()))
        # Use the same verifier as the upload helper before making this dist visible.
        verified_release, verified_inventory = local_inventory(stage)
        if verified_release['github']['commit'] != args.source_commit or not verified_inventory:
            raise ValueError('Prepared release failed its final shared-manifest check')
        if args.dist.exists():
            args.dist.rmdir()
        stage.rename(args.dist)
    except BaseException:
        # Preserve the staging directory to diagnose large-copy/build issues.
        raise
    print(json.dumps({'prepared': True, 'dist': str(args.dist), 'image_id': args.image_id,
                      'source_commit': args.source_commit, 'file_count': len(files(args.dist))}))


if __name__ == '__main__':
    run_safely(main)
