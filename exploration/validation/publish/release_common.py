"""Release helpers. Credentials are handled only by huggingface_hub itself."""
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re

DEFAULT_ROOT = Path('/mnt/hdd/ssy/p9-exploration-release-20261112')
DEFAULT_REPO = 'sssssssssssssssy/go2-p9-arm64'
DEFAULT_PREFIX = 'releases/p9-explore-20261112'
SECRET = re.compile(rb'(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|hf_[A-Za-z0-9]{20,}|tskey-auth-[A-Za-z0-9-]{12,}|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----)')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for data in iter(lambda: handle.read(8 * 1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def safe_relative(name):
    p = PurePosixPath(name)
    if not name or p.is_absolute() or '..' in p.parts or '.' in p.parts or '\\' in name or '\n' in name or '\r' in name:
        raise ValueError('Unsafe relative release path')
    if p.as_posix() != name or any(part == '.git' for part in p.parts):
        raise ValueError('Non-canonical release path')
    return name


def safe_prefix(prefix):
    safe_relative(prefix)
    if not prefix.startswith('releases/') or len(PurePosixPath(prefix).parts) < 2:
        raise ValueError('Publication must use a named releases/ subdirectory')
    return prefix


def reject_recognizable_secret(data):
    if SECRET.search(data):
        raise ValueError('Recognizable access credential found; content suppressed')


def files(root):
    result = {}
    for path in sorted(Path(root).rglob('*')):
        if path.is_symlink():
            raise ValueError('Release files may not be symlinks')
        if path.is_file():
            name = safe_relative(path.relative_to(root).as_posix())
            result[name] = path
    return result


def local_inventory(dist):
    dist = Path(dist)
    listed = {}
    for line in (dist/'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        safe_relative(name)
        if not re.fullmatch('[0-9a-f]{64}', digest) or name in listed or name == 'SHA256SUMS':
            raise ValueError('Invalid checksum manifest entry')
        listed[name] = digest
    actual = files(dist)
    if set(actual) != set(listed) | {'SHA256SUMS'}:
        raise ValueError('Local release file set differs from SHA256SUMS')
    result = {}
    for name, path in actual.items():
        digest = sha(path)
        if name != 'SHA256SUMS' and digest != listed[name]:
            raise ValueError('Local release checksum mismatch: ' + name)
        result[name] = {'sha256': digest, 'size_bytes': path.stat().st_size}
    release = json.loads((dist/'release.json').read_text())
    return release, result


def api_client(repo):
    os.environ['HF_DEBUG'] = '0'
    os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
    from huggingface_hub import HfApi
    from huggingface_hub.utils import logging
    logging.set_verbosity_warning()
    api = HfApi()
    if api.whoami()['name'] != repo.split('/')[0]:
        raise ValueError('Logged-in HF account does not own the requested release repository')
    return api


def info_snapshot(api, repo):
    info = api.repo_info(repo, repo_type='dataset', files_metadata=True)
    if info.id != repo:
        raise ValueError('Unexpected HF repository identity')
    result = {}
    for item in info.siblings:
        lfs = item.lfs
        lfs_sha = (lfs.get('sha256') if isinstance(lfs, dict) else getattr(lfs, 'sha256', None)) if lfs else None
        result[item.rfilename] = {'size_bytes': item.size, 'blob_id': item.blob_id, 'lfs_sha256': lfs_sha}
    return {'repo': repo, 'commit': info.sha, 'private': info.private, 'files': result}


def outside_prefix(snapshot, prefix):
    return {name: item for name, item in snapshot['files'].items() if not name.startswith(prefix + '/')}


def verify_remote(snapshot, dist, expected, prefix):
    actual_names = {name[len(prefix)+1:] for name in snapshot['files'] if name.startswith(prefix + '/')}
    if actual_names != set(expected):
        raise ValueError('Remote release-prefix file set differs from the local manifest')
    checked = []
    for name, entry in expected.items():
        item = snapshot['files'][prefix + '/' + name]
        if item['size_bytes'] != entry['size_bytes']:
            raise ValueError('Remote file size mismatch: ' + name)
        if item['lfs_sha256']:
            if item['lfs_sha256'] != entry['sha256']:
                raise ValueError('Remote LFS SHA256 mismatch: ' + name)
            method = 'remote LFS SHA256 and size'
        else:
            path = Path(dist)/name
            h = hashlib.sha1(('blob ' + str(path.stat().st_size)).encode() + b'\0')
            with path.open('rb') as handle:
                for data in iter(lambda: handle.read(8*1024*1024), b''):
                    h.update(data)
            if item['blob_id'] != h.hexdigest():
                raise ValueError('Remote Git blob ID mismatch: ' + name)
            method = 'remote Git blob ID and size; local SHA256'
        checked.append({'file': prefix+'/'+name, **entry, 'verified_by': method})
    return checked


def run_safely(main):
    try:
        main()
    except Exception as error:
        # Do not echo third-party request exceptions, signed URLs, or headers.
        print(json.dumps({'error': type(error).__name__, 'details': 'suppressed; operation did not pass verification'}))
        raise SystemExit(1)
