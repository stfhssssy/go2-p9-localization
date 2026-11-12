#!/usr/bin/env python3
"""Upload only a new release subdirectory; preserve all other files/visibility."""
import argparse
import json
from pathlib import Path

from release_common import (DEFAULT_ROOT, DEFAULT_REPO, DEFAULT_PREFIX, api_client, info_snapshot,
                            local_inventory, outside_prefix, run_safely, safe_prefix, verify_remote, write_json)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dist', type=Path, default=DEFAULT_ROOT/'dist')
    parser.add_argument('--repo', default=DEFAULT_REPO)
    parser.add_argument('--prefix', default=DEFAULT_PREFIX)
    parser.add_argument('--report', type=Path, default=DEFAULT_ROOT/'evidence/huggingface-exploration-published.json')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    safe_prefix(args.prefix)
    if args.report.resolve().is_relative_to(args.dist.resolve()):
        raise ValueError('Publication reports must stay outside the immutable dist directory')
    release, expected = local_inventory(args.dist)
    if release['huggingface']['repo_id'] != args.repo or release['huggingface']['prefix'] != args.prefix:
        raise ValueError('Requested destination differs from the prepared release manifest')
    api = api_client(args.repo)
    before = info_snapshot(api, args.repo)
    baseline_path = args.report.with_name(args.report.stem+'-before.json')
    if baseline_path.exists():
        raise ValueError('Baseline report exists; choose a new --report path for this attempt')
    write_json(baseline_path, before)
    exists = any(name.startswith(args.prefix+'/') for name in before['files'])
    if exists:
        # A complete matching release is idempotent. Different or partial
        # contents require a new version directory, never silent overwriting.
        verified = verify_remote(before, args.dist, expected, args.prefix)
        result = {'repo': args.repo, 'prefix': args.prefix, 'private': before['private'],
                  'commit': before['commit'], 'already_published_identical': True,
                  'all_uploaded_files_content_verified': True, 'outside_prefix_preserved': True,
                  'visibility_preserved': True, 'files': verified}
        write_json(args.report, result)
        print(json.dumps({key: value for key, value in result.items() if key != 'files'}))
        return
    if args.dry_run:
        print(json.dumps({'dry_run': True, 'repo': args.repo, 'prefix': args.prefix,
                          'private': before['private'], 'parent_commit': before['commit'],
                          'files': len(expected), 'bytes': sum(item['size_bytes'] for item in expected.values()),
                          'baseline': str(baseline_path)}))
        return
    commit = api.upload_folder(
        repo_id=args.repo, repo_type='dataset', folder_path=str(args.dist),
        path_in_repo=args.prefix, allow_patterns=list(expected),
        commit_message='Add calibrated P9 and TARE exploration ARM64 release 20261112',
        parent_commit=before['commit'],
    )
    after = info_snapshot(api, args.repo)
    verified = verify_remote(after, args.dist, expected, args.prefix)
    if after['private'] != before['private']:
        raise ValueError('Repository visibility changed during publication')
    if outside_prefix(after, args.prefix) != outside_prefix(before, args.prefix):
        raise ValueError('Files outside the new release prefix changed during publication')
    result = {'repo': args.repo, 'prefix': args.prefix, 'private': after['private'],
              'commit': after['commit'], 'upload_commit': getattr(commit, 'oid', None),
              'source_commit': release['github']['commit'], 'image_id': release['image']['id'],
              'all_uploaded_files_content_verified': True, 'outside_prefix_preserved': True,
              'visibility_preserved': True, 'baseline': str(baseline_path), 'files': verified}
    write_json(args.report, result)
    print(json.dumps({key: value for key, value in result.items() if key != 'files'}))


if __name__ == '__main__':
    run_safely(main)
