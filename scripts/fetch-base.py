#!/usr/bin/env python3
"""Fetch the pinned public v4 build input, verifying the entire archive."""
import argparse
import hashlib
from pathlib import Path
import urllib.request

REVISION = 'f5d01e5821f69b0eba8c3dde14197c05388d11db'
SHA256 = 'f2ec60225d0d07f9d66663a93005aeafca7e56979ed6a04809cf178578a2e99d'
URL = f'https://huggingface.co/datasets/xuanjohnny/go2-control-v4/resolve/{REVISION}/v0.4.0-arm64/go2-control-arm64-v4.tar'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    if a.output.exists():
        p.error('Output exists; verify it separately rather than replacing it')
    partial = a.output.with_name(a.output.name+'.part')
    digest = hashlib.sha256()
    with urllib.request.urlopen(URL, timeout=90) as source, partial.open('xb') as target:
        while chunk := source.read(8*1024*1024):
            target.write(chunk)
            digest.update(chunk)
    if digest.hexdigest() != SHA256:
        raise RuntimeError('Base archive SHA256 mismatch; partial file retained for diagnosis')
    partial.rename(a.output)
    print(f'Verified {SHA256}  {a.output}')


if __name__ == '__main__':
    main()
