#!/usr/bin/env python3
"""Fetch immutable official sources/SDKs; verify SHA-256 before atomic install."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parent


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    manifest = json.loads((ROOT / "sources.json").read_text())
    for artifact in manifest["artifacts"]:
        target = ROOT / artifact["filename"]
        if target.exists() and sha256(target) == artifact["sha256"]:
            print(f"Verified existing {target.name}", flush=True)
            continue
        pending = target.with_suffix(target.suffix + ".part")
        request = urllib.request.Request(artifact["url"], headers={"User-Agent": "P9-TARE-reproducible-build"})
        try:
            with urllib.request.urlopen(request, timeout=120) as response, pending.open("wb") as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
            actual = sha256(pending)
            if actual != artifact["sha256"]:
                raise RuntimeError(f"Checksum mismatch for {target.name}: {actual}")
            os.replace(pending, target)
            print(f"Downloaded and verified {target.name}", flush=True)
        finally:
            pending.unlink(missing_ok=True)
    patch = ROOT / "P9_TARE.patch"
    if sha256(patch) != manifest["integration_patch_sha256"]:
        raise RuntimeError("Integration patch checksum mismatch")


if __name__ == "__main__":
    main()
