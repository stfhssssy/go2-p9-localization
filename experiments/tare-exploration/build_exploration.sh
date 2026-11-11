#!/usr/bin/env bash
# Run on the AMD64 Linux Docker host. No build or publication runs automatically.
set -euo pipefail
p9_bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
p9_image=${1:-go2-p9-tare:amd64-20261111}
p9_base=${P9_EXPLORATION_BASE_IMAGE:-go2-p9-nav:amd64-20261110}
p9_jobs=${P9_BUILD_JOBS:-2}
[[ "$p9_jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'P9_BUILD_JOBS must be a positive integer' >&2; exit 2; }
p9_base_id=$(docker image inspect "$p9_base" --format '{{.Id}}')
p9_base_arch=$(docker image inspect "$p9_base" --format '{{.Architecture}}')
[[ "$p9_base_arch" == amd64 ]] || { echo 'This bundle requires the AMD64 P9 simulation base' >&2; exit 2; }
p9_patch_sha=$(sha256sum "$p9_bundle_dir/P9_TARE.patch" | cut -d ' ' -f 1)
printf 'Building %s from %s (%s)\n' "$p9_image" "$p9_base" "$p9_base_id"
docker build --platform linux/amd64 --progress plain \
  --build-arg "BASE_IMAGE=$p9_base" --build-arg "BASE_IMAGE_ID=$p9_base_id" \
  --build-arg "TARE_COMMIT=b6760adb4280b379567c76f4e684ce628e28ef71" \
  --build-arg "TARE_PATCH_SHA256=$p9_patch_sha" --build-arg "BUILD_JOBS=$p9_jobs" \
  -f "$p9_bundle_dir/Dockerfile.exploration" -t "$p9_image" "$p9_bundle_dir"
docker image inspect "$p9_image" --format 'Built {{.Id}} ({{.Architecture}})'
