#!/usr/bin/env bash
# Run inside the release image, with no physical sensors in this DDS domain.
set -euo pipefail
if [ "${ROS_DOMAIN_ID:-0}" = 0 ]; then
  echo 'Set an isolated nonzero ROS_DOMAIN_ID for synthetic testing.' >&2; exit 2
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
p9_smoke_root="${1:?Usage: smoke-live.sh NEW_OUTPUT_DIR [--raw]}"
mkdir -p "$p9_smoke_root"
p9_smoke_config=/opt/p9/config/smoke.yaml
if [ "${2:-}" = --raw ]; then
  python3 - "$p9_smoke_root/smoke-raw.yaml" <<'PY'
import sys,yaml
c=yaml.safe_load(open('/opt/p9/config/smoke.yaml'))
c['sensor']['imu_mode']='raw_linear'
with open(sys.argv[1], 'x') as f: yaml.safe_dump(c,f)
PY
  p9_smoke_config="$p9_smoke_root/smoke-raw.yaml"
fi
p9 live --config "$p9_smoke_config" --output "$p9_smoke_root/run" > "$p9_smoke_root/launch.log" 2>&1 &
p9_launch_pid=$!
cleanup() {
  kill -INT "$p9_launch_pid" 2>/dev/null || true
  wait "$p9_launch_pid" || true
}
trap cleanup EXIT
sleep 4
p9_smoke_args=()
if [ "${2:-}" = --raw ]; then p9_smoke_args+=(--raw); fi
python3 /opt/p9/tests/ros_smoke.py --seconds 12 --output "$p9_smoke_root/report.json" "${p9_smoke_args[@]}"
timeout 15 p9 save > "$p9_smoke_root/save.log" 2>&1
