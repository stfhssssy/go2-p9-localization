#!/usr/bin/env bash
# Create one isolated simulator container; preserve all existing runs/containers.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo 'Usage: run_container.sh /absolute/NEW-output /absolute/compiled-go2-autonomy [image]' >&2
  exit 2
fi
p9_output=$1
p9_workspace=$2
p9_image=${3:-go2-p9-tare:amd64-20261111}
p9_container=${P9_EXPLORATION_CONTAINER:-p9-tare-experiment}
p9_port=${P9_NOVNC_PORT:-16097}
p9_seconds=${P9_EXPLORATION_SECONDS:-900}
p9_gpu=${P9_GPU_DEVICE:-0}
[[ "$p9_output" == /* && "$p9_workspace" == /* ]] || { echo 'Use absolute output and workspace paths' >&2; exit 2; }
[[ ! -e "$p9_output" ]] || { echo 'Output already exists; use a new directory' >&2; exit 2; }
[[ -f "$p9_workspace/install/setup.bash" ]] || { echo 'Compiled simulator workspace install/setup.bash is missing' >&2; exit 2; }
[[ "$p9_container" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || { echo 'Invalid container name' >&2; exit 2; }
[[ "$p9_port" =~ ^[0-9]+$ && "$p9_port" -ge 1024 && "$p9_port" -le 65535 ]] || { echo 'Invalid noVNC port' >&2; exit 2; }
[[ "$p9_seconds" =~ ^[1-9][0-9]*$ ]] || { echo 'P9_EXPLORATION_SECONDS must be a positive integer' >&2; exit 2; }
[[ "$p9_gpu" =~ ^[0-9]+$ ]] || { echo 'P9_GPU_DEVICE must be a numeric GPU index' >&2; exit 2; }
docker image inspect "$p9_image" >/dev/null
if docker container inspect "$p9_container" >/dev/null 2>&1; then
  echo 'Container already exists; choose P9_EXPLORATION_CONTAINER without deleting prior evidence' >&2
  exit 2
fi
mkdir -p -- "$p9_output"
docker run -d --name "$p9_container" --network bridge --stop-timeout 30 \
  --gpus "device=$p9_gpu" --cpus 8 --memory 20g --pids-limit 4096 --shm-size 1g \
  -p "127.0.0.1:$p9_port:6090" \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e LIBGL_ALWAYS_SOFTWARE=0 -e OMP_NUM_THREADS=2 \
  -e "P9_EXPERIMENT_SECONDS=$p9_seconds" \
  --mount "type=bind,src=$p9_output,dst=/output" \
  --mount "type=bind,src=$p9_workspace,dst=/ws,readonly" \
  "$p9_image" bash -c '
    set -eo pipefail
    mkdir -p /output/integration
    cp -a /opt/p9-exploration/. /output/integration/
    python3 /output/integration/make_config.py
    # The display belongs to the supervisor. VNC becomes available after X starts.
    (
      for attempt in $(seq 1 120); do
        if xdpyinfo -display :94 >/dev/null 2>&1; then break; fi
        sleep 1
      done
      exec x11vnc -display :94 -forever -shared -nopw -rfbport 5900
    ) > /output/vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 6090 localhost:5900 > /output/novnc.log 2>&1 &
    exec bash /output/integration/inside.sh python3 /output/integration/run_exploration.py \
      --output /output/run01 --seconds "$P9_EXPERIMENT_SECONDS"
  '
printf 'Container: %s\nEvidence: %s/run01\nRViz: http://127.0.0.1:%s/vnc.html?autoconnect=1&resize=scale\n' \
  "$p9_container" "$p9_output" "$p9_port"
printf 'Initially STOPPED. Inspect inputs and explicitly call /p9/exploration/start through /output/integration/inside.sh.\n'
