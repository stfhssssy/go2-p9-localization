#!/usr/bin/env bash
# Capture actual RViz pixels; run on the Docker host with docker and ffmpeg access.
# Use the system ffmpeg with libx264 support; minimal/Conda builds may omit it.
# Override the executable, for example P9_FFMPEG=/path/to/ffmpeg ./record_host.sh ...
set -euo pipefail
p9_ffmpeg=${P9_FFMPEG:-/usr/bin/ffmpeg}
p9_container=${1:?container name required}
p9_container_run=${2:?absolute run directory inside container required}
p9_output=${3:?new host MP4 path required}
p9_display=${4:-:96}
[[ "$p9_container_run" = /* && "$p9_output" = /* ]] || { echo 'Use absolute run/output paths' >&2; exit 2; }
[[ ! -e "$p9_output" ]] || { echo 'Refusing to overwrite video' >&2; exit 2; }
command -v "$p9_ffmpeg" >/dev/null || { echo "FFmpeg executable not found: $p9_ffmpeg; set P9_FFMPEG" >&2; exit 2; }
p9_encoder_list=$("$p9_ffmpeg" -hide_banner -encoders 2>&1)
grep -qE '[[:space:]]libx264[[:space:]]' <<< "$p9_encoder_list" || {
  echo "FFmpeg must include the libx264 H.264 encoder: $p9_ffmpeg; use a system build or set P9_FFMPEG" >&2
  exit 2
}
docker exec "$p9_container" python3 /opt/p9-exploration/capture_video.py \
  --display "$p9_display" --raw-stdout --metadata "$p9_container_run/recording.json" \
  --max-seconds 600 --stop-file "$p9_container_run/stop-recording" | \
  "$p9_ffmpeg" -hide_banner -loglevel warning -n -f rawvideo -pixel_format rgb24 \
    -video_size 1440x900 -framerate 10 -i pipe:0 -an -c:v libx264 \
    -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart "$p9_output"
