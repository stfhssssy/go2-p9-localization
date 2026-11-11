#!/usr/bin/env bash
set -euo pipefail
p9_run=${1:?new run basename required}
[[ "$p9_run" =~ ^run[0-9]+$ ]] || exit 2
p9_root=/mnt/hdd/ssy/p9-exploration-20261111
p9_container=ssy-p9-tare-20261111
docker exec "$p9_container" python3 /output/integration/capture_video.py --raw-stdout --metadata "/output/$p9_run/recording.json" --max-seconds 600 --stop-file "/output/$p9_run/stop-recording" | /usr/bin/ffmpeg -hide_banner -loglevel warning -n -f rawvideo -pixel_format rgb24 -video_size 1440x900 -framerate 10 -i pipe:0 -an -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart "$p9_root/$p9_run/exploration-rviz.mp4"
