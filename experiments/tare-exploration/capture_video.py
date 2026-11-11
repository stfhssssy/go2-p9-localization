#!/usr/bin/env python3
"""Record actual X display :94 pixels at 1440x900 / 10 FPS, with timing metadata.

Raw mode supports the existing container -> host ffmpeg pipe. MP4 mode requires
ffmpeg inside the executing environment. No ROS dependencies or generated graphics.
"""
import argparse
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--output', help='New MP4 file; requires a local ffmpeg executable')
    target.add_argument('--raw-stdout', action='store_true', help='Emit RGB24 frames for host-side ffmpeg')
    parser.add_argument('--metadata', help='JSON recording metadata (required with --raw-stdout)')
    parser.add_argument('--display', default=':94')
    parser.add_argument('--max-seconds', type=float, default=600.)
    parser.add_argument('--stop-file', help='Stop when this file exists')
    parser.add_argument('--ffmpeg', default='ffmpeg')
    args = parser.parse_args()
    if not 1 <= args.max_seconds <= 600:
        parser.error('--max-seconds must be between 1 and 600')
    if args.raw_stdout and not args.metadata:
        parser.error('--metadata is required with --raw-stdout')
    from PIL import ImageGrab, features
    if not features.check_feature('xcb'):
        parser.error('Pillow must have XCB support for a real X display capture')
    output = Path(args.output).resolve() if args.output else None
    metadata = Path(args.metadata).resolve() if args.metadata else output.with_suffix('.recording.json')
    metadata.parent.mkdir(parents=True, exist_ok=True)
    if metadata.exists():
        parser.error('Metadata file already exists: ' + str(metadata))
    stop_file = Path(args.stop_file).resolve() if args.stop_file else metadata.parent / 'stop-recording'
    encoder = None
    if output:
        if output.exists():
            parser.error('MP4 already exists: ' + str(output))
        ffmpeg = shutil.which(args.ffmpeg)
        if not ffmpeg:
            parser.error('ffmpeg unavailable here; use --raw-stdout piped to host ffmpeg')
        output.parent.mkdir(parents=True, exist_ok=True)
        encoder = subprocess.Popen([
            ffmpeg, '-hide_banner', '-loglevel', 'warning', '-n',
            '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '1440x900',
            '-framerate', '10', '-i', 'pipe:0', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(output)], stdin=subprocess.PIPE)
        sink = encoder.stdin
    else:
        sink = sys.stdout.buffer
    running = True
    signal_number = None

    def stop(signum, _frame):
        nonlocal running, signal_number
        running, signal_number = False, signum

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    start = time.monotonic()
    frames = duplicates = 0
    maximum_lateness = 0.
    maximum_capture_duration = 0.
    previous_frame = None
    failure = None
    reason = 'max_seconds'
    encoder_exit = None
    try:
        while running:
            now = time.monotonic()
            if now - start >= args.max_seconds:
                reason = 'max_seconds'; break
            if stop_file.exists():
                reason = 'stop_file'; break
            target_time = start + frames / 10.
            delay = target_time - now
            if delay > 0:
                time.sleep(delay)
            if not running:
                reason = 'signal'; break
            lateness = max(0., time.monotonic() - target_time)
            maximum_lateness = max(maximum_lateness, lateness)
            if lateness > .1 and previous_frame is not None:
                # Preserve wall-time playback when capture/pipe is briefly late.
                # Every duplicate is counted, never presented as a fresh screenshot.
                frame = previous_frame
                duplicates += 1
            else:
                capture_start = time.monotonic()
                picture = ImageGrab.grab(xdisplay=args.display).convert('RGB')
                maximum_capture_duration = max(maximum_capture_duration, time.monotonic() - capture_start)
                if picture.size != (1440, 900):
                    raise RuntimeError('Unexpected X display dimensions: ' + str(picture.size))
                frame = picture.tobytes()
                previous_frame = frame
            sink.write(frame)
            sink.flush()
            frames += 1
        if signal_number is not None:
            reason = 'signal'
    except Exception as error:
        failure = type(error).__name__ + ': ' + str(error)
        reason = 'capture_error'
        print(failure, file=sys.stderr)
    finally:
        capture_wall = time.monotonic() - start
        if encoder is not None:
            try:
                sink.close()
            except BrokenPipeError:
                pass
            try:
                encoder_exit = encoder.wait(timeout=30)
            except subprocess.TimeoutExpired:
                encoder.terminate()
                encoder_exit = encoder.wait(timeout=10)
            if encoder_exit and failure is None:
                failure = 'ffmpeg exited with code ' + str(encoder_exit)
        record = dict(source='actual X display via Pillow XCB', display=args.display,
                      resolution=[1440,900], fps=10, frames=frames, video_seconds=frames/10.,
                      wall_seconds=capture_wall, duplicates_for_delayed_capture=duplicates,
                      maximum_capture_lateness_seconds=maximum_lateness,
                      maximum_capture_duration_seconds=maximum_capture_duration,
                      playback_speed=1, audio=False, stop_reason=reason,
                      stop_file=str(stop_file), stop_file_exists=stop_file.exists(),
                      received_signal=signal_number, output=str(output) if output else 'raw RGB24 stdout',
                      encoder_exit_code=encoder_exit, error=failure)
        temporary = metadata.with_suffix(metadata.suffix + '.tmp')
        temporary.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
        temporary.replace(metadata)
    return 1 if failure is not None else 0


if __name__ == '__main__':
    raise SystemExit(main())
