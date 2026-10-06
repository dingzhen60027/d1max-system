#!/usr/bin/env python3
"""Record only two explicitly supplied X11 window IDs; never the desktop.

Examples (the root operator supplies the actual windows):
  python3 record_windows.py capabilities
  python3 record_windows.py capture --rviz-window 0x123 --isaac-window 0x456 \
      --output-dir /path/to/new/video --max-seconds 240
  python3 record_windows.py postprocess --output-dir /path/to/new/video

SIGINT/SIGTERM requests a graceful stop of this script's ffmpeg child only.
Capture uses software encoding. The common wall-clock timeline is retained in
windows.mkv and its two MP4 extracts; composition occurs only on explicit request.
This video wall clock is not ROS/PhysX source-time acceptance evidence.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


NAMES = ('rviz', 'isaacsim')
FILES = ('windows.mkv', 'rviz.mp4', 'isaacsim.mp4', 'overview.mp4',
         'recording.json', 'recording.json.tmp', 'ffmpeg.log', 'ffmpeg.progress')


def window_id(value):
    number = int(value, 0)
    if not 0 < number <= 0xffffffff:
        raise argparse.ArgumentTypeError('A nonzero explicit X11 window ID is required.')
    return number


def size(value):
    match = re.fullmatch(r'(\d+)x(\d+)', value)
    if not match or min(map(int, match.groups())) < 2:
        raise argparse.ArgumentTypeError('Size must be WIDTHxHEIGHT, at least 2x2.')
    return tuple(map(int, match.groups()))


def output_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def checked(command, timeout=30):
    result = subprocess.run(command, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError('Command failed: '+command[0]+'\n'+result.stderr[-4000:])
    return result.stdout


def capabilities():
    for tool in ('ffmpeg', 'ffprobe', 'xwininfo'):
        if shutil.which(tool) is None:
            raise RuntimeError('Required tool is unavailable: '+tool)
    demuxer = checked(['ffmpeg', '-hide_banner', '-h', 'demuxer=x11grab'])
    encoders = checked(['ffmpeg', '-hide_banner', '-encoders'])
    filters = checked(['ffmpeg', '-hide_banner', '-filters'])
    if '-window_id' not in demuxer or 'libx264' not in encoders:
        raise RuntimeError('This ffmpeg must support X11 window IDs and libx264.')
    required = ('hstack', 'setpts', 'scale', 'pad', 'format', 'setsar', 'fps')
    if any(not re.search(r'\s'+name+r'\s', filters) for name in required):
        raise RuntimeError('Required ffmpeg filters are unavailable.')
    return dict(ffmpeg=checked(['ffmpeg', '-version']).splitlines()[0],
                window_capture=True, software_encoder='libx264',
                display=os.environ.get('DISPLAY'),
                drawtext=bool(re.search(r'\sdrawtext\s', filters)))


def geometry(display, identifier):
    # Read only the explicitly supplied target. No title search or window inventory.
    root = checked(['xwininfo', '-display', display, '-root'])
    root_id = re.search(r'Window id:\s*(0x[0-9a-fA-F]+)', root)
    if not root_id or identifier == int(root_id.group(1), 16):
        raise RuntimeError('Desktop/root window capture is forbidden.')
    text = checked(['xwininfo', '-display', display, '-id', hex(identifier)])
    fields = {}
    for field in ('Width', 'Height', 'Depth', 'Map State'):
        match = re.search(r'^\s*'+field+r':\s*(.+)$', text, re.MULTILINE)
        if not match:
            raise RuntimeError('Cannot read target window geometry: '+hex(identifier))
        fields[field] = match.group(1).strip()
    if fields['Map State'] != 'IsViewable':
        raise RuntimeError('Target window must be visible: '+hex(identifier))
    width, height = int(fields['Width']), int(fields['Height'])
    if width < 2 or height < 2:
        raise RuntimeError('Target window is too small.')
    return dict(window_id=identifier, window_id_hex=hex(identifier), width=width,
                height=height, depth=int(fields['Depth']), geometry_verified=True)


def capture_command(args, windows, epoch_ns):
    epoch = f'{epoch_ns//10**9}.{epoch_ns%10**9:09d}'
    command = ['ffmpeg', '-hide_banner', '-nostdin', '-n', '-copyts',
               '-filter_complex_threads', '1']
    for window in windows:
        command += ['-thread_queue_size', '8', '-f', 'x11grab',
                    '-use_wallclock_as_timestamps', '1', '-framerate', str(args.fps),
                    '-draw_mouse', '0', '-window_id', str(window['window_id']),
                    '-video_size', f"{window['width']}x{window['height']}", '-i', args.display]
    filters = ';'.join(f'[{i}:v]setpts=PTS-{epoch}/TB,'
                       f'pad=ceil(iw/2)*2:ceil(ih/2)*2,format=yuv420p[v{i}]'
                       for i in range(2))
    command += ['-filter_complex', filters, '-map', '[v0]', '-map', '[v1]',
                '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
                '-crf', '22', '-threads:v:0', '2', '-threads:v:1', '2',
                '-enc_time_base:v', '-1', '-vsync', '0', '-avoid_negative_ts', 'disabled',
                '-metadata:s:v:0', 'title=RViz', '-metadata:s:v:1', 'title=Isaac Sim',
                '-t', str(args.max_seconds), '-progress', str(args.output_dir/'ffmpeg.progress'),
                str(args.output_dir/'windows.mkv')]
    nice = shutil.which('nice')
    return ([nice, '-n', '10'] if nice else [])+command


def probe(path):
    return json.loads(checked(['ffprobe', '-v', 'error', '-threads', '1', '-count_frames',
        '-show_entries', 'stream=index,codec_name,width,height,start_time,duration,avg_frame_rate,nb_read_frames:format=duration,start_time',
        '-of', 'json', str(path)], timeout=180))


def extract(output_dir, manifest):
    source = output_dir/'windows.mkv'
    info = probe(source)
    streams = info.get('streams', [])
    if len(streams) != 2 or any(int(s.get('nb_read_frames', '0')) < 1 for s in streams):
        raise RuntimeError('The recording must contain two nonempty decoded video streams.')
    manifest['source_video'] = info
    manifest['individual_videos'] = {}
    for index, name in enumerate(NAMES):
        destination = output_dir/(name+'.mp4')
        if not destination.exists():
            checked(['ffmpeg', '-hide_banner', '-nostdin', '-n', '-copyts', '-i', str(source),
                     '-map', f'0:v:{index}', '-c:v', 'copy', '-an', '-vsync', '0',
                     '-avoid_negative_ts', 'disabled', '-movflags', '+faststart', str(destination)], timeout=180)
        manifest['individual_videos'][name] = dict(path=str(destination), **probe(destination))
    output_json(output_dir/'recording.json', manifest)


def compose(output_dir, manifest, caps):
    destination = output_dir/'overview.mp4'
    if destination.exists():
        raise RuntimeError('Refusing to overwrite an existing overview.')
    font = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    branches = []
    for index, title in enumerate(('RViz', 'Isaac Sim')):
        branch = (f'[0:v:{index}]scale=960:1080:force_original_aspect_ratio=decrease,'
                  'pad=960:1080:(ow-iw)/2:(oh-ih)/2,setsar=1')
        if caps['drawtext'] and font.is_file():
            branch += f",drawtext=fontfile={font}:text='{title}':x=16:y=16:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.7:boxborderw=8"
        branches.append(branch+f'[p{index}]')
    # Only the joined timeline is reset. Independent STARTPTS resets would lose sync.
    filters = ';'.join(branches)+f";[p0][p1]hstack=inputs=2:shortest=1,fps={manifest['fps']},setpts=PTS-STARTPTS[out]"
    command = ['ffmpeg', '-hide_banner', '-nostdin', '-n', '-copyts',
               '-filter_complex_threads', '1', '-i', str(output_dir/'windows.mkv'),
               '-filter_complex', filters, '-map', '[out]', '-an', '-c:v', 'libx264',
               '-preset', 'veryfast', '-crf', '20', '-threads', '2',
               '-movflags', '+faststart', str(destination)]
    nice = shutil.which('nice')
    checked(([nice, '-n', '10'] if nice else [])+command, timeout=600)
    manifest['overview'] = dict(path=str(destination), **probe(destination))
    output_json(output_dir/'recording.json', manifest)


def capture(args, caps):
    if args.rviz_window == args.isaac_window:
        raise RuntimeError('The two target window IDs must differ.')
    if not args.display or not math.isfinite(args.max_seconds) or not 1 <= args.max_seconds <= 1800:
        raise RuntimeError('DISPLAY and a maximum duration between 1 and 1800 seconds are required.')
    ids = (args.rviz_window, args.isaac_window)
    supplied_sizes = (args.rviz_size, args.isaac_size)
    windows = []
    for identifier, supplied in zip(ids, supplied_sizes):
        if args.dry_run and supplied:
            windows.append(dict(window_id=identifier, window_id_hex=hex(identifier),
                                width=supplied[0], height=supplied[1], geometry_verified=False))
        else:
            window = geometry(args.display, identifier)
            if supplied and supplied != (window['width'], window['height']):
                raise RuntimeError('Supplied dimensions differ from the actual target window.')
            windows.append(window)
    epoch_ns = time.time_ns()
    command = capture_command(args, windows, epoch_ns)
    if args.dry_run:
        print(json.dumps(dict(dry_run=True, windows=windows, command=command,
                              capabilities=caps), indent=2))
        return
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if any((args.output_dir/name).exists() or (args.output_dir/name).is_symlink() for name in FILES):
        raise RuntimeError('Use a new recording directory; existing outputs will not be overwritten.')
    manifest = dict(schema=1, display=args.display, windows=dict(zip(NAMES, windows)),
                    fps=args.fps, maximum_seconds=args.max_seconds, capabilities=caps,
                    wall_clock_origin_ns=epoch_ns, capture_command=command,
                    status='starting', scope='only_explicit_window_ids',
                    synchronization='common_wall_clock_PTS_not_ROS_source_time')
    output_json(args.output_dir/'recording.json', manifest)
    stop = dict(requested=False, reason=None, sent=False)
    process = None

    def request_stop(number, _frame):
        if not stop['requested']:
            stop.update(requested=True, reason=signal.Signals(number).name)
        if process is not None and process.poll() is None and not stop['sent']:
            process.send_signal(signal.SIGINT)
            stop['sent'] = True

    old = {number: signal.signal(number, request_stop) for number in (signal.SIGINT, signal.SIGTERM)}
    started = time.monotonic()
    stopped_at = None
    forced = False
    try:
        with (args.output_dir/'ffmpeg.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            manifest.update(status='recording', recorder_pid=process.pid)
            output_json(args.output_dir/'recording.json', manifest)
            while process.poll() is None:
                elapsed = time.monotonic()-started
                if elapsed >= args.max_seconds and not stop['requested']:
                    stop.update(requested=True, reason='maximum_duration')
                # A signal may arrive before Popen has assigned the owned child.
                if stop['requested'] and not stop['sent']:
                    process.send_signal(signal.SIGINT)
                    stop['sent'] = True
                if stop['requested'] and stopped_at is None:
                    stopped_at = time.monotonic()
                if stopped_at is not None and time.monotonic()-stopped_at > 10:
                    forced = True
                    process.terminate()  # Only the ffmpeg child created above.
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()  # Own child only; never a process group/other application.
                        process.wait(timeout=5)
                    break
                time.sleep(.1)
        manifest.update(status='captured' if not forced and (process.returncode == 0 or
                        (process.returncode == 255 and stop['requested'])) else 'capture_failed',
                        ffmpeg_returncode=process.returncode, stop_reason=stop['reason'], forced_stop=forced,
                        capture_elapsed_monotonic_s=time.monotonic()-started,
                        wall_clock_end_ns=time.time_ns())
        output_json(args.output_dir/'recording.json', manifest)
    finally:
        for number, handler in old.items():
            signal.signal(number, handler)
    extract(args.output_dir, manifest)
    if args.compose_on_stop:
        compose(args.output_dir, manifest, caps)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    if manifest['status'] == 'capture_failed':
        raise RuntimeError('Capture was not completed gracefully; inspect ffmpeg.log and recording.json.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('capabilities')
    record = commands.add_parser('capture')
    record.add_argument('--rviz-window', type=window_id, required=True)
    record.add_argument('--isaac-window', type=window_id, required=True)
    record.add_argument('--output-dir', type=Path, required=True)
    record.add_argument('--display', default=os.environ.get('DISPLAY'))
    record.add_argument('--max-seconds', type=float, default=240.)
    record.add_argument('--fps', type=int, choices=(10, 15), default=15)
    record.add_argument('--dry-run', action='store_true')
    record.add_argument('--rviz-size', type=size)
    record.add_argument('--isaac-size', type=size)
    record.add_argument('--compose-on-stop', action='store_true')
    post = commands.add_parser('postprocess')
    post.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    caps = capabilities()
    if args.command == 'capabilities':
        print(json.dumps(caps, indent=2))
    elif args.command == 'capture':
        capture(args, caps)
    else:
        manifest = json.loads((args.output_dir/'recording.json').read_text())
        if manifest.get('schema') != 1 or manifest.get('scope') != 'only_explicit_window_ids':
            raise RuntimeError('Recording metadata does not match this window recorder.')
        extract(args.output_dir, manifest)
        compose(args.output_dir, manifest, caps)
        print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
