"""Sustained PC-replay or camera run with per-minute throughput, latency and temperature.

  --source pc      loop --video [--start, --end) at 10 Hz through the PC input
  --source camera  live camera after an explicit camera selection

Records compact telemetry frames and `oakctl device stats -a` samples, then
reports per minute (by device publication time):
  published_fps     frames published / minute duration
  consumption_rate  pc: unique (media_epoch, frame_index) / source frames spanned
                    camera: unique frame_seq / camera sequence numbers spanned
  drop_rate         1 - consumption_rate (cause is not attributed)
  latency p50/p95   capture/injection -> telemetry publication, ms
  temperatures      max per sensor group, CPU usage and SoC power
No algorithm, threshold or model setting is changed. Run under WSL (OpenCV).
Re-analyze an existing run with --analyze DIR.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
OAKCTL = '/mnt/c/Program Files/oakctl/bin/oakctl.exe'
FIELDS = ('input_source', 'source_epoch', 'source_media_epoch', 'source_frame_index', 'source_media_time_s',
          'processing_mode','processing_epoch',
          'frame_seq', 'nn_seq', 'depth_seq', 'capture_monotonic_est_s', 'sent_monotonic_s',
          'nn_timestamp_s', 'depth_available', 'boundary_state', 'left_boundary_state',
          'right_boundary_state', 'stop_line_state')
GROUPS = dict(cpu=('cpu-', 'cpuss-'), npu=('nspss-',), gpu=('gpuss-',), ddr=('ddr',),
              skin=('skin-msm-therm',), camera=('camera-', 'cam-flash-therm'))


def percentile(values, q):
    ordered = sorted(values)
    return ordered[int((len(ordered)-1)*q)] if ordered else None  # analyze_oak_profile.py rule


def device_stats(ip):
    r = subprocess.run([OAKCTL, 'device', 'stats', '-d', ip, '-a', '--format', 'json'],
                       capture_output=True, text=True, timeout=30)
    data = json.loads(r.stdout[r.stdout.index('{'):])
    temps = {}
    for zone in data.get('temperatures', []):
        for group, prefixes in GROUPS.items():
            if zone['sensor'].startswith(prefixes):
                temps[group] = max(temps.get(group, -1e9), zone['millidegrees_celsius']/1000)
    return dict(sampled_at_unix_ms=data['sampled_at_unix_ms'], cpu_usage_percent=data['cpu']['usage_percent'],
                soc_power_w=data.get('power', {}).get('soc', {}).get('power_microwatts', 0)/1e6,
                temperatures_c=temps, warnings=data.get('warnings', []))


def record(args):
    sys.path.insert(0, str(ROOT/'tools'))
    from video_file_sender import VideoFileSender, select_camera, set_processing_mode
    args.output.mkdir(parents=True, exist_ok=True)
    frames, stats, events = [], [], []
    stop = threading.Event()

    def read():
        while not stop.is_set():
            try:
                with socket.create_connection((args.device_ip, 8557), timeout=5) as connection:
                    connection.settimeout(10)
                    connection.sendall(b'HELLO\n')
                    with connection.makefile('rb') as stream:
                        while not stop.is_set():
                            line = stream.readline()
                            if not line:
                                break
                            payload = json.loads(line)
                            if 'frame_seq' in payload:
                                row = {k: payload.get(k) for k in FIELDS}
                                row['depth_valid_objects'] = sum(bool(o.get('depth_valid'))
                                                                 for o in payload.get('objects', []))
                                frames.append(row)
            except (OSError, ValueError) as exc:
                if not stop.is_set():
                    events.append(dict(t=time.time(), event='telemetry_reconnect', error=str(exc)))
                    time.sleep(1)

    def sample():
        while not stop.is_set():
            try:
                stats.append(dict(device_stats(args.device_ip), wall_s=time.time()))
            except Exception as exc:  # stats are auxiliary; never end the run
                events.append(dict(t=time.time(), event='stats_error', error=str(exc)))
            stop.wait(args.stats_interval)

    set_processing_mode(args.device_ip,args.processing)
    select_camera(args.device_ip)
    threads = [threading.Thread(target=f, daemon=True) for f in (read, sample)]
    for t in threads:
        t.start()
    sender = None
    try:
        deadline = time.monotonic()+15
        while not frames and time.monotonic() < deadline:
            time.sleep(.1)
        if not frames:
            raise RuntimeError(f'No telemetry; events={events}')
        if args.source == 'pc':
            sender = VideoFileSender(args.device_ip, args.video, loop=True, start_s=args.start, end_s=args.end)
            sender.start()
            deadline = time.monotonic()+20
            while not any(f['input_source'] == 'pc' for f in frames) and time.monotonic() < deadline:
                time.sleep(.1)
        begin = len(frames)
        events.append(dict(t=time.time(), event='measurement_start', frame=begin))
        end = time.monotonic()+args.minutes*60
        while time.monotonic() < end:
            time.sleep(1)
            if sender is not None and sender.snapshot().get('error'):
                events.append(dict(t=time.time(), event='sender_error', error=sender.snapshot()['error']))
                break
        events.append(dict(t=time.time(), event='measurement_end', frame=len(frames)))
        sender_status = sender.snapshot() if sender is not None else None
    finally:
        if sender is not None:
            sender.close()
        select_camera(args.device_ip)
        set_processing_mode(args.device_ip,'basic')
        stop.set()
        for t in threads:
            t.join(timeout=15)
    measured = [f for f in frames[begin:] if f['input_source'] == args.source]
    (args.output/'frames.jsonl').write_text('\n'.join(json.dumps(f) for f in measured))
    (args.output/'stats.jsonl').write_text('\n'.join(json.dumps(s) for s in stats))
    (args.output/'run.json').write_text(json.dumps(dict(
        source=args.source, processing_mode=args.processing, video=str(args.video) if args.video else None, start_s=args.start, end_s=args.end,
        minutes=args.minutes, device_ip=args.device_ip, events=events, sender=sender_status,
        other_source_frames=len(frames[begin:])-len(measured)), indent=2, ensure_ascii=False), encoding='utf-8')
    return args.output


def analyze(directory):
    run = json.loads((directory/'run.json').read_text(encoding='utf-8'))
    frames = [json.loads(line) for line in (directory/'frames.jsonl').read_text(encoding='utf-8').splitlines() if line]
    stats = [json.loads(line) for line in (directory/'stats.jsonl').read_text(encoding='utf-8').splitlines() if line]
    source = run['source']
    t0 = frames[0]['sent_monotonic_s']
    # Map device stats wall time onto the device publication clock via the start offset.
    start_wall = next(e['t'] for e in run['events'] if e['event'] == 'measurement_start')
    minutes = []
    last = frames[-1]['sent_monotonic_s']
    for m in range(int((last-t0)//60)+1):
        rows = [f for f in frames if m*60 <= f['sent_monotonic_s']-t0 < (m+1)*60]
        if len(rows) < 2:
            continue
        duration = min(60., last-t0-m*60)
        if source == 'pc':
            keys = [(f['source_media_epoch'], f['source_frame_index']) for f in rows]
            expected = sum(max(i for e, i in keys if e == epoch)-min(i for e, i in keys if e == epoch)+1
                           for epoch in set(k[0] for k in keys))
        else:
            keys = [f['frame_seq'] for f in rows]
            expected = max(keys)-min(keys)+1
        unique = len(set(keys))
        latency = [(f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in rows]
        samples = [s for s in stats if m*60 <= s['wall_s']-start_wall < (m+1)*60]
        temps = {g: max(s['temperatures_c'].get(g, float('nan')) for s in samples) for g in GROUPS} if samples else {}
        minutes.append(dict(
            minute=m+1, duration_s=duration, partial=duration < 59, frames=len(rows),
            # A partial tail bucket can hold frames queued before its short span.
            published_fps=len(rows)/duration if duration >= 59 else None,
            unique_inputs=unique, expected_inputs=expected, consumption_rate=unique/expected,
            drop_rate=1-unique/expected, duplicates=len(keys)-unique,
            latency_p50_ms=percentile(latency, .5), latency_p95_ms=percentile(latency, .95),
            latency_max_ms=max(latency),
            frame_nn_mismatches=sum(f['frame_seq'] != f['nn_seq'] for f in rows),
            depth_minus_frame_seq=dict(Counter(f['depth_seq']-f['frame_seq'] for f in rows).most_common(3)),
            depth_available_frames=sum(bool(f['depth_available']) for f in rows),
            depth_valid_objects=sum(f['depth_valid_objects'] for f in rows),
            temperatures_max_c=temps,
            cpu_usage_mean_percent=sum(s['cpu_usage_percent'] for s in samples)/len(samples) if samples else None,
            soc_power_mean_w=sum(s['soc_power_w'] for s in samples)/len(samples) if samples else None,
            stats_samples=len(samples)))
    full = [m for m in minutes if m['duration_s'] >= 59]
    latency = [(f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in frames]
    overall = dict(source=source, frames=len(frames), span_s=last-t0,
                   published_fps=(len(frames)-1)/(last-t0),
                   consumption_rate=sum(m['unique_inputs'] for m in minutes)/sum(m['expected_inputs'] for m in minutes),
                   latency_p50_ms=percentile(latency, .5), latency_p95_ms=percentile(latency, .95),
                   latency_max_ms=max(latency),
                   min_full_minute_fps=min(m['published_fps'] for m in full) if full else None,
                   max_full_minute_p95_ms=max(m['latency_p95_ms'] for m in full) if full else None,
                   duplicates=sum(m['duplicates'] for m in minutes),
                   frame_nn_mismatches=sum(m['frame_nn_mismatches'] for m in minutes),
                   depth_available_frames=sum(m['depth_available_frames'] for m in minutes),
                   depth_valid_objects=sum(m['depth_valid_objects'] for m in minutes),
                   reconnects=sum(e['event'] == 'telemetry_reconnect' for e in run['events']),
                   stats_errors=sum(e['event'] == 'stats_error' for e in run['events']),
                   device_warnings=sorted({w if isinstance(w, str) else json.dumps(w)
                                           for s in stats for w in s['warnings']}))
    overall['drop_rate'] = 1-overall['consumption_rate']
    result = dict(overall=overall, minutes=minutes)
    (directory/'summary.json').write_text(json.dumps(result, indent=2))
    print(f"{source}: {overall['published_fps']:.2f} fps, consumed {overall['consumption_rate']*100:.1f}%, "
          f"latency p50/p95 {overall['latency_p50_ms']:.0f}/{overall['latency_p95_ms']:.0f} ms, "
          f"min minute {overall['min_full_minute_fps']}")
    for m in minutes:
        t = m['temperatures_max_c']
        fps = f"{m['published_fps']:.2f} fps" if not m['partial'] else f"partial {m['duration_s']:.0f}s"
        print(f"  min {m['minute']:2d}: {fps}, drop {m['drop_rate']*100:.1f}%, "
              f"p50/p95 {m['latency_p50_ms']:.0f}/{m['latency_p95_ms']:.0f} ms, "
              f"cpu {t.get('cpu', float('nan')):.1f}C npu {t.get('npu', float('nan')):.1f}C "
              f"skin {t.get('skin', float('nan')):.1f}C, usage {m['cpu_usage_mean_percent'] or 0:.0f}%")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--analyze', type=Path, help='only summarize an existing output directory')
    p.add_argument('--device-ip')
    p.add_argument('--source', choices=('pc', 'camera'))
    p.add_argument('--processing',choices=('basic','full'),default='full')
    p.add_argument('--video', type=Path)
    p.add_argument('--start', type=float, default=0.)
    p.add_argument('--end', type=float)
    p.add_argument('--minutes', type=float, default=11)
    p.add_argument('--stats-interval', type=float, default=15)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    if not (args.device_ip and args.source and args.output) or (args.source == 'pc' and not args.video):
        p.error('recording needs --device-ip, --source, --output (and --video for pc)')
    analyze(record(args))


if __name__ == '__main__':
    main()
