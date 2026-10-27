"""Compare uninstrumented PC replay runs (verify_pc_video_device.py outputs).

Per run, over the same continuous window the verifier uses (pause excluded):
  publication_fps   output throughput from telemetry publication times
  input_clock_fps   preserved NN/input timestamps (optimistic under queue growth)
  consumption_rate  unique (media_epoch, frame_index) published / source frames
                    spanned in the window; drop_rate = 1 - consumption_rate
  latency_ms        capture/injection -> telemetry publication: p50/p95/mean,
                    plus steady-state (after the first 10 s) and growth slope
Gaps cannot be attributed to the sender, the 1-frame hub, or the camera tick
from telemetry alone; they are reported as total missing source frames.
"""
import argparse
import json
from pathlib import Path
import statistics


def percentile(values, q):
    ordered = sorted(values)
    return ordered[int((len(ordered)-1)*q)]  # same rule as analyze_oak_profile.py


def summarize(directory):
    report = json.loads((directory/'report.json').read_text())
    frames = [json.loads(line) for line in (directory/'frames.jsonl').read_text().splitlines()]
    pc = [f for f in frames if f.get('input_source') == 'pc']
    first = pc[0]['nn_timestamp_s']
    seconds = report.get('measurement_seconds', 20)  # older reports: analyze_oak_profile default
    window = [f for f in pc if f['nn_timestamp_s']-first < seconds-1]
    span = window[-1]['sent_monotonic_s']-window[0]['sent_monotonic_s']
    keys = [(f.get('source_media_epoch'), f['source_frame_index']) for f in window]
    expected = 0
    for epoch in sorted(set(k[0] for k in keys)):
        indices = [k[1] for k in keys if k[0] == epoch]
        expected += max(indices)-min(indices)+1
    latency = [(f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in window]
    steady = [v for f, v in zip(window, latency) if f['sent_monotonic_s']-window[0]['sent_monotonic_s'] >= 10]
    n = list(range(len(latency)))
    slope = statistics.linear_regression(n, latency).slope if len(latency) > 2 else None
    return dict(run=directory.name, passed=report['passed'], window_frames=len(window),
                publication_fps=(len(window)-1)/span,
                input_clock_fps=(len(window)-1)/(window[-1]['nn_timestamp_s']-window[0]['nn_timestamp_s']),
                unique_frames=len(set(keys)), duplicate_frames=len(keys)-len(set(keys)),
                expected_frames=expected, consumption_rate=len(set(keys))/expected,
                drop_rate=1-len(set(keys))/expected,
                latency_ms=dict(p50=percentile(latency, .5), p95=percentile(latency, .95),
                                mean=statistics.mean(latency), max=max(latency)),
                steady_latency_ms=dict(p50=percentile(steady, .5), p95=percentile(steady, .95)) if steady else None,
                latency_growth_ms_per_frame=slope,
                sequence_mismatches=report['pc_sequence_mismatches'],
                depth_available_in_pc=report['depth_available_in_pc'],
                depth_valid_objects_in_pc=report['depth_valid_objects_in_pc'],
                csv_frames=report['csv_frames'])


def aggregate(rows):
    keys = ('publication_fps', 'input_clock_fps', 'consumption_rate', 'drop_rate')
    out = {k: statistics.mean(r[k] for r in rows) for k in keys}
    for k in ('p50', 'p95', 'mean'):
        out['latency_'+k+'_ms'] = statistics.mean(r['latency_ms'][k] for r in rows)
    out['steady_latency_p95_ms'] = statistics.mean(r['steady_latency_ms']['p95'] for r in rows
                                                   if r['steady_latency_ms'])
    out['latency_growth_ms_per_frame'] = statistics.mean(r['latency_growth_ms_per_frame'] for r in rows)
    out['runs'] = len(rows)
    out['all_passed'] = all(r['passed'] for r in rows)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--before', nargs='+', type=Path, required=True)
    p.add_argument('--after', nargs='+', type=Path, required=True)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    result = {}
    for name in ('before', 'after'):
        rows = [summarize(d) for d in getattr(args, name)]
        result[name] = dict(runs=rows, aggregate=aggregate(rows))
    if args.output:
        args.output.write_text(json.dumps(result, indent=2))
    for name in ('before', 'after'):
        a = result[name]['aggregate']
        print(f"{name}: pub {a['publication_fps']:.2f} fps, input {a['input_clock_fps']:.2f}, "
              f"consumed {a['consumption_rate']*100:.1f}%, drop {a['drop_rate']*100:.1f}%, "
              f"latency p50/p95 {a['latency_p50_ms']:.0f}/{a['latency_p95_ms']:.0f} ms, "
              f"steady p95 {a['steady_latency_p95_ms']:.0f} ms, growth {a['latency_growth_ms_per_frame']:.2f} ms/frame")


if __name__ == '__main__':
    main()
