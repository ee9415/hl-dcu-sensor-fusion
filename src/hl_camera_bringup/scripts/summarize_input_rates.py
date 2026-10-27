"""Compare input-rate runs using output publication clocks and unique frames.

Native-frame gaps are intentional when sampling a 30-FPS file at 10/15/20Hz;
do not report them as receiver packet loss.
"""
import argparse
import json
from pathlib import Path
import statistics


def percentile(values,q):
    return sorted(values)[int((len(values)-1)*q)]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    args=p.parse_args();results=[]
    for path in sorted(args.root.glob('*fps/report.json')):
        report=json.loads(path.read_text())
        frames=[json.loads(s) for s in path.with_name('frames.jsonl').read_text().splitlines()]
        pc=[f for f in frames if f.get('input_source')=='pc']
        first=pc[0]['nn_timestamp_s']
        pc=[f for f in pc if f['nn_timestamp_s']-first<report['measurement_seconds']-1]
        keys=[(f['source_epoch'],f['source_frame_index']) for f in pc]
        span=pc[-1]['sent_monotonic_s']-pc[0]['sent_monotonic_s']
        latency=[(f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in pc]
        results.append(dict(run=path.parent.name,requested_fps=report['requested_send_fps'],
            effective_send_cap=report['sender']['effective_fps'],
            publication_fps=(len(pc)-1)/span,unique_output_fps=(len(set(keys))-1)/span,
            duplicate_outputs=len(keys)-len(set(keys)),frames=len(pc),
            latency_p50_ms=statistics.median(latency),latency_p95_ms=percentile(latency,.95),
            latency_first_ms=latency[0],latency_last_ms=latency[-1],
            latency_slope_ms_per_frame=statistics.linear_regression(range(len(latency)),latency).slope,
            sequence_mismatches=report['pc_sequence_mismatches'],passed=report['passed']))
    results.sort(key=lambda v:v['requested_fps'])
    (args.root/'comparison.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
