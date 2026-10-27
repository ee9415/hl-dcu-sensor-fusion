"""Summarize explicit device callback measurements without adding overlapping times."""
import argparse
import json
from pathlib import Path
import statistics


def stats(values):
    ordered=sorted(values)
    return dict(count=len(values),mean_ms=statistics.mean(values),median_ms=statistics.median(values),
                p95_ms=ordered[int((len(ordered)-1)*.95)],max_ms=max(values)) if values else None


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    args=p.parse_args();root=args.directory
    profiles=json.loads((root/'profile.json').read_text())
    assert profiles,'No callback profile files; do not interpret empty data as zero latency'
    post=profiles['pipeline_timing_postprocess']
    sequences={e['sequence'] for e in post['events']}
    stages={}
    for key,stage in profiles.items():
        events=[e for e in stage['events'] if e['sequence'] in sequences]
        if events:stages[key]=stats([e['ms'] for e in events])
    pre={e['sequence']:e['end'] for e in profiles['pipeline_timing_preprocess']['events']}
    nn_wait=[(e['start']-pre[e['sequence']])*1000 for e in post['events'] if e['sequence'] in pre]
    frames=[json.loads(line) for line in (root/'normal/frames.jsonl').read_text().splitlines()]
    pc=[f for f in frames if f.get('input_source')=='pc']
    latency=[(f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in pc]
    normal=json.loads((root/'normal/report.json').read_text())
    instrumented=json.loads((root/'instrumented/report.json').read_text())
    functions=post.get('functions',[])
    # Preserved NN timestamps are input times. Queue growth can make this FPS
    # optimistic: separately measure publication intervals at the output.
    first=pc[0]['nn_timestamp_s']
    window=normal.get('measurement_seconds',20)-1
    continuous=[f for f in pc if f['nn_timestamp_s']-first<window]
    publication_fps=(len(continuous)-1)/(continuous[-1]['sent_monotonic_s']-continuous[0]['sent_monotonic_s'])
    report=dict(normal_fps_from_preserved_input_clock=normal.get('input_clock_fps',normal['output_fps']),
        normal_publication_fps=publication_fps,
        publication_window_frames=len(continuous),
        instrumented_fps_not_baseline=instrumented['output_fps'],
        profiled_frames=post['frames'],stages=stages,nn_path_including_queues_and_join=stats(nn_wait),
        normal_capture_to_telemetry=stats(latency),functions=functions,
        caveats=['Nested function times and parallel node times overlap: do not sum them.',
                 'Timing also perturbs runtime. normal_publication_fps is the uninstrumented throughput.',
                 'NN path includes output transfer and input synchronization; not pure accelerator time.',
                 'RTSP enqueue is not hardware encoding/network/client display latency.'])
    (root/'summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='functions'},indent=2))
    for row in functions[:30]:
        print(f"{Path(row['file']).name}:{row['line']} {row['name']} "
              f"{row['cumulative_per_frame_ms']:.3f}ms/frame inclusive")


if __name__=='__main__':main()
