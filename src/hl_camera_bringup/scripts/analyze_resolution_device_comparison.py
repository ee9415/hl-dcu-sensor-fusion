"""Summarize paired device runs without treating model agreement as accuracy."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def percentile(values, q):
    return sorted(values)[int((len(values)-1)*q)] if values else None


def box_iou(a, b):
    w = max(0., min(a['xmax'], b['xmax'])-max(a['xmin'], b['xmin']))
    h = max(0., min(a['ymax'], b['ymax'])-max(a['ymin'], b['ymin']))
    intersection = w*h
    area = lambda c: max(0., c['xmax']-c['xmin'])*max(0., c['ymax']-c['ymin'])
    union = area(a)+area(b)-intersection
    return intersection/union if union else 0.


def read_run(path):
    report = json.loads((path/'report.json').read_text())
    frames = [json.loads(line) for line in (path/'frames.jsonl').read_text().splitlines()]
    frames = [f for f in frames if f.get('input_source') == 'pc']
    delay = [(f['sent_monotonic_s']-f['capture_monotonic_est_s'])*1000 for f in frames
             if f.get('sent_monotonic_s') is not None and f.get('capture_monotonic_est_s') is not None]
    epoch = min(f['source_media_epoch'] for f in frames)
    first = {f['source_frame_index']: f for f in frames if f['source_media_epoch']==epoch}
    return report, frames, delay, first


def compare(a, b):
    common = sorted(set(a)&set(b)); distances = {'left': [], 'right': []}
    same_counts = 0; matched = 0; counts = [0, 0]; ious = []
    for index in common:
        x, y = a[index], b[index]
        for side in distances:
            pa = {int(pt[1]): pt[0] for pt in x.get(side+'_boundary_image', []) if pt[0] is not None}
            pb = {int(pt[1]): pt[0] for pt in y.get(side+'_boundary_image', []) if pt[0] is not None}
            rows = set(pa)&set(pb)
            if rows:
                distances[side].append(statistics.mean(abs(pa[r]-pb[r]) for r in rows))
        aa, bb = x.get('objects', []), y.get('objects', [])
        same_counts += len(aa)==len(bb)
        counts[0] += len(aa); counts[1] += len(bb)
        unused = set(range(len(bb)))
        for obj in sorted(aa, key=lambda o: o['confidence'], reverse=True):
            candidates = [(box_iou(obj,bb[j]),j) for j in unused if obj['label']==bb[j]['label']]
            if candidates:
                overlap, j = max(candidates)
                if overlap >= .5:
                    unused.remove(j);matched += 1;ious.append(overlap)
    return dict(matched_source_frames=len(common), same_object_count_frames=same_counts,
                object_count_320=counts[0], object_count_192=counts[1], object_pairs_iou50=matched,
                median_paired_object_iou=statistics.median(ious) if ious else None,
                boundary_frame_mean_difference_px={k:dict(median=statistics.median(v) if v else None,
                                                          p95=percentile(v,.95), frames=len(v))
                                                   for k,v in distances.items()})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    args=p.parse_args();root=args.directory
    summaries={};data={}
    for block,height in ((1,320),(2,192),(3,192),(4,320)):
        for start in (60,303):
            name=f'block{block}_height{height}_sec{start}'
            report,frames,delays,first=read_run(root/name)
            summaries[name]=dict(output_fps=report['output_fps'], input_clock_fps=report['input_clock_fps'],
                                 latency_p50_ms=statistics.median(delays),latency_p95_ms=percentile(delays,.95),
                                 frames=len(frames), sequence_mismatches=report['pc_sequence_mismatches'],
                                 camera_recovered=report['camera_recovered'],
                                 left_states=dict(Counter(f['left_boundary_state'] for f in frames)),
                                 right_states=dict(Counter(f['right_boundary_state'] for f in frames)))
            data[name]=first
    comparisons={}
    for start in (60,303):
        for first,second in ((1,2),(4,3)):
            comparisons[f'sec{start}_block{first}_vs_{second}']=compare(data[f'block{first}_height320_sec{start}'],data[f'block{second}_height192_sec{start}'])
    output=dict(runs=summaries, matched_frame_comparisons=comparisons,
                limitation='Agreement is not labelled accuracy; tracker histories and capture times can differ.')
    (root/'comparison_summary.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    main()
