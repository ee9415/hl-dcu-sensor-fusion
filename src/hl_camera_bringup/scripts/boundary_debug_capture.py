"""Opt-in bounded variable capture; never changes tracker inputs or outputs."""
import base64
import io
import json
import os
from pathlib import Path
import sys
import numpy as np

COUNT = min(100, max(0, int(os.environ.get('BOUNDARY_DEBUG_FRAMES', '0'))))
WARMUP = max(0, int(os.environ.get('BOUNDARY_DEBUG_WARMUP', '60')))
seen = 0
arrays = {}
records = []
DEST = Path('/app/boundary_debug.npz')


def frozen(value, key):
    if isinstance(value, np.ndarray):
        arrays[key] = value.copy()
        return {'array': key}
    if isinstance(value, (list, tuple)):
        return [frozen(v, key+'_'+str(i)) for i,v in enumerate(value)]
    if isinstance(value, (str, bool, int, float, np.generic)):
        return value.item() if isinstance(value, np.generic) else value
    return None


def update(tracker, *args, **kwargs):
    global seen
    seen += 1
    if not COUNT or seen <= WARMUP or len(records) >= COUNT:
        return tracker.update(*args, **kwargs)
    prefix = 'f%03d' % len(records)
    record = {'history_before': frozen(tracker.history,prefix+'_history'),
              'scores_before': list(tracker.scores),
              'width_before': frozen(tracker.width,prefix+'_width'), 'boxes': [[b.xmin,b.ymin,b.xmax,b.ymax] for b in kwargs.get('detections',())], 'calls': []}
    def profile(frame, event, result):
        if event != 'return' or frame.f_code.co_filename != tracker.update.__code__.co_filename:
            return
        name = frame.f_code.co_name
        if name not in ('update','_detect','_complete','_warp','_fuse'):
            return
        call = {'name':name}
        key = prefix+'_c'+str(len(record['calls']))
        allowed = ('side','timestamp_s','image','road','lane','paint','allowed','threshold',
                   'motion','motion_score','predicted','observations','output','states','scores',
                   'common','widths','learned','loser','fraction','final_error','delta','target',
                   'points','coeff','values','prior','best','seeds','support','domain',
                   'measured','quality','existing','fresh','finite','observations')
        for k in allowed:
            if k in frame.f_locals:
                call[k] = frozen(frame.f_locals[k],key+'_'+k)
        if name != 'update': call['return'] = frozen(result,key+'_return')
        record['calls'].append(call)
    previous = sys.getprofile()
    try:
        sys.setprofile(profile)
        estimate = tracker.update(*args, **kwargs)
    finally:
        sys.setprofile(previous)
    record['telemetry'] = estimate.telemetry()
    record['region_segments'] = frozen(tracker.region_segments,prefix+'_segments')
    record['detection_diagnostics'] = tracker.detection_diagnostics
    records.append(record)
    if len(records) == COUNT:
        arrays['records_json'] = np.array(json.dumps(records))
        np.savez_compressed(DEST, **arrays)
        arrays.clear()
        print('BOUNDARY_DEBUG_COMPLETE '+str(COUNT),flush=True)
    return estimate


if __name__ == '__main__' and sys.argv[1:] == ['export']:
    print('BOUNDARY_DATA='+base64.b64encode(DEST.read_bytes()).decode())
