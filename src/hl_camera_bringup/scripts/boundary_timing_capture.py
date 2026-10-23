"""Bounded, opt-in device timings; no frame arrays or line-level profiling."""
import json,sys,time
from pathlib import Path
records=[];seen=0
def update(tracker,*args,**kwargs):
    global seen
    seen+=1
    if seen<=30 or len(records)>=100:return tracker.update(*args,**kwargs)
    timings={};originals={}
    def timed(name,fn):
        def call(*a,**k):
            start=time.perf_counter()
            try:return fn(*a,**k)
            finally:timings[name]=timings.get(name,0.)+(time.perf_counter()-start)*1000
        return call
    module=sys.modules[tracker.__class__.__module__];motion=module.road_motion
    for name in ('_detect_regions','_detect_full','_fuse','_temporal_fragments'):
        originals[name]=getattr(tracker,name);setattr(tracker,name,timed(name,originals[name]))
    module.road_motion=timed('road_motion',motion)
    start=time.perf_counter()
    try:estimate=tracker.update(*args,**kwargs)
    finally:
        module.road_motion=motion
        for name in originals:delattr(tracker,name)
    timings['total']=(time.perf_counter()-start)*1000
    records.append(timings)
    if len(records)==100:Path('/app/boundary_timing.json').write_text(json.dumps(records))
    return estimate
