"""Compare baseline/current full detector cost on identical regression masks."""
import json,subprocess,sys,time,types
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
old=types.ModuleType('stripe_baseline');sys.modules[old.__name__]=old
source=subprocess.check_output(['git','show','4b32752:apps/yolopv2_standalone/image_boundaries.py'],cwd=root,text=True)
exec(compile(source,'baseline','exec'),old.__dict__)
report={}
for name,side in [('left_straight',0),('right_upper_gap',1)]:
    z=np.load(root/'apps/yolopv2_standalone/tests/data'/f'{name}.npz')
    report[name]={}
    for label,cls in [('baseline',old.ImageBoundaryTracker),('current',ImageBoundaryTracker)]:
        t=cls();t.nn_rank_mask=z['lane']>0
        samples=[]
        for i in range(53):
            start=time.perf_counter();t._detect_full(z['paint'],side,z['prior'])
            if i>=3:samples.append((time.perf_counter()-start)*1000)
        report[name][label]=dict(median_ms=float(np.median(samples)),p95_ms=float(np.percentile(samples,95)))
print(json.dumps(report,indent=2))
