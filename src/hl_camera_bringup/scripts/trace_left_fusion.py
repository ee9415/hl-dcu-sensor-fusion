import sys,json,numpy as np,inspect
from pathlib import Path
root=Path('artifacts/left_debug_0322');sys.path.insert(0,str(root.resolve()))
from image_boundaries import ImageBoundaryTracker
z=np.load(root/'boundary_debug.npz');r=json.loads(str(z['records_json']))[0]
def unpack(v):
 if isinstance(v,dict) and 'array' in v:return z[v['array']].copy()
 if isinstance(v,list):return [unpack(x) for x in v]
 return v
f=next(c for c in r['calls'] if c['name']=='_fuse');obs=unpack(f['observations']);pred=unpack(f['predicted'])
t=ImageBoundaryTracker();t.history=unpack(r['history_before']);t.last_valid_pair=unpack(r['history_before']);t.scores=r['scores_before'];t.width=unpack(r['width_before'])
source=Path(inspect.getfile(ImageBoundaryTracker)).read_text().splitlines()
def summary(pair):
 common=np.isfinite(pair[0][:,0])&np.isfinite(pair[1][:,0]);gap=pair[1][common,0]-pair[0][common,0]
 return dict(common=int(common.sum()),min_gap=float(gap.min()),smooth=[t._smooth_shape(p) for p in pair],range=[[float(np.nanmin(p[:,0])),float(np.nanmax(p[:,0]))] for p in pair])
print('prior',summary(pred))
for s,o in enumerate(obs):
 print('observation',s,'quality',o[1], 'rows',o[0][np.isfinite(o[0][:,0])].round(2).tolist())
def trace(frame,event,arg):
 if frame.f_code.co_name=='_fuse' and event=='line':
  line=source[frame.f_lineno-1].strip()
  if line=='restored=False':
   print('before_validation',summary(frame.f_locals['output']));print('states',frame.f_locals['states'],'scores',frame.f_locals['scores']);np.save(root/'proposed.npy',np.array(frame.f_locals['output']))
  if line=='if choices:':print('choices',[(v[0],v[1]) for v in frame.f_locals['choices']])
 return trace
sys.settrace(trace)
b=t._fuse(obs,pred,unpack(f['motion']),0,.1)
sys.settrace(None)
print('after',summary([b.image_left,b.image_right]),b.left_state,b.right_state)
np.save(root/'observations.npy',np.array([o[0] for o in obs]));np.save(root/'final.npy',np.array([b.image_left,b.image_right]))
