import numpy as np,json,cv2
from pathlib import Path
p=Path('artifacts/left_debug_0322');z=np.load(p/'boundary_debug.npz');rs=json.loads(str(z['records_json']))
for i,r in enumerate(rs):
 u=next(c for c in r['calls'] if c['name']=='update'); f=next(c for c in r['calls'] if c['name']=='_fuse'); d=[c for c in r['calls'] if c['name']=='_detect' and c['side']==0]
 print(i,r['telemetry']['left_boundary_state'],r['telemetry']['boundary_detection_diagnostics'][0], 'left_obs',d[0]['return'] is not None,'restore',r['telemetry']['boundary_restored_pair'])
 if i==0:
  image=z[u['image']['array']];cv2.imwrite(str(p/'rgb.jpg'),cv2.resize(image,(960,540)))
  mask=z[u['paint']['array']]; image=image.copy();image[mask]=(0,0,255);cv2.imwrite(str(p/'paint.jpg'),cv2.resize(image,(960,540)))
