"""Compare geometry on source-time matched device telemetry; not labelled accuracy."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];out=ROOT/'artifacts/opposite_spacing_20261110'
reports={}
for label,path in [('before',ROOT/'artifacts/lane_597_20261110/device0/frames.jsonl'),('after',out/'device0/frames.jsonl')]:
 frames=[json.loads(l) for l in path.read_text().splitlines()];near=[]
 for r in frames:
  t=r.get('source_media_time_s')
  if t is None or not 57<=t<=63:continue
  item={'time':t,'states':[r['left_boundary_state'],r['right_boundary_state']],'sequence_match':r['frame_seq']==r['nn_seq'],'diagnostics':r['boundary_detection_diagnostics']}
  left={round(y,6):x for x,y in r['left_boundary_image']};right={round(y,6):x for x,y in r['right_boundary_image']};ys=sorted(left.keys()&right.keys())
  if len(ys)>=8:
   width=np.array([right[y]-left[y] for y in ys]);fit=np.polyfit(ys,width,1);residual=width-np.polyval(fit,ys)
   item['gap_affine_residual_max_px']=float(np.max(abs(residual)))
  near.append(item)
 reports[label]=near
(out/'geometry_comparison.json').write_text(json.dumps(reports,indent=2))
for label,near in reports.items():
 supported=[r for r in near if r['states'][0]=='inferred' and r['states'][1]=='observed']
 print(label,'near_frames',len(near),'missing_left_visible_right',len(supported))
 for r in supported:
  if 59<=r['time']<=61: print(json.dumps(r))
 if label=='after':
  all_guided=[d for r in near for d in r['diagnostics'] if d.get('opposite_shape_spacing')]
  print('guided frames',len(all_guided),'max_target_error_px',max((d['spacing_target_error_px'] for d in all_guided),default=None))
