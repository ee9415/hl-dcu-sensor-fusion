"""Summarize actual device evidence without treating model agreement as accuracy."""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parents[3]/'artifacts/curvature_20261110'
summary={}
for folder in ('device0','device378','device428'):
 p=ROOT/folder/'frames.jsonl'
 if not p.exists():continue
 rows=[json.loads(s) for s in p.read_text().splitlines()]
 rows=[r for r in rows if r.get('input_source')=='pc' and r.get('source_media_time_s') is not None]
 targets=(44.,45.,45.3,46.,46.6,58.5,59.7) if folder=='device0' else (378.,378.5,380.) if folder=='device378' else (428.,428.5,430.)
 selected=[]
 for target in targets:
  if not rows:continue
  r=min(rows,key=lambda r:abs(r['source_media_time_s']-target))
  selected.append({k:r.get(k) for k in ('source_media_time_s','left_boundary_state','right_boundary_state','left_display_coefficients','right_display_coefficients','boundary_detection_diagnostics')})
 report=ROOT/folder/'report.json'
 summary[folder]=dict(frames=len(rows),selected=selected,polynomial_error_max=0.)
 errors=[]
 for r in rows:
  for side in ('left','right'):
   xy=np.asarray(r.get(side+'_boundary_image',[]));coef=r.get(side+'_display_coefficients')
   if xy.size and coef:errors.extend(abs(xy[:,0]-np.polyval(coef,xy[:,1])).tolist())
 if errors:summary[folder]['polynomial_error_max']=max(errors)
 if report.exists():
  rep=json.loads(report.read_text());summary[folder].update({k:rep[k] for k in ('lossless','sender','errors')})
(ROOT/'device_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
