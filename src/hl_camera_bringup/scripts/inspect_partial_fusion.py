"""Inspect a rejected fusion's observed-side geometry from retained device samples."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from ego_lane_quadratic import fit_quadratic
F=ROOT/'artifacts/partial_spacing_20261110'
rs=[json.loads(l) for l in (F/'code7_device0/frames.jsonl').read_text().splitlines()];rs=[r for r in rs if r.get('source_media_time_s') is not None]
t=ImageBoundaryTracker()
for i,r in enumerate(rs):
 if r['source_media_time_s'] not in [58.,58.2] or i==0:continue
 prev=rs[i-1];side=1;name='right';pts={round(y,6):x for x,y in prev[name+'_boundary_image']};prior=np.column_stack(([pts.get(round(y,6),np.nan) for y in t.rows],t.rows))
 z=np.load(ROOT/f'artifacts/lane_597_20261110/raw_device/{round(r["source_media_time_s"]*10):06d}.npz');curves=json.loads(str(z['curves_json']))
 for idx,raw in enumerate(curves):
  curve=np.asarray(raw);fit=fit_quadratic(curve,180,idx)
  if fit is None:continue
  anchor=np.polyval(fit.coefficients,153)
  if anchor<160:continue
  m=curve[curve[:,2]>0];ys=np.unique(m[:,1]);xs=np.array([np.median(m[m[:,1]==y,0]) for y in ys]);support=(t.rows>=ys.min())&(t.rows<=ys.max())&(np.min(abs(t.rows[:,None]-ys[None,:]),axis=1)<=1.1)
  points=np.column_stack((np.interp(t.rows,ys,xs),t.rows));points[~support,0]=np.nan
  common=support&np.isfinite(prior[:,0])
  if common.sum()<8:continue
  quality=float(np.clip((.55+.4*min(1.,np.ptp(fit.points[:,1])/72))*np.exp(-fit.residual_px/6),.15,.95))
  dt=r['nn_timestamp_s']-prev['nn_timestamp_s'];delta=np.where(common,points[:,0]-prior[:,0],0.)
  fraction=min((1-np.exp(-dt/(.08/3)))*quality,360*dt/max(np.max(abs(delta[common])),1e-9));out=prior.copy();out[common,0]+=fraction*delta[common]
  new=support&~np.isfinite(prior[:,0]);order=np.argsort(t.rows[common]);res=out[common,0]-points[common,0]
  out[new,0]=points[new,0]+np.interp(t.rows[new],t.rows[common][order],res[order])
  p=out[np.isfinite(out[:,0])];sl=np.diff(p[:,0])/np.diff(p[:,1]);j=np.argmax(abs(np.diff(sl)))
  print(json.dumps({'time':r['source_media_time_s'],'candidate':idx,'raw_smooth':t._smooth_shape(points),'prior_smooth':t._smooth_shape(prior),'output_smooth':t._smooth_shape(out),'max_slope':float(max(abs(sl))),'slope_change':float(max(abs(np.diff(sl)))),'triple':p[j:j+3].tolist()}))
