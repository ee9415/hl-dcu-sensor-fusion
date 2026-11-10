"""Explain eligibility gates on exact device masks without altering perception."""
from pathlib import Path
import json,sys
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from ego_lane_quadratic import fit_quadratic
from image_boundaries import ImageBoundaryTracker
F=ROOT/'artifacts/lane_597_20261110/raw_device';records=[]
tracker=ImageBoundaryTracker()
for path in sorted(F.glob('*.npz')):
 z=np.load(path);source=json.loads(str(z['source_json']));curves=json.loads(str(z['curves_json']))
 row={'time':source['source_media_time_s'],'index':source['source_frame_index'],'lane_pixels':int(np.count_nonzero(z['lane'])),'groups':[]}
 for index,raw in enumerate(curves):
  c=np.asarray(raw);measured=c[c[:,2]>0];entry={'group':index,'n':len(measured),'y_min':float(measured[:,1].min()),'y_max':float(measured[:,1].max()),'x_min':float(measured[:,0].min()),'x_max':float(measured[:,0].max())}
  fit=fit_quadratic(c,180,index)
  if fit is None:reason='quadratic_fit_rejected'
  elif fit.points[-1,1]<180*.68:reason='too_far_no_near_support'
  else:
   anchor=float(np.polyval(fit.coefficients,180*.85));entry['anchor']=anchor
   rows=tracker.rows;support=(rows>=fit.points[0,1])&(rows<=fit.points[-1,1]);entry['domain_rows']=int(support.sum())
   if not -32<=anchor<=352:reason='anchor_out_of_range'
   elif support.sum()<8:reason='too_few_domain_rows'
   else:
    ys=np.unique(measured[:,1]);xs=np.array([np.median(measured[measured[:,1]==y,0]) for y in ys]);support &= np.min(abs(rows[:,None]-ys[None,:]),axis=1)<=1.1;entry['supported_rows']=int(support.sum())
    points=np.column_stack((np.interp(rows,ys,xs),rows));points[~support,0]=np.nan
    if support.sum()<8:reason='too_few_measured_rows'
    elif not tracker._smooth_shape(points):
     reason='shape_fold_or_slope_rejected'
     good=points[np.isfinite(points[:,0])];slopes=np.diff(good[:,0])/np.diff(good[:,1]);entry['max_abs_slope']=float(np.max(abs(slopes)))
     entry['max_slope_change']=float(np.max(abs(np.diff(slopes))))
     j=int(np.argmax(abs(np.diff(slopes))));entry['worst_three_points']=good[j:j+3].tolist()

    else:reason='eligible'
  entry['gate']=reason;row['groups'].append(entry)
 records.append(row)
 if 59.3<=row['time']<=60.3:
  img=cv2.resize(z['rgb'],(960,540));mask=cv2.resize(z['lane'],(960,540),interpolation=cv2.INTER_NEAREST)>0;img[mask]=(0,255,255)
  cv2.putText(img,f'ACTUAL DEVICE MASK t={row["time"]:.1f}',(10,25),0,.65,(0,255,255),2)
  cv2.imwrite(str(F.parent/f'device_mask_{row["time"]:g}.jpg'),img)
(F.parent/'mask_gates.json').write_text(json.dumps(records,indent=2))
for r in records:
 if 59.3<=r['time']<=60.3:print(json.dumps(r))
