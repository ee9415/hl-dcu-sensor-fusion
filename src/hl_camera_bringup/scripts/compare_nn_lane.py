"""Compare synchronized NN lane input, RGB evidence and selected curves."""
import sys,json
from pathlib import Path
import numpy as np
import cv2
root=Path(sys.argv[1]);sys.path.insert(0,str(root.resolve()))
from lane_boundaries import white_lane_evidence
z=np.load(root/'boundary_debug.npz');records=json.loads(str(z['records_json']))
reports=[]
for i,r in enumerate(records):
 u=next(c for c in r['calls'] if c['name']=='update')
 def arr(k):return z[u[k]['array']]
 rgb=arr('image');lane=arr('lane')>0;paint=arr('paint');allowed=arr('allowed')
 combined=white_lane_evidence(rgb,lane.astype(np.uint8)*255)>0
 rgb_only=white_lane_evidence(rgb,np.zeros_like(lane,dtype=np.uint8))>0
 roi=np.indices(lane.shape)[0]>=72
 counts=lambda m:int(np.count_nonzero(m&roi))
 report=dict(frame=i,nn=counts(lane),rgb=counts(rgb_only),combined=counts(combined),nn_added=counts(combined&~rgb_only),paint=counts(paint),nn_survives=counts(lane&paint),nn_allowed=counts(lane&allowed),nn_bright=counts(lane&(rgb.mean(2)>=u['threshold'])),threshold=u['threshold'])
 reports.append(report)
 report['nn_added_survives']=counts(combined&~rgb_only&paint)
 report['combined_after_brightness']=counts(combined&(rgb.mean(2)>=u['threshold']))
 # Visible near-left marking in this capture; diagnostic box, not ground truth.
 yy,xx=np.indices(lane.shape)
 left_window=(yy>=148)&(yy<=164)&(xx>=97)&(xx<=119)
 report['near_left_window']={name:int(np.count_nonzero(mask&left_window))
   for name,mask in [('nn',lane),('rgb',rgb_only),('combined',combined),('paint',paint),('nn_added_survives',combined&~rgb_only&paint)]}
 if i==0:
  panels=[]
  masks=[None,lane,rgb_only,combined,paint,None]
  labels=['1 RGB','2 YOLOPv2 lane (binary >=0.5)','3 RGB evidence only','4 NN + RGB evidence','5 Filtered paint','6 Final curves + NN pixels']
  for j,(label,mask) in enumerate(zip(labels,masks)):
   im=rgb.copy()
   if mask is not None:im[mask]=(255,0,255)
   if j==5:
    im[lane]=(255,0,255)
    for side,color in [('left',(0,220,255)),('right',(255,180,0))]:
     points=np.array(r['telemetry'][side+'_boundary_image']).astype(np.int32)
     if len(points)>1:cv2.polylines(im,[points],False,color,1)
   im=cv2.resize(im,(640,360),interpolation=cv2.INTER_NEAREST)
   im=cv2.copyMakeBorder(im,28,0,0,0,cv2.BORDER_CONSTANT,value=(20,20,20))
   cv2.putText(im,label,(8,19),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
   panels.append(im)
  cv2.imwrite(str(root/'comparison.jpg'),np.vstack([np.hstack(panels[j:j+2]) for j in (0,2,4)]))
  np.savez_compressed(root/'masks.npz',rgb=rgb,lane=lane,paint=paint,combined=combined,rgb_only=rgb_only,allowed=allowed)
(root/'comparison.json').write_text(json.dumps(reports,indent=2))
print(json.dumps(reports))
