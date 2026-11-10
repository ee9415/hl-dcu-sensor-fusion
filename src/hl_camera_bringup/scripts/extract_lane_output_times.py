"""Extract actual rendered frames by the source time burned into the camera output."""
from pathlib import Path
import ast,json,re,argparse
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args()
module=ast.parse((ROOT/'apps/yolopv2_standalone/yolopv2_node.py').read_text())
font_node=next(n.value for n in module.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FONT_5X7' for t in n.targets))
font={ast.literal_eval(k):(('00000',)*7 if ast.literal_eval(k)==' ' else ast.literal_eval(v)) for k,v in zip(font_node.keys,font_node.values) if ast.literal_eval(k) in '0123456789.S '}
templates={c:np.array([[v=='1' for v in row] for row in font[c]],float) for c in '0123456789.S '}
telemetry=[json.loads(l) for l in (a.folder/'frames.jsonl').read_text().splitlines()]
known={round(r['source_media_time_s'],1) for r in telemetry if r.get('source_media_time_s') is not None}
report={}
for stream,y in [('result',12),('top',10)]:
 cap=cv2.VideoCapture(str(a.folder/(stream+'_actual.mp4')));assert cap.isOpened()
 best={};seen=[];index=0
 while True:
  ok,img=cap.read()
  if not ok:break
  chars=[];scores=[]
  for i in range(6):
   x=10+i*12;crop=img[y:y+14,x:x+10].astype(float)
   yellow=(crop[:,:,2]>100)&(crop[:,:,1]>90)&(crop[:,:,0]<.65*np.minimum(crop[:,:,1],crop[:,:,2]))
   cells=yellow.reshape(7,2,5,2).mean(axis=(1,3))
   score,c=min((float(np.mean(abs(cells-glyph))),c) for c,glyph in templates.items())
   chars.append(c);scores.append(score)
  text=''.join(chars);match=re.match(r'^(\d+\.\d)S',text)
  if match:
   t=float(match.group(1));score=float(np.mean(scores[:match.end()]))
   if score<.25 and t in known:
    seen.append(t)
    for target in [45.,45.3,46.,46.6,54.5,55.,55.5,56.,56.5,57.,58.,58.5,59.,59.7,60.,60.5,378.,378.5,428.,428.5]:
     rank=(abs(t-target),score)
     if target not in best or rank<best[target]['rank']:
      best[target]={'rank':rank,'time':t,'frame':index,'header':text,'image':img.copy()}
  index+=1
 cap.release()
 saved=[]
 for target,r in best.items():
  if abs(r['time']-target)>.3:continue
  name=f'actual_{stream}_{target:g}.jpg';cv2.imwrite(str(a.folder/name),r['image'])
  saved.append({'requested':target,'source_time':r['time'],'video_frame':r['frame'],'match_error':r['rank'][1],'path':name})
 report[stream]={'decoded_frames':len(seen),'min':min(seen,default=None),'max':max(seen,default=None),'saved':saved}
(a.folder/'actual_frames.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
