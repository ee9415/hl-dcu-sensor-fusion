"""Analyze bounded profiler captures from unmodified 0.25 tracker math."""
import argparse
import csv
import json
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    args=parser.parse_args();data=np.load(args.directory/'variables.npz')
    records=json.loads(str(data['records_json']))
    def thaw(v):
        if isinstance(v,dict) and 'array' in v:return data[v['array']]
        if isinstance(v,list):return [thaw(x) for x in v]
        return v
    rows=[];images=[];replay_errors=[]
    tracker=ImageBoundaryTracker()
    for i,record in enumerate(records):
        calls=[{k:thaw(v) for k,v in c.items()} for c in record['calls']]
        u=next(c for c in calls if c['name']=='update')
        detects=[c for c in calls if c['name']=='_detect']
        output=u['output'];history=thaw(record['history_before'])
        common=np.isfinite(output[0][:,0])&np.isfinite(output[1][:,0])
        row={'frame':i,'timestamp':u['timestamp_s'],'motion_score':u['motion_score'],
             'state':record['telemetry']['boundary_state'],'reference':record['telemetry']['boundary_reference_side'],
             'output_cross_rows':int(np.sum(output[0][common,0]+6>=output[1][common,0])),
             'actual_cross_rows':int(np.sum(output[0][common,0]>=output[1][common,0])),
             'threshold':u['threshold'],'paint_pixels':int(u['paint'].sum()),
             'road_pixels':int(u['road'].sum())}
        image=u['image'].astype(np.uint8);overlay=image.copy()
        observed=[]
        for side,c in enumerate(detects):
            label=('left','right')[side];candidate=c['return'];observed.append(candidate)
            row[label+'_state']=record['telemetry'][label+'_boundary_state']
            row[label+'_confidence']=record['telemetry'][label+'_boundary_confidence']
            replay=tracker._detect(c['paint'],side,c['prior'])
            if candidate is not None:
                replay_errors.append(float(np.nanmax(abs(replay[0]-candidate[0]))))
                p,score=candidate;good=np.isfinite(p[:,0]);extended=tracker._complete(p)
                row[label+'_raw_score']=score
                row[label+'_support_y_min']=float(p[good,1].min());row[label+'_support_y_max']=float(p[good,1].max())
                row[label+'_extension_rows']=int((~good).sum())
                row[label+'_extended_x_min']=float(extended[:,0].min());row[label+'_extended_x_max']=float(extended[:,0].max())
                coeff=np.polynomial.polynomial.polyfit((p[good,1]-125)/50,p[good,0],2)
                row[label+'_c2']=float(coeff[2])
                row[label+'_display_candidate_error_px']=float(np.median(abs(output[side][good,0]-p[good,0])))
                for x,y in p[good]:cv2.circle(overlay,(int(x),int(y)),1,(0,0,255),-1)
            prior=history[side]
            row[label+'_same_as_previous']=bool(prior is not None and np.allclose(output[side],prior,equal_nan=True))
            p=output[side];good=np.isfinite(p).all(axis=1)
            xy=np.rint(p[good]).astype(int)
            inside=(xy[:,0]>=0)&(xy[:,0]<320)&(xy[:,1]>=0)&(xy[:,1]<180)
            row[label+'_offroad_rows']=int((~inside).sum()+np.sum(~u['road'][xy[inside,1],xy[inside,0]]))
            cv2.polylines(overlay,[xy],False,((0,220,255),(255,180,0))[side],1)
        if len(observed)==2 and all(v is not None for v in observed):
            pair=[tracker._complete(v[0]) for v in observed]
            row['raw_completed_cross_rows']=int(np.sum(pair[0][:,0]+6>=pair[1][:,0]))
        rows.append(row)
        mask=np.repeat((u['paint'].astype(np.uint8)*255)[:,:,None],3,axis=2)
        tile=np.hstack([cv2.resize(v,(480,270)) for v in (image,mask,overlay)])
        cv2.putText(tile,f'frame {i} | RGB / paint / output + red candidate samples',
                    (8,18),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        images.append(tile)
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with (args.directory/'frame_variables.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,keys);writer.writeheader();writer.writerows(rows)
    for start in range(0,len(images),6):cv2.imwrite(str(args.directory/f'variables_{start//6:02d}.jpg'),np.vstack(images[start:start+6]))
    crossing=next((r['frame'] for r in rows if r['actual_cross_rows']),0)
    selected=sorted(set([max(0,crossing-1),crossing,min(len(images)-1,crossing+1),len(images)-1]))
    cv2.imwrite(str(args.directory/'crossing_trace.jpg'),np.vstack([images[i] for i in selected]))
    summary={'frames':len(rows),'max_detect_replay_error_px':max(replay_errors,default=None),
        'output_cross_frames':[r['frame'] for r in rows if r['output_cross_rows']],
        'actual_cross_frames':[r['frame'] for r in rows if r['actual_cross_rows']],
        'completed_candidate_cross_frames':[r['frame'] for r in rows if r.get('raw_completed_cross_rows',0)],
        'both_reference_but_held':[r['frame'] for r in rows if r['reference']=='both' and 'held' in (r['left_state'],r['right_state'])],
        'max_offroad_rows':{s:max(r[s+'_offroad_rows'] for r in rows) for s in ('left','right')},
        'largest_candidate_display_error':{s:max((r.get(s+'_display_candidate_error_px',0) for r in rows)) for s in ('left','right')}}
    (args.directory/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))


if __name__=='__main__':main()
