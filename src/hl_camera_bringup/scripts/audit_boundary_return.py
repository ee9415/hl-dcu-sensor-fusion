import argparse,csv,json
from pathlib import Path
parser=argparse.ArgumentParser(description='Audit common-range movement when an inferred boundary becomes observed.')
parser.add_argument('csv',type=Path)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--rate',type=float,default=1.)
args=parser.parse_args()
p=args.csv
with p.open(encoding='utf-8-sig') as f: rows=[r for r in csv.DictReader(f) if r.get('right_boundary_dev')]
rows=[r for r in rows if r['session']==rows[-1]['session']]
events=[]
for side in ('left','right'):
 last=None
 for a,b in zip(rows,rows[1:]):
  if a[side+'_boundary_state']=='observed': last=float(a['nn_timestamp_s'])
  if a[side+'_boundary_state'] not in ('inferred','held') or b[side+'_boundary_state']!='observed': continue
  dt=float(b['nn_timestamp_s'])-float(a['nn_timestamp_s'])
  if not 0<dt<=.5 or int(b['frame_seq'])!=int(a['frame_seq'])+1: continue
  x=dict(zip(json.loads(a['boundary_forward_dev']),json.loads(a[side+'_boundary_dev'])))
  y=dict(zip(json.loads(b['boundary_forward_dev']),json.loads(b[side+'_boundary_dev'])))
  delta=[abs(x[k]-y[k]) for k in x.keys()&y.keys() if x[k] is not None and y[k] is not None]
  if delta: events.append(dict(side=side,frame=int(b['frame_seq']),dt=dt,gap_s=None if last is None else float(b['nn_timestamp_s'])-last,max_delta=max(delta),limit=args.rate*min(dt,.2),common=len(delta)))
result=dict(session=rows[-1]['session'],frames=len(rows),events=events)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(dict(frames=len(rows),events=len(events),over_limit=[e for e in events if e['max_delta']>e['limit']+1e-5],long_gaps=[e for e in events if (e['gap_s'] or 0)>2]),indent=2))
