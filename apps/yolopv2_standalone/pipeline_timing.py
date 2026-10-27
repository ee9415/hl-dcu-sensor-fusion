"""Explicit opt-in callback timing; normal application does not import this."""
from collections import defaultdict, deque
import cProfile
import json
from pathlib import Path
import pstats
import time

STATES=defaultdict(lambda:dict(count=0,events=deque(maxlen=2000),profile=None))
DETAIL=Path('/app/pipeline_timing_detail').exists()


def begin(stage,sequence,active=True):
    if not active:return None
    state=STATES[stage]
    if stage=='postprocess' and DETAIL:
        if state['profile'] is None:state['profile']=cProfile.Profile()
        state['profile'].enable()
    return stage,sequence,time.monotonic()


def finish(token):
    if token is None:return
    stage,sequence,start=token;state=STATES[stage]
    if stage=='postprocess' and DETAIL:state['profile'].disable()
    end=time.monotonic();state['count']+=1
    state['events'].append(dict(sequence=sequence,start=start,end=end,ms=(end-start)*1000))
    if state['count']%10:return
    result=dict(stage=stage,frames=state['count'],events=list(state['events']))
    if stage=='postprocess' and DETAIL:
        entries=[]
        for (file,line,name),(_,calls,own,total,_) in pstats.Stats(state['profile']).stats.items():
            entries.append(dict(file=file,line=line,name=name,calls=calls,own_ms=own*1000,
                cumulative_ms=total*1000,cumulative_per_frame_ms=total*1000/state['count']))
        result['functions']=sorted(entries,key=lambda x:x['cumulative_ms'],reverse=True)[:80]
    target=Path('/app/pipeline_timing_'+stage+'.json')
    temporary=target.with_suffix('.json.new');temporary.write_text(json.dumps(result));temporary.replace(target)
