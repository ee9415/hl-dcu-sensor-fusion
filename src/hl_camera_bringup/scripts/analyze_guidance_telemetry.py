"""Check real device guidance geometry, reference rejection and temporal bounds."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('frames',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    frames=[json.loads(line) for line in a.frames.read_text().splitlines() if line.strip()]
    frames=[r for r in frames if r.get('input_source')=='pc']
    polynomial_error=midpoint_error=start_error=max_rate=0.
    rate_violations=reference_violations=comparisons=0
    previous=None
    previous_boundary=None
    boundary_comparisons=boundary_rate_violations=0
    boundary_max_rate=0.
    display_path_rate_violations=0
    display_path_max_rate=display_path_error=display_midpoint_error=0.
    for r in frames:
        epoch=tuple(r.get(k) for k in ('source_epoch','source_media_epoch','processing_epoch'))
        if r.get('guidance_boundaries_available'):
            if previous_boundary is not None and epoch==previous_boundary[1]:
                old=previous_boundary[0];dt=r['nn_timestamp_s']-old['nn_timestamp_s']
                if 0<dt<=1:
                    change=0.
                    path_change=0.
                    for key in ('guidance_left_image','guidance_right_image','guidance_display_path_image'):
                        if not r.get(key) or not old.get(key):continue
                        now=np.array(r[key]);before=np.array(old[key])
                        lo=max(now[:,1].min(),before[:,1].min())
                        hi=min(now[:,1].max(),before[:,1].max())
                        if hi<=lo:continue
                        dc=(np.polynomial.polynomial.polyfit(now[:,1],now[:,0],2)-
                            np.polynomial.polynomial.polyfit(before[:,1],before[:,0],2))
                        ys=[lo,hi]
                        if abs(dc[2])>1e-15:
                            vertex=-dc[1]/(2*dc[2])
                            if lo<vertex<hi:ys.append(vertex)
                        delta=float(np.max(abs(np.polynomial.polynomial.polyval(ys,dc))))
                        if key=='guidance_display_path_image':path_change=delta
                        else:change=max(change,delta)
                    boundary_max_rate=max(boundary_max_rate,change/dt)
                    boundary_comparisons+=1
                    boundary_rate_violations+=int(change>40*min(dt,.2)+1e-6)
                    display_path_max_rate=max(display_path_max_rate,path_change/dt)
                    display_path_rate_violations+=int(path_change>40*min(dt,.2)+1e-6)
            previous_boundary=(r,epoch)
        else:
            previous_boundary=None
        if r.get('guidance_display_path_image'):
            shown=np.array(r['guidance_display_path_image'])
            cf=np.polynomial.polynomial.polyfit(shown[:,1],shown[:,0],2)
            display_path_error=max(display_path_error,float(np.max(abs(np.polynomial.polynomial.polyval(shown[:,1],cf)-shown[:,0]))))
            display_midpoint_error=max(display_midpoint_error,float(abs(shown[-1,0]-.5*(r['guidance_left_image'][0][0]+r['guidance_right_image'][0][0]))))
        if not r.get('guidance_valid'):
            previous=None
            continue
        path=np.array(r['guidance_path_image']);left=np.array(r['guidance_left_image']);right=np.array(r['guidance_right_image'])
        c=np.array(r['guidance_coefficients'])
        polynomial_error=max(polynomial_error,float(np.max(abs(np.polynomial.polynomial.polyval(path[:,1],c)-path[:,0]))))
        midpoint_error=max(midpoint_error,float(abs(path[-1,0]-.5*(left[0,0]+right[0,0]))))
        start_error=max(start_error,float(np.max(abs(path[0]-[159.5,179*.995]))))
        for side in ('left','right'):
            reference_violations+=int(r['guidance_'+side+'_real'] and
                (r[side+'_boundary_state']!='observed' or r[side+'_boundary_confidence']<.55))
        epoch=tuple(r.get(k) for k in ('source_epoch','source_media_epoch','processing_epoch'))
        if previous is not None and epoch==previous[1]:
            old=previous[0];dt=r['nn_timestamp_s']-old['nn_timestamp_s']
            if 0<dt<=1 and np.isclose(path[-1,1],old['guidance_target'][1]):
                y=np.linspace(path[-1,1],path[0,1],64)
                change=0.
                for key in ('guidance_left_image','guidance_right_image','guidance_path_image'):
                    now=np.array(r[key]);before=np.array(old[key])
                    nc=np.polynomial.polynomial.polyfit(now[:,1],now[:,0],2)
                    oc=np.polynomial.polynomial.polyfit(before[:,1],before[:,0],2)
                    change=max(change,float(np.max(abs(np.polynomial.polynomial.polyval(y,nc-oc)))))
                rate=change/dt;max_rate=max(max_rate,rate);comparisons+=1
                rate_violations+=int(change>40*min(dt,.2)+1e-6)
        previous=(r,epoch)
    report=dict(frames=len(frames),valid=sum(bool(r.get('guidance_valid')) for r in frames),
        boundaries_available=sum(bool(r.get('guidance_boundaries_available')) for r in frames),
        boundary_display_states=dict(Counter(r.get('guidance_boundary_state') for r in frames)),
        boundary_rate_comparisons=boundary_comparisons,boundary_max_rate_px_s=boundary_max_rate,
        boundary_rate_violations=boundary_rate_violations,
        path_display_available=sum(bool(r.get('guidance_path_display_available')) for r in frames),
        display_path_max_rate_px_s=display_path_max_rate,
        display_path_rate_violations=display_path_rate_violations,
        display_polynomial_error_px=display_path_error,display_midpoint_error_px=display_midpoint_error,
        reasons=dict(Counter(r.get('guidance_reason') for r in frames)),
        sources=dict(Counter(r.get('guidance_source') for r in frames)),
        polynomial_error_px=polynomial_error,midpoint_error_px=midpoint_error,
        start_error_px=start_error,reference_violations=reference_violations,
        rate_comparisons=comparisons,max_rate_px_s=max_rate,rate_violations=rate_violations)
    report['passed']=bool(report['valid'] and polynomial_error<1e-7 and midpoint_error<1e-7 and
        start_error<1e-7 and display_path_error<1e-7 and display_midpoint_error<1e-7 and
        reference_violations==rate_violations==boundary_rate_violations==display_path_rate_violations==0)
    a.output.write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
    if not report['passed']:raise RuntimeError('Guidance checks failed')


if __name__=='__main__':main()
