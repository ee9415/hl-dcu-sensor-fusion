"""Fixed image start to the exact midpoint of displayed lanes in TOP space."""
import numpy as np
from image_boundaries import quadratic_display
from virtual_guidance import GuidanceEstimate


def _map(points, matrix):
    h=np.column_stack((points,np.ones(len(points))))@matrix.T
    if not np.isfinite(h).all() or np.any(abs(h[:,2])<1e-9):return None
    if np.any(h[:,2]*h[0,2]<=0):return None
    return h[:,:2]/h[:,2,None]


def _intersection(coef, domain, matrix, row):
    """Intersect the projected quadratic with one horizontal TOP row exactly."""
    q=matrix[1]-row*matrix[2]
    poly=np.array([q[0]*coef[0],q[0]*coef[1]+q[1],q[0]*coef[2]+q[2]])
    roots=np.roots(np.trim_zeros(poly,'f')) if np.any(abs(poly)>1e-12) else []
    hits=[]
    for value in roots:
        if abs(value.imag)>1e-7 or not domain[0]-1e-7<=value.real<=domain[1]+1e-7:continue
        y=float(value.real);xy=_map(np.array([[np.polyval(coef,y),y]]),matrix)
        if xy is not None and abs(xy[0,1]-row)<1e-5:hits.append(xy[0,0])
    return hits[0] if len(hits)==1 else None


def displayed_midpoint_guidance(boundaries, projection, config, valid=None, near_row=419):
    """One TOP x(y) quadratic with zero lateral tangent at the fixed start.

    The farthest common visible row below the header (72px) supplies the target.
    No separately learned width or independently smoothed endpoint is used.
    """
    if not config.enabled:return GuidanceEstimate()
    curves=[];displays=[]
    for side in ('left','right'):
        p,c=quadratic_display(getattr(boundaries,'image_'+side),getattr(boundaries,side+'_support'))
        good=np.isfinite(p).all(axis=1)
        if c is None or good.sum()<8:return GuidanceEstimate(reason='displayed_lane_pair_missing')
        curves.append((c,(float(p[good,1].min()),float(p[good,1].max()))))
        displays.append(p[good])
    matrix=np.linalg.inv(projection)
    start_image=np.array([319*config.start_x,179*config.start_y])
    start=_map(np.array([start_image]),matrix)
    if start is None:return GuidanceEstimate(reason='invalid_top_start')
    start=start[0];height,width=(480,640) if valid is None else valid.shape
    if not 0<=start[0]<width or not 0<=start[1]<=near_row+1:
        return GuidanceEstimate(reason='fixed_start_outside_top')
    target=None;edges=None
    for row in range(72,min(int(start[1]-20),near_row)+1):
        xs=[_intersection(c,d,matrix,row) for c,d in curves]
        if any(x is None for x in xs):continue
        if not 0<=xs[0]<xs[1]<=width-1 or xs[1]-xs[0]<6:continue
        middle=(xs[0]+xs[1])/2
        row_points=np.array([[x,row] for x in (xs[0],middle,xs[1])])
        source=_map(row_points,projection)
        if source is None or np.any(source<0) or np.any(source>np.array([319.,179.])):continue
        if valid is not None and not all(valid[row,int(round(x))] for x in (xs[0],middle,xs[1])):continue
        target=np.array([middle,float(row)]);edges=np.array([[xs[0],row],[xs[1],row]],float);break
    if target is None:return GuidanceEstimate(reason='no_common_visible_top_lane_row')
    t=np.linspace(0.,1.,128);span=start[1]-target[1]
    top=np.column_stack((start[0]+(target[0]-start[0])*t*t,start[1]-span*t))
    image=_map(top,projection)
    if image is None:return GuidanceEstimate(reason='invalid_top_path_projection')
    # Keep the exact existing image start, rather than accumulating roundoff.
    image[0]=start_image
    a=(target[0]-start[0])/(span*span)
    coefficients=np.array([a,-2*a*start[1],start[0]+a*start[1]*start[1]])
    real=tuple(getattr(boundaries,s+'_state')=='observed' for s in ('left','right'))
    current=all(real)
    return GuidanceEstimate(valid=current,reason='displayed_top_midpoint' if current else 'displayed_top_midpoint_estimated',
        source='displayed_pair',left=displays[0],right=displays[1],path=image if current else None,
        display_path=None if current else image,target=image[-1].copy(),real=real,
        width_source='displayed_pair',boundary_state='current' if current else 'displayed_estimate',
        road_supported=False,path_top=top,target_top=target,start_top=start,target_edges_top=edges,
        top_coefficients=coefficients,path_geometry='top_quadratic')
