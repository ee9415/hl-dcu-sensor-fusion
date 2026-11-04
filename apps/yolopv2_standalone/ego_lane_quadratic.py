"""Select ego-lane skeleton groups and fit at most one quadratic per side.

The selector does not fabricate an unobserved side. The separate explicit
add_virtual_side visualization fallback can supply a labeled virtual side.
Coefficients use image pixels, x = a*y**2 + b*y + c.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class QuadraticLane:
    coefficients: np.ndarray
    points: np.ndarray
    group_index: int
    support_rows: int
    residual_px: float
    anchor_x: float
    source_groups: tuple = ()
    is_virtual: bool = False
    width_source: str = 'observed'
    perspective: object = None
    extension: object = None


@dataclass
class EgoLaneQuadratics:
    left: object
    right: object
    reference_y: float
    vehicle_x: float
    reason: str


def complete_partial_lanes(selected, result, image_shape):
    """Use a long opposite lane plus a measured near fragment for extension.

    Observed polynomials are unchanged. The attached separate quadratic is
    position/tangent continuous at the near fragment's far end and constrained
    by the opposite lane's convergence. No image-prior width is used here.
    """
    from dataclasses import replace
    height,width=image_shape[:2]
    left,right=selected.left,selected.right
    # Recover near fragments too short to pass the ordinary full-lane gate.
    for side in ('left','right'):
        near=left if side=='left' else right
        opposite=right if side=='left' else left
        if opposite is None or opposite.is_virtual:
            continue
        sign=-1 if side=='left' else 1
        if near is None or near.is_virtual:
            options=[]
            for index,curve in enumerate(result.group_curves):
                if index in opposite.source_groups:
                    continue
                samples=curve[curve[:,2]>0]
                if len(samples)<6 or np.ptp(samples[:,1])<4:
                    continue
                x,y=samples[:,:2].T
                if y.max()<opposite.points[-1,1]-height*.12 or np.ptp(y)>.65*np.ptp(opposite.points[:,1]):
                    continue
                slope,intercept=np.polyfit(y,x,1)
                if np.quantile(abs(x-(slope*y+intercept)),.9)>2.5:
                    continue
                anchor=float(slope*y.max()+intercept)
                if (side=='left' and anchor>=selected.vehicle_x) or (side=='right' and anchor<=selected.vehicle_x):
                    continue
                gap=sign*(x-np.polyval(opposite.coefficients,y))
                if np.any(gap<width*.08) or np.any(gap>width*.95):
                    continue
                rows=np.arange(int(y.min()),int(y.max())+1,dtype=float)
                fit=QuadraticLane(np.array([0.,slope,intercept]),
                    np.column_stack((slope*rows+intercept,rows)),index,len(y),
                    float(np.median(abs(x-(slope*y+intercept)))),anchor,(index,))
                options.append((abs(anchor-selected.vehicle_x),fit))
            options.sort(key=lambda item:item[0])
            if not options or (len(options)>1 and options[1][0]-options[0][0]<width*.025):
                continue
            near=options[0][1]
        y0=float(near.points[0,1]);far=float(opposite.points[0,1])
        if y0-far<height*.08 or np.ptp(near.points[:,1])>.65*np.ptp(opposite.points[:,1]):
            continue
        x0=float(np.polyval(near.coefficients,y0))
        anchor_width=sign*(x0-float(np.polyval(opposite.coefficients,y0)))
        # Narrow parking-bay/edge pairs are not enough evidence for a full
        # ego-lane extension. Leave the measured short line unchanged.
        if not width*.2<=anchor_width<=width*.95:
            continue
        slope=float(2*near.coefficients[0]*y0+near.coefficients[1])
        horizon=max(height*.25,far-height*.05)
        estimate=estimate_lane_perspective(opposite,result.group_curves,image_shape,side)
        if estimate is not None:
            horizon=estimate['y']
        delta=horizon-y0
        if delta>=-height*.04:
            continue
        curvature=(float(np.polyval(opposite.coefficients,horizon))-x0-slope*delta)/delta**2
        if abs(curvature-opposite.coefficients[0])>.015:
            continue
        coefficients=np.array([curvature,slope-2*curvature*y0,
                               x0-slope*y0+curvature*y0*y0])
        rows=np.arange(int(np.ceil(far)),int(y0)+1,dtype=float)
        x=np.polyval(coefficients,rows)
        gaps=sign*(x-np.polyval(opposite.coefficients,rows))
        if np.any(gaps<=2) or np.any(gaps>width) or np.any(np.diff(gaps)<-.3):
            continue
        extension=QuadraticLane(coefficients,np.column_stack((x,rows)),-1,0,
            float('nan'),float(np.polyval(coefficients,selected.reference_y)),
            near.source_groups,True,'opposite_plus_measured_near_fragment',
            dict(x=float(np.polyval(opposite.coefficients,horizon)),y=horizon,
                 source='fragment_convergence' if estimate is not None else 'opposite_far_direction'))
        completed=replace(near,extension=extension)
        if side=='left':left=completed
        else:right=completed
    if left is selected.left and right is selected.right:
        return selected
    return EgoLaneQuadratics(left,right,selected.reference_y,selected.vehicle_x,'partial_extension')


def estimate_lane_perspective(real, curves, image_shape, side):
    """Estimate a local convergence point from observed longitudinal pieces.

    Intersections are in image space, bounded above observed paint. Curves
    have a local convergence estimate, not a calibrated global vanishing point.
    A short fragment can constrain convergence without determining lane width.
    """
    height,width=image_shape[:2]
    sign=1 if side=='right' else -1
    hypotheses=[]
    for index,curve in enumerate(curves):
        if index in real.source_groups:
            continue
        observed=curve[curve[:,2]>0]
        if len(observed)<6 or np.ptp(observed[:,1])<4:
            continue
        x,y=observed[:,:2].T
        slope,intercept=np.polyfit(y,x,1)
        # Exclude transverse branches and candidates pointing towards the
        # opposite side. Near-horizontal image branches have large dx/dy.
        if abs(slope)>5 or sign*slope<.15:
            continue
        if np.median(abs(x-(slope*y+intercept)))>2:
            continue
        separation=sign*(x-np.polyval(real.coefficients,y))
        if np.any(separation<=3):
            continue
        difference=real.coefficients-np.array([0.,slope,intercept])
        roots=np.roots(np.trim_zeros(difference,'f'))
        far=min(real.points[0,1],y.min())
        for root in roots:
            if abs(root.imag)>1e-6:
                continue
            horizon=float(root.real)
            if not height*.25<=horizon<=min(height*.65,far-2):
                continue
            vx=float(np.polyval(real.coefficients,horizon))
            if not width*.1<=vx<=width*.9:
                continue
            hypotheses.append(dict(x=vx,y=horizon,group=index,rows=len(y),
                                   span=float(np.ptp(y)),scale=float(np.median(separation/(y-horizon)))))
    if not hypotheses:
        return None
    heights=np.array([item['y'] for item in hypotheses])
    median=float(np.median(heights))
    cluster=[item for item in hypotheses if abs(item['y']-median)<=height*.04]
    if not cluster or len(cluster)*2<len(hypotheses):
        return None
    # Large disagreement means that paint identities cannot be reconciled.
    if np.ptp([item['y'] for item in cluster])>height*.06:
        return None
    horizon=float(np.median([item['y'] for item in cluster]))
    return dict(x=float(np.polyval(real.coefficients,horizon)),y=horizon,
                source='observed_fragment_convergence',
                groups=[item['group'] for item in cluster],
                support_rows=sum(item['rows'] for item in cluster),
                confidence='limited' if len(cluster)==1 else 'multiple_fragments',
                width_scale=float(np.median([item['scale'] for item in cluster]))
                    if max(item['span'] for item in cluster)>=height*.12 else None)


def add_virtual_side(selected, result, image_shape, width_at_reference=None,
                     horizon_y=None):
    """Explicit visualization fallback; never relabel virtual paint as real.

    Optional calibrated/history width is preferred. Estimate local convergence
    from observed fragments when possible; otherwise use the declared image
    prior (0.55W at 0.85H, zero width at 0.45H). Short fragments
    only adjust that prior if compatible; they cannot drag it to an outer edge.
    No camera calibration or metric lane width is claimed by this fallback.
    """
    if (selected.left is None)==(selected.right is None):
        return selected
    height,width=image_shape[:2]
    reference=selected.reference_y
    horizon=height*.45 if horizon_y is None else float(horizon_y)
    near_width=width*.55 if width_at_reference is None else float(width_at_reference)
    if not np.isfinite([horizon,near_width]).all() or horizon>=reference or near_width<=0:
        raise ValueError('positive finite width and horizon above reference required')
    real=selected.left if selected.left is not None else selected.right
    sign=1 if selected.left is not None else -1
    perspective=None
    if horizon_y is None:
        perspective=estimate_lane_perspective(real,result.group_curves,image_shape,
                                              'right' if sign==1 else 'left')
        if perspective is not None:
            horizon=perspective['y']
            estimated_width=perspective['width_scale']
            if width_at_reference is None and estimated_width is not None:
                estimate=estimated_width*(reference-horizon)
                if width*.3<=estimate<=width*.8:
                    near_width=estimate
    scale=near_width/(reference-horizon)
    source='image_prior' if width_at_reference is None else 'provided_width'
    if perspective is not None:
        source='estimated_convergence_'+('fragment_width' if perspective['width_scale'] is not None
                 and width_at_reference is None and near_width!=width*.55 else source)
    options=[]
    for index,curve in enumerate(result.group_curves):
        if index in real.source_groups:
            continue
        observed=curve[curve[:,2]>0]
        if len(observed)<6 or np.ptp(observed[:,1])<4:
            continue
        x,y=observed[:,:2].T
        actual=sign*(x-np.polyval(real.coefficients,y))
        predicted=scale*(y-horizon)
        if np.any(predicted<=3) or np.any(actual<=3):
            continue
        slope=np.polyfit(y,x,1)[0]
        expected_slope=2*real.coefficients[0]*float(y.mean())+real.coefficients[1]+sign*scale
        if abs(np.arctan(slope)-np.arctan(expected_slope))>np.deg2rad(25):
            continue
        error=abs(actual-predicted)/np.maximum(6.,predicted*.25)
        if np.quantile(error,.8)>1:
            continue
        candidate=float(np.median(actual/(y-horizon)))
        options.append((float(np.median(error)),candidate,index))
    options.sort()
    anchors=()
    if options and (len(options)==1 or options[1][0]-options[0][0]>=.2):
        _,scale,index=options[0]
        source+='_fragment_anchored'
        anchors=(index,)
    coefficients=real.coefficients.copy()
    coefficients[1]+=sign*scale
    coefficients[2]-=sign*scale*horizon
    rows=real.points[:,1].copy()
    points=np.column_stack((np.polyval(coefficients,rows),rows))
    # Do not display crossed or wholly off-screen fallback geometry.
    widths=scale*(rows-horizon)
    valid=(widths>2)&(points[:,0]>=0)&(points[:,0]<width)
    if np.count_nonzero(valid)<12:
        return selected
    virtual=QuadraticLane(coefficients,points[valid],-1,0,float('nan'),
                          float(np.polyval(coefficients,reference)),anchors,True,source,
                          perspective if perspective is not None else
                          dict(x=float(np.polyval(real.coefficients,horizon)),y=horizon,
                               source='provided_horizon' if horizon_y is not None else 'image_prior',
                               confidence='unmeasured'))
    left,right=(real,virtual) if sign==1 else (virtual,real)
    return EgoLaneQuadratics(left,right,reference,selected.vehicle_x,
                             'right_virtual' if sign==1 else 'left_virtual')


def fit_quadratic(curve, image_height, group_index=0):
    """Robust fit from measured skeleton rows; inferred gap rows get no vote."""
    curve = np.asarray(curve,dtype=float)
    if curve.ndim != 2 or curve.shape[1] != 3 or not np.isfinite(curve).all():
        raise ValueError('expected finite Nx3 x,y,observed samples')
    observed = curve[curve[:,2]>0]
    if len(observed)<12 or np.ptp(observed[:,1])<image_height*.1:
        return None
    x,y = observed[:,:2].T
    z = (y-image_height/2)/image_height
    weights = np.ones(len(y))
    for _ in range(8):
        fit = np.polynomial.polynomial.polyfit(z,x,2,w=np.sqrt(weights))
        error = x-np.polynomial.polynomial.polyval(z,fit)
        scale = max(.6,1.4826*np.median(abs(error-np.median(error))))
        weights = np.minimum(1.,1.5*scale/np.maximum(abs(error),1e-9))
    error = x-np.polynomial.polynomial.polyval(z,fit)
    residual = float(np.median(abs(error)))
    if residual>4.:
        return None
    # Convert normalized coefficients to conventional a,b,c pixel units.
    c,b,a = fit
    coefficients = np.array([a/image_height**2,(b-a)/image_height,c-b/2+a/4])
    rows = np.arange(int(y.min()),int(y.max())+1,dtype=float)
    points = np.column_stack((np.polyval(coefficients,rows),rows))
    return QuadraticLane(coefficients,points,group_index,len(y),residual,float('nan'),(group_index,))


def extend_from_opposite(primary, opposite, curves, image_shape, side):
    """Recover a measured far fragment using perspective width as a gate.

    The opposite lane supplies only association checks. Its samples are never
    added to the target fit, and it never creates a missing-side boundary.
    """
    height,width=image_shape[:2]
    sign=1 if side=='right' else -1
    lo=max(primary.points[0,1],opposite.points[0,1])
    hi=min(primary.points[-1,1],opposite.points[-1,1])
    if hi-lo<height*.08:
        return primary
    rows=np.linspace(lo,hi,32)
    widths=sign*(np.polyval(primary.coefficients,rows)-np.polyval(opposite.coefficients,rows))
    width_fit=np.polyfit(rows,widths,1)
    if width_fit[0]<=0:
        return primary
    options=[]
    for index,curve in enumerate(curves):
        if index in primary.source_groups or index in opposite.source_groups:
            continue
        observed=curve[curve[:,2]>0]
        if len(observed)<8 or np.ptp(observed[:,1])<height*.04:
            continue
        x,y=observed[:,:2].T
        if y.min()>primary.points[0,1]-height*.05 or y.max()>primary.points[0,1]+3:
            continue
        if primary.points[0,1]-y.max()>height*.25 or y.min()<opposite.points[0,1]-height*.05:
            continue
        expected=np.polyval(width_fit,y)
        actual=sign*(x-np.polyval(opposite.coefficients,y))
        if np.any(expected<=2) or np.any(actual<=2):
            continue
        normalized=abs(actual-expected)/np.maximum(6.,expected*.4)
        if np.quantile(normalized,.8)>1:
            continue
        # Longitudinal fragments must agree with the target's direction.
        slope=np.polyfit(y,x,1)[0]
        target_slope=2*primary.coefficients[0]*float(y.mean())+primary.coefficients[1]
        if abs(np.arctan(slope)-np.arctan(target_slope))>np.deg2rad(30):
            continue
        combined=np.concatenate([curves[primary.group_index],curve])
        fit=fit_quadratic(combined,height,primary.group_index)
        if fit is None:
            continue
        # Both observed fragments must actually support the new polynomial.
        if any(np.quantile(abs(c[c[:,2]>0,0]-np.polyval(fit.coefficients,c[c[:,2]>0,1])),.9)>4.
               for c in (curves[primary.group_index],curve)):
            continue
        check=np.linspace(max(fit.points[0,1],opposite.points[0,1]),hi,64)
        corridor=sign*(np.polyval(fit.coefficients,check)-np.polyval(opposite.coefficients,check))
        if np.any(corridor<=2) or np.any(np.diff(corridor)<-.5):
            continue
        # Preserve the already observed near section rather than replacing it
        # with a shape dictated by the opposite boundary.
        if np.max(abs(np.polyval(fit.coefficients,rows)-np.polyval(primary.coefficients,rows)))>5:
            continue
        fit.anchor_x=float(np.polyval(fit.coefficients,height*.85))
        fit.source_groups=(primary.group_index,index)
        options.append((float(np.median(normalized)),fit))
    options.sort(key=lambda item:item[0])
    if not options or (len(options)>1 and options[1][0]-options[0][0]<.2):
        return primary
    return options[0][1]


def select_ego_quadratics(result, image_shape, vehicle_x=None):
    """Select nearest supported left/right groups at a common lower-image row.

    Reject short distant/parking branches, crossing pairs and inverted fits.
    Draw only the supported row span; extrapolation is for selection only and
    limited to 15% of image height. No temporal tracking or virtual boundary.
    """
    height,width = image_shape[:2]
    center = width/2 if vehicle_x is None else float(vehicle_x)
    if not 0 <= center < width:
        raise ValueError('vehicle_x must lie inside the image')
    reference_y = height*.85
    candidates = [[],[]]
    for index,curve in enumerate(result.group_curves):
        fit = fit_quadratic(curve,height,index)
        if fit is None or fit.points[-1,1]<height*.68:
            continue
        if reference_y-fit.points[-1,1]>height*.15:
            continue
        fit.anchor_x = float(np.polyval(fit.coefficients,reference_y))
        if not -width*.1<=fit.anchor_x<=width*1.1:
            continue
        # A candidate must move away from the vehicle towards the bottom;
        # strong reversal usually means a short branch dominated the fit.
        side = 0 if fit.anchor_x<center else 1
        slope = 2*fit.coefficients[0]*reference_y+fit.coefficients[1]
        if (side==0 and slope>.5) or (side==1 and slope<-.5):
            continue
        distance = abs(fit.anchor_x-center)
        score = distance + fit.residual_px*2
        candidates[side].append((score,fit))
    candidates = [sorted(side,key=lambda item:item[0]) for side in candidates]
    for _,left in candidates[0]:
        for _,right in candidates[1]:
            lo=max(left.points[0,1],right.points[0,1])
            hi=min(left.points[-1,1],right.points[-1,1])
            if hi-lo<height*.08:
                continue
            rows=np.linspace(lo,hi,32)
            widths=np.polyval(right.coefficients,rows)-np.polyval(left.coefficients,rows)
            if np.any(widths<=2) or right.anchor_x-left.anchor_x<width*.08:
                continue
            extended_left=extend_from_opposite(left,right,result.group_curves,image_shape,'left')
            extended_right=extend_from_opposite(right,left,result.group_curves,image_shape,'right')
            # Recheck the final pair because both sides may have been extended.
            check=np.linspace(max(extended_left.points[0,1],extended_right.points[0,1]),
                              min(extended_left.points[-1,1],extended_right.points[-1,1]),64)
            if np.all(np.polyval(extended_right.coefficients,check)-np.polyval(extended_left.coefficients,check)>2):
                left,right=extended_left,extended_right
            return EgoLaneQuadratics(left,right,reference_y,center,'pair')
    left=candidates[0][0][1] if candidates[0] else None
    right=candidates[1][0][1] if candidates[1] else None
    if left is not None and right is not None:
        # Keep only the better supported side if the pair is inconsistent.
        if left.support_rows>=right.support_rows:
            right=None
        else:
            left=None
    reason='left_only' if left else 'right_only' if right else 'no_supported_lane'
    return EgoLaneQuadratics(left,right,reference_y,center,reason)


def tracked_quadratics(estimate, image_shape):
    """Adapt the common 320x180 tracking estimate for RGB and TOP drawing.

    No detection or temporal state lives here. Non-observed rows are dashed,
    and the adapter cannot create a side absent from the tracking estimate.
    """
    height, width = image_shape[:2]
    reference = height*.85
    fits = []
    for side in ('left', 'right'):
        points = getattr(estimate, 'image_'+side)
        good = np.isfinite(points).all(axis=1)
        if good.sum() < 8:
            fits.append(None)
            continue
        points = points[good].copy()
        points *= np.array([width/320., height/180.])
        points = points[np.argsort(points[:,1])]
        coefficients = np.polyfit(points[:,1], points[:,0], 2)
        state = getattr(estimate, side+'_state')
        support = getattr(estimate, side+'_support')
        virtual = state not in ('observed', 'partial', 'aligning')
        rows = points[:,1]
        extension = None
        if not virtual and support is not None:
            measured = getattr(estimate, 'image_'+side)[support]
            measured = measured[np.isfinite(measured).all(axis=1)]
            if len(measured):
                low, high = measured[:,1].min()*height/180., measured[:,1].max()*height/180.
                tail = rows[rows < low]
                if len(tail) >= 2:
                    extension = QuadraticLane(coefficients.copy(),
                        np.column_stack((np.polyval(coefficients,tail),tail)), -1, 0, float('nan'),
                        float(np.polyval(coefficients,reference)), is_virtual=True,
                        width_source='tracked_extension')
                rows = rows[(rows >= low) & (rows <= high)]
        if len(rows) < 2:
            fits.append(None)
            continue
        fits.append(QuadraticLane(coefficients, np.column_stack((np.polyval(coefficients,rows),rows)),
            -1, int(np.count_nonzero(support)) if support is not None else 0,
            getattr(estimate, side+'_residual_px'), float(np.polyval(coefficients,reference)),
            is_virtual=virtual, width_source='tracked_'+state, extension=extension))
    return EgoLaneQuadratics(*fits, reference, width/2., 'temporal_'+estimate.state)
