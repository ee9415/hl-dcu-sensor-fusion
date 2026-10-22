"""Image-first white boundaries with sparse road-motion compensation (NumPy only)."""
from dataclasses import dataclass
import numpy as np
from lane_boundaries import BoundaryConfig, projection_for, white_lane_evidence


@dataclass
class ImageBoundaryEstimate:
    image_left: np.ndarray
    image_right: np.ndarray
    projection: np.ndarray
    left_state: str
    right_state: str
    left_confidence: float
    right_confidence: float
    motion_confidence: float
    motion_matrix: np.ndarray
    state: str
    reference_side: str

    @property
    def left_valid(self):
        return bool(np.isfinite(self.image_left).all(axis=1).sum() >= 8)

    @property
    def right_valid(self):
        return bool(np.isfinite(self.image_right).all(axis=1).sum() >= 8)

    @property
    def valid(self):
        return self.left_valid or self.right_valid

    @property
    def pair_valid(self):
        common = np.isfinite(self.image_left[:,0]) & np.isfinite(self.image_right[:,0])
        return bool(common.sum() >= 8 and np.all(self.image_right[common,0] > self.image_left[common,0]))

    def telemetry(self, config=None):
        def points(p):
            return [[float(x),float(y)] for x,y in p if np.isfinite([x,y]).all()]
        return dict(boundary_valid=self.valid,boundary_pair_valid=self.pair_valid,
                    left_boundary_valid=self.left_valid,right_boundary_valid=self.right_valid,
                    boundary_state=self.state,boundary_reference_side=self.reference_side,
                    left_boundary_state=self.left_state,right_boundary_state=self.right_state,
                    left_boundary_confidence=self.left_confidence,right_boundary_confidence=self.right_confidence,
                    boundary_confidence_kind='heuristic_not_probability',boundary_units='image_pixels_320x180',
                    boundary_model='image_motion',left_boundary_image=points(self.image_left),
                    right_boundary_image=points(self.image_right),motion_confidence=self.motion_confidence,
                    motion_matrix=self.motion_matrix.tolist(),corridor_width_m=None,
                    normal_width_error_dev=None,left_boundary_coefficients=[],right_boundary_coefficients=[],
                    boundary_forward_dev=[],left_boundary_dev=[],right_boundary_dev=[],
                    corridor_feasible=None,clearance_sufficient=None)


def road_motion(previous, current, allowed):
    """Robust affine patch flow at 160x90; reject ambiguous/large scene changes."""
    identity=np.array([[1.,0.,0.],[0.,1.,0.]])
    if previous is None:
        return identity,0.
    yy,xx=np.mgrid[44:82:7,12:150:10]
    x,y=xx.ravel(),yy.ravel()
    offsets=np.arange(-3,4)
    patches=previous[y[:,None,None]+offsets[None,:,None],x[:,None,None]+offsets[None,None,:]]
    texture=patches.std(axis=(1,2))
    chosen=np.flatnonzero((texture>7)&allowed[y,x])
    chosen=chosen[np.argsort(texture[chosen])[-40:]]
    if len(chosen)<8:
        return identity,0.
    x,y,patches=x[chosen],y[chosen],patches[chosen]
    dy,dx=np.mgrid[-5:6,-6:7]
    dx,dy=dx.ravel(),dy.ravel()
    trials=current[y[:,None,None,None]+dy[None,:,None,None]+offsets[None,None,:,None],
                   x[:,None,None,None]+dx[None,:,None,None]+offsets[None,None,None,:]]
    patches=patches-patches.mean(axis=(1,2),keepdims=True)
    trials=trials-trials.mean(axis=(2,3),keepdims=True)
    error=np.mean(abs(trials-patches[:,None]),axis=(2,3))
    best=np.argmin(error,axis=1)
    best_error=error[np.arange(len(x)),best]
    other=error.copy()
    nearby=(abs(dx[None,:]-dx[best,None])<=1)&(abs(dy[None,:]-dy[best,None])<=1)
    other[nearby]=np.inf
    good=(best_error<18)&(other.min(axis=1)>best_error*1.12+0.3)
    if good.sum()<8:
        return identity,0.
    source=np.column_stack((x,y,np.ones(len(x))))
    target=np.column_stack((x+dx[best],y+dy[best]))
    for _ in range(3):
        if good.sum()<8:
            return identity,0.
        matrix=np.linalg.lstsq(source[good],target[good],rcond=None)[0].T
        residual=np.linalg.norm(source@matrix.T-target,axis=1)
        good &= residual<1.8
    if good.sum()<8 or np.ptp(y[good])<14 or np.ptp(x[good])<40:
        return identity,0.
    if np.max(abs(matrix[:,:2]-np.eye(2)))>.12:
        return identity,0.
    matrix[:,2]*=2  # Motion in the 320x180 tracking coordinates.
    return matrix,float(good.sum()/len(x))


class ImageBoundaryTracker:
    def __init__(self, config=None):
        self.config=config or BoundaryConfig.load()  # Projection is display-only.
        self.rows=np.linspace(172,78,48)
        self.history=[None,None]
        self.scores=[0.,0.]
        self.previous=None
        self.last_time=None
        self.width=None
        self.last_candidate_count=0
        self.motion_velocity=np.zeros((2,3))
        self.missing_seconds=0.

    def _complete(self, points):
        """Keep a full curve model; visibility must not erode temporal memory."""
        good=np.isfinite(points).all(axis=1)
        if good.sum()<8:
            return None
        coeff=np.polynomial.polynomial.polyfit((points[good,1]-125)/50,points[good,0],2)
        values=np.polynomial.polynomial.polyval((self.rows-125)/50,coeff)
        return np.column_stack((values,self.rows))

    def _warp(self, points, matrix):
        if points is None:
            return None
        good=np.isfinite(points).all(axis=1)
        p=np.column_stack((points[good],np.ones(good.sum())))@matrix.T
        order=np.argsort(p[:,1]);p=p[order]
        if len(p)<8:
            return None
        result=self._complete(p)
        if result is None or np.count_nonzero((result[:,0]>0)&(result[:,0]<319))<12:
            return points.copy()
        return result

    def _detect(self, paint, side, prior):
        # Start at any supported near/middle row, not at a fixed C0 anchor.
        seeds=[]
        for row in (168,156,144,132):
            xs=np.flatnonzero(paint[row])
            xs=xs[(xs>8)&(xs<312)&((xs<175) if side==0 else (xs>145))]
            if len(xs):
                groups=np.split(xs,np.flatnonzero(np.diff(xs)>2)+1)
                seeds.extend((row,float(np.mean(g))) for g in groups if len(g)<=12)
        best=None
        for start,seed in seeds[:28]:
            ys,xs=[],[];value=seed;slope=0.;last=start
            for y in range(start,76,-2):
                options=np.flatnonzero(paint[y])
                predicted=value+slope*(y-last)
                gate=min(16.,5.+.25*(last-y))
                options=options[abs(options-predicted)<gate]
                if not len(options):
                    continue
                value=float(options[np.argmin(abs(options-predicted))])
                ys.append(y);xs.append(value);last=y
                if len(ys)>=4:
                    slope=float(np.clip((xs[-1]-xs[-4])/(ys[-1]-ys[-4]),-2.,2.))
            if len(ys)<10 or np.ptp(ys)<20:
                continue
            ys,xs=np.array(ys),np.array(xs)
            good=np.ones(len(xs),bool)
            for _ in range(3):
                if good.sum()<10:break
                coeff=np.polynomial.polynomial.polyfit((ys[good]-125)/50,xs[good],2)
                residual=abs(np.polynomial.polynomial.polyval((ys-125)/50,coeff)-xs)
                good=residual<3.5
            if good.sum()<10 or np.ptp(ys[good])<20:
                continue
            residual=float(np.mean(residual[good]))
            score=float(.45*min(1,np.ptp(ys[good])/75)+.3*good.mean()+.25*np.exp(-residual/2))
            values=np.polynomial.polynomial.polyval((self.rows-125)/50,coeff)
            # Only short extrapolation; don't fabricate an unconstrained horizon curve.
            valid=(self.rows>=ys[good].min()-6)&(self.rows<=ys[good].max()+8)&(values>0)&(values<319)
            values[~valid]=np.nan
            if valid.sum()<12 or score<.65:
                continue
            near=values[np.flatnonzero(valid)[0]]
            if (side==0 and near>185) or (side==1 and near<135):
                continue
            temporal=0.
            if prior is not None:
                common=valid&np.isfinite(prior[:,0])
                if common.sum()>=8:
                    temporal=float(np.exp(-np.median(abs(values[common]-prior[common,0]))/20))
            # Long continuous paint dominates; past position is only a tie-breaker.
            rank=score+.10*temporal+.08*min(1,len(xs)/40)
            if best is None or rank>best[0]:
                best=(rank,np.column_stack((values,self.rows)),score)
        return None if best is None else best[1:]

    def update(self, road, lane, timestamp_s, rgb=None, detections=()):
        if rgb is None:
            raise ValueError('Image boundary tracking requires RGB')
        # Full RGB resize matches segmentation coordinates; fixed-cost CPU path.
        image=rgb[np.linspace(0,rgb.shape[0]-1,180).astype(int)[:,None],
                  np.linspace(0,rgb.shape[1]-1,320).astype(int)[None,:]]
        if lane.shape!=(180,320):
            lane=lane[np.linspace(0,lane.shape[0]-1,180).astype(int)[:,None],
                      np.linspace(0,lane.shape[1]-1,320).astype(int)[None,:]]
        paint=white_lane_evidence(image,lane)>0
        road=road[np.linspace(0,road.shape[0]-1,180).astype(int)[:,None],
                  np.linspace(0,road.shape[1]-1,320).astype(int)[None,:]]>0
        # Segmentation often excludes paint itself. A narrow dilation admits
        # markings next to pavement without admitting treetops/vehicle roofs.
        padded=np.pad(road,((2,2),(5,5)))
        allowed=np.zeros_like(road)
        for dy in (0,2,4):
            for dx in (0,2,5,8,10):
                allowed |= padded[dy:dy+180,dx:dx+320]
        brightness=image.mean(axis=2)
        pavement=brightness[road & (np.indices(road.shape)[0]>90)]
        threshold=max(125.,float(np.median(pavement))+25.) if len(pavement) else 150.
        paint &= brightness>=threshold
        for box in detections:
            a,b=np.clip([int(box.xmin*320),int(np.ceil(box.xmax*320))],0,320)
            c,d=np.clip([int(box.ymin*180),int(np.ceil(box.ymax*180))],0,180)
            # Bounding boxes include pavement at their bottom corners. Keep a
            # small bottom strip available for paint alongside parked tires.
            d=max(c,d-int((d-c)*.10))
            allowed[c:d,a:b]=False
        paint &= allowed
        gray=image[::2,::2].mean(axis=2).astype(np.float32)
        dt=.1 if self.last_time is None else timestamp_s-self.last_time
        reset=not np.isfinite(timestamp_s) or dt<=0 or dt>1.
        if reset:
            self.history=[None,None];self.previous=None;self.width=None;self.scores=[0.,0.]
            self.motion_velocity[:]=0.;self.missing_seconds=0.
        motion,motion_score=road_motion(self.previous,gray,allowed[::2,::2])
        # A discontinuous source cut must not carry old boundaries into a new scene.
        if self.previous is not None and motion_score==0 and np.mean(abs(gray-self.previous))>45:
            self.history=[None,None];self.width=None;self.scores=[0.,0.]
            self.motion_velocity[:]=0.;self.missing_seconds=0.
        self.previous=gray;self.last_time=timestamp_s if np.isfinite(timestamp_s) else None
        identity=np.array([[1.,0.,0.],[0.,1.,0.]])
        step=float(np.clip(dt if np.isfinite(dt) else .1,.01,.2))
        if motion_score:
            # Filter motion velocity, so noisy alternating flow cannot reverse
            # the boundary instantly. This is visual inertia, not measured IMU.
            alpha=1-np.exp(-step/.25)
            self.motion_velocity=(1-alpha)*self.motion_velocity+alpha*(motion-identity)/step
        else:
            self.motion_velocity*=np.exp(-step/.6)
        # Unobserved motion decays to a retained shape instead of drifting away.
        motion=identity+self.motion_velocity*step*np.exp(-self.missing_seconds/1.5)
        predicted=[self._warp(p,motion) for p in self.history]
        observations=[self._detect(paint,s,predicted[s]) for s in (0,1)]
        self.missing_seconds=0. if any(o is not None for o in observations) else self.missing_seconds+step
        output=[];states=[];scores=[]
        for s,observation in enumerate(observations):
            prior=predicted[s]
            if observation is not None:
                p,score=observation
                if prior is not None:
                    common=np.isfinite(p[:,0])&np.isfinite(prior[:,0])
                    old_good=np.isfinite(prior[:,0]);new_good=np.isfinite(p[:,0])
                    old_coeff=np.polynomial.polynomial.polyfit((prior[old_good,1]-125)/50,prior[old_good,0],2)
                    new_coeff=np.polynomial.polynomial.polyfit((p[new_good,1]-125)/50,p[new_good,0],2)
                    old_curve=np.polynomial.polynomial.polyval((self.rows-125)/50,old_coeff)
                    new_curve=np.polynomial.polynomial.polyval((self.rows-125)/50,new_coeff)
                    delta=new_curve-old_curve
                    # Blend whole polynomials, not separately clipped fragments:
                    # refitting inconsistent fragments can amplify a tiny clamp.
                    fraction=min(.65,8./max(float(np.max(abs(delta[old_good|new_good]))),1e-6))
                    p[new_good,0]=(old_curve+fraction*delta)[new_good]
                    final_error=np.median(abs(delta[common])) if common.any() else 30.
                    state='observed' if final_error<6 else 'aligning'
                    score*=float(np.exp(-max(0.,final_error-3)/30))
                else:
                    state='observed'
            elif prior is not None:
                p=prior;score=self.scores[s]*np.exp(-max(.01,dt)*(.25 if motion_score else .8))
                state='predicted' if motion_score else 'held'
            else:
                p=np.column_stack((np.full(48,np.nan),self.rows));score=0.;state='unavailable'
            good=np.isfinite(p[:,0])
            if good.sum()>=8:
                # Blending partial supports pointwise can create a kink at their
                # junction. Refit the entire displayed curve, unconstrained C0.
                coeff=np.polynomial.polynomial.polyfit((p[good,1]-125)/50,p[good,0],2)
                p[good,0]=np.polynomial.polynomial.polyval((p[good,1]-125)/50,coeff)
                p=self._complete(p)
            output.append(p);states.append(state);scores.append(float(score))
        common=np.isfinite(output[0][:,0])&np.isfinite(output[1][:,0])
        if common.sum()>=8 and np.any(output[0][common,0]+6>=output[1][common,0]):
            loser=int(scores[1]<scores[0])
            if predicted[loser] is not None:
                output[loser]=predicted[loser].copy();states[loser]='held';scores[loser]*=.5
            else:
                output[loser][:,0]=np.nan;states[loser]='unavailable';scores[loser]=0.
        if all(o is not None for o in observations):
            widths=output[1][:,0]-output[0][:,0]
            valid=np.isfinite(widths)&(widths>12)
            if valid.sum()>=12:
                learned=np.interp(self.rows[::-1],self.rows[valid][::-1],widths[valid][::-1])[::-1]
                self.width=learned if self.width is None else .9*self.width+.1*learned
        # Only a previously observed width profile may supply an unseen side.
        for s in (0,1):
            if observations[s] is None and observations[1-s] is not None:
                # Bootstrap is a low-confidence image perspective prior, never
                # a physical 2.1m constraint. Replace it with measured spacing.
                width=self.width if self.width is not None else np.clip(1.6*(self.rows-65),24,240)
                target=output[1-s][:,0]+width*(-1 if s==0 else 1)
                if predicted[s] is not None:
                    delta=target-predicted[s][:,0]
                    fraction=min(.15,3./max(float(np.max(abs(delta))),1e-6))
                    target=predicted[s][:,0]+fraction*delta
                output[s]=np.column_stack((target,self.rows))
                scores[s]=min(scores[1-s]*.4,max(scores[s],.05));states[s]='inferred'
        common=np.isfinite(output[0][:,0])&np.isfinite(output[1][:,0])
        if common.any() and np.any(output[0][common,0]+6>=output[1][common,0]):
            # Keep the last coherent pair if an observation would cross sides.
            if all(p is not None for p in self.history):
                output=[p.copy() for p in self.history];states=['held','held']
        self.history=[p.copy() if np.isfinite(p[:,0]).sum()>=8 else None for p in output]
        self.scores=scores
        observed=[s for s,o in enumerate(observations) if o is not None]
        reference='both' if len(observed)==2 else ('left' if observed==[0] else 'right' if observed==[1] else 'none')
        state='tracking' if observed else 'predicted' if any(p is not None for p in self.history) else 'unavailable'
        return ImageBoundaryEstimate(*output,projection_for((180,320),self.config),*states,*scores,
                                     motion_score,motion,state,reference)
