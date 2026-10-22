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
    left_support: object = None
    right_support: object = None
    left_residual_px: object = None
    right_residual_px: object = None
    left_observation_age_s: float = 0.
    right_observation_age_s: float = 0.

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
                    motion_matrix=self.motion_matrix.tolist(),
                    left_display_residual_px=self.left_residual_px,right_display_residual_px=self.right_residual_px,
                    left_observation_age_s=self.left_observation_age_s,right_observation_age_s=self.right_observation_age_s,
                    corridor_width_m=None,
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
        self.ages=[0.,0.]
        self.pending=[[],[]]
        self.support=[None,None]
        self.width_pair=None
        self.predicted_pair=[None,None]

    def _complete(self, points):
        """Interpolate support, extend endpoint tangents; never extrapolate c2."""
        good=np.isfinite(points).all(axis=1)
        p=points[good]
        if len(p)<8:
            return None
        p=p[np.argsort(p[:,1])]
        y=p[:,1];x=p[:,0]
        if np.ptp(y)<1:
            return None
        values=np.interp(self.rows,y,x)
        for end,indices in ((0,slice(0,5)),(-1,slice(-5,None))):
            slope=float(np.clip(np.polyfit(y[indices],x[indices],1)[0],-2.,2.))
            outside=self.rows<y[0] if end==0 else self.rows>y[-1]
            values[outside]=x[end]+slope*(self.rows[outside]-y[end])
        # An affine road patch model is valid locally, not indefinitely outside
        # the image. Bound latent geometry to a small offscreen margin so repeated
        # expansion cannot grow to thousands of pixels and block reacquisition.
        values=np.clip(values,-32.,352.)
        return np.column_stack((values,self.rows))

    def _warp(self, points, matrix):
        if points is None:
            return None
        good=np.isfinite(points).all(axis=1)
        p=np.column_stack((points[good],np.ones(good.sum())))@matrix.T
        return self._complete(p)

    def _associate(self, observation, prior, side):
        """Reject isolated remote paint; three consistent candidates in five frames."""
        history=self.pending[side]
        candidate=None if observation is None else observation[0]
        history.append(None if candidate is None else candidate.copy())
        del history[:-5]
        if observation is None or prior is None:
            return observation
        valid=np.isfinite(candidate[:,0])&np.isfinite(prior[:,0])
        if valid.sum()<8:
            return None
        innovation=float(np.percentile(abs(candidate[valid,0]-prior[valid,0]),75))
        gate=min(28.,7.+2.*self.ages[side])
        if innovation<=gate:
            return observation
        matches=0
        for past in history:
            if past is None:continue
            common=valid&np.isfinite(past[:,0])
            if common.sum()>=8 and np.percentile(abs(past[common,0]-candidate[common,0]),75)<5:
                matches+=1
        return observation if matches>=3 else None

    def _detect(self, paint, side, prior):
        # Start at any supported near/middle row, not at a fixed C0 anchor.
        seeds=[]
        for row in (168,156,144,132):
            xs=np.flatnonzero(paint[row])
            xs=xs[(xs>3)&(xs<316)]
            if prior is None:
                xs=xs[(xs<175) if side==0 else (xs>145)]
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
                degree=1 if np.ptp(ys[good])<40 else 2
                coeff=np.polynomial.polynomial.polyfit((ys[good]-125)/50,xs[good],degree)
                if degree==2:
                    coeff[2]=np.clip(coeff[2],-12.,12.)
                    coeff[:2]=np.polynomial.polynomial.polyfit((ys[good]-125)/50,
                                xs[good]-coeff[2]*((ys[good]-125)/50)**2,1)
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
            other=self.predicted_pair[1-side]
            if other is not None and self.scores[1-side]>.2:
                common=valid&np.isfinite(other[:,0])
                signed=(other[:,0]-values) if side==0 else (values-other[:,0])
                if common.sum()>=8 and np.median(signed[common])<8:
                    continue  # Never acquire the reliable opposite paint as this side.
            near=values[np.flatnonzero(valid)[0]]
            if prior is None and ((side==0 and near>185) or (side==1 and near<135)):
                continue
            temporal=0.;inside=False
            if prior is not None:
                common=valid&np.isfinite(prior[:,0])
                if common.sum()>=8:
                    error=float(np.percentile(abs(values[common]-prior[common,0]),75))
                    gate=min(28.,7.+2.*self.ages[side])
                    inside=error<=gate
                    temporal=float(np.exp(-error/gate))
            # Prefer plausible nearby paint while a reliable history exists.
            rank=(bool(inside and self.scores[side]>.2),score+.8*temporal+.08*min(1,len(xs)/40))
            if best is None or rank>best[0]:
                best=(rank,np.column_stack((values,self.rows)),score)
        return None if best is None else best[1:]

    def update(self, road, lane, timestamp_s, rgb=None, detections=()):
        if rgb is None:
            raise ValueError('Image boundary tracking requires RGB')
        # Four subpixel samples reduce thin-paint aliasing without full-frame filtering.
        if rgb.shape[:2]==(180,320):
            image=rgb
        else:
            image=np.zeros((180,320,3),np.float32)
            for offset in (.2,.7):
                for offset_x in (.2,.7):
                    yy=np.minimum(((np.arange(180)+offset)*rgb.shape[0]/180).astype(int),rgb.shape[0]-1)
                    xx=np.minimum(((np.arange(320)+offset_x)*rgb.shape[1]/320).astype(int),rgb.shape[1]-1)
                    image+=rgb[yy[:,None],xx[None,:]]*.25
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
        thresholds=np.full(180,125.)
        for row in range(78,180):
            pavement=brightness[row,road[row]]
            if len(pavement):thresholds[row]=max(110.,float(np.median(pavement))+18.)
        paint &= brightness>=thresholds[:,None]
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
            self.motion_velocity[:]=0.;self.ages=[0.,0.];self.pending=[[],[]]
            self.support=[None,None];self.width_pair=None
        motion,motion_score=road_motion(self.previous,gray,allowed[::2,::2])
        if self.previous is not None and motion_score==0 and np.mean(abs(gray-self.previous))>45:
            self.history=[None,None];self.width=None;self.scores=[0.,0.]
            self.motion_velocity[:]=0.;self.ages=[0.,0.];self.pending=[[],[]]
            self.support=[None,None];self.width_pair=None
        self.previous=gray;self.last_time=timestamp_s if np.isfinite(timestamp_s) else None
        identity=np.array([[1.,0.,0.],[0.,1.,0.]])
        step=float(np.clip(dt if np.isfinite(dt) else .1,.01,.2))
        if motion_score:
            # A valid registration maps old pixels to THIS frame. Never lag or
            # attenuate it. Smoothed velocity is only a fallback motion prior.
            alpha=1-np.exp(-step/.25)
            self.motion_velocity=(1-alpha)*self.motion_velocity+alpha*(motion-identity)/step
        else:
            self.motion_velocity*=np.exp(-step/.6)
            motion=identity+self.motion_velocity*step
        predicted=[self._warp(p,motion) for p in self.history]
        self.predicted_pair=predicted
        if self.width_pair is not None:
            self.width_pair=[self._warp(p,motion) for p in self.width_pair]
            if all(p is not None for p in self.width_pair):
                self.width=self.width_pair[1][:,0]-self.width_pair[0][:,0]
        for side in (0,1):
            self.ages[side]+=step
            if self.support[side] is not None and self.history[side] is not None:
                lo,hi=self.support[side]
                old=self.history[side]
                ends=np.column_stack((np.interp([lo,hi],self.rows[::-1],old[::-1,0]),[lo,hi],np.ones(2)))@motion.T
                self.support[side]=(float(ends[:,1].min()),float(ends[:,1].max()))
        previous_ages=self.ages.copy()
        previous_support=self.support.copy()
        observations=[self._associate(self._detect(paint,s,predicted[s]),predicted[s],s) for s in (0,1)]
        output=[];states=[];scores=[];residuals=[None,None];supports=[]
        for side,observation in enumerate(observations):
            prior=predicted[side]
            decay=self.scores[side]*np.exp(-step*(.3 if motion_score else .8))
            support=np.zeros(48,bool)
            if observation is not None:
                measured,score=observation
                support=np.isfinite(measured[:,0])
                lo=float(self.rows[support].min());hi=float(self.rows[support].max())
                target=self._complete(measured)
                if prior is not None:
                    delta=target[:,0]-prior[:,0]
                    fraction=min(.65,8./max(float(np.max(abs(delta))),1e-6))
                    # Blend complete curves without refitting or a local taper:
                    # tapers accumulate kinks between old and observed sections.
                    # Outside support the target is only an endpoint tangent.
                    p=prior.copy();p[:,0]+=fraction*delta
                else:
                    p=target
                residual=float(np.percentile(abs(p[support,0]-measured[support,0]),90))
                state='observed' if residual<=4. else 'aligning'
                score=float(score*np.exp(-residual/6.))
                self.support[side]=(lo,hi)
                self.ages[side]=0.
                residuals[side]=residual
            elif prior is not None:
                p=prior;score=decay
                state='predicted' if motion_score or np.max(abs(motion-identity))>.02 else 'held'
            else:
                p=np.column_stack((np.full(48,np.nan),self.rows));score=0.;state='unavailable'
            output.append(p);states.append(state);scores.append(float(score));supports.append(support)
        # Reject crossing observations only where BOTH have actual support.
        common=supports[0]&supports[1]
        if common.any() and np.any(output[0][common,0]+2>=output[1][common,0]):
            loser=int(scores[1]<scores[0])
            observations[loser]=None;supports[loser][:]=False;residuals[loser]=None
            self.ages[loser]=previous_ages[loser]
            self.support[loser]=previous_support[loser]
            if predicted[loser] is not None:
                output[loser]=predicted[loser].copy()
                states[loser]='predicted' if motion_score else 'held'
                scores[loser]=self.scores[loser]*np.exp(-step*.8)*.5
            else:
                output[loser][:,0]=np.nan;states[loser]='unavailable';scores[loser]=0.
        if states==['observed','observed']:
            common=supports[0]&supports[1]
            if common.sum()>=8 and np.all(output[1][common,0]-output[0][common,0]>2):
                self.width_pair=[p.copy() for p in output]
                self.width=output[1][:,0]-output[0][:,0]
        for side in (0,1):
            # Own warped history is primary. Opposite-side geometry must never
            # overwrite a valid independent prediction on every frame.
            stale=predicted[side] is not None and self.scores[side]<.1 and self.ages[side]>1.
            if (predicted[side] is None or stale) and observations[side] is None and states[1-side]=='observed':
                width=self.width if self.width is not None else np.clip(1.6*(self.rows-65),24,240)
                target=output[1-side][:,0]+np.clip(width,12,280)*(-1 if side==0 else 1)
                target=np.clip(target,-32,352)
                if stale:
                    delta=target-output[side][:,0]
                    output[side][:,0]+=delta*min(.25,8./max(np.max(abs(delta)),1e-6))
                    scores[side]=min(scores[side],.09)
                else:
                    output[side]=np.column_stack((target,self.rows))
                    scores[side]=min(.2,scores[1-side]*.3)
                states[side]='inferred'
                self.support[side]=self.support[1-side]
        if all(o is None for o in observations) and all(p is not None for p in self.history):
            crossed=output[0][:,0]+2>=output[1][:,0]
            visible_counts=[np.count_nonzero((p[:,0]>=0)&(p[:,0]<=319)) for p in output]
            if crossed.any() or min(visible_counts)<8:
                # Registration is local. When its long extrapolation collapses
                # or leaves the view, retain the last drawable pair explicitly
                # as HELD with reduced confidence, not as a measured prediction.
                output=[p.copy() for p in self.history]
                states=['held','held'];scores=[s*.5 for s in scores]
        self.history=[p.copy() if np.isfinite(p[:,0]).sum()>=8 else None for p in output]
        self.scores=scores
        # Do not render extrapolated crossings above the measured region.
        visible=[p.copy() for p in output]
        both=np.isfinite(visible[0][:,0])&np.isfinite(visible[1][:,0])
        crossed=both&(visible[0][:,0]+2>=visible[1][:,0])
        for side in (0,1):
            visible[side][crossed,0]=np.nan
        observed=[s for s,o in enumerate(observations) if o is not None]
        reference='both' if len(observed)==2 else ('left' if observed==[0] else 'right' if observed==[1] else 'none')
        state='tracking' if observed else 'predicted' if any(p is not None for p in self.history) else 'unavailable'
        return ImageBoundaryEstimate(*visible,projection_for((180,320),self.config),*states,*scores,
                                     motion_score,motion,state,reference,*supports,*residuals,*self.ages)
