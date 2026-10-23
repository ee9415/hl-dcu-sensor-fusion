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
    restored_pair: bool = False
    detection_diagnostics: object = None

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
        def coefficients(p):
            good=np.isfinite(p[:,0])
            return np.polynomial.polynomial.polyfit(p[good,1],p[good,0],2).tolist() if good.sum()>=3 else None
        return dict(image_curve_degree=2,
                    left_image_coefficients=coefficients(self.image_left),
                    right_image_coefficients=coefficients(self.image_right),
                    boundary_valid=self.valid,boundary_pair_valid=self.pair_valid,
                    left_boundary_valid=self.left_valid,right_boundary_valid=self.right_valid,
                    boundary_state=self.state,boundary_reference_side=self.reference_side,
                    left_boundary_state=self.left_state,right_boundary_state=self.right_state,
                    left_boundary_confidence=self.left_confidence,right_boundary_confidence=self.right_confidence,
                    boundary_confidence_kind='heuristic_not_probability',boundary_units='image_pixels_320x180',
                    boundary_model='image_motion',left_boundary_image=points(self.image_left),
                    left_display_residual_px=self.left_residual_px,
                    right_display_residual_px=self.right_residual_px,
                    boundary_restored_pair=self.restored_pair,
                    boundary_detection_diagnostics=self.detection_diagnostics,
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
        self.last_valid_pair=None
        self.side_missing=np.zeros(2)
        self.observed_streak=np.zeros(2,dtype=int)
        self.width_samples=np.zeros(48,dtype=int)
        self.detection_diagnostics=[{},{}]

    def _complete(self, points):
        """Interpolate observed support only; never extrapolate a quadratic."""
        good=np.isfinite(points).all(axis=1)
        if good.sum()<8:
            return None
        p=points[good];p=p[np.argsort(p[:,1])]
        values=np.interp(self.rows,p[:,1],p[:,0])
        values[(self.rows<p[0,1])|(self.rows>p[-1,1])]=np.nan
        return np.column_stack((values,self.rows))

    def _quadratic(self, points):
        if points is None:return None
        p=points.copy();good=np.isfinite(p[:,0])
        if good.sum()<3:return p
        u=(p[:,1]-125.)/50.
        c=np.polynomial.polynomial.polyfit(u[good],p[good,0],2)
        p[good,0]=np.polynomial.polynomial.polyval(u[good],c)
        return p

    def _warp(self, points, matrix):
        if points is None:
            return None
        good=np.isfinite(points).all(axis=1)
        p=np.column_stack((points[good],np.ones(good.sum())))@matrix.T
        order=np.argsort(p[:,1]);p=p[order]
        if len(p)<8:
            return None
        # Preserve previously displayed support during loss, without re-fitting
        # its shape or expanding into previously unseen rows.
        values=np.interp(self.rows,p[:,1],p[:,0])
        values[~good]=np.nan
        result=np.column_stack((values,self.rows))
        if result is None or np.count_nonzero((result[:,0]>0)&(result[:,0]<319))<12:
            return points.copy()
        return self._quadratic(result)

    @staticmethod
    def _smooth_shape(points):
        good=np.isfinite(points[:,0])
        p=points[good]
        if len(p)<8:return False
        slope=np.diff(p[:,0])/np.diff(p[:,1])
        # Broad image-space limits: reject folds/seams, retain normal curves.
        return bool(np.max(abs(slope))<=4 and
                    (len(slope)<2 or np.max(abs(np.diff(slope)))<=1.8))

    @staticmethod
    def _valid_pair(pair):
        if any(p is None for p in pair):
            return False
        common=np.isfinite(pair[0][:,0])&np.isfinite(pair[1][:,0])
        return bool(common.sum()>=8 and all(ImageBoundaryTracker._smooth_shape(p) for p in pair) and np.all(pair[1][common,0]-pair[0][common,0]>6)
                    and all(np.count_nonzero(np.isfinite(p[:,0])&(p[:,0]>=0)&(p[:,0]<=319))>=8 for p in pair))

    def _detect(self, paint, side, prior):
        regional=self._detect_regions(paint,side,prior)
        regional_stats=dict(self.detection_diagnostics[side])
        legacy=self._detect_legacy(paint,side,prior)
        if regional is not None and (legacy is None or regional[1]>legacy[1]):
            self.detection_diagnostics[side]=regional_stats
            return regional
        return legacy

    def _detect_regions(self, paint, side, prior):
        """Independent overlapping bands in the bottom 60%, then joint fit."""
        from itertools import product, combinations
        bands=((72,124),(100,160),(136,178))
        candidates=[]
        for lo,hi in bands:
            segments=[];seeds=[]
            for y in range(hi-2,lo+5,-4):
                xs=np.flatnonzero(paint[y])
                xs=xs[(xs>8)&(xs<312)&((xs<185) if side==0 else (xs>135))]
                groups=np.split(xs,np.flatnonzero(np.diff(xs)>2)+1) if len(xs) else []
                for g in groups:
                    if len(g)<=12:seeds.append((y,float(np.mean(g))))
            for start,x in seeds[:24]:
                pts=[];last=start;slope=0.
                for y in range(start,lo-1,-2):
                    if last-y>6:break
                    options=np.flatnonzero(paint[y]);prediction=x+slope*(y-last)
                    options=options[abs(options-prediction)<4]
                    if not len(options):continue
                    x=float(options[np.argmin(abs(options-prediction))]);pts.append((x,y));last=y
                    if len(pts)>=2:
                        back=min(4,len(pts));slope=(pts[-1][0]-pts[-back][0])/(pts[-1][1]-pts[-back][1])
                        slope=float(np.clip(slope,-2,2))
                if len(pts)<3:continue
                q=np.array(pts);span=np.ptp(q[:,1])
                if span<4:continue
                c=np.polynomial.polynomial.polyfit((q[:,1]-125)/50,q[:,0],1)
                error=np.max(abs(np.polynomial.polynomial.polyval((q[:,1]-125)/50,c)-q[:,0]))
                if error>2:continue
                if any(abs(np.median(q[:,0])-np.median(v[:,0]))<3 and abs(np.median(q[:,1])-np.median(v[:,1]))<8 for v in segments):continue
                segments.append(q)
            segments.sort(key=lambda q:np.ptp(q[:,1]),reverse=True)
            candidates.append(segments[:4])
        best=None
        for count in (3,2):
            for band_ids in combinations(range(3),count):
                for group in product(*(candidates[k] for k in band_ids)):
                    # Repeated support in overlap is not a second segment.
                    ordered=sorted(group,key=lambda q:q[:,1].mean())
                    compatible=True
                    for far,near in zip(ordered,ordered[1:]):
                        if near[:,1].min()<=far[:,1].max()+2:
                            compatible=False;break
                        sf=np.polyfit(far[:,1],far[:,0],1)[0];sn=np.polyfit(near[:,1],near[:,0],1)[0]
                        a=far[np.argmax(far[:,1])];b=near[np.argmin(near[:,1])]
                        join=(b[0]-a[0])/(b[1]-a[1])
                        if not min(sf,sn)-.25<=join<=max(sf,sn)+.25:
                            compatible=False;break
                    if not compatible:continue
                    q=np.vstack(group);u=(q[:,1]-125)/50
                    if len(q)<10 or np.ptp(q[:,1])<20:continue
                    c=np.polynomial.polynomial.polyfit(u,q[:,0],2)
                    errors=abs(np.polynomial.polynomial.polyval(u,c)-q[:,0])
                    if errors.max()>2:continue
                    values=np.polynomial.polynomial.polyval((self.rows-125)/50,c)
                    domain=(self.rows>=q[:,1].min())&(self.rows<=q[:,1].max())
                    values[~domain]=np.nan
                    curve=np.column_stack((values,self.rows))
                    if not self._smooth_shape(curve):continue
                    if np.any((values[domain]<0)|(values[domain]>319)):continue
                    support=domain&(np.min(abs(self.rows[:,None]-q[None,:,1]),axis=1)<=1)
                    score=float(min(.85,.55+.2*min(1,len(q)/30))*np.exp(-errors.mean()/3))
                    rank=len(q)-errors.mean()*5
                    if prior is not None:
                        common=support&np.isfinite(prior[:,0])
                        if common.any():rank+=2*np.exp(-np.median(abs(values[common]-prior[common,0]))/20)
                    if best is None or rank>best[0]:best=(rank,curve,score,support)
        if best is None:return None
        self.detection_diagnostics[side]={'regional':True,'bands':[len(v) for v in candidates],
                                        'observed_rows':int(best[3].sum())}
        return best[1:]

    def _detect_legacy(self, paint, side, prior):
        full=self._detect_full(paint,side,prior)
        if full is not None:return full
        # The fallback only supplies supported linear fragments. Leave the
        # established long-curve detector unchanged.
        # Start at any supported near/middle row, not at a fixed C0 anchor.
        seeds=[]
        for row in (174,168,156,144,132):
            xs=np.flatnonzero(paint[row])
            xs=xs[(xs>8)&(xs<312)&((xs<175) if side==0 else (xs>145))]
            if len(xs):
                groups=np.split(xs,np.flatnonzero(np.diff(xs)>2)+1)
                seeds.extend((row,float(np.mean(g))) for g in groups if len(g)<=12)
        stats=dict(seeds=len(seeds),short=0,fit=0,quality=0,side=0,accepted=0)
        self.detection_diagnostics[side]=stats
        best=None;partial=None
        for start,seed in seeds[:28]:
            ys,xs=[],[];value=seed;slope=0.;last=start
            for y in range(start,76,-2):
                options=np.flatnonzero(paint[y])
                predicted=value+slope*(y-last)
                gap=last-y
                if gap>14:break
                gate=5. if gap<=4 else 6.
                if gap>4 and len(ys)<3:continue
                options=options[abs(options-predicted)<gate]
                if not len(options):
                    continue
                if len(ys)>=3 and gap>4:
                    trial=(options-value)/(y-last)
                    options=options[abs(trial-slope)<=.7]
                    if not len(options):continue
                value=float(options[np.argmin(abs(options-predicted))])
                ys.append(y);xs.append(value);last=y
                if len(ys)>=2:
                    back=min(4,len(ys))
                    slope=float(np.clip((xs[-1]-xs[-back])/(ys[-1]-ys[-back]),-2.,2.))
            if len(ys):
                stats["short"]+=1
                # A short fragment may correct existing geometry only. It is
                # never used to initialize a whole boundary or learn its width.
                if prior is not None and start>=160 and len(ys)>=5 and np.ptp(ys)>=8:
                    # Prefer the near contiguous fragment for correction. A
                    # linked further dash (>=3 samples, direction-gated above)
                    # is kept only if the pair is one smoothly bending marking:
                    # the slope across the gap lies between both dash slopes.
                    # A laterally offset (parallel) stripe fails this check.
                    ys,xs=np.array(ys),np.array(xs)
                    near=ys>=start-10
                    if near.sum()<5:continue
                    coef=np.polynomial.polynomial.polyfit(ys[near],xs[near],1)
                    err=float(np.mean(abs(np.polynomial.polynomial.polyval(ys[near],coef)-xs[near])))
                    chained=False
                    if (~near).sum()>=3:
                        far=np.polynomial.polynomial.polyfit(ys[~near],xs[~near],1)[1]
                        a,b=np.argmin(ys[near]),np.argmax(ys[~near])
                        join=(xs[near][a]-xs[~near][b])/(ys[near][a]-ys[~near][b])
                        if min(coef[1],far)-.25<=join<=max(coef[1],far)+.25:
                            # Support rows only; the quadratic never extends past them.
                            chain=np.polynomial.polynomial.polyfit(ys,xs,2)
                            chain_res=abs(np.polynomial.polynomial.polyval(ys,chain)-xs)
                            if chain_res.max()<=1.5:
                                coef,err,chained=chain,float(chain_res.mean()),True
                    if not chained:
                        ys,xs=ys[near],xs[near]
                    values=np.polynomial.polynomial.polyval(self.rows,coef)
                    valid=(self.rows>=min(ys))&(self.rows<=max(ys))
                    # Do not label an unobserved gap as current paint support.
                    valid &= np.min(abs(self.rows[:,None]-np.array(ys)[None,:]),axis=1)<=1
                    common=valid&np.isfinite(prior[:,0])
                    if (err<=1.5 and common.sum()>=4 and
                            np.median(abs(values[common]-prior[common,0]))<=100):
                        values[~valid]=np.nan
                        score=.55*np.exp(-err/3)
                        candidate=(score,np.column_stack((values,self.rows)),score)
                        if partial is None or score>partial[0]:partial=candidate
                continue
        if best is None and partial is not None:
            stats['partial']=1
            return partial[1:]
        return None if best is None else best[1:]

    def _detect_full(self, paint, side, prior):
        # Start at any supported near/middle row, not at a fixed C0 anchor.
        seeds=[]
        for row in (168,156,144,132):
            xs=np.flatnonzero(paint[row])
            xs=xs[(xs>8)&(xs<312)&((xs<175) if side==0 else (xs>145))]
            if len(xs):
                groups=np.split(xs,np.flatnonzero(np.diff(xs)>2)+1)
                seeds.extend((row,float(np.mean(g))) for g in groups if len(g)<=12)
        stats=dict(seeds=len(seeds),short=0,fit=0,quality=0,side=0,accepted=0)
        self.detection_diagnostics[side]=stats
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
                stats["short"]+=1
                continue
            ys,xs=np.array(ys),np.array(xs)
            good=np.ones(len(xs),bool)
            for _ in range(3):
                if good.sum()<10:break
                coeff=np.polynomial.polynomial.polyfit((ys[good]-125)/50,xs[good],2)
                residual=abs(np.polynomial.polynomial.polyval((ys-125)/50,coeff)-xs)
                good=residual<3.5
            if good.sum()<10 or np.ptp(ys[good])<20:
                stats["fit"]+=1
                continue
            residual=float(np.mean(residual[good]))
            score=float(.45*min(1,np.ptp(ys[good])/75)+.3*good.mean()+.25*np.exp(-residual/2))
            values=np.polynomial.polynomial.polyval((self.rows-125)/50,coeff)
            # Only short extrapolation; don't fabricate an unconstrained horizon curve.
            valid=(self.rows>=ys[good].min()-6)&(self.rows<=ys[good].max()+8)&(values>0)&(values<319)
            values[~valid]=np.nan
            if valid.sum()<12 or score<.65:
                stats["quality"]+=1
                continue
            near=values[np.flatnonzero(valid)[0]]
            if (side==0 and near>185) or (side==1 and near<135):
                stats["side"]+=1
                continue
            stats["accepted"]+=1
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
        # Paint cannot exceed 255: on bright (overcast/concrete) pavement a
        # fixed +25 margin rejects worn markings that still form a local ridge.
        # The margin shrinks with remaining headroom; ridge contrast still applies.
        median=float(np.median(pavement)) if len(pavement) else 125.
        threshold=max(125.,median+min(25.,.2*(255.-median)))
        # Poles/lamp posts: a near-vertical ridge run (<~0.5px/row drift) that
        # continues above the admitted road area. Lane paint ends at the road;
        # a post's base would otherwise extend a boundary into the horizon.
        # Uses ridge evidence before the brightness gate, which fragments posts.
        wide=paint.copy();wide[:,1:]|=paint[:,:-1];wide[:,:-1]|=paint[:,1:]
        index=np.arange(180)[:,None]
        run_start=np.maximum.accumulate(np.where(wide,-1,index),axis=0)
        outside=np.vstack((np.zeros((1,320),int),np.cumsum(wide&~allowed,axis=0)))
        above=outside[index+1,np.arange(320)]-outside[run_start+1,np.arange(320)]
        # Only suppress the transition at the road edge. An imperfect road
        # mask must not make a post label persist down an entire valid lane.
        last_outside=np.maximum.accumulate(np.where(~allowed,index,-1),axis=0)
        inside_depth=index-last_outside
        pole_transition=wide&(index-run_start>=10)&(above>=4)&(inside_depth<=12)
        paint &= ~pole_transition
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
            self.last_valid_pair=None
            self.side_missing[:]=0;self.observed_streak[:]=0;self.width_samples[:]=0
            self.motion_velocity[:]=0.;self.missing_seconds=0.
        motion,motion_score=road_motion(self.previous,gray,allowed[::2,::2])
        # A discontinuous source cut must not carry old boundaries into a new scene.
        if self.previous is not None and motion_score==0 and np.mean(abs(gray-self.previous))>45:
            self.history=[None,None];self.width=None;self.scores=[0.,0.]
            self.last_valid_pair=None
            self.side_missing[:]=0;self.observed_streak[:]=0;self.width_samples[:]=0
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
        return self._fuse(observations,predicted,motion,motion_score,step)

    def _fuse(self, observations, predicted, motion, motion_score, step):
        """Fuse only supported rows; commit history only after pair validation."""
        predicted=[self._quadratic(p) for p in predicted]
        output=[];states=[];scores=[];supports=[];residuals=[None,None]
        for side,o in enumerate(observations):
            self.side_missing[side]=self.side_missing[side]+step if o is None else 0.

        for side,observation in enumerate(observations):
            prior=predicted[side]
            p=(prior.copy() if prior is not None else
               np.column_stack((np.full(48,np.nan),self.rows)))
            support=np.zeros(48,bool)
            score=self.scores[side]*np.exp(-step*(.25 if motion_score else .8))
            state='predicted' if motion_score else 'held'
            if observation is not None:
                measured,quality=observation[:2]
                support=observation[2].copy() if len(observation)>2 else np.isfinite(measured[:,0])
                existing=support&np.isfinite(p[:,0])
                fresh=support&~np.isfinite(p[:,0])
                # Blend polynomial coefficients, with ONE scale for all rows.
                # Per-row clipping would turn a quadratic into a piecewise line.
                alpha=1-(1-(.2+.45*quality))**(step/.1)
                u=(self.rows-125.)/50.
                finite=np.isfinite(p[:,0])
                if existing.sum()>=3:
                    old=np.polynomial.polynomial.polyfit(u[finite],p[finite,0],2)
                    diff=measured[existing,0]-p[existing,0]
                    degree=2 if quality>=.65 else (1 if np.ptp(self.rows[existing])>=16 else 0)
                    dc=np.polynomial.polynomial.polyfit(u[existing],diff,degree)
                    delta=np.polynomial.polynomial.polyval(u,dc)
                    domain=finite|np.isfinite(measured[:,0])
                    fraction=min(alpha,80*step/max(float(np.max(abs(delta[domain]))),1e-9))
                    values=np.polynomial.polynomial.polyval(u,old)+fraction*delta
                    p[domain,0]=values[domain]
                elif finite.sum()>=3:
                    # Disjoint fragment cannot safely change the retained model.
                    support[:]=False
                else:
                    domain=np.isfinite(measured[:,0])
                    p[domain,0]=measured[domain,0]
                    p=self._quadratic(p)
                residuals[side]=float(np.percentile(abs(p[support,0]-measured[support,0]),90)) if support.any() else 100.
                score=float(quality*np.exp(-residuals[side]/6.))
                state=('partial' if quality<.65 else 'observed') if residuals[side]<=4 else 'aligning'
            if np.isfinite(p[:,0]).sum()<8:
                state='unavailable';score=0.
            output.append(p);states.append(state);scores.append(float(score));supports.append(support)
            self.observed_streak[side]=(self.observed_streak[side]+1
                if state=='observed' and score>=.65 else 0)

        # Bootstrap only a missing side from visible support; owned history is
        # not dragged by the opposite boundary on every frame.
        for side in (0,1):
            if states[side]=='unavailable' and states[1-side] in ('observed','aligning'):
                width=self.width if self.width is not None else np.clip(1.6*(self.rows-65),24,240)
                target=output[1-side].copy()
                target[:,0]+=width*(-1 if side==0 else 1)
                output[side]=self._quadratic(target);states[side]='inferred';scores[side]=min(.15,scores[1-side]*.3)

        # Only long-lost sides may follow a stable opposite. Use spacing from
        # repeated TWO-sided observations, never a guessed metric/default width.
        for side in (0,1):
            other=1-side
            if (observations[side] is None and self.side_missing[side]>=1.
                    and self.observed_streak[other]>=5 and self.width is not None):
                trusted=(self.width_samples>=5)&supports[other]&np.isfinite(output[side][:,0])
                if trusted.sum()>=8:
                    target=output[other][:,0]+self.width*(-1 if side==0 else 1)
                    candidate=output[side].copy()
                    delta=np.zeros(48)
                    delta[trusted]=np.clip((target[trusted]-candidate[trusted,0])*.1,-20*step,20*step)
                    for _ in range(2):
                        delta=np.convolve(np.pad(delta,1,mode='edge'),[.25,.5,.25],mode='valid')
                    finite=np.isfinite(candidate[:,0]);candidate[finite,0]+=delta[finite]
                    candidate=self._quadratic(candidate)
                    common=np.isfinite(candidate[:,0])&np.isfinite(output[side][:,0])
                    change=candidate[common,0]-output[side][common,0]
                    scale=min(1.,20*step/max(float(np.max(abs(change))),1e-9))
                    candidate[common,0]=output[side][common,0]+scale*change
                    pair=list(output);pair[side]=candidate
                    if self._valid_pair(pair):
                        output[side]=candidate;states[side]='inferred';scores[side]=min(scores[side],.15)

        restored=False
        if not self._valid_pair(output):
            # Try accepting either side independently against the last VALID
            # opposite, before holding both. Never reuse arbitrary history.
            fallback=predicted if self._valid_pair(predicted) else ([self._quadratic(p) for p in self.last_valid_pair] if self.last_valid_pair is not None else None)
            choices=[]
            if fallback is not None and self._valid_pair(fallback):
                for side in (0,1):
                    if observations[side] is None:continue
                    pair=[p.copy() for p in fallback];pair[side]=output[side].copy()
                    if self._valid_pair(pair):choices.append((scores[side],side,pair))
                if choices:
                    _,side,output=max(choices,key=lambda v:v[0])
                    states[1-side]='held';scores[1-side]=min(.15,self.scores[1-side]*np.exp(-step*.8))
                    supports[1-side][:]=False;residuals[1-side]=None
                else:
                    output=[p.copy() for p in fallback];states=['held','held']
                    scores=[min(.15,v*np.exp(-step*.8)) for v in self.scores]
                    supports=[np.zeros(48,bool),np.zeros(48,bool)];residuals=[None,None]
                restored=True
            else:
                # No good pair has ever existed. Keep the stronger measured
                # side and infer the other only over that same supported range.
                side=int(scores[1]>scores[0]);other=1-side
                if np.isfinite(output[side][:,0]).sum()>=8:
                    width=np.clip(1.6*(self.rows-65),24,240)
                    output[other]=output[side].copy()
                    output[other][:,0]+=width*(-1 if other==0 else 1)
                    output[other]=self._quadratic(output[other])
                    states[other]='inferred';scores[other]=min(.15,scores[side]*.3)
                    supports[other][:]=False;residuals[other]=None
                if not self._valid_pair(output):
                    output=[np.column_stack((np.full(48,np.nan),self.rows)) for _ in (0,1)]
                    states=['unavailable','unavailable'];scores=[0.,0.]
                    supports=[np.zeros(48,bool),np.zeros(48,bool)];residuals=[None,None]

        if self._valid_pair(output):
            self.last_valid_pair=[p.copy() for p in output]
            self.history=[p.copy() for p in output]
            # Learn spacing only from concurrently observed pixels, never from
            # virtual extensions or a pair that was rejected/restored.
            common=supports[0]&supports[1]
            if not restored and states==['observed','observed'] and common.sum()>=8:
                widths=output[1][:,0]-output[0][:,0]
                if self.width is None:self.width=np.clip(1.6*(self.rows-65),24,240)
                self.width[common]=.9*self.width[common]+.1*widths[common]
                self.width_samples[common]+=1
        self.scores=scores
        observed=[s for s in (0,1) if supports[s].any() and states[s] in ('observed','aligning','partial')]
        reference='both' if len(observed)==2 else 'left' if observed==[0] else 'right' if observed==[1] else 'none'
        state='tracking' if observed else 'held' if restored else 'predicted' if self.last_valid_pair is not None else 'unavailable'
        return ImageBoundaryEstimate(*output,projection_for((180,320),self.config),*states,*scores,
                                     motion_score,motion,state,reference,*supports,*residuals,restored,
                                     [dict(v) for v in self.detection_diagnostics])
