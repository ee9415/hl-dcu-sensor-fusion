"""Image-first white boundaries with sparse road-motion compensation (NumPy only)."""
from dataclasses import dataclass
import numpy as np
from lane_boundaries import BoundaryConfig, projection_for, white_lane_evidence


def quadratic_display(points, support=None):
    """One shared degree-two display curve; raw measured samples stay intact."""
    display = np.array(points,dtype=float,copy=True)
    valid = np.isfinite(display).all(axis=1)
    if valid.sum() < 3:
        return display,None
    fit_rows = valid.copy()
    if support is not None:
        measured = valid & np.asarray(support,bool)
        if measured.sum() >= 8 and np.ptp(display[measured,1]) >= 20:
            fit_rows = measured
    coefficients = np.polyfit(display[fit_rows,1],display[fit_rows,0],2)
    display[valid,0] = np.polyval(coefficients,display[valid,1])
    return display,coefficients


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
    model: str = 'image_motion'

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
        c = config or BoundaryConfig()
        def points(p):
            return [[float(x),float(y)] for x,y in p if np.isfinite([x,y]).all()]
        def coefficients(p):
            good=np.isfinite(p[:,0])
            return np.polynomial.polynomial.polyfit(p[good,1],p[good,0],2).tolist() if good.sum()>=3 else None
        left_display,left_curve = quadratic_display(self.image_left,self.left_support)
        right_display,right_curve = quadratic_display(self.image_right,self.right_support)
        return dict(display_curve_degree=2,display_geometry='quadratic',
                    left_display_image=points(left_display),right_display_image=points(right_display),
                    left_display_coefficients=left_curve.tolist() if left_curve is not None else [],
                    right_display_coefficients=right_curve.tolist() if right_curve is not None else [],
                    boundary_response=dict(observed_max_rate_px_s=c.skeleton_observed_max_rate_px_s,
                    observed_tau_s=c.skeleton_observed_tau_s,
                    observed_fine_tau_s=c.skeleton_observed_fine_tau_s,
                    retained_max_rate_px_s=c.skeleton_retained_max_rate_px_s,
                    retained_tau_s=c.skeleton_retained_tau_s) if self.model=='persistent_skeleton' else None,
                    image_curve_degree=2,
                    image_geometry='tracked_quadratic_model' if self.model=='persistent_skeleton' else 'quadratic',
                    image_coefficients_kind='quadratic_model',
                    left_image_coefficients=coefficients(self.image_left),
                    right_image_coefficients=coefficients(self.image_right),
                    boundary_valid=self.valid,boundary_pair_valid=self.pair_valid,
                    left_boundary_valid=self.left_valid,right_boundary_valid=self.right_valid,
                    boundary_state=self.state,boundary_reference_side=self.reference_side,
                    left_boundary_state=self.left_state,right_boundary_state=self.right_state,
                    left_boundary_confidence=self.left_confidence,right_boundary_confidence=self.right_confidence,
                    boundary_confidence_kind='heuristic_not_probability',boundary_units='image_pixels_320x180',
                    boundary_model=self.model,left_boundary_image=points(self.image_left),
                    left_boundary_supported_image=points(self.image_left[np.asarray(self.left_support,bool)]) if self.left_support is not None else [],
                    right_boundary_supported_image=points(self.image_right[np.asarray(self.right_support,bool)]) if self.right_support is not None else [],
                    left_display_residual_px=self.left_residual_px,
                    right_display_residual_px=self.right_residual_px,
                    boundary_restored_pair=self.restored_pair,
                    boundary_detection_diagnostics=self.detection_diagnostics,
                    right_boundary_image=points(self.image_right),motion_confidence=self.motion_confidence,
                    motion_matrix=self.motion_matrix.tolist(),corridor_width_m=None,
                    normal_width_error_dev=None,left_boundary_coefficients=[],right_boundary_coefficients=[],
                    boundary_forward_dev=[],left_boundary_dev=[],right_boundary_dev=[],
                    corridor_feasible=None,clearance_sufficient=None)


def nanmedian_columns(values):
    """Bit-identical np.nanmedian(values, axis=0) for a 2-D float array.

    For a short axis NumPy builds masked arrays (np.ma.median) per call. This
    keeps the same NaN exclusion: NaN sorts last, then the same low/high order
    statistics are summed and halved. An all-NaN column stays NaN.
    """
    ordered=np.sort(values,axis=0)
    count=np.count_nonzero(~np.isnan(values),axis=0)
    columns=np.arange(values.shape[1])
    low=ordered[np.maximum(count-1,0)//2,columns]
    high=ordered[count//2-(count==0),columns]
    result=(low+high)/2
    result[count==0]=np.nan
    return result


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
    # Opt-in diagnostics only; does not change detection, fusion, or drawing.
    trace_fit_evidence = False

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
        self.fragment_hits=np.zeros((2,48),dtype=int)
        self.fragment_offsets=np.full((2,48),np.nan)
        self.region_segments=[[],[]]
        self.reacquire=[None,None]
        self.reacquire_hits=[0,0]
        self.short_pair_hits=[0,0]
        self.short_pair_previous=[None,None]
        self.nn_rank_mask=None
        self.curve_reference_hits=np.zeros(2,dtype=int)
        self.curve_guidance=np.zeros(2)
        self.current_pair_hits=0
        self.current_pair_previous=None
        self.observation_motion=np.array([[1.,0.,0.],[0.,1.,0.]])

    def update_contours(self, result, image_shape, timestamp_s, rgb=None):
        """Track actual mask contours once for rendering, guidance and telemetry.

        Coordinates are normalized to the existing 320x180 image model. No
        virtual/extended samples enter the observations. Motion prediction is
        hold-only; a learned bilateral width is a weak, rate-limited constraint.
        """
        from ego_lane_quadratic import fit_quadratic
        height, width = image_shape[:2]
        shape = (height, width)
        last = getattr(self, '_contour_time', None)
        dt = 0. if last is None else timestamp_s-last
        gray = None
        if rgb is not None:
            gray = rgb[np.linspace(0, rgb.shape[0]-1, 24).astype(int)[:, None],
                       np.linspace(0, rgb.shape[1]-1, 32).astype(int)[None, :]].mean(axis=2)
        previous = getattr(self, '_contour_gray', None)
        cut = gray is not None and previous is not None and np.mean(abs(gray-previous)) > 45
        reset = bool(last is None or not np.isfinite(timestamp_s) or dt <= 0 or dt > 1.
                     or shape != getattr(self, '_contour_shape', None) or cut)
        if reset:
            self.history = [None, None]
            self.scores = [0., 0.]
            self.width = None
            self.width_samples[:] = 0
            self._contour_seen = [None, None]
            self._contour_pending = [None, None]
            self._contour_curve_pending = [None, None]
            self._contour_width_time = None
            self._contour_loss_anchor = [None, None]
            dt = 0.
        self._contour_time = float(timestamp_s) if np.isfinite(timestamp_s) else None
        self._contour_shape = shape
        self._contour_gray = gray
        empty = lambda: np.column_stack((np.full(48, np.nan), self.rows))
        identity = np.array([[1., 0., 0.], [0., 1., 0.]])
        stats = [dict(source='mask_contours', reset=reset, candidates=0),
                 dict(source='mask_contours', reset=reset, candidates=0)]
        if not np.isfinite(timestamp_s):
            return ImageBoundaryEstimate(empty(), empty(), projection_for((180,320), self.config),
                'unavailable', 'unavailable', 0., 0., 0., identity, 'unavailable', 'none',
                np.zeros(48,bool), np.zeros(48,bool), detection_diagnostics=stats, model='persistent_skeleton')

        # Previously observed skeletons survive a detection loss. Only source,
        # timestamp, image geometry or scene resets discard their identity.
        candidates = []
        for index, curve in enumerate(result.group_curves):
            raw = np.asarray(curve)
            raw_measured = raw[raw[:,2] > 0]
            raw_side = 0 if len(raw_measured) and np.median(raw_measured[:,0])*320/width < 160 else 1
            def rejected(reason):
                counts = stats[raw_side].setdefault('candidate_rejections',{})
                counts[reason] = counts.get(reason,0)+1
            fit = fit_quadratic(curve, height, index)
            if fit is None:
                rejected('quadratic_fit_or_support')
                continue
            anchor = np.polyval(fit.coefficients, height*.85)*320/width
            if fit.points[-1,1] < height*.68:
                owned_side = 0 if anchor < 160 else 1
                if self.history[owned_side] is None or np.ptp(fit.points[:,1]) < height*.22:
                    rejected('far_short_unowned')
                    continue
            if not -32 <= anchor <= 352:
                rejected('anchor_outside')
                continue
            rows = self.rows*height/180
            support = (rows >= fit.points[0,1]) & (rows <= fit.points[-1,1])
            if support.sum() < 8:
                rejected('support_rows')
                continue
            measured = np.asarray(curve)[np.asarray(curve)[:,2] > 0]
            ys = np.unique(measured[:,1])
            xs = np.array([np.median(measured[measured[:,1]==y,0]) for y in ys])
            distance = np.min(abs(rows[:,None]-ys[None,:]),axis=1)
            support &= distance <= 1.1
            if support.sum() < 8:
                rejected('measured_support_rows')
                continue
            points = np.column_stack((np.interp(rows,ys,xs)*320/width,self.rows))
            points[~support,0] = np.nan
            if not self._smooth_shape(points):
                rejected('raw_shape')
                continue
            quality = float(np.clip((.55+.4*min(1., np.ptp(fit.points[:,1])/(height*.4)))
                                   *np.exp(-fit.residual_px*320/width/6.), .15, .95))
            candidates.append((index, points, support, quality, anchor))

        self.last_candidate_count = len(candidates)

        # Associate BEFORE any bilateral pair rejection. A weak/short opposite
        # cannot delete an otherwise supported, temporally consistent boundary.
        options = [[], []]
        for side in (0, 1):
            prior = self.history[side]
            for item in candidates:
                index, points, support, quality, anchor = item
                if prior is None:
                    if (anchor < 160) != (side == 0):
                        continue
                    cost = .1*abs(anchor-160)+4*(1-quality)
                else:
                    common = support & np.isfinite(prior[:,0])
                    if common.sum() < 8:
                        continue
                    cost = float(np.percentile(abs(points[common,0]-prior[common,0]), 80))
                    cost += 4*(1-quality)
                    # Never force an unrelated opposite/outer boundary into an
                    # owned track merely to maximize the number of assignments.
                    if cost > 60.:
                        age = timestamp_s-self._contour_seen[side]
                        if age < 1.5 or cost > 120. or ((anchor < 160) != (side == 0)):
                            continue
                options[side].append((cost, index, item))
            options[side].sort(key=lambda entry: entry[:2])
            stats[side]['candidates'] = len(options[side])
        # Joint assignment prevents both sides consuming the same skeleton.
        assignments = []
        for left in [None]+options[0]:
            for right in [None]+options[1]:
                if left is not None and right is not None:
                    if left[1] == right[1]:
                        continue
                    pair = [left[2][1], right[2][1]]
                    if not self._valid_pair(pair):
                        continue
                cost = sum(160. if entry is None else entry[0] for entry in (left,right))
                assignments.append((cost, left, right))
        _, left, right = min(assignments, key=lambda entry: entry[0])
        chosen = [left, right]
        observations = [None, None]
        for side, entry in enumerate(chosen):
            if entry is None:
                self._contour_pending[side] = None
                self._contour_curve_pending[side] = None
                continue
            _, index, item = entry
            _, points, support, quality, _ = item
            prior = self.history[side]
            accepted = prior is None
            error = 0.
            if prior is not None:
                common = support & np.isfinite(prior[:,0])
                error = float(np.percentile(abs(points[common,0]-prior[common,0]), 90))
                slope_old = np.polyfit(self.rows[common], prior[common,0], 1)[0]
                slope_new = np.polyfit(self.rows[common], points[common,0], 1)[0]
                age = timestamp_s-self._contour_seen[side]
                accepted = (error <= min(32., 12.+20.*age)
                            and abs(np.arctan(slope_new)-np.arctan(slope_old)) <= np.deg2rad(25))
            stats[side].update(group=index, prior_error_px=error, accepted=bool(accepted),
                support_rows=int(support.sum()),support_span_px=float(np.ptp(self.rows[support])),
                candidate_quality=float(quality))
            # Small residual on a short visible segment does not validate the
            # curvature extrapolated over the retained display domain.
            if prior is not None:
                measured_curve = np.polyfit(self.rows[support],points[support,0],2)
                retained = np.isfinite(prior[:,0])
                previous_curve = np.polyfit(self.rows[retained],prior[retained,0],2)
                span = float(np.ptp(self.rows[retained | support]))
                bow_change = abs(measured_curve[0]-previous_curve[0])*span*span/4.
                stats[side]['curvature_change_px'] = float(bow_change)
                if bow_change > 2.5:
                    pending = self._contour_curve_pending[side]
                    domain = retained | support
                    repeat = (pending is not None and np.max(abs(np.polyval(
                        measured_curve-pending[0],self.rows[domain]))) <= 4.)
                    pending = (measured_curve.copy(),pending[1],pending[2]+1) if repeat else (measured_curve.copy(),timestamp_s,1)
                    self._contour_curve_pending[side] = pending
                    enough = (support.sum() >= 16 and np.ptp(self.rows[support]) >= max(40.,.65*span))
                    confirmed = bool(enough and pending[2] >= 3 and timestamp_s-pending[1] >= .3-1e-9)
                    stats[side].update(curvature_pending=not confirmed,
                        curvature_confirmation_hits=pending[2],curvature_support_sufficient=bool(enough),
                        curvature_confirmed=confirmed)
                    if not confirmed:
                        stats[side]['accepted'] = False
                        self._contour_pending[side] = None
                        continue
                else:
                    self._contour_curve_pending[side] = None
            if not accepted:
                pending = self._contour_pending[side]
                common = support & (np.isfinite(pending[0][:,0]) if pending is not None else False)
                repeat = (pending is not None and common.sum() >= 8
                          and np.max(abs(points[common,0]-pending[0][common,0])) <= 4.)
                pending = (points.copy(), pending[1], pending[2]+1) if repeat else (points.copy(), timestamp_s, 1)
                self._contour_pending[side] = pending
                accepted = pending[2] >= 3 and timestamp_s-pending[1] >= .2-1e-9
                stats[side]['confirmed_reacquisition'] = accepted
                if not accepted:
                    continue
            else:
                self._contour_pending[side] = None
            observations[side] = (points, support, quality)

        strength = []
        for side,observation in enumerate(observations):
            error = 0. if stats[side].get('confirmed_reacquisition') else float(stats[side].get('prior_error_px') or 0.)
            strength.append(float(observation[2])*int(observation[1].sum())*np.exp(-min(error,60.)/8.) if observation is not None else 0.)
        trusted_width = (self.width is not None and np.count_nonzero(
            (self.width_samples >= 3) & np.isfinite(self.width) & (self.width > 6.)) >= 8)
        weak = [bool(trusted_width and observations[side] is not None
                     and observations[1-side] is not None
                     and strength[1-side] > 1.15*strength[side]
                     and (observations[side][1].sum() < .9*observations[1-side][1].sum()
                          or float(stats[side].get('prior_error_px') or 0.) > 4.)) for side in (0,1)]
        seen_before = list(self._contour_seen)
        scores_before = list(self.scores)
        output = [p.copy() if p is not None else empty() for p in self.history]
        states = ['unavailable', 'unavailable']
        supports = [np.zeros(48,bool), np.zeros(48,bool)]
        residuals = [None, None]
        observed = [False, False]
        for side, observation in enumerate(observations):
            if observation is None or weak[side]:
                if weak[side]:
                    stats[side]['weak_observation_rows'] = int(observation[1].sum())
                self.scores[side] *= np.exp(-dt/ .6)
                if self.history[side] is not None:
                    states[side] = 'held'
                continue
            points, support, quality = observation
            prior = self.history[side]
            measured_curve = np.polyfit(self.rows[support],points[support,0],2)
            if prior is None:
                domain = (self.rows >= self.rows[support].min()) & (self.rows <= self.rows[support].max())
                values = np.polyval(measured_curve,self.rows)
            else:
                retained = np.isfinite(prior[:,0])
                domain = retained | support
                domain = (self.rows >= self.rows[domain].min()) & (self.rows <= self.rows[domain].max())
                previous_curve = np.polyfit(self.rows[retained],prior[retained,0],2)
                previous_values = np.polyval(previous_curve,self.rows)
                delta = np.polyval(measured_curve,self.rows)-previous_values
                tau = (self.config.skeleton_observed_tau_s if stats[side]['prior_error_px'] > 4
                       else self.config.skeleton_observed_fine_tau_s)
                fraction = min((1.-np.exp(-dt/tau))*quality,
                               self.config.skeleton_observed_max_rate_px_s*dt
                               /max(float(np.max(abs(delta[domain]))),1e-9))
                # A coefficient blend is still one quadratic across measured,
                # occluded and newly exposed rows. It cannot leave stale holes.
                values = previous_values+fraction*delta
            output[side] = np.column_stack((np.where(domain,values,np.nan),self.rows))
            supports[side] = support.copy()
            residuals[side] = float(np.percentile(abs(output[side][support,0]-points[support,0]),90))
            states[side] = 'observed' if residuals[side] <= 4 else 'aligning'
            self.scores[side] = quality*float(np.exp(-residuals[side]/12.))
            self._contour_seen[side] = timestamp_s
            observed[side] = True

        # Learn width only from actual, aligned bilateral measurements.
        common = supports[0] & supports[1]
        balanced = min(strength) >= max(strength)/1.15
        bootstrap = (not trusted_width and min(int(v.sum()) for v in supports) >= 24
                     and common.sum() >= 16 and np.ptp(self.rows[common]) >= 40)
        if (balanced or bootstrap) and all(observed) and states == ['observed','observed'] and common.sum() >= 8 and self._valid_pair(output):
            measured_width = observations[1][0][:,0]-observations[0][0][:,0]
            if self.width is None:
                self.width = np.full(48, np.nan)
            first = common & (self.width_samples == 0)
            known = common & ~first
            self.width[first] = measured_width[first]
            self.width[known] += (1.-np.exp(-dt/.8))*(measured_width[known]-self.width[known])
            self.width_samples[common] += 1
            self._contour_width_time = timestamp_s
        for side in (0, 1):
            other = 1-side
            partial = bool(observed[side] or weak[side])
            if stats[side].get('curvature_pending'):
                # The suspect candidate contributes nothing. A separately
                # validated opposite model may still supply trusted spacing.
                partial = True
            if observed[side] and not weak[side]:
                continue
            if partial and strength[other] <= 1.15*strength[side]:
                continue
            # 'aligning' is a confirmed observation whose display is catching
            # up. Its filtered motion can already guide the retained side.
            if not observed[other] or self.history[side] is None:
                continue
            # Copy the supported opposite shape with the last reliably measured
            # perspective spacing. An affine width cannot preserve an obsolete
            # bend of the missing side or invent curvature by extrapolation.
            if self.width is None:
                continue
            learned = (self.width_samples >= 3) & np.isfinite(self.width) & (self.width > 6.)
            if learned.sum() < 8 or np.ptp(self.rows[learned]) < 20:
                continue
            u = (self.rows-125.)/50.
            spacing_fit = np.polynomial.polynomial.polyfit(u[learned], self.width[learned], 1)
            spacing = np.polynomial.polynomial.polyval(u, spacing_fit)
            reliable = supports[other] & np.isfinite(output[other][:,0])
            if reliable.sum() < 8 or np.ptp(self.rows[reliable]) < 20:
                continue
            # Extend only the affine perspective gap, never an old-side bend.
            # Keep the existing display domain; do not initialize unseen sides.
            domain = np.isfinite(output[side][:,0]) & (spacing > 6.)
            if domain.sum() < 8:
                continue
            if np.ptp(self.rows[domain]) < 20:
                continue
            _,reference_coefficients = quadratic_display(output[other],supports[other])
            _,previous_coefficients = quadratic_display(output[side])
            target = np.polyval(reference_coefficients,self.rows) + (-1. if side == 0 else 1.)*spacing
            previous_values = np.polyval(previous_coefficients,self.rows)
            delta = np.where(domain,target-previous_values,0.)
            fraction = min(1.-np.exp(-dt/self.config.skeleton_retained_tau_s),
                           self.config.skeleton_retained_max_rate_px_s*dt
                           /max(float(np.max(abs(delta[domain]))),1e-9))
            candidate = np.column_stack((np.where(domain,previous_values+fraction*delta,np.nan),self.rows))
            pair = list(output); pair[side] = candidate
            if self._valid_pair(pair):
                output[side] = candidate
                states[side] = 'inferred'
                if partial:
                    # Weak detections do not become new truth merely because a
                    # display curve was reconstructed from the stronger side.
                    stats[side]['weak_observation_rows'] = stats[side].get('weak_observation_rows',int(supports[side].sum()))
                    observed[side] = False
                    supports[side][:] = False
                    self._contour_seen[side] = seen_before[side]
                    self.scores[side] = scores_before[side]*np.exp(-dt/.6)
                stats[side]['partial_spacing_guidance'] = partial
                stats[side]['opposite_curve_applied_px'] = float(np.max(abs(fraction*delta[domain])))
                stats[side]['opposite_shape_spacing'] = True
                stats[side]['spacing_reference'] = 'right' if other == 1 else 'left'
                stats[side]['spacing_target_error_px'] = float(np.max(abs(candidate[domain,0]-target[domain])))
                stats[side]['spacing_rows'] = int(domain.sum())
            else:
                stats[side]['opposite_curve_rejected'] = True

        # Retention is geometric memory, not a new observation. Age and
        # confidence remain honest even after a long one- or two-sided loss.
        any_observed = any(observed)
        for side in (0,1):
            age = None if self._contour_seen[side] is None else timestamp_s-self._contour_seen[side]
            stats[side].update(missing_s=age, observed=observed[side],
                              retained=bool(not observed[side] and self.history[side] is not None),
                              width_samples=int(np.max(self.width_samples)))
            if age is None:
                output[side] = empty(); states[side] = 'unavailable'; self.scores[side] = 0.
        # Reject incompatible fusion without discarding a valid observed side.
        if all(np.isfinite(p[:,0]).sum() >= 8 for p in output) and not self._valid_pair(output):
            if self._valid_pair(self.history):
                output = [p.copy() for p in self.history]
                states = ['held','held']; supports = [np.zeros(48,bool),np.zeros(48,bool)]
                self._contour_seen = seen_before
                observed = [False,False]; any_observed = False
                self.scores = [min(.15,float(score*np.exp(-dt/.6))) for score in scores_before]
                residuals = [None,None]
                for side in (0,1):
                    stats[side].update(observed=False,retained=True,pair_conflict_held=True,
                                      opposite_curve_applied_px=0.,missing_s=timestamp_s-self._contour_seen[side])
            else:
                side = int(self.scores[1] < self.scores[0])
                output[side] = empty(); states[side] = 'unavailable'; supports[side][:] = False
                self.scores[side] = 0.; stats[side]['pair_rejected'] = True
        self.history = [p.copy() if np.isfinite(p[:,0]).sum() >= 8 else None for p in output]
        self.side_missing = np.array([0. if seen is None else timestamp_s-seen for seen in self._contour_seen])
        self.detection_diagnostics = stats
        reference = 'both' if all(observed) else 'left' if observed[0] else 'right' if observed[1] else 'none'
        state = 'tracking' if any_observed else 'held' if any(p is not None for p in self.history) else 'unavailable'
        return ImageBoundaryEstimate(*output, projection_for((180,320), self.config), *states, *self.scores,
            0., identity, state, reference, *supports, *residuals, False, stats, 'persistent_skeleton')

    def _nn_support(self, points):
        """Tie-break only on observed paint; never change acceptance/confidence."""
        if self.nn_rank_mask is None or not len(points):return 0.
        q=np.asarray(points)
        q=q[np.isfinite(q).all(axis=1)]
        if len(q)<6 or np.ptp(q[:,1])<10:return 0.
        xy=np.rint(q).astype(int)
        good=(xy[:,0]>=0)&(xy[:,0]<320)&(xy[:,1]>=0)&(xy[:,1]<180)
        if not good.all():return 0.
        # Equal weight per image row; thick masks cannot inflate support.
        rows,inverse=np.unique(xy[:,1],return_inverse=True)
        support=np.zeros(len(rows),bool)
        np.logical_or.at(support,inverse,self.nn_rank_mask[xy[:,1],xy[:,0]])
        return float(support.mean()) if support.mean()>=.5 else 0.

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

    def _plausible_pair(self, pair, learned=True):
        if not self._valid_pair(pair):return False
        common=np.isfinite(pair[0][:,0])&np.isfinite(pair[1][:,0])
        gap=pair[1][:,0]-pair[0][:,0]
        baseline=np.clip(1.6*(self.rows-65),24,240)
        minimum=np.where(self.rows>=150,self.config.image_min_width_ratio*baseline,6.)
        if learned and self.width is not None:
            trusted=self.width_samples>=5
            minimum[trusted]=np.maximum(minimum[trusted],self.config.image_retained_width_ratio*self.width[trusted])
        return bool(np.all(gap[common]>=minimum[common]))

    def _confirm_current_pair(self, observations, predicted):
        """Repeated independent observations may supersede a stale width prior."""
        if any(o is None or o[1]<.65 or (len(o)>3 and o[3]) for o in observations):
            self.current_pair_hits=0;self.current_pair_previous=None
            return predicted
        pair=[]
        for o in observations:
            p=o[0].copy()
            if len(o)>2:p[~o[2],0]=np.nan
            pair.append(p)
        if not self._plausible_pair(pair,learned=False):
            self.current_pair_hits=0;self.current_pair_previous=None
            return predicted
        repeat=self.current_pair_previous is not None
        if repeat:
            for p,old in zip(pair,self.current_pair_previous):
                old=self._warp(old,self.observation_motion)
                if old is None:
                    repeat=False
                    break
                common=np.isfinite(p[:,0])&np.isfinite(old[:,0])
                repeat=repeat and common.sum()>=8 and np.max(abs(p[common,0]-old[common,0]))<=3
        self.current_pair_hits=self.current_pair_hits+1 if repeat else 1
        self.current_pair_previous=[p.copy() for p in pair]
        for stats in self.detection_diagnostics:stats['current_pair_hits']=self.current_pair_hits
        if self.current_pair_hits<3:return predicted
        common=np.isfinite(pair[0][:,0])&np.isfinite(pair[1][:,0])
        if self.width is not None and not self._plausible_pair(pair):
            self.width[common]=pair[1][common,0]-pair[0][common,0]
            self.width_samples[common]=3
            for stats in self.detection_diagnostics:stats['width_prior_reacquired']=True
        result=list(predicted)
        for side in (0,1):
            if predicted[side] is None:continue
            overlap=np.isfinite(pair[side][:,0])&np.isfinite(predicted[side][:,0])
            if overlap.sum()<3:
                result[side]=None
                self.detection_diagnostics[side]['disjoint_reacquired']=True
            elif np.percentile(abs(pair[side][overlap,0]-predicted[side][overlap,0]),90)>6:
                # Three independent, motion-consistent BOTH-side observations
                # may replace stale geometry. Use only observed support; never
                # transplant an extrapolated tail from the rejected history.
                result[side]=pair[side].copy()
                self.detection_diagnostics[side]['stale_prior_reacquired']=True
        return result

    def _resolve_duplicate_observations(self, observations, predicted):
        if any(o is None for o in observations):
            self.short_pair_hits=[0,0];self.short_pair_previous=[None,None]
            return observations,predicted
        pair=[o[0] for o in observations]
        if self._plausible_pair(pair):
            self.short_pair_hits=[0,0];self.short_pair_previous=[None,None]
            return observations,predicted
        # Only reassign the weaker side; never use the same paint twice.
        side=int(observations[1][1]<observations[0][1]);other=1-side
        best=None
        for q in self.region_segments[side]:
            if len(q)<6 or np.ptp(q[:,1])<10:continue
            c=np.polynomial.polynomial.polyfit((q[:,1]-125)/50,q[:,0],1)
            error=np.max(abs(np.polynomial.polynomial.polyval((q[:,1]-125)/50,c)-q[:,0]))
            if error>1.5:continue
            domain=(self.rows>=q[:,1].min())&(self.rows<=q[:,1].max())
            if domain.sum()<8:continue
            values=np.polynomial.polynomial.polyval((self.rows-125)/50,c);values[~domain]=np.nan
            curve=np.column_stack((values,self.rows));candidate=list(pair);candidate[side]=curve
            if not self._plausible_pair(candidate):continue
            if best is None or domain.sum()>best[0]:best=(domain.sum(),curve,domain)
        if best is None:
            self.short_pair_hits[side]=0;self.short_pair_previous[side]=None
            return observations,predicted
        _,curve,support=best;previous=self.short_pair_previous[side]
        common=support&(np.isfinite(previous[:,0]) if previous is not None else False)
        repeat=common.sum()>=8 and np.max(abs(curve[common,0]-previous[common,0]))<=3
        self.short_pair_hits[side]=self.short_pair_hits[side]+1 if repeat else 1
        self.short_pair_previous[side]=curve.copy()
        self.detection_diagnostics[side]['duplicate_rejected']=True
        self.detection_diagnostics[side]['short_pair_hits']=self.short_pair_hits[side]
        if self.short_pair_hits[side]<3:return observations,predicted
        observations=list(observations);observations[side]=(curve,.5,support,True)
        # A collapsed prior is not a trustworthy anchor. Recover only the
        # actually observed extent; normal pairs keep rate-limited fusion.
        overlap=support&(np.isfinite(predicted[side][:,0]) if predicted[side] is not None else False)
        stale=False
        if overlap.sum()>=3 and self.scores[side]<.25:
            offset=np.max(abs(predicted[side][overlap,0]-curve[overlap,0]))
            slope_old=np.polyfit(self.rows[overlap],predicted[side][overlap,0],1)[0]
            slope_new=np.polyfit(self.rows[overlap],curve[overlap,0],1)[0]
            stale=offset>6 or abs(slope_old-slope_new)>.6
        if not self._plausible_pair(predicted) or stale:
            predicted=list(predicted);predicted[side]=None
        self.detection_diagnostics[side]['short_pair_selected']=True
        return observations,predicted

    def _display_continuation(self, output, states, supports, step=.1):
        """Bounded virtual tail; never feeds tracking, support or width learning."""
        result=[p.copy() for p in output]
        for side in (0,1):
            reliable=(states[side]=='observed' and self.scores[side]>=.6
                      and supports[side].sum()>=12)
            self.curve_reference_hits[side]=self.curve_reference_hits[side]+1 if reliable else 0
        for side in (0,1):
            other=1-side
            if (states[side]!='partial' or states[other]!='observed'
                    or self.short_pair_hits[side]<3):
                self.curve_guidance[side]=0.
                continue
            p=output[side];good=np.isfinite(p[:,0]);observed=good&supports[side]
            if observed.sum()<8 or np.ptp(self.rows[observed])<14:continue
            # A short fit's curvature is poorly constrained. Reject nonlinear
            # extrapolation unless its departure from the observed tangent is small.
            u=(self.rows-125)/50
            c=np.polynomial.polynomial.polyfit(u[good],p[good,0],2)
            edge=float(self.rows[good].min());ue=(edge-125)/50
            slope=(c[1]+2*c[2]*ue)/50
            if abs(slope)>1.5:continue
            edge_x=float(np.polynomial.polynomial.polyval(ue,c))
            values=np.polynomial.polynomial.polyval(u,c)
            tail=(self.rows<edge)&(self.rows>=edge-24)&np.isfinite(output[other][:,0])
            # Subtract the opposite's local tangent: perspective convergence
            # (constant/linear terms) is NOT transferred. This is an image-space
            # curvature prior, not a calibrated road-curvature measurement.
            target=0.
            reference=supports[other]&np.isfinite(output[other][:,0])
            if (self.curve_reference_hits[other]>=5 and self.scores[other]-self.scores[side]>=.08
                    and reference.sum()>=12 and np.ptp(self.rows[reference])>=24):
                oc=np.polynomial.polynomial.polyfit(u[reference],output[other][reference,0],2)
                target=self.config.opposite_curve_weight*(oc[2]-c[2])/2500
            # Smooth coefficient changes, capped at 2 px at the 24 px endpoint.
            target=float(np.clip(target,-2/576,2/576))
            alpha=1-np.exp(-max(0.,step)/.5)
            delta=alpha*(target-self.curve_guidance[side])
            self.curve_guidance[side]+=float(np.clip(delta,-2*step/576,2*step/576))
            guidance=self.curve_guidance[side] if self.config.opposite_curve_weight else 0.
            values[tail]+=guidance*(self.rows[tail]-edge)**2
            added=0
            for index in np.flatnonzero(tail):
                tangent=edge_x+slope*(self.rows[index]-edge)
                if abs(values[index]-tangent)>4 or not 0<values[index]<319:break
                candidate=[v.copy() for v in result]
                candidate[side][index,0]=values[index]
                if not self._plausible_pair(candidate):break
                result=candidate;added+=1
            self.detection_diagnostics[side]['virtual_extension_rows']=added
            self.detection_diagnostics[side]['virtual_extension_limit_px']=24
            self.detection_diagnostics[side]['opposite_curve_target']=target
            self.detection_diagnostics[side]['opposite_curve_applied']=guidance
            self.detection_diagnostics[side]['opposite_curve_reference_hits']=int(self.curve_reference_hits[other])
        return result

    def _reacquire_observation(self, paint, side, prior, selected):
        if prior is None or (self.scores[side]>=.25 and self.side_missing[side]<.5):
            self.reacquire[side]=None;self.reacquire_hits[side]=0
            return selected
        stats=dict(self.detection_diagnostics[side])
        candidate=self._detect_full(paint,side,None)
        self.detection_diagnostics[side]=stats
        if candidate is None or candidate[1]<.75:
            self.reacquire[side]=None;self.reacquire_hits[side]=0
            return selected
        curve=candidate[0];previous=self.reacquire[side]
        if previous is not None:previous=self._warp(previous,self.observation_motion)
        common=np.isfinite(curve[:,0])&(np.isfinite(previous[:,0]) if previous is not None else False)
        repeat=common.sum()>=8 and np.max(abs(curve[common,0]-previous[common,0]))<=4
        self.reacquire_hits[side]=self.reacquire_hits[side]+1 if repeat else 1
        self.reacquire[side]=curve.copy()
        stats['reacquire_hits']=self.reacquire_hits[side]
        stats['reacquired']=self.reacquire_hits[side]>=3
        return candidate if self.reacquire_hits[side]>=3 else selected

    def _detect(self, paint, side, prior):
        regional=self._detect_regions(paint,side,prior)
        regional_stats=dict(self.detection_diagnostics[side])
        legacy=self._detect_legacy(paint,side,prior)
        corrected=self._regional_upper_refit(paint,regional,legacy)
        if corrected is not None:
            self.detection_diagnostics[side]=regional_stats
            self.detection_diagnostics[side]['regional_upper_refit']=True
            return corrected
        extends=False
        if regional is not None and legacy is not None:
            common=np.isfinite(regional[0][:,0])&np.isfinite(legacy[0][:,0])
            legacy_support=legacy[2] if len(legacy)>2 else np.isfinite(legacy[0][:,0])
            extends=(regional[1]>=.45 and np.count_nonzero(regional[2])>np.count_nonzero(legacy_support)
                     and common.sum()>=3 and np.max(abs(regional[0][common,0]-legacy[0][common,0]))<=3)
        if regional is not None and (legacy is None or regional[1]>legacy[1] or extends):
            self.detection_diagnostics[side]=regional_stats
            return self._temporal_fragments(self._reacquire_observation(paint,side,prior,regional),side,prior)
        return self._temporal_fragments(self._reacquire_observation(paint,side,prior,legacy),side,prior)

    def _regional_upper_refit(self, paint, regional, full):
        """Use independently supported upper paint to challenge a long fit."""
        if regional is None or full is None or regional[1]<.5:return None
        r,f=regional[0],full[0]
        support=regional[2]&np.isfinite(r[:,0])
        upper=support&(self.rows<124)&np.isfinite(f[:,0])
        if upper.sum()<4:return None
        def on_paint(p):
            hit=np.zeros(48,bool);good=np.isfinite(p[:,0])
            idx=np.flatnonzero(good);xx=np.rint(p[good,0]).astype(int);yy=self.rows[good].astype(int)
            for dx in (-2,-1,0,1,2):
                inside=(xx+dx>=0)&(xx+dx<320)
                hit[idx[inside]]|=paint[yy[inside],(xx+dx)[inside]]
            return hit
        rh,fh=on_paint(r),on_paint(f)
        if rh[upper].mean()<.75 or fh[upper].mean()>.25:return None
        if np.median(abs(r[upper,0]-f[upper,0]))<6:return None
        near=support&(self.rows>=136)&np.isfinite(f[:,0])
        if near.sum()<6 or np.median(abs(r[near,0]-f[near,0]))>3:return None
        # Keep the independently joined regional observations plus real lower
        # support beyond them. Never treat the full fit's unsupported top as data.
        chosen=support&rh
        lower=(self.rows>self.rows[support].max())&fh
        x=r[:,0].copy();x[lower]=f[lower,0];chosen|=lower
        if chosen.sum()<12:return None
        u=(self.rows-125)/50
        c=np.polynomial.polynomial.polyfit(u[chosen],x[chosen],2)
        values=np.polynomial.polynomial.polyval(u,c)
        if np.max(abs(values[chosen]-x[chosen]))>3:return None
        domain=(self.rows>=self.rows[chosen].min())&(self.rows<=self.rows[chosen].max())
        values[~domain]=np.nan
        curve=np.column_stack((values,self.rows))
        if not self._smooth_shape(curve):return None
        if np.any((values[domain]<0)|(values[domain]>319)):return None
        return curve,min(.7,full[1]),chosen

    def _temporal_fragments(self, observation, side, prior):
        # Accumulate repeatability relative to the motion-compensated prior.
        current=np.full(48,np.nan)
        if prior is not None:
            for q in self.region_segments[side]:
                order=np.argsort(q[:,1]);q=q[order]
                rows=(self.rows>=q[0,1])&(self.rows<=q[-1,1])&np.isfinite(prior[:,0])
                if rows.sum()<3:continue
                slope=np.polyfit(q[:,1],q[:,0],1)[0]
                previous=np.polyfit(self.rows[rows],prior[rows,0],1)[0]
                if abs(slope-previous)>.6:continue
                x=np.interp(self.rows[rows],q[:,1],q[:,0])
                indices=np.flatnonzero(rows);offset=x-prior[rows,0]
                for k,d in zip(indices,offset):
                    if abs(d)<=6 and (not np.isfinite(current[k]) or abs(d)<abs(current[k])):current[k]=d
        visible=np.isfinite(current)
        repeat=visible&np.isfinite(self.fragment_offsets[side])&(abs(current-self.fragment_offsets[side])<=2)
        self.fragment_hits[side]=np.where(repeat,np.minimum(self.fragment_hits[side]+1,5),visible.astype(int))
        self.fragment_offsets[side]=current
        stable=visible&(self.fragment_hits[side]>=3)
        stats=self.detection_diagnostics[side]
        stats.update(temporal_visible=int(visible.sum()),temporal_stable=int(stable.sum()),temporal_used=False)
        if observation is None and prior is not None:
            # Only repeated, prior-aligned paint may make a local correction.
            # Do not turn its unobserved extension into measured support.
            if stable.sum() < 5 or np.ptp(self.rows[stable]) < 8:return None
            measured=prior.copy()
            measured[stable,0]+=current[stable]
            measured[~stable,0]=np.nan
            stats['temporal_used']=True
            stats['temporal_fragment_only']=True
            return measured,.45,stable.copy(),True
        if observation is None or prior is None or observation[1]>=.65:return observation
        measured,quality=observation[:2]
        support=observation[2].copy() if len(observation)>2 else np.isfinite(measured[:,0])
        added=stable&~support
        if added.sum()<3:return observation
        measured=measured.copy();measured[added,0]=prior[added,0]+current[added]
        support|=added
        stats['temporal_used']=True
        return measured,min(quality,.55),support,True

    def _detect_regions(self, paint, side, prior):
        """Independent overlapping bands in the bottom 60%, then joint fit."""
        from itertools import product, combinations
        bands=((72,124),(100,160),(136,178))
        row_pixels={y:np.flatnonzero(paint[y]) for y in range(72,178,2)}
        candidates=[]
        for lo,hi in bands:
            segments=[];seeds=[];centroids=[]
            for y in range(hi-2,lo+5,-4):
                xs=row_pixels[y]
                xs=xs[(xs>8)&(xs<312)&((xs<185) if side==0 else (xs>135))]
                groups=np.split(xs,np.flatnonzero(np.diff(xs)>2)+1) if len(xs) else []
                for g in groups:
                    if len(g)<=12:seeds.append((y,float(np.mean(g))))
            for start,x in seeds[:24]:
                pts=[];last=start;slope=0.
                for y in range(start,lo-1,-2):
                    if last-y>6:break
                    options=row_pixels[y];prediction=x+slope*(y-last)
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
                center=np.median(q,axis=0)
                if any(abs(center[0]-v[0])<3 and abs(center[1]-v[1])<8 for v in centroids):continue
                segments.append(q);centroids.append(center)
            segments.sort(key=lambda q:np.ptp(q[:,1]),reverse=True)
            candidates.append(segments[:4])
        self.region_segments[side]=[q for band in candidates for q in band]
        # The same segment occurs in many band combinations. Compute its
        # row-aligned samples and tangent once, preserving ALL candidates.
        row_grid=np.arange(72,178,2)
        segment_data={}
        for segment in self.region_segments[side]:
            aligned=np.full(len(row_grid),np.nan)
            aligned[((segment[:,1]-72)/2).astype(int)]=segment[:,0]
            local=np.polyfit(segment[:,1],segment[:,0],1)
            noise=np.max(abs(np.polyval(local,segment[:,1])-segment[:,0]))
            segment_data[id(segment)]=(aligned,local[0],noise,np.ptp(segment[:,1]),segment[:,1].mean())
        best=None;fit_cache={}
        for count in (3,2):
            for band_ids in combinations(range(3),count):
                for group in product(*(candidates[k] for k in band_ids)):
                    # Merge overlapping evidence once; reject conflicting locations.
                    aligned=np.vstack([segment_data[id(segment)][0] for segment in group])
                    active=np.isfinite(aligned).any(axis=0)
                    aligned=aligned[:,active]
                    if np.any(np.nanmax(aligned,axis=0)-np.nanmin(aligned,axis=0)>3):continue
                    q=np.column_stack((nanmedian_columns(aligned),row_grid[active]))
                    if len(q)<10 or np.ptp(q[:,1])<20:continue
                    # Overlapping segment combinations can yield identical
                    # samples. Reuse ONLY identical fits, keeping each group's
                    # independent tangent checks and original candidate order.
                    key=q.tobytes()
                    if key not in fit_cache:
                        u=(q[:,1]-125)/50
                        c=np.polynomial.polynomial.polyfit(u,q[:,0],2)
                        errors=abs(np.polynomial.polynomial.polyval(u,c)-q[:,0])
                        fit_cache[key]=(c,errors)
                    c,errors=fit_cache[key]
                    if errors.max()>2:continue
                    # Short fragments have uncertain tangent estimates. Check the
                    # fitted tangent against each fragment, with length-scaled tolerance.
                    compatible=True
                    for segment in group:
                        _,slope,noise,span,mean_y=segment_data[id(segment)]
                        tangent=(c[1]+2*c[2]*(mean_y-125)/50)/50
                        if abs(tangent-slope)>.2+min(1.,noise*2)*4/max(span,4):
                            compatible=False;break
                    if not compatible:continue
                    values=np.polynomial.polynomial.polyval((self.rows-125)/50,c)
                    domain=(self.rows>=q[:,1].min())&(self.rows<=q[:,1].max())
                    values[~domain]=np.nan
                    curve=np.column_stack((values,self.rows))
                    if not self._smooth_shape(curve):continue
                    if np.any((values[domain]<0)|(values[domain]>319)):continue
                    support=domain&(np.min(abs(self.rows[:,None]-q[None,:,1]),axis=1)<=1)
                    score=float(min(.85,.55+.2*min(1,len(q)/30))*np.exp(-errors.mean()/3))
                    rank=len(q)-errors.mean()*5
                    rank+=30*self.config.image_nn_rank_weight*self._nn_support(q)
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
        # Use whole stripe centers BEFORE applying the search gate. Selecting
        # the nearest individual pixel follows alternating edges of thick paint
        # and creates curvature even on a straight stripe. Cache per frame/call.
        # Vectorized run endpoints avoid a Python loop/mean for every paint
        # component, especially on busy road texture.
        transitions=np.diff(np.pad(paint[78:169:2].astype(np.int8),((0,0),(1,1))),axis=1)
        rr,starts=np.nonzero(transitions==1)
        _,ends=np.nonzero(transitions==-1)
        narrow=(ends-starts)<=12
        rr=rr[narrow];mid=(starts[narrow]+ends[narrow]-1)*.5
        centers={row:mid[rr==i] for i,row in enumerate(range(78,169,2))}
        seeds=[]
        for row in (168,156,144,132):
            xs=centers[row]
            xs=xs[(xs>8)&(xs<312)&((xs<175) if side==0 else (xs>145))]
            seeds.extend((row,float(x)) for x in xs)
        stats=dict(seeds=len(seeds),short=0,fit=0,quality=0,side=0,accepted=0)
        self.detection_diagnostics[side]=stats
        best=None
        for start,seed in seeds[:28]:
            ys,xs=[],[];value=seed;slope=0.;last=start
            for y in range(start,76,-2):
                options=centers[y]
                predicted=value+slope*(y-last)
                gap=last-y
                if gap>24:break
                gate=min(11.,5.+.25*gap)
                options=options[abs(options-predicted)<gate]
                if gap>4:
                    if len(ys)<4:continue
                    # A distant dot must agree with the incoming tangent.
                    # Otherwise it can turn the search away from real upper paint.
                    options=options[abs((options-value)/(y-last)-slope)<=.5]
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
            # Prefer a line only when it explains the same robust inliers at
            # subpixel precision and bending adds negligible improvement.
            # Strong real curves retain their quadratic; fusion still smooths
            # changes over time instead of snapping the displayed boundary.
            line=np.polynomial.polynomial.polyfit((ys[good]-125)/50,xs[good],1)
            line_error=abs(np.polynomial.polynomial.polyval((ys-125)/50,line)-xs)
            if (line_error[good].mean()<=.8 and line_error[good].max()<=2.
                    and line_error[good].mean()-residual[good].mean()<=.25):
                coeff=np.array([line[0],line[1],0.])
                residual=line_error
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
            # Equal band weighting prevents plentiful near pixels from hiding
            # a poorly supported upper curve. Use the fitted observed span only.
            evidence=[]
            for lo,hi in ((72,124),(124,148),(148,180)):
                rows=np.arange(max(lo,int(ys[good].min())),min(hi,int(ys[good].max())+1),2)
                if len(rows)<3:continue
                xx=np.rint(np.polynomial.polynomial.polyval((rows-125)/50,coeff)).astype(int)
                hits=np.zeros(len(rows),bool)
                for dx in (-2,-1,0,1,2):
                    x=xx+dx;inside=(x>=0)&(x<320)
                    hits[inside]|=paint[rows[inside],x[inside]]
                evidence.append(float(hits.mean()))
            rank+=.12*min(evidence,default=0.)
            rank+=self.config.image_nn_rank_weight*self._nn_support(np.column_stack((xs[good],ys[good])))
            # Translation-equivalent stripes can differ only by fit roundoff.
            # Keep deterministic seed order for numerical ties.
            if best is None or rank>best[0]+1e-9:
                best=(rank,np.column_stack((values,self.rows)),score)
                if self.trace_fit_evidence:
                    # These are the selected fit inliers, not every row on
                    # the displayed/extended quadratic and not ground truth.
                    stats['fit_observed_points'] = [
                        [float(x), int(y)] for x,y in zip(xs[good],ys[good])
                    ]
                    stats['fit_curve_rows'] = [int(y) for y in self.rows[valid]]
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
        # Replace on every frame (including empty masks); no stale NN support.
        self.nn_rank_mask=lane>0
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
        # Global pavement brightness can suppress genuine paint in shadows
        # (especially when filming a monitor). Admit locally contrasted paint
        # supported by the network, not the network mask alone. Keep pole,
        # road and vehicle exclusions identical for both admission paths.
        nn_support=self.nn_rank_mask.copy()
        for offset in (1,2,3):
            nn_support[:,offset:] |= self.nn_rank_mask[:,:-offset]
            nn_support[:,:-offset] |= self.nn_rank_mask[:,offset:]
        paint &= (brightness>=threshold) | (nn_support & (brightness>=85.))
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
            self.fragment_hits[:]=0;self.fragment_offsets[:]=np.nan
            self.region_segments=[[],[]];self.reacquire=[None,None];self.reacquire_hits=[0,0]
            self.short_pair_hits=[0,0];self.short_pair_previous=[None,None]
            self.curve_reference_hits[:]=0;self.curve_guidance[:]=0.
            self.current_pair_hits=0;self.current_pair_previous=None
        motion,motion_score=road_motion(self.previous,gray,allowed[::2,::2])
        # A discontinuous source cut must not carry old boundaries into a new scene.
        if self.previous is not None and motion_score==0 and np.mean(abs(gray-self.previous))>45:
            self.history=[None,None];self.width=None;self.scores=[0.,0.]
            self.last_valid_pair=None
            self.side_missing[:]=0;self.observed_streak[:]=0;self.width_samples[:]=0
            self.motion_velocity[:]=0.;self.missing_seconds=0.
            self.fragment_hits[:]=0;self.fragment_offsets[:]=np.nan
            self.region_segments=[[],[]];self.reacquire=[None,None];self.reacquire_hits=[0,0]
            self.short_pair_hits=[0,0];self.short_pair_previous=[None,None]
            self.curve_reference_hits[:]=0;self.curve_guidance[:]=0.
            self.current_pair_hits=0;self.current_pair_previous=None
        self.previous=gray;self.last_time=timestamp_s if np.isfinite(timestamp_s) else None
        identity=np.array([[1.,0.,0.],[0.,1.,0.]])
        # Compare independently detected candidates in the CURRENT frame.
        # Reliable measured flow is geometric compensation, not inertia.
        self.observation_motion=motion.copy() if motion_score>=.5 else identity.copy()
        step=float(np.clip(dt if np.isfinite(dt) else .1,.01,.2))
        if motion_score:
            # Filter motion velocity, so noisy alternating flow cannot reverse
            # the boundary instantly. This is visual inertia, not measured IMU.
            alpha=1-np.exp(-step/.25)
            self.motion_velocity=(1-alpha)*self.motion_velocity+alpha*(motion-identity)/step
        else:
            self.motion_velocity*=np.exp(-step/.6)
        # Unobserved motion decays to a retained shape instead of drifting away.
        if motion_score<.5:
            motion=identity+self.motion_velocity*step*np.exp(-self.missing_seconds/1.5)
        else:
            motion=identity+(motion-identity)*np.exp(-self.missing_seconds/1.5)
        predicted=[self._warp(p,motion) for p in self.history]
        observations=[self._detect(paint,s,predicted[s]) for s in (0,1)]
        observations,predicted=self._resolve_duplicate_observations(observations,predicted)
        self.missing_seconds=0. if any(o is not None for o in observations) else self.missing_seconds+step
        return self._fuse(observations,predicted,motion,motion_score,step)

    def _fuse(self, observations, predicted, motion, motion_score, step):
        """Fuse only supported rows; commit history only after pair validation."""
        for side,o in enumerate(observations):
            stats=self.detection_diagnostics[side]
            stats['observation_quality']=float(o[1]) if o is not None else None
            stats['observation_prior_error_px']=None
            if o is not None and predicted[side] is not None:
                common=np.isfinite(o[0][:,0])&np.isfinite(predicted[side][:,0])
                if len(o)>2:common &= o[2]
                if common.any():
                    stats['observation_prior_error_px']=float(np.percentile(abs(o[0][common,0]-predicted[side][common,0]),90))
        predicted=self._confirm_current_pair(observations,predicted)
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
                    temporal=len(observation)>3 and observation[3]
                    degree=2 if quality>=.65 or temporal else (1 if np.ptp(self.rows[existing])>=16 else 0)
                    if temporal:
                        # Anchor unobserved rows; keep one quadratic without
                        # letting a weak local fragment move the entire boundary.
                        targets=np.zeros(finite.sum());targets[existing[finite]]=diff
                        weights=np.where(existing[finite],1.,4.)
                        dc=np.polynomial.polynomial.polyfit(u[finite],targets,2,w=weights)
                    else:
                        dc=np.polynomial.polynomial.polyfit(u[existing],diff,degree)
                    delta=np.polynomial.polynomial.polyval(u,dc)
                    domain=finite|np.isfinite(measured[:,0])
                    fraction=min(alpha,80*step/max(float(np.max(abs(delta[domain]))),1e-9))
                    if temporal:
                        fraction=min(fraction,5*step/max(float(np.max(abs(delta[domain]))),1e-9))
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
                    if self._plausible_pair(pair):
                        output[side]=candidate;states[side]='inferred';scores[side]=min(scores[side],.15)

        # A valid confirmed current pair must not be discarded because its
        # fusion with unobserved OLD tails violates spacing. Preserve all
        # absolute/learned pair guards; replace only after motion-consistent
        # bilateral confirmation, and only over current supported rows.
        if (not self._plausible_pair(output) and self.current_pair_hits>=3
                and self.current_pair_previous is not None
                and self._plausible_pair(self.current_pair_previous)):
            reason='learned_width' if self._plausible_pair(output,learned=False) else 'shape_or_absolute_width'
            output=[p.copy() for p in self.current_pair_previous]
            states=['observed','observed'];scores=[float(o[1]) for o in observations]
            supports=[np.isfinite(p[:,0]) for p in output];residuals=[0.,0.]
            for stats in self.detection_diagnostics:
                stats['confirmed_pair_replaced_invalid_fusion']=reason
        restored=False
        if not self._plausible_pair(output):
            common=np.isfinite(output[0][:,0])&np.isfinite(output[1][:,0])
            gap=output[1][:,0]-output[0][:,0]
            basic=self._plausible_pair(output,learned=False)
            reason='learned_width' if basic else 'shape_or_absolute_width'
            for stats in self.detection_diagnostics:
                stats['fusion_pair_rejection']=reason
                stats['fusion_pair_common_rows']=int(common.sum())
                stats['fusion_pair_min_gap']=float(gap[common].min()) if common.any() else None
            # Try accepting either side independently against the last VALID
            # opposite, before holding both. Never reuse arbitrary history.
            fallback=predicted if self._plausible_pair(predicted) else ([self._quadratic(p) for p in self.last_valid_pair] if self.last_valid_pair is not None else None)
            for stats in self.detection_diagnostics:
                stats['fallback_source']='predicted' if fallback is predicted else 'last_valid_pair'
            choices=[]
            if fallback is not None and self._plausible_pair(fallback):
                for side in (0,1):
                    if observations[side] is None:continue
                    # Weak temporal evidence must not win a pair conflict and
                    # force the independently detected opposite side into HELD.
                    if len(observations[side])>3 and observations[side][3]:continue
                    pair=[p.copy() for p in fallback];pair[side]=output[side].copy()
                    if self._plausible_pair(pair):choices.append((scores[side],side,pair))
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
                if not self._plausible_pair(output):
                    output=[np.column_stack((np.full(48,np.nan),self.rows)) for _ in (0,1)]
                    states=['unavailable','unavailable'];scores=[0.,0.]
                    supports=[np.zeros(48,bool),np.zeros(48,bool)];residuals=[None,None]

        if self._plausible_pair(output):
            if not restored and states==['observed','observed'] and min(scores)>=.5:
                self.last_valid_pair=[p.copy() for p in output]
            self.history=[p.copy() for p in output]
            # Learn spacing only from concurrently observed pixels, never from
            # virtual extensions or a pair that was rejected/restored.
            common=supports[0]&supports[1]
            if not restored and states==['observed','observed'] and common.sum()>=8:
                widths=output[1][:,0]-output[0][:,0]
                if self.width is None:self.width=np.clip(1.6*(self.rows-65),24,240)
                first=common&(self.width_samples==0)
                seen=common&~first
                self.width[first]=widths[first]
                self.width[seen]=.9*self.width[seen]+.1*widths[seen]
                self.width_samples[common]+=1
        self.scores=scores
        observed=[s for s in (0,1) if supports[s].any() and states[s] in ('observed','aligning','partial')]
        reference='both' if len(observed)==2 else 'left' if observed==[0] else 'right' if observed==[1] else 'none'
        state='tracking' if observed else 'held' if restored else 'predicted' if self.last_valid_pair is not None else 'unavailable'
        output=self._display_continuation(output,states,supports,step)
        return ImageBoundaryEstimate(*output,projection_for((180,320),self.config),*states,*scores,
                                     motion_score,motion,state,reference,*supports,*residuals,restored,
                                     [dict(v) for v in self.detection_diagnostics])
