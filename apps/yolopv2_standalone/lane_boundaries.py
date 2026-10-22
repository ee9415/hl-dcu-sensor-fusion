"""Halla campus fixed-anchor BEV boundary pair. No center path is displayed.

Distances use the existing *uncalibrated* development IPM. Width in meters
is a requested physical width, not a claim of metric accuracy before calibration.
Only NumPy is required on the device.
"""
from dataclasses import dataclass, fields
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class BoundaryConfig:
    vehicle_width_m: float = 1.8
    left_margin_m: float = .3
    right_margin_m: float = .3
    max_lateral_rate_m_s: float = 1.0
    max_forward_m: float = 25.0
    max_curvature_coefficient: float = .02
    width_tolerance_m: float = .08
    road_support_threshold: float = .9
    max_heading_coefficient: float = .2
    near_left_u: float = .12
    near_right_u: float = .88
    near_v: float = .98
    far_left_u: float = .46
    far_right_u: float = .54
    far_v: float = .60
    projection_reference_width_m: float = 2.4

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f'{field.name} must be a finite number')
            if value < 0 or (value == 0 and field.name not in ('left_margin_m', 'right_margin_m')):
                raise ValueError(f'{field.name} is out of range')
        if self.width_m >= 12 or self.max_forward_m > 40:
            raise ValueError('Requested corridor exceeds development BEV extent')
        if max(self.vehicle_width_m/2+self.left_margin_m,
               self.vehicle_width_m/2+self.right_margin_m) >= 6:
            raise ValueError('Fixed anchor exceeds development BEV extent')
        if self.road_support_threshold > 1 or self.width_tolerance_m >= self.width_m*.25:
            raise ValueError('Invalid road support or normal-width tolerance')
        if not (0 < self.near_left_u < self.far_left_u < self.far_right_u
                < self.near_right_u < 1 and 0 < self.far_v < self.near_v <= 1):
            raise ValueError('Invalid image projection anchors')
        if self.max_heading_coefficient > .25:
            raise ValueError('Heading exceeds fixed-anchor normal-width envelope')
        if self.projection_reference_width_m >= 12:
            raise ValueError('Projection reference exceeds BEV extent')

    @property
    def width_m(self):
        return self.vehicle_width_m + self.left_margin_m + self.right_margin_m

    @classmethod
    def load(cls, path=None):
        # Fail explicitly on malformed settings; do not silently use another width.
        with Path(path or Path(__file__).with_name('lane_boundaries.json')).open(encoding='utf-8') as stream:
            return cls(**json.load(stream))


def transform(points, matrix):
    points = np.asarray(points, dtype=float)
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    return homogeneous[:, :2] / homogeneous[:, 2:3]


def projection_for(shape, config=None):
    """Map fixed corridor anchors, rather than the whole 12-unit BEV, to image.

    Normalized points are installation settings, not metric camera calibration.
    The same mapping is used for evidence sampling and boundary rendering.
    """
    h, w = shape
    c = config or BoundaryConfig()
    source = np.array([[c.near_left_u, c.near_v], [c.near_right_u, c.near_v],
                       [c.far_right_u, c.far_v], [c.far_left_u, c.far_v]]) * [w-1,h-1]
    # Camera mapping must not rescale with the requested corridor width:
    # otherwise reducing margins leaves the displayed width unchanged.
    left = -c.projection_reference_width_m/24+.5
    right = c.projection_reference_width_m/24+.5
    destination = np.array([[left*639,479], [right*639,479],
                            [right*639,0], [left*639,0]])
    a, b = [], []
    for (x, y), (u, v) in zip(source, destination):
        a.extend([[x,y,1,0,0,0,-u*x,-u*y], [0,0,0,x,y,1,-v*x,-v*y]])
        b.extend([u, v])
    return np.linalg.inv(np.append(np.linalg.solve(a, b), 1).reshape(3, 3))


def white_lane_evidence(rgb, lane):
    """Find neutral paint ridges, including regions missed by the NN lane head."""
    h, w = lane.shape
    sy, sx = rgb.shape[0]//h, rgb.shape[1]//w
    if rgb.shape[:2] != (h*sy, w*sx):
        raise ValueError('RGB size must be an integer multiple of the lane mask')
    image = rgb.reshape(h, sy, w, sx, 3).mean(axis=(1, 3))
    brightness = image.mean(axis=2)
    # Suppress asphalt grain before looking for narrow paint ridges.
    padded_image = np.pad(brightness, 1, mode='edge')
    brightness = sum(weight*padded_image[dy:dy+h, dx:dx+w]
                     for dy, wy in enumerate((1, 2, 1))
                     for dx, wx in enumerate((1, 2, 1))
                     for weight in (wy*wx,))/16.
    padded = np.pad(brightness, ((0, 0), (4, 4)), mode='edge')
    background = (padded[:, :w]+padded[:, 8:8+w])*.5
    supported = np.zeros_like(lane, bool)
    for offset in range(-3, 4):
        source = np.arange(w)+offset
        valid = (source >= 0) & (source < w)
        supported[:, valid] |= lane[:, source[valid]] != 0
    neutral = image.max(axis=2)-image.min(axis=2) < 75
    # Independent RGB evidence must be brighter than BOTH shoulders; a single
    # bright vehicle/curb edge is insufficient. Multiple scales cover perspective.
    ridge = np.zeros_like(lane, bool)
    for radius in (2, 4, 6):
        padded = np.pad(brightness, ((0, 0), (radius, radius)), mode='edge')
        shoulders = np.maximum(padded[:, :w], padded[:, 2*radius:2*radius+w])
        ridge |= brightness-shoulders >= 12
    evidence = (supported & (brightness-background >= 10)) | ridge
    return (evidence & neutral & (brightness >= 75)).astype(np.uint8)*255


@dataclass(frozen=True)
class BoundaryEstimate:
    valid: bool
    state: str
    forward: np.ndarray
    left: np.ndarray
    right: np.ndarray
    projection: np.ndarray
    left_state: str = 'unavailable'
    right_state: str = 'unavailable'
    left_coefficients: tuple = ()
    right_coefficients: tuple = ()
    width_error: float = 0.
    road_coverage: float = 0.
    corridor_feasible: object = None
    reference_side: str = 'none'
    left_confidence: float = 0.
    right_confidence: float = 0.

    @property
    def left_valid(self):
        return bool(np.count_nonzero(np.isfinite(self.left)) >= 8)

    @property
    def right_valid(self):
        return bool(np.count_nonzero(np.isfinite(self.right)) >= 8)

    @property
    def pair_valid(self):
        common = np.isfinite(self.left) & np.isfinite(self.right)
        return bool(np.count_nonzero(common) >= 8 and np.all(self.right[common]-self.left[common] > 0))

    def telemetry(self, config):
        width = self.right-self.left
        width = width[np.isfinite(width)] if self.pair_valid else np.empty(0)
        def values(array):
            return [float(v) if np.isfinite(v) else None for v in array] if self.valid else []
        return dict(boundary_valid=self.valid, boundary_pair_valid=self.pair_valid,
                    boundary_reference_side=self.reference_side,
                    left_boundary_confidence=self.left_confidence,
                    right_boundary_confidence=self.right_confidence,
                    boundary_confidence_kind='heuristic_not_probability',
                    left_boundary_valid=self.left_valid, right_boundary_valid=self.right_valid,
                    boundary_state=self.state,
                    left_boundary_state=self.left_state, right_boundary_state=self.right_state,
                    vehicle_width_m=config.vehicle_width_m, left_margin_m=config.left_margin_m,
                    right_margin_m=config.right_margin_m, corridor_width_m=config.width_m,
                    minimum_clearance_width_m=config.width_m,
                    boundary_width_min_dev=float(np.min(width)) if len(width) else None,
                    boundary_width_max_dev=float(np.max(width)) if len(width) else None,
                    clearance_sufficient=self.corridor_feasible,
                    boundary_units='uncalibrated_bev_development_units',
                    boundary_forward_dev=self.forward.tolist() if self.valid else [],
                    left_boundary_dev=values(self.left), right_boundary_dev=values(self.right),
                    left_boundary_coefficients=list(self.left_coefficients),
                    right_boundary_coefficients=list(self.right_coefficients),
                    normal_width_error_dev=self.width_error,
                    road_coverage=self.road_coverage, corridor_feasible=self.corridor_feasible)


class BoundaryTracker:
    """Joint anchored quadratic boundaries; no displayed or estimated center path.

    The left quadratic supplies a normal-offset construction for the right one.
    Both C0 anchors are exact. Only pairs within the normal-width tolerance are
    searched. Connected paint steers shape; traversable area determines feasibility.
    """
    def __init__(self, config=None):
        self.config = config or BoundaryConfig()
        self.forward = np.linspace(0., self.config.max_forward_m, 81)
        self.curvature = 0.
        self.heading = 0.
        self.last_time = None
        self.search_frames = 0
        self.last_feasible = False
        self.last_candidate_count = 0
        self.side_history = [None,None]
        self.candidates = []
        for value in np.linspace(-self.config.max_curvature_coefficient,
                                 self.config.max_curvature_coefficient, 21):
            geometry = self._geometry(float(value))
            if geometry is not None:
                self.candidates.append((float(value), geometry))

        # Nonzero heading frees the tangent without moving either anchor.
        for heading in np.linspace(-self.config.max_heading_coefficient,
                                   self.config.max_heading_coefficient, 5):
            if abs(heading) < 1e-10:
                continue
            for curvature in np.linspace(-self.config.max_curvature_coefficient,
                                         self.config.max_curvature_coefficient, 21):
                geometry = self._geometry(float(curvature), float(heading))
                if geometry is not None:
                    self.candidates.append(((float(curvature),float(heading)),geometry))
        self.candidates = [(v if isinstance(v,tuple) else (v,0.),g)
                           for v,g in self.candidates]

    def _geometry(self, curvature, heading=0.):
        c, x = self.config, self.forward
        left_anchor = -c.vehicle_width_m/2-c.left_margin_m
        right_anchor = c.vehicle_width_m/2+c.right_margin_m
        # Exact constant normal offsets are generally not quadratics. Fit with
        # both C0 anchors fixed and reject pairs exceeding the width tolerance.
        sample = np.linspace(-c.width_m, c.max_forward_m+c.width_m, 181)
        slope = heading+2*curvature*sample
        norm = np.sqrt(1+slope*slope)
        offset_x = sample-c.width_m*slope/norm
        offset_y = left_anchor+heading*sample+curvature*sample**2+c.width_m/norm
        if np.any(np.diff(offset_x) <= 0):
            return None
        use = (offset_x >= 0) & (offset_x <= c.max_forward_m)
        basis = np.column_stack((offset_x[use], offset_x[use]**2))
        b1, b2 = np.linalg.lstsq(basis, offset_y[use]-right_anchor, rcond=None)[0]
        lc = np.array([left_anchor, heading, curvature])
        rc = np.array([right_anchor, b1, b2])
        left = np.polynomial.polynomial.polyval(x, lc)
        right = np.polynomial.polynomial.polyval(x, rc)
        if np.any(right <= left) or np.min(left) < -6 or np.max(right) > 6:
            return None
        # Intersect each left normal with the right quadratic, measuring the
        # actual fitted pair, not merely the exact offset used as fitting input.
        slope = heading+2*curvature*x
        norm = np.sqrt(1+slope*slope)
        low, high = np.full_like(x,c.width_m*.5), np.full_like(x,c.width_m*1.5)
        for _ in range(30):
            distance = (low+high)*.5
            hit_x = x-distance*slope/norm
            residual = np.polynomial.polynomial.polyval(hit_x,rc)-left-distance/norm
            low = np.where(residual > 0, distance, low)
            high = np.where(residual > 0, high, distance)
        distance = (low+high)*.5
        hit_x = x-distance*slope/norm
        inside = (hit_x >= 0) & (hit_x <= c.max_forward_m)
        error = float(np.max(abs(distance[inside]-c.width_m)))
        if error > c.width_tolerance_m:
            return None
        return left, right, lc, rc, error

    def update(self, road, lane, timestamp_s, rgb=None):
        road, lane = np.asarray(road), np.asarray(lane)
        if road.ndim != 2 or min(road.shape) < 2 or road.shape != lane.shape:
            raise ValueError('Road and lane masks must have the same non-empty 2-D shape')
        if rgb is not None:
            lane = white_lane_evidence(rgb, lane)
        return self.update_bev(*self._sample(road, lane), timestamp_s,
                               projection=projection_for(road.shape,self.config))

    def _sample(self, road, lane):
        # The segmentation heads may exclude painted lane pixels from the road.
        # Restore only short, lane-supported horizontal holes bracketed by road;
        # a marking must not split otherwise connected pavement into two roads.
        road = road.copy()
        max_gap = max(1, round(road.shape[1] * 6 / 320))
        for y in range(road.shape[0]):
            indices = np.flatnonzero(road[y])
            gaps = np.flatnonzero(np.diff(indices) > 1)
            for a, b in zip(indices[gaps], indices[gaps+1]):
                if 1 < b-a <= max_gap+1 and np.any(lane[y, a+1:b]):
                    road[y, a+1:b] = 255
        # A small fixed BEV raster bounds runtime. Nearest-neighbor binary sampling.
        lateral = np.linspace(-6, 6, 241)
        forward = np.linspace(0., self.config.max_forward_m, 81)
        xx, yy = np.meshgrid((lateral/12+.5)*639, (1-forward/40)*479)
        pixels = np.rint(transform(np.column_stack((xx.ravel(), yy.ravel())),
                                    projection_for(road.shape,self.config))).astype(int)
        x, y = pixels.T
        inside = (x >= 0) & (x < road.shape[1]) & (y >= 0) & (y < road.shape[0])
        outputs = []
        for mask in (road, lane):
            sampled = np.zeros(len(x), dtype=bool)
            sampled[inside] = mask[y[inside], x[inside]] != 0
            outputs.append(sampled.reshape(xx.shape))
        return forward, lateral, *outputs

    def _observations(self, forward, lateral, lane, geometry):
        """Follow separate near-connected paint chains, allowing gaps/occlusion.

        Association starts at each fixed anchor, not at a road-mask edge.
        Missing paint never becomes a fabricated measurement.
        """
        observations = []
        for side,anchor in enumerate((geometry[0][0],geometry[1][0])):
            rows,values = [],[]
            slope = 0.
            for row,x in enumerate(forward):
                if x > self.config.max_forward_m:
                    break
                if values:
                    predicted=values[-1]+slope*(x-forward[rows[-1]])
                    gate=min(.75,.25+.2*(x-forward[rows[-1]]))
                else:
                    predicted=anchor
                    gate=.75
                    if x>self.config.max_forward_m*.35:
                        break
                paint=lateral[lane[row]]
                if not len(paint):
                    continue
                value=paint[np.argmin(abs(paint-predicted))]
                if abs(value-predicted)>gate:
                    continue
                rows.append(row);values.append(float(value))
                if len(rows)>=5:
                    dx=x-forward[rows[-5]]
                    if dx>0:
                        slope=float(np.clip((value-values[-5])/dx,-.6,.6))
            # Isolated paint spots/crosswalk fragments are not a usable side.
            if len(rows)>=12 and forward[rows[-1]]-forward[rows[0]]>=4:
                observations.append((side,np.asarray(rows),np.asarray(values)))
        return observations

    def _support(self, geometry, forward, lateral, road, lane, paint_distance=None):
        def nearest(axis, values):
            upper = np.clip(np.searchsorted(axis,values),1,len(axis)-1)
            return upper-(abs(values-axis[upper-1]) <= abs(values-axis[upper]))
        left, right = geometry[:2]
        rows = nearest(forward,self.forward)
        cross = left[:,None]+(right-left)[:,None]*np.linspace(0,1,25)[None,:]
        columns = nearest(lateral,cross)
        longitudinal = (self.forward >= forward[0]) & (self.forward <= forward[-1])
        inside = (cross >= lateral[0]) & (cross <= lateral[-1]) & longitudinal[:,None]
        coverage_rows = (road[rows[:,None],columns] & inside).mean(axis=1)
        # Additional pavement outside the pair differentiates wider free space.
        spare = np.stack((left-.3, right+.3),axis=1)
        spare_cols = nearest(lateral,spare)
        spare_inside = (spare >= lateral[0]) & (spare <= lateral[-1]) & longitudinal[:,None]
        clearance = np.mean(road[rows[:,None],spare_cols] & spare_inside)
        paint = 0.
        for edge in (left,right):
            if paint_distance is None:
                distance = np.where(lane[rows],abs(lateral[None,:]-edge[:,None]),np.inf).min(axis=1)
            else:
                distance = paint_distance[rows,nearest(lateral,edge)]
            # Broad continuous proximity gives a gradient towards paint instead
            # of leaving a misplaced boundary trapped outside a narrow gate.
            proximity = np.exp(-.5*(distance/.35)**2)*longitudinal
            # Require paint to connect to the near corridor. Remote parking
            # stripes on a suddenly widened road must not pull the pair aside.
            if np.mean(proximity[:16]) >= .15:
                paint += np.mean(proximity)*.5
        coverage = float(np.mean(coverage_rows))
        feasible = bool(np.all(coverage_rows >= self.config.road_support_threshold))
        # Free-space support dominates paint and preference for a gentle curve.
        score = coverage+clearance*.08+paint*.35
        return coverage, feasible, score

    def update_bev(self, forward, lateral, road, lane, timestamp_s, projection=None):
        c = self.config
        projection = np.eye(3) if projection is None else projection
        forward, lateral = np.asarray(forward), np.asarray(lateral)
        road, lane = np.asarray(road,dtype=bool), np.asarray(lane,dtype=bool)
        if (forward.ndim != 1 or lateral.ndim != 1 or len(forward)<2 or len(lateral)<2
                or road.shape != (len(forward),len(lateral)) or lane.shape != road.shape
                or not np.all(np.isfinite(forward)) or not np.all(np.isfinite(lateral))
                or np.any(np.diff(forward)<=0) or np.any(np.diff(lateral)<=0)):
            raise ValueError('BEV masks require increasing finite coordinate axes')
        geometry = self._geometry(self.curvature,self.heading)
        observations=self._observations(forward,lateral,lane,geometry)
        confidence=[0.,0.]
        accepted=[]
        for side,rows,values in observations:
            xs=forward[rows]
            fit=np.polynomial.polynomial.polyfit(xs,values,2)
            residual=float(np.mean(abs(np.polynomial.polynomial.polyval(xs,fit)-values)))
            continuity=len(rows)/max(1,rows[-1]-rows[0]+1)
            span=min(1.,(xs[-1]-xs[0])/10.)
            temporal=1.
            if self.side_history[side] is not None:
                old=np.interp(xs,self.forward,self.side_history[side])
                temporal=float(np.exp(-np.mean(abs(old-values))/1.5))
            confidence[side]=float(np.clip(.35*continuity+.3*span+
                                          .25*np.exp(-residual/.15)+.1*temporal,0,1))
            if confidence[side]>=.65:
                accepted.append((side,rows,values))
        observations=accepted
        observed_sides={o[0] for o in observations}
        side_states=[('observed' if s in observed_sides else
                      'held' if self.side_history[s] is not None else
                      'inferred' if observations else 'initial') for s in (0,1)]
        reference=('both' if len(observations)==2 else
                   ('left' if observations[0][0]==0 else 'right') if observations else 'none')
        self.last_candidate_count = 0
        state, feasible, coverage = 'held', None, 0.
        time_valid = np.isfinite(timestamp_s) and (self.last_time is None or timestamp_s>self.last_time)
        if time_valid:
            dt = .1 if self.last_time is None else min(timestamp_s-self.last_time,.2)
            self.last_time = timestamp_s
            if observations:
                # Compute the paint distance field once per frame, not once per
                # candidate. Supports nonuniform lateral axes without SciPy.
                paint_x = np.where(lane,lateral[None,:],-np.inf)
                previous = np.maximum.accumulate(paint_x,axis=1)
                paint_x = np.where(lane,lateral[None,:],np.inf)
                following = np.minimum.accumulate(paint_x[:,::-1],axis=1)[:,::-1]
                paint_distance = np.minimum(lateral[None,:]-previous,following-lateral[None,:])
                ranked = []
                self.search_frames += 1
                candidates = self.candidates
                # Reacquire globally whenever unsupported and periodically even
                # while tracking, so a coarse local search cannot lock the route.
                if self.last_feasible and self.search_frames % 5:
                    candidates = [(v,g) for v,g in candidates
                                  if abs(v[0]-self.curvature) <= c.max_curvature_coefficient*.21
                                  and abs(v[1]-self.heading) <= c.max_heading_coefficient*.51]
                # Keep the current interpolated pair as a candidate to avoid
                # grid snapping when no better coarse route is supported.
                candidates = candidates+[((self.curvature,self.heading),geometry)]
                self.last_candidate_count = len(candidates)
                for (value,heading), candidate in candidates:
                    support, fits, score = self._support(candidate,forward,lateral,road,lane,paint_distance)
                    # Prefer any feasible full-width route, then broader support.
                    score -= .01*abs(value)/c.max_curvature_coefficient
                    score -= .01*abs(value-self.curvature)/c.max_curvature_coefficient
                    score -= .005*abs(heading-self.heading)/c.max_heading_coefficient
                    score -= .005*abs(heading)/c.max_heading_coefficient
                    errors=[]
                    weights=[]
                    for side,rows,values in observations:
                        predicted=np.polynomial.polynomial.polyval(forward[rows],candidate[2+side])
                        errors.append(float(np.mean(np.minimum(abs(predicted-values),1.5))))
                        weights.append(confidence[side])
                    for side in (0,1):
                        if side not in observed_sides and self.side_history[side] is not None:
                            errors.append(float(np.mean(abs(candidate[side]-self.side_history[side]))))
                            weights.append(.65)
                    # The visible side is the primary geometry reference. Its
                    # paired offset supplies the missing side; road is a weak
                    # tie-breaker and still determines displayed space status.
                    rank=-np.average(errors,weights=weights)+.02*score
                    ranked.append(((rank,fits),(value,heading),candidate))
                _, (target,target_heading), desired = max(ranked,key=lambda item:item[0])
                movement = max(np.max(abs(desired[0]-geometry[0])),np.max(abs(desired[1]-geometry[1])))
                fraction = min(1.,c.max_lateral_rate_m_s*dt/max(movement,1e-9))
                # Regenerate paired geometry during smoothing; independent
                # coefficient interpolation would no longer enforce the width.
                for _ in range(12):
                    value = self.curvature+(target-self.curvature)*fraction
                    heading = self.heading+(target_heading-self.heading)*fraction
                    trial = self._geometry(value,heading)
                    if trial is not None and max(np.max(abs(trial[0]-geometry[0])),
                            np.max(abs(trial[1]-geometry[1]))) <= c.max_lateral_rate_m_s*dt+1e-8:
                        self.curvature, geometry = value, trial
                        self.heading = heading
                        break
                    fraction *= .5
                coverage, feasible, _ = self._support(geometry,forward,lateral,road,lane,paint_distance)
                state = 'tracking' if feasible else 'blocked'
                for side in observed_sides:
                    self.side_history[side]=geometry[side].copy()
            self.last_feasible = bool(feasible)
            if not observations and np.any(road):
                coverage,feasible,_=self._support(geometry,forward,lateral,road,lane)
        else:
            state = 'invalid_timestamp'
            if np.isfinite(timestamp_s):
                self.last_time = timestamp_s
        left,right,lc,rc,error = geometry
        result = BoundaryEstimate(True,state,self.forward.copy(),left.copy(),right.copy(),
                                  projection,*side_states,tuple(lc),tuple(rc),
                                  error,coverage,feasible,reference,*confidence)
        return result


def draw_polyline(frame, points, color, thickness=2, dashed=False):
    h, w = frame.shape[:2]
    strokes = []
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        if dashed and index % 6 >= 3:
            continue
        # Bound all raster work even for projections near the horizon.
        if not np.all(np.isfinite([start, end])) or np.max(np.abs([start, end])) > 10000:
            continue
        count = min(2000, int(np.max(np.abs(end-start)))+2)
        strokes.append(np.rint(np.linspace(start, end, count)).astype(int))
    if not strokes:
        return
    # Rasterize all segments before thickening. The identical square brush is
    # applied once per polyline rather than 81 times, preserving solid pixels.
    x, y = np.concatenate(strokes).T
    for dy in range(-thickness, thickness+1):
        for dx in range(-thickness, thickness+1):
            xx, yy = x+dx, y+dy
            valid = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
            frame[yy[valid], xx[valid]] = color


def boundary_points(estimate, side):
    if hasattr(estimate, 'image_'+side):
        return transform(getattr(estimate,'image_'+side),np.linalg.inv(estimate.projection))
    return np.column_stack(((getattr(estimate, side)/12+.5)*639,
                            (1-estimate.forward/40)*479))


def draw_boundaries(frame, estimate, mask_size=None):
    if not estimate.valid:
        return
    for side, color, state in (('left', (0, 220, 255), estimate.left_state),
                               ('right', (255, 180, 0), estimate.right_state)):
        if not getattr(estimate, side+'_valid'):
            continue
        points = boundary_points(estimate, side)
        if mask_size is not None:
            points = (getattr(estimate,'image_'+side).copy() if hasattr(estimate,'image_'+side)
                      else transform(points, estimate.projection))
            points *= np.array([frame.shape[1]/mask_size[0], frame.shape[0]/mask_size[1]])
        draw_polyline(frame, points, color, thickness=3, dashed=False)


def render_boundary_bev(estimate):
    frame = np.full((480, 640, 3), (14, 22, 30), np.uint8)
    frame[::60] = (43, 58, 68)
    draw_boundaries(frame, estimate)
    return frame
