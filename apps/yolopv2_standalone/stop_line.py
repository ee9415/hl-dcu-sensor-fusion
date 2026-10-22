"""Conservative RGB stop-bar candidates; confidence is heuristic, not probability."""
from dataclasses import dataclass
import numpy as np
from lane_boundaries import boundary_points, transform, draw_polyline


@dataclass(frozen=True)
class StopLineEstimate:
    state: str = 'unavailable'
    confidence: float = 0.
    points: object = None  # normalized image coordinates

    def telemetry(self):
        return dict(stop_line_state=self.state,stop_line_confidence=self.confidence,
                    stop_line_confidence_kind='heuristic_not_probability',
                    stop_line_valid=self.state in ('detected','estimated'),
                    stop_line_points=self.points.tolist() if self.points is not None else [])


class StopLineTracker:
    def __init__(self):
        self.last_time=None
        self.last_observed=None
        self.points=None
        self.score=0.
        self.hits=0
        self.candidate_y=None

    def update(self,rgb,boundaries,timestamp,detections=()):
        # Cheap fixed 320x180 nearest sampling; independent from lane ridges
        # (those intentionally suppress broad transverse paint).
        h,w=180,320
        image=rgb[np.linspace(0,rgb.shape[0]-1,h).astype(int)[:,None],
                  np.linspace(0,rgb.shape[1]-1,w).astype(int)[None,:]].astype(float)
        bright=image.mean(axis=2)
        neutral=image.max(axis=2)-image.min(axis=2)<55
        vertical=np.pad(bright,((4,4),(0,0)),mode='edge')
        contrast=bright-np.maximum(vertical[:h],vertical[8:8+h])
        white=neutral & (bright>110) & (contrast>12)
        for box in detections:
            x1,x2=np.clip([int(box.xmin*w),int(np.ceil(box.xmax*w))],0,w)
            y1,y2=np.clip([int(box.ymin*h),int(np.ceil(box.ymax*h))],0,h)
            white[y1:y2,x1:x2]=False
        # Projection is expressed in the segmentation's fixed 320x180 image.
        edges=[transform(boundary_points(boundaries,s),boundaries.projection)
               for s in ('left','right')]
        rows=[]
        for y in range(int(h*.5),int(h*.94)):
            if any(y<e[:,1].min() or y>e[:,1].max() for e in edges):
                continue
            limits=[np.interp(y,e[::-1,1],e[::-1,0]) for e in edges]
            a,b=int(np.ceil(limits[0])),int(np.floor(limits[1]))
            a,b=max(1,a),min(w-2,b)
            if b-a<20:
                continue
            paint=white[y,a:b+1]
            coverage=float(paint.mean())
            indices=np.flatnonzero(paint)
            if len(indices)<2:
                continue
            span=(indices[-1]-indices[0])/max(1,b-a)
            if coverage>=.7 and span>=.8:
                rows.append((y,a,b,.6*coverage+.4*span))
        groups=[]
        for row in rows:
            if not groups or row[0]>groups[-1][-1][0]+1:
                groups.append([])
            groups[-1].append(row)
        groups=[g for g in groups if 1<=len(g)<=7]
        # Repeated transverse bars are ambiguous (e.g. crosswalk), never assert
        # a stop line merely because one of them is nearest.
        candidate=max((max(g,key=lambda r:r[3]) for g in groups),
                      key=lambda r:r[0],default=None) if len(groups)==1 else None
        if not np.isfinite(timestamp) or (self.last_time is not None and timestamp<=self.last_time):
            self.hits=0;self.points=None;self.last_observed=None
            self.last_time=timestamp if np.isfinite(timestamp) else None
            return StopLineEstimate()
        dt=0.1 if self.last_time is None else timestamp-self.last_time
        self.last_time=timestamp
        if candidate is not None:
            y,a,b,score=candidate
            close=self.candidate_y is not None and abs(y/h-self.candidate_y)<.04 and dt<.3
            self.hits=self.hits+1 if close else 1
            self.candidate_y=y/h
            points=np.array([[a/(w-1),y/(h-1)],[b/(w-1),y/(h-1)]])
            if self.hits>=3:
                self.points=points
                self.last_observed=timestamp
                self.score=float(score)
                return StopLineEstimate('detected',self.score,points.copy())
            return StopLineEstimate('candidate',float(score)*self.hits/3,points)
        self.hits=0;self.candidate_y=None
        if self.points is not None and timestamp-self.last_observed<=.4:
            confidence=self.score*max(0.,1-(timestamp-self.last_observed)/.4)
            return StopLineEstimate('estimated',confidence,self.points.copy())
        self.points=None;self.last_observed=None
        return StopLineEstimate()


def draw_stop_line(frame,estimate,boundaries=None):
    if estimate.state not in ('detected','estimated'):
        return
    if boundaries is None:
        points=estimate.points*[frame.shape[1]-1,frame.shape[0]-1]
    else:
        points=transform(estimate.points*[319,179],np.linalg.inv(boundaries.projection))
    points=np.linspace(points[0],points[-1],19)
    draw_polyline(frame,points,(255,0,255),2,dashed=estimate.state=='estimated')
