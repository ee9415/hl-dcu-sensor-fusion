"""Mask contours and conservative, frame-local grouping of lane paint pieces.

Contours remain measured pixel outlines. Links express membership only: they
do not fill the mask, select ego lanes, or create guidance boundaries.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LanePiece:
    contour: np.ndarray
    rows: np.ndarray
    centers: np.ndarray
    widths: np.ndarray
    skeleton_curve: object = None

    def endpoint(self, top):
        count = min(12, len(self.rows))
        take = slice(0, count) if top else slice(-count, None)
        slope, intercept = np.polyfit(self.rows[take], self.centers[take], 1)
        y = float(self.rows[0 if top else -1])
        return y, float(slope * y + intercept), float(slope)


@dataclass
class LaneContourResult:
    pieces: list
    groups: list
    links: list
    rejected_contours: list
    directional_mask: object = None
    group_curves: object = None
    skeleton_mask: object = None
    curve_segment_observed: object = None


def observed_curve_segments(curve, mask):
    """A line is observed only if its complete raster lies in measured paint."""
    segments = []
    for a,b in zip(curve[:-1],curve[1:]):
        ax,ay = np.rint(a[:2]).astype(int)
        bx,by = np.rint(b[:2]).astype(int)
        if not a[2] or not b[2]:
            segments.append(False)
            continue
        left,right = min(ax,bx),max(ax,bx)+1
        top,bottom = min(ay,by),max(ay,by)+1
        raster = np.zeros((bottom-top,right-left),np.uint8)
        cv2.line(raster,(ax-left,ay-top),(bx-left,by-top),1,1)
        inside = (left>=0 and top>=0 and right<=mask.shape[1] and bottom<=mask.shape[0])
        segments.append(bool(inside and np.all(mask[top:bottom,left:right][raster>0]>0)))
    return np.asarray(segments,dtype=bool)


def connect_lane_pieces(pieces, image_shape, max_gap_fraction=.22,
                        ambiguity_margin=.2):
    """Associate monotone-y continuations, including overlapping cut ends.

    Tangent comparisons use angles so shallow perspective lanes are not
    penalized by large dx/dy values. Nearby intermediate pieces block skips.
    Each endpoint has at most one unambiguous continuation; paths cannot fork.
    """
    candidates = []
    for lower,a in enumerate(pieces):
        ay,ax,sa = a.endpoint(True)
        for upper,b in enumerate(pieces):
            by,bx,sb = b.endpoint(False)
            gap = ay-by
            overlap = min(6.,.5*min(np.ptp(a.rows),np.ptp(b.rows)))
            if lower == upper or not -overlap <= gap <= image_shape[0]*max_gap_fraction:
                continue
            if a.rows.mean() <= b.rows.mean() or a.rows[-1] <= b.rows[-1]:
                continue
            angle = abs(np.arctan(sa)-np.arctan(sb))
            if angle > np.deg2rad(28):
                continue
            wa,wb = float(np.median(a.widths[:8])),float(np.median(b.widths[-8:]))
            if max(wa,wb) > 3*min(wa,wb)+2:
                continue
            middle = (ay+by)/2
            error = abs((ax+sa*(middle-ay))-(bx+sb*(middle-by)))
            tolerance = max(2.5,image_shape[1]*.008,1.25*max(wa,wb))
            if error > tolerance:
                continue
            if gap > 3:
                chord = (ax-bx)/gap
                if max(abs(np.arctan(chord)-np.arctan(sa)),
                       abs(np.arctan(chord)-np.arctan(sb))) > np.deg2rad(28):
                    continue
            # Never skip over a closer observed branch to form a long link.
            skipped = False
            for other,c in enumerate(pieces):
                if other in (lower,upper):
                    continue
                rows = c.rows[(c.rows >= min(ay,by)-1)&(c.rows <= max(ay,by)+1)]
                if len(rows) < 2:
                    continue
                expected = bx+(rows-by)*(ax-bx)/gap if abs(gap)>1e-6 else np.full(len(rows),(ax+bx)/2)
                observed = np.interp(rows,c.rows,c.centers)
                if np.count_nonzero(abs(expected-observed) < max(2.,np.median(c.widths)/2)) >= 2:
                    skipped = True
                    break
            if skipped:
                continue
            score = error/tolerance + angle/np.deg2rad(28)*.25 + max(gap,0)/(image_shape[0]*max_gap_fraction)*.25
            candidates.append((score,lower,upper))
    def best(options):
        options = sorted(options)
        if not options or (len(options)>1 and options[1][0]-options[0][0] < ambiguity_margin):
            return None
        return options[0]
    links = [(a,b) for candidate in sorted(candidates) for _,a,b in [candidate]
             if best([c for c in candidates if c[1] == a]) == candidate
             and best([c for c in candidates if c[2] == b]) == candidate]
    parent = list(range(len(pieces)))
    def root(i):
        while parent[i] != i:
            i = parent[i]
        return i
    for a,b in links:
        parent[root(a)] = root(b)
    groups = {}
    for i in range(len(pieces)):
        groups.setdefault(root(i),[]).append(i)
    return list(groups.values()),links


def lane_group_curves(pieces, groups):
    """Skeleton centers plus local interpolation, separate from contours.

    Each output is Nx3: x, y, observed-row flag. Missing rows are inferred;
    they are never written into the measured mask. Local smoothing does not
    replace the whole curved lane with a global straight/quadratic fit.
    """
    curves = []
    samples = [p.skeleton_curve if p.skeleton_curve is not None
               else np.column_stack((p.centers,p.rows)) for p in pieces]
    for members in groups:
        rows = np.arange(min(samples[i][:,1].min() for i in members),
                         max(samples[i][:,1].max() for i in members)+1)
        x = np.full(len(rows),np.nan)
        observed_columns = []
        for j,y in enumerate(rows):
            values = [point[0] for i in members for point in samples[i]
                      if point[1] == y]
            observed_columns.append(np.asarray(values))
            if len(values):
                # Select an actual skeleton pixel, never the midpoint of two
                # disconnected components in an overlap row.
                target = np.median(values)
                x[j] = min(values,key=lambda value: abs(value-target))
        observed = np.isfinite(x)
        x = np.interp(rows,rows[observed],x[observed])
        known = np.flatnonzero(observed)
        for left,right in zip(known[:-1],known[1:]):
            if right-left <= 1:
                continue
            # Cubic Hermite interpolation matches the measured end tangents;
            # shape-preserving tangent bounds prevent bridge overshoot.
            lower = known[(known >= left-7)&(known <= left)]
            upper = known[(known >= right)&(known <= right+7)]
            chord = (x[right]-x[left])/(right-left)
            sl = np.polyfit(rows[lower],x[lower],1)[0] if len(lower)>1 else chord
            sr = np.polyfit(rows[upper],x[upper],1)[0] if len(upper)>1 else chord
            if abs(chord)<1e-8:
                sl=sr=0.
            else:
                sl = np.clip(sl/chord,0,3)*chord
                sr = np.clip(sr/chord,0,3)*chord
            t = np.arange(1,right-left)/(right-left)
            x[left+1:right] = ((2*t**3-3*t**2+1)*x[left]
                +(t**3-2*t**2+t)*(right-left)*sl
                +(-2*t**3+3*t**2)*x[right]
                +(t**3-t**2)*(right-left)*sr)
        # Short local fit preserves curvature while blending cut-end offsets.
        smooth = x.copy()
        if len(members)>1:
            for j in range(len(rows)):
                lo,hi = max(0,j-4),min(len(rows),j+5)
                delta = rows[lo:hi]-rows[j]
                smooth[j] = np.polynomial.polynomial.polyfit(delta,x[lo:hi],min(2,hi-lo-1))[0]
        for j,columns in enumerate(observed_columns):
            if len(columns) and any(pieces[i].skeleton_curve is not None for i in members):
                smooth[j] = columns[np.argmin(abs(columns-smooth[j]))]
        curves.append(np.column_stack((smooth,rows,observed)))
    return curves


def longitudinal_mask(binary, min_angle_deg=20., length=21):
    """Local directional opening, retaining only observed mask pixels.

    Angles are measured from the horizontal. A bank of short line elements
    follows changing tangents on curves, rather than fitting one global line.
    No closing or dilation beyond each opening is used to invent lane pixels.
    """
    if not 0 < min_angle_deg < 90 or length < 5 or length % 2 != 1:
        raise ValueError('expected angle in (0,90) and odd length >= 5')
    source = np.asarray(binary, dtype=np.uint8)
    kept = np.zeros_like(source)
    radius = length // 2
    for angle in np.linspace(min_angle_deg, 180-min_angle_deg, 29):
        radians = np.deg2rad(angle)
        # Short near-vertical dashes need less longitudinal support. Shallow
        # tangents need a longer probe to distinguish them from wide bars.
        probe = min(radius, 5) if 40 <= angle <= 140 else radius
        dx, dy = np.rint(probe * np.array([np.cos(radians), np.sin(radians)])).astype(int)
        kernel = np.zeros((length, length), np.uint8)
        cv2.line(kernel, (radius-dx, radius-dy), (radius+dx, radius+dy), 1, 1)
        opened = cv2.morphologyEx(source, cv2.MORPH_OPEN, kernel,
                                 borderType=cv2.BORDER_CONSTANT, borderValue=0)
        kept |= opened
    return kept & source


def thin_mask(binary):
    """Zhang-Suen thinning; retain topology without an optional cv2 package."""
    image = np.pad(np.asarray(binary, dtype=np.uint8), 1)
    while True:
        changed = False
        for phase in (0, 1):
            p = [image[:-2,1:-1], image[:-2,2:], image[1:-1,2:],
                 image[2:,2:], image[2:,1:-1], image[2:,:-2],
                 image[1:-1,:-2], image[:-2,:-2]]
            count = sum(p)
            transitions = sum((p[i] == 0) & (p[(i+1)%8] == 1) for i in range(8))
            if phase == 0:
                safe = (p[0]*p[2]*p[4] == 0) & (p[2]*p[4]*p[6] == 0)
            else:
                safe = (p[0]*p[2]*p[6] == 0) & (p[0]*p[4]*p[6] == 0)
            remove = (image[1:-1,1:-1] > 0) & (count >= 2) & (count <= 6) & (transitions == 1) & safe
            if remove.any():
                image[1:-1,1:-1][remove] = 0
                changed = True
        if not changed:
            return image[1:-1,1:-1]


def longitudinal_components(binary):
    """Split skeleton junctions, reject transverse branches, recover paint.

    Each returned label is a separate observed branch, not an inferred lane
    identity. Ambiguous junction pixels are omitted instead of joining lanes.
    Curves are classified by their vertical extent, not a straight-line fit.
    """
    source = np.asarray(binary, dtype=np.uint8)
    skeleton = thin_mask(source)
    # Local principal tangent also cuts horizontal connectors with no fork
    # (e.g. the bridge between two lanes at their distant ends).
    yy,xx = np.indices(source.shape,dtype=np.float32)
    weight = skeleton.astype(np.float32)
    def total(values):
        return cv2.boxFilter(values,-1,(15,15),normalize=False,
                             borderType=cv2.BORDER_CONSTANT)
    n = np.maximum(total(weight),1)
    mx,my = total(weight*xx)/n,total(weight*yy)/n
    vx = np.maximum(0,total(weight*xx*xx)/n-mx*mx)
    vy = np.maximum(0,total(weight*yy*yy)/n-my*my)
    cov = total(weight*xx*yy)/n-mx*my
    norm = np.sqrt((vx-vy)**2+4*cov**2)
    tangent_y2 = .5*(1-(vx-vy)/np.maximum(norm,1e-6))
    longitudinal = tangent_y2 >= np.sin(np.deg2rad(8))**2
    # Count connected neighbor groups around a skeleton pixel: >=3 is a fork.
    padded = np.pad(skeleton, 1)
    p = [padded[:-2,1:-1],padded[:-2,2:],padded[1:-1,2:],
         padded[2:,2:],padded[2:,1:-1],padded[2:,:-2],
         padded[1:-1,:-2],padded[:-2,:-2]]
    branches = sum((p[i] == 0) & (p[(i+1)%8] == 1) for i in range(8))
    junctions = ((branches >= 3) & (skeleton > 0)).astype(np.uint8)
    cuts = cv2.dilate(junctions, np.ones((3,3),np.uint8))
    paths = skeleton & (1-cuts) & longitudinal.astype(np.uint8)
    count, labels = cv2.connectedComponents(paths, connectivity=8)
    radius = cv2.distanceTransform(source,cv2.DIST_L2,5)
    seeds = np.zeros_like(labels, dtype=np.int32)
    accepted = []
    for label in range(1,count):
        y,x = np.nonzero(labels == label)
        if len(y) < 4 or np.ptp(y) < 4 or np.ptp(y) < .12 * np.ptp(x):
            continue
        accepted.append(label)
        seeds[y,x] = label
    # Recover the width of each measured branch, limited by its local radius.
    recovered = np.zeros_like(labels,dtype=np.int32)
    best = np.full(source.shape,np.inf)
    for label in accepted:
        branch = np.zeros_like(source)
        y,x = np.nonzero(seeds == label)
        for py,px in zip(y,x):
            cv2.circle(branch,(int(px),int(py)),max(1,int(np.ceil(radius[py,px]))),1,-1)
        distance = cv2.distanceTransform((seeds != label).astype(np.uint8),cv2.DIST_L2,5)
        take = (branch > 0) & (source > 0) & (distance < best)
        recovered[take] = label
        best[take] = distance[take]
    # A one-pixel seam prevents recovered widths from rejoining distinct paths.
    seam = np.zeros_like(source,bool)
    for dy,dx in ((1,0),(0,1),(1,1),(1,-1)):
        a = recovered[max(0,dy):source.shape[0],max(0,dx):source.shape[1]+min(0,dx)]
        b = recovered[:source.shape[0]-dy,max(0,-dx):source.shape[1]-max(0,dx)]
        different = (a > 0) & (b > 0) & (a != b)
        seam[max(0,dy):source.shape[0],max(0,dx):source.shape[1]+min(0,dx)] |= different
    recovered[seam] = 0
    return recovered


def extract_lane_contours(mask, threshold=0.5, min_height=5,
                          max_gap_fraction=0.22, ambiguity_margin=0.2,
                          direction_filter=True):
    """Extract external outlines; join only mutual, unambiguous continuations.

    Floating masks are probabilities in [0,1]; bool/integer masks are binary.
    All geometry is in input pixels. Horizontal blobs are kept as rejected
    outlines for inspection. Local directional opening can remove transverse
    branches before extracting contours; it never joins separate mask pieces.
    """
    mask = np.asarray(mask)
    if mask.ndim != 2 or not np.isfinite(mask).all():
        raise ValueError('lane mask must be a finite two-dimensional array')
    if not 0 <= threshold <= 1:
        raise ValueError('threshold must be in [0,1]')
    binary = (mask >= threshold if mask.dtype.kind == 'f' else mask > 0)
    original = binary
    if direction_filter:
        labels = longitudinal_components(longitudinal_mask(binary))
        binary = labels > 0
        contours = []
        for label in np.unique(labels[labels > 0]):
            found, _ = cv2.findContours((labels == label).astype(np.uint8),
                                       cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            contours.extend(found)
    else:
        contours, _ = cv2.findContours(binary.astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
    removed, _ = cv2.findContours((original & ~binary).astype(np.uint8),
                                  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pieces, rejected = [], list(removed)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        # Perspective lanes near the image edge can be strongly diagonal.
        if h < min_height or h < 0.2 * w:
            rejected.append(contour)
            continue
        local = np.zeros((h, w), np.uint8)
        shifted = contour - np.array([[[x, y]]])
        cv2.drawContours(local, [shifted], -1, 1, cv2.FILLED)
        # Intersect with the original mask so holes remain unobserved.
        local &= binary[y:y+h, x:x+w].astype(np.uint8)
        rows, centers, widths = [], [], []
        for row in range(h):
            columns = np.flatnonzero(local[row])
            if len(columns):
                rows.append(y + row)
                centers.append(x + float(np.median(columns)))
                widths.append(float(columns[-1] - columns[0] + 1))
        skeleton = thin_mask(local)
        sy,sx = np.nonzero(skeleton)
        # Preserve all skeleton pixels (including horizontal pixel steps).
        # Association continues using the previous geometry, so extracting
        # centers cannot silently change which lane pieces belong together.
        skeleton_curve = np.column_stack((sx+x,sy+y)).astype(float)
        pieces.append(LanePiece(contour, np.asarray(rows),
                                np.asarray(centers), np.asarray(widths),
                                skeleton_curve))
    pieces.sort(key=lambda p: (p.rows[0], p.centers[0]))
    groups,links = connect_lane_pieces(pieces,mask.shape,max_gap_fraction,
                                       ambiguity_margin)
    curves = lane_group_curves(pieces,groups)
    skeleton_mask = np.zeros(mask.shape,np.uint8)
    for piece in pieces:
        xy = piece.skeleton_curve.astype(int)
        skeleton_mask[xy[:,1],xy[:,0]] = 1
    return LaneContourResult(pieces,groups,links,rejected,
                             binary.astype(np.uint8),curves,skeleton_mask,
                             [observed_curve_segments(curve,binary) for curve in curves])
