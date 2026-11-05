"""RGB lane-score previews only. Never supplies a mask to perception/tracking."""
import math
import numpy as np


def contour_skeleton_layer(result,show_skeleton=False):
    """Only measured outlines and true thinning pixels; no inferred segments."""
    import cv2
    layer=np.zeros((*result.directional_mask.shape,3),np.uint8)
    for piece in result.pieces:
        cv2.drawContours(layer,[piece.contour],-1,(0,255,255),1)
    if show_skeleton:
        layer[result.skeleton_mask>0]=(255,255,0)
    return layer


def overlay_contour_skeleton(frame,result):
    """Scale measured source pixels onto RGB without fitted center curves."""
    import cv2
    layer=contour_skeleton_layer(result)
    scaled=cv2.resize(layer,(frame.shape[1],frame.shape[0]),interpolation=cv2.INTER_NEAREST)
    selected=scaled.any(axis=2)
    frame[selected]=scaled[selected]
    return frame


def overlay_lane_quadratics(frame,selected,mask_shape,projection=None,valid=None):
    """Magenta polynomial fits, distinct from measured cyan skeleton pixels.

    Observed fits are solid magenta; explicitly virtual fits are dashed yellow.
    A TOP homography maps canvas to mask; use its inverse for these vectors.
    """
    import cv2
    if selected is None:
        return frame
    h,w=frame.shape[:2]
    matrix=(np.diag([w/mask_shape[1],h/mask_shape[0],1.]) if projection is None
            else np.linalg.inv(projection))
    fits=[]
    for boundary in (selected.left,selected.right):
        if boundary is not None:
            fits.append(boundary)
            if getattr(boundary,'extension',None) is not None:
                fits.append(boundary.extension)
    for fit in fits:
        if fit is None:
            continue
        dash_distance=0.
        rows=np.arange(fit.points[0,1],fit.points[-1,1]+.01,.25)
        source=np.column_stack((np.polyval(fit.coefficients,rows),rows,np.ones(len(rows))))
        mapped=source@matrix.T
        denominator=mapped[:,2]
        good=np.isfinite(mapped).all(axis=1)&(abs(denominator)>1e-9)
        xy=mapped[:,:2]/np.where(good,denominator,1)[:,None]
        good &= (abs(xy)<1e7).all(axis=1)
        for i in range(len(xy)-1):
            if not (good[i] and good[i+1]) or denominator[i]*denominator[i+1]<=0:
                continue
            a,b=np.rint(xy[i:i+2]).astype(int)
            visible,start,end=cv2.clipLine((0,0,w,h),tuple(a),tuple(b))
            if visible:
                if not fit.is_virtual:
                    cv2.line(frame,start,end,(255,0,255),3)
                else:
                    start=np.asarray(start,dtype=float)
                    end=np.asarray(end,dtype=float)
                    length=float(np.linalg.norm(end-start))
                    if length>0:
                        for distance in np.arange(0,length+.01,.75):
                            if int((dash_distance+distance)//10)%2==0:
                                point=np.rint(start+(end-start)*(distance/length)).astype(int)
                                cv2.circle(frame,tuple(point),1,(0,255,255),-1)
                        dash_distance+=length
    if valid is not None:
        frame[~valid]=0
    return frame


def overlay_tracked_skeleton(frame, estimate, mask_shape, projection=None, valid=None):
    """Draw the retained tracker samples directly, without a quadratic refit.

    Current supported skeleton segments are magenta; remembered/deformed ones
    are dashed yellow. The same 320x180 samples are exported in telemetry.
    """
    import cv2
    h,w = frame.shape[:2]
    matrix = (np.diag([w/320.,h/180.,1.]) if projection is None else
              np.linalg.inv(projection) @ np.diag([mask_shape[1]/320.,mask_shape[0]/180.,1.]))
    for side in ('left','right'):
        points = getattr(estimate,'image_'+side)
        if len(points) < 2:
            continue
        mapped = np.column_stack((points,np.ones(len(points)))) @ matrix.T
        denominator = mapped[:,2]
        good = np.isfinite(mapped).all(axis=1) & (abs(denominator)>1e-9)
        xy = mapped[:,:2]/np.where(good,denominator,1.)[:,None]
        good &= (abs(xy)<1e7).all(axis=1)
        support = getattr(estimate,side+'_support')
        supported = np.zeros(len(points),bool) if support is None else np.asarray(support,bool)
        measured = getattr(estimate,side+'_state') in ('observed','partial','aligning')
        distance = 0.
        for i in range(len(points)-1):
            if not (good[i] and good[i+1]) or denominator[i]*denominator[i+1] <= 0:
                continue
            a,b = np.rint(xy[i:i+2]).astype(int)
            visible,start,end = cv2.clipLine((0,0,w,h),tuple(a),tuple(b))
            if not visible:
                continue
            if measured and supported[i] and supported[i+1]:
                cv2.line(frame,start,end,(255,0,255),3)
                continue
            start,end = np.asarray(start,float),np.asarray(end,float)
            length = float(np.linalg.norm(end-start))
            if length <= 0:
                continue
            for offset in np.arange(0.,length+.01,.75):
                if int((distance+offset)//10)%2 == 0:
                    point = np.rint(start+(end-start)*min(1.,offset/length)).astype(int)
                    cv2.circle(frame,tuple(point),1,(0,255,255),-1)
            distance += length
    if valid is not None:
        frame[~valid] = 0
    return frame


def overlay_pixel_confidence(rgb, scores, threshold):
    """RGB preview only: absolute per-pixel score colors, low red to high violet."""
    h, w = scores.shape
    sy, sx = rgb.shape[0]//h, rgb.shape[1]//w
    if rgb.shape[:2] != (h*sy, w*sx):
        raise ValueError('RGB must be an integer multiple of score dimensions')
    sampled = np.repeat(np.repeat(scores, sy, axis=0), sx, axis=1)
    selected = sampled >= threshold
    frame = rgb.copy()
    colors = lane_score_colors(1-sampled[selected])
    frame[selected] = np.rint(.2*frame[selected]+.8*colors).astype(np.uint8)
    return frame


class TopViewRenderer:
    """Cached inverse perspective sampling for display, never metric geometry."""

    def __init__(self, near_row=479, horizontal_scale=3.75):
        if not 1 <= near_row <= 479:
            raise ValueError('TOP near row must be in [1,479]')
        self.near_row = near_row
        if not np.isfinite(horizontal_scale) or not 0 < horizontal_scale <= 20:
            raise ValueError('TOP horizontal scale must be in (0,20]')
        self.horizontal_scale = float(horizontal_scale)
        self.key = None

    def render(self, rgb, lane, projection, mode='normal', threshold=.5, scores=None):
        # Existing projection maps BEV -> native mask. The horizontal display
        # scale controls zoom; lower values show more road on both sides.
        scale=self.horizontal_scale
        expand = np.array([[1/scale, 0, 319.5*(1-1/scale)],
                           [0, 479/self.near_row, 0], [0, 0, 1.]])
        self.projection = np.asarray(projection) @ expand
        key = (rgb.shape, lane.shape, self.projection.tobytes())
        if key != self.key:
            yy, xx = np.indices((480, 640))
            mapped = self.projection @ np.stack((xx.ravel(), yy.ravel(), np.ones(xx.size)))
            denominator = mapped[2]
            finite = np.isfinite(mapped).all(axis=0) & (np.abs(denominator) > 1e-9)
            uv = mapped[:2] / np.where(finite, denominator, 1)
            uv[:, ~finite] = -1
            h, w = lane.shape
            self.valid = (finite & (uv[0] >= 0) & (uv[0] <= w-1) &
                          (uv[1] >= 0) & (uv[1] <= h-1)).reshape(480, 640)
            self.valid &= yy <= self.near_row
            self.mx = np.clip(np.rint(uv[0]), 0, w-1).astype(int).reshape(480, 640)
            self.my = np.clip(np.rint(uv[1]), 0, h-1).astype(int).reshape(480, 640)
            self.rx = np.clip(np.rint(uv[0]*rgb.shape[1]/w), 0, rgb.shape[1]-1).astype(int).reshape(480, 640)
            self.ry = np.clip(np.rint(uv[1]*rgb.shape[0]/h), 0, rgb.shape[0]-1).astype(int).reshape(480, 640)
            self.key = key
        frame = rgb[self.ry, self.rx].copy()
        frame[~self.valid] = 0
        if mode == 'raw_rgb':
            return frame
        if mode == 'normal' and scores is None:
            return frame  # Never imply pixel confidence using a binary mask.
        if scores is None:
            selected = (lane[self.my, self.mx] != 0) & self.valid
            colors = np.array([255, 255, 0])
        else:
            sampled = scores[self.my, self.mx]
            selected = (sampled >= effective_preview_threshold(mode, threshold)) & self.valid
            colors = (lane_score_colors(1-sampled[selected]) if mode == 'normal' else
                      reference_band_colors(sampled[selected]) if mode in BAND_MODES else
                      lane_score_colors(sampled[selected]) if mode == 'lane_score' else
                      np.array([255, 255, 0]))
        if mode == 'lane_bands' and scores is not None:
            frame[:] = 0
            frame[selected] = colors
        else:
            frame[selected] = np.rint(.2*frame[selected]+.8*colors).astype(np.uint8)
        return frame

    def overlay_lane_groups(self, frame, result):
        """Use the RGB inverse map for measured yellow outlines/cyan skeleton.

        Pixel sampling preserves mask membership and never joins components.
        """
        import cv2
        if self.key is None:
            raise ValueError('render TOP before drawing lane groups')
        if result.directional_mask.shape != (self.key[1]):
            raise ValueError('group mask must match TOP source mask dimensions')
        if frame.shape != (480,640,3):
            raise ValueError('expected 480x640 TOP frame')
        layer=contour_skeleton_layer(result)
        sampled=layer[self.my,self.mx]
        selected=sampled.any(axis=2)&self.valid
        frame[selected]=sampled[selected]
        frame[~self.valid]=0
        return frame

MODES=('normal','lane_score','lane_mask','raw_rgb','lane_bands','lane_bands_rgb','lane_compare')
BAND_MODES=('lane_bands','lane_bands_rgb','lane_compare')
BAND_LEVELS=np.array([.05,.15,.25,.35,.45,.55,.65,.75,.85,.95],dtype=np.float32)
# Reference-image convention: low red -> high violet. Index 0 is <.05.
BAND_COLORS=np.array([(0,0,0),(0,0,255),(0,128,255),(0,255,255),
    (0,255,128),(0,255,0),(128,255,0),(255,255,0),(255,128,0),
    (255,0,0),(255,0,180)],dtype=np.uint8)
RAINBOW_EDGES=(0.,.20,.35,.50,.65,.80,.90,1.)
RAINBOW_ANCHORS=(0.,.20,.35,.50,.65,.80,1.)
# BGR: violet, blue, cyan, green, yellow, orange, red.
RAINBOW_COLORS=np.array([(211,0,148),(255,0,0),(255,255,0),(0,200,0),
                         (0,255,255),(0,165,255),(0,0,255)],dtype=np.float32)


def lane_score_colors(scores):
    """Per-pixel continuous rainbow on an absolute 0..1 score scale.

    No per-lane average and no per-frame normalization: nearby but different
    scores can differ within the formerly single-colored score bands too.
    """
    values=np.asarray(scores,dtype=np.float32)
    anchors=np.array(RAINBOW_ANCHORS,dtype=np.float32)
    return np.stack([np.interp(values,anchors,RAINBOW_COLORS[:,channel])
                     for channel in range(3)],axis=-1).astype(np.float32)


def reference_band_colors(scores):
    indices=np.searchsorted(BAND_LEVELS,np.asarray(scores,dtype=np.float32),side='right')
    return BAND_COLORS[indices]


def effective_preview_threshold(mode,threshold):
    return max(.05,threshold) if mode in BAND_MODES else threshold


def validate_preview(mode,threshold):
    if mode not in MODES:
        raise ValueError('Unknown lane preview mode')
    if type(threshold) not in (int,float) or not math.isfinite(threshold) or not 0<=threshold<=1:
        raise ValueError('Preview threshold must be in [0,1]')


def crop_lane_scores(tensor,content_shape,model_size):
    """Remove letterbox from either NCHW or NHWC without modifying NN output."""
    raw=np.asarray(tensor)
    if raw.shape==(1,1,model_size[1],model_size[0]):
        score=raw[0,0]
    elif raw.shape==(1,model_size[1],model_size[0],1):
        score=raw[0,:,:,0]
    else:
        raise ValueError(f'Unexpected lane tensor {raw.shape}')
    h,w=content_shape
    top,left=(score.shape[0]-h)//2,(score.shape[1]-w)//2
    if min(top,left)<0:
        raise ValueError('Lane content exceeds model size')
    score=score[top:top+h,left:left+w]
    if not np.isfinite(score).all():
        raise ValueError('Lane scores must be finite')
    # The currently deployed segmentation contract uses a 0..1 lane score.
    # Do not silently reinterpret logits as probabilities.
    if score.min() < -.01 or score.max() > 1.01:
        raise ValueError('Lane output is not in the expected score range')
    return np.clip(score,0,1)


def render_lane_overlay(bgr,scores,threshold,mode):
    """Color selected lane-mask pixels only; preserve every background pixel."""
    validate_preview(mode,threshold)
    if mode in ('normal','raw_rgb'):
        return bgr.copy()
    if mode=='lane_compare':
        if bgr.shape[0]%3 or bgr.shape[1]%3:
            raise ValueError('Comparison view requires dimensions divisible by three')
        raw=bgr[::3,::3]
        bands=render_lane_overlay(bgr,scores,threshold,'lane_bands')[::3,::3]
        overlay=render_lane_overlay(bgr,scores,threshold,'lane_bands_rgb')[::3,::3]
        result=np.zeros_like(bgr)
        top=(bgr.shape[0]-raw.shape[0])//2
        result[top:top+raw.shape[0]]=np.hstack((raw,bands,overlay))
        return result
    h,w=bgr.shape[:2]
    sh,sw=scores.shape
    if h%sh or w%sw:
        raise ValueError('Preview size must be integer multiples of score map')
    # Work at mask resolution first, then expand. RGB and scores belong to the
    # same callback/frame; there is no independent-stream alignment guess.
    score=np.asarray(scores,dtype=np.float32)
    selected=score>=effective_preview_threshold(mode,threshold)
    if mode in BAND_MODES:
        color=reference_band_colors(score).astype(np.float32)
        alpha=selected.astype(np.float32)*(1. if mode=='lane_bands' else .85)
        if mode=='lane_bands':
            bgr=np.zeros_like(bgr)
    elif mode=='lane_mask':
        color=np.empty((sh,sw,3),np.float32);color[:]=(255,255,0)
        alpha=selected.astype(np.float32)*.7
    else:
        # Use each native model pixel's own score, before nearest expansion.
        color=lane_score_colors(score)
        # The threshold selects the mask. Non-mask RGB is bit-identical.
        # Fixed opacity makes color, rather than opacity, encode the score.
        alpha=selected.astype(np.float32)*.8
    scale=(h//sh,w//sw)
    up_color=np.repeat(np.repeat(color,scale[0],axis=0),scale[1],axis=1)
    up_alpha=np.repeat(np.repeat(alpha,scale[0],axis=0),scale[1],axis=1)[:,:,None]
    return np.clip(bgr.astype(np.float32)*(1-up_alpha)+up_color*up_alpha,0,255).astype(np.uint8)
