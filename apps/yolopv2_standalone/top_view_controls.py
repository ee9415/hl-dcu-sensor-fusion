"""Display-only TOP perspective controls; detection geometry stays unchanged."""
from dataclasses import replace
import math


def validate_top_view(spread=0., ahead=0.):
    for value,lo,hi,name in ((spread,-30,100,'상단 벌어짐 보정'),(ahead,0,100,'앞쪽 도로 포함 범위')):
        if type(value) not in (int,float) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'{name}: {lo}~{hi} 범위의 숫자를 입력하세요')
    return float(spread),float(ahead)


def top_view_config(base, spread=0., ahead=0.):
    """Positive spread narrows the TOP far end; ahead moves source .62 -> .56.

    Extend the original road rays to the new source row before changing far
    width. Near anchors stay fixed. The result is one homography shared by
    the road image and its overlays, not a change to tracked lane coordinates.
    """
    spread,ahead=validate_top_view(spread,ahead)
    if spread == 0. and ahead == 0.:
        return base
    far=base.far_v-.06*ahead/100.
    t=(far-base.near_v)/(base.far_v-base.near_v)
    left=base.near_left_u+t*(base.far_left_u-base.near_left_u)
    right=base.near_right_u+t*(base.far_right_u-base.near_right_u)
    center=(left+right)/2.;half=(right-left)/2.*(1.+spread/100.)
    return replace(base,far_v=far,far_left_u=center-half,far_right_u=center+half)
