import depthai as dai
import numpy as np
from depth_color import colorize_depth


class DepthPreview(dai.node.HostNode):
    def __init__(self):
        super().__init__()
        self.preview_out = self.createOutput(possibleDatatypes=[
            dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)])

    def build(self, aligned_depth):
        self.link_args(aligned_depth)
        return self

    def process(self, depth):
        raw = depth.getFrame()
        # Exact 3:1 sampling of the 1920x1080 RGB-aligned frame. Do not average
        # invalid zeros into neighbouring valid depth or change the raw stream.
        if raw.shape != (1080, 1920):
            raise ValueError(f'Unexpected aligned depth shape: {raw.shape}')
        color = colorize_depth(raw[::3, ::3])
        frame = dai.ImgFrame()
        frame.setCvFrame(np.ascontiguousarray(color), dai.ImgFrame.Type.NV12)
        frame.setTimestamp(depth.getTimestamp())
        frame.setSequenceNum(depth.getSequenceNum())
        self.preview_out.send(frame)
