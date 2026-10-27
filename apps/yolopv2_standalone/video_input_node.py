"""Select live camera or PC frames before RGB encoding and NN preprocessing."""
import depthai as dai
import numpy as np


class VideoInputSelector(dai.node.HostNode):
    def __init__(self):
        super().__init__()
        kinds = [dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)]
        self.video_out = self.createOutput(possibleDatatypes=kinds)
        self.depth_out = self.createOutput(possibleDatatypes=kinds)
        self.hub = None
        self.zero_depth = np.zeros((1080, 1920), np.uint16)

    def build(self, camera, aligned_depth, hub):
        self.hub = hub
        self.link_args(camera, aligned_depth)
        return self

    def process(self, camera: dai.ImgFrame, aligned_depth: dai.ImgFrame):
        payload, metadata = self.hub.take()
        if metadata is None:
            return
        frame, depth = camera, aligned_depth
        if metadata['input_source'] == 'pc':
            import cv2
            bgr = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            bgr = cv2.resize(bgr, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            frame = dai.ImgFrame()
            frame.setCvFrame(np.ascontiguousarray(bgr), dai.ImgFrame.Type.NV12)
            frame.setTimestamp(camera.getTimestamp())
            frame.setSequenceNum(camera.getSequenceNum())
            depth = dai.ImgFrame()
            depth.setFrame(self.zero_depth)
            depth.setType(dai.ImgFrame.Type.RAW16)
            depth.setWidth(1920)
            depth.setHeight(1080)
            depth.setTimestamp(frame.getTimestamp())
            depth.setSequenceNum(frame.getSequenceNum())
        self.hub.remember(frame.getSequenceNum(), metadata)
        self.video_out.send(frame)
        self.depth_out.send(depth)

    def onStop(self):
        if self.hub is not None:
            self.hub.close()
