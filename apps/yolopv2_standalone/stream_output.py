import depthai as dai

from rtsp_server import RTSPServer


class StreamOutput(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()

    def build(
        self,
        stream: dai.Node.Output,
        fps: int,
        port: int,
        mount_path: str,
    ) -> "StreamOutput":
        self._server = RTSPServer(fps, port, mount_path)
        self.link_args(stream)
        self.sendProcessingToPipeline(True)
        return self

    def process(self, stream: dai.EncodedFrame) -> None:
        self._server.send_data(bytes(stream.getData()))
