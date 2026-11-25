import os
import json
import hashlib
import tarfile
import time
from pathlib import Path

import depthai as dai

from yolopv2_node import (
    DiagnosticFrameSource,
    Yolopv2NnProbe,
    Yolopv2Postprocess,
    Yolopv2Preprocess,
)
from stream_output import StreamOutput
from depth_preview import DepthPreview
from video_transport import VideoInputServer
from video_input_node import VideoInputSelector
from perception_export import load_config


CAMERA_SIZE = (1920, 1080)
STEREO_SIZE = (640, 400)
MODEL_CONFIG = json.loads(Path(__file__).with_name('model_config.json').read_text())
MODEL_FAMILY = os.environ.get('YOLOPV2_MODEL_FAMILY', MODEL_CONFIG['family'])
MODEL_SIZE = (int(os.environ.get('YOLOPV2_INPUT_WIDTH', MODEL_CONFIG['input_width'])),
              int(os.environ.get('YOLOPV2_INPUT_HEIGHT', MODEL_CONFIG['input_height'])))
if MODEL_FAMILY not in ('yolopv2', 'ayolom_v4'):
    raise ValueError(f'Unsupported model family: {MODEL_FAMILY}')
if MODEL_SIZE not in ((320, 320), (320, 192), (672, 384)):
    raise ValueError('Supported model sizes are 320x320, 320x192 and 672x384')
FPS = int(os.environ.get('OAK_INPUT_FPS', str(load_config().get('input_fps', 20))))
if FPS not in (10, 15, 20):
    raise ValueError('OAK_INPUT_FPS must be 10, 15 or 20')
MODEL_ARCHIVE = Path(os.environ.get(
    'YOLOPV2_MODEL_ARCHIVE',
    str(Path(__file__).parent / MODEL_CONFIG['archive']),
))
with tarfile.open(MODEL_ARCHIVE, 'r:xz') as model_archive:
    archive_config = json.load(model_archive.extractfile('config.json'))['model']
if archive_config['inputs'][0]['shape'] not in (
        [1, MODEL_SIZE[1], MODEL_SIZE[0], 3], [1, 3, MODEL_SIZE[1], MODEL_SIZE[0]]):
    raise ValueError('Configured input size does not match the model archive')
if MODEL_FAMILY == 'ayolom_v4':
    from ayolom_postprocess import TENSOR_NAMES
    if {output['name'] for output in archive_config['outputs']} != set(TENSOR_NAMES):
        raise ValueError('A-YOLOM requires both detection heads and both segmentation heads')
PIPELINE_STAGE = os.environ.get("PIPELINE_STAGE", "full").lower()
VALID_STAGES = {"camera", "depth", "manip", "nn", "static_nn", "full"}

if PIPELINE_STAGE not in VALID_STAGES:
    raise ValueError(
        f"PIPELINE_STAGE must be one of {sorted(VALID_STAGES)}, got {PIPELINE_STAGE!r}"
    )
if PIPELINE_STAGE == 'static_nn' and MODEL_SIZE != (320, 320):
    raise ValueError('static_nn diagnostic currently requires 320x320')


class LocalOnlyConnection:
    """Desktop RTSP mode: do not start a separate web visualizer by default."""
    def addTopic(self, *args, **kwargs):
        pass

    def registerPipeline(self, pipeline):
        pass

    def waitKey(self, milliseconds):
        time.sleep(milliseconds/1000)


visualizer = (dai.RemoteConnection(httpPort=8082)
              if os.environ.get('OAK_WEB_VISUALIZER') == '1' else LocalOnlyConnection())
device = dai.Device()
print(f"Device: {device.getDeviceInfo()}", flush=True)
print(f"Pipeline stage: {PIPELINE_STAGE}", flush=True)
print(f'Camera/input clock limit: {FPS} FPS', flush=True)
print(f'MODEL_CONFIG {json.dumps(dict(family=MODEL_FAMILY, input_size=MODEL_SIZE, archive=str(MODEL_ARCHIVE), archive_sha256=hashlib.sha256(MODEL_ARCHIVE.read_bytes()).hexdigest()))}', flush=True)

with dai.Pipeline(device) as pipeline:
    if PIPELINE_STAGE == "static_nn":
        print(f"Model: {MODEL_ARCHIVE}", flush=True)
        diagnostic_source = pipeline.create(DiagnosticFrameSource)
        preprocess = pipeline.create(Yolopv2Preprocess).build(
            diagnostic_source.out
        )
        archive = dai.NNArchive(MODEL_ARCHIVE)
        neural_network = pipeline.create(dai.node.NeuralNetwork).build(
            preprocess.tensor_out,
            archive,
        )
        pipeline.create(Yolopv2NnProbe).build(neural_network.out)
        visualizer.addTopic("DiagnosticInput", diagnostic_source.out)

        print("YOLOPv2 static_nn pipeline created", flush=True)
        pipeline.start()
        visualizer.registerPipeline(pipeline)
        print("YOLOPv2 static_nn pipeline running", flush=True)
        while pipeline.isRunning():
            visualizer.waitKey(100)
        raise SystemExit(0)

    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camera_out = camera.requestOutput(
        CAMERA_SIZE,
        dai.ImgFrame.Type.NV12,
        fps=FPS,
        enableUndistortion=True,
    )

    if PIPELINE_STAGE in {"depth", "full"}:
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.ROBOTICS
        )
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        left_out = left.requestOutput(STEREO_SIZE, fps=FPS)
        right_out = right.requestOutput(STEREO_SIZE, fps=FPS)
        left_out.link(stereo.left)
        right_out.link(stereo.right)

        depth_align = pipeline.create(dai.node.ImageAlign)
        stereo.depth.link(depth_align.input)
        camera_out.link(depth_align.inputAlignTo)
        aligned_depth = depth_align.outputAligned
        visualizer.addTopic("DepthAligned", aligned_depth)
        print(
            "DepthAligned: CAM_B/C stereo depth aligned to CAM_A "
            f"({CAMERA_SIZE[0]}x{CAMERA_SIZE[1]}, millimeters)",
            flush=True,
        )

    video_input = None
    if PIPELINE_STAGE == 'full':
        video_input = VideoInputServer(input_fps=FPS,
            preserve_frames=not load_config().get('camera_postprocess', True))
        source = pipeline.create(VideoInputSelector).build(camera_out, aligned_depth, video_input)
        camera_out, aligned_depth = source.video_out, source.depth_out
        print('PC video input: TCP 8558 (camera/PC source selection)', flush=True)

    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
    encoder.setKeyframeFrequency(FPS)
    encoder.setNumBFrames(0)
    camera_out.link(encoder.input)
    pipeline.create(StreamOutput).build(encoder.out, FPS, 8555, '/rgb')
    print('RGB RTSP: rtsp://<device-ip>:8555/rgb', flush=True)
    if PIPELINE_STAGE != 'full':
        visualizer.addTopic('Video', encoder.out, 'images')
    if PIPELINE_STAGE in {'depth', 'full'}:
        depth_preview = pipeline.create(DepthPreview).build(aligned_depth)
        depth_encoder = pipeline.create(dai.node.VideoEncoder)
        depth_encoder.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
        depth_encoder.setKeyframeFrequency(FPS)
        depth_encoder.setNumBFrames(0)
        depth_preview.preview_out.link(depth_encoder.input)
        pipeline.create(StreamOutput).build(depth_encoder.out, FPS, 8556, '/depth')
        print('Depth preview RTSP: rtsp://<device-ip>:8556/depth', flush=True)

    if PIPELINE_STAGE not in {"camera", "depth"}:
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setOutputSize(
            *MODEL_SIZE,
            dai.ImageManipConfig.ResizeMode.LETTERBOX,
        )
        # ImageManip produces the fixed-size frame. Yolopv2Preprocess below
        # performs BGR->RGB and 0..255->0..1 explicitly before inference.
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888i)
        if MODEL_FAMILY == 'ayolom_v4':
            manip.initialConfig.setBackgroundColor(114, 114, 114)
        manip.setMaxOutputFrameSize(MODEL_SIZE[0] * MODEL_SIZE[1] * 3)
        camera_out.link(manip.inputImage)

        if PIPELINE_STAGE == "manip":
            visualizer.addTopic("ManipRGB320", manip.out)
        else:
            print(f"Model: {MODEL_ARCHIVE}", flush=True)
            archive = dai.NNArchive(MODEL_ARCHIVE)
            preprocess = pipeline.create(Yolopv2Preprocess).build(manip.out)
            preprocess.model_size = MODEL_SIZE
            neural_network = pipeline.create(dai.node.NeuralNetwork).build(
                preprocess.tensor_out, archive
            )

            if PIPELINE_STAGE == "nn":
                probe = pipeline.create(Yolopv2NnProbe).build(neural_network.out)
                probe.model_size = MODEL_SIZE
                probe.model_family = MODEL_FAMILY
            else:
                postprocess = pipeline.create(Yolopv2Postprocess).build(
                    camera_out,
                    neural_network.out,
                    aligned_depth,
                )
                postprocess.set_source_size(*CAMERA_SIZE)
                postprocess.model_size = MODEL_SIZE
                postprocess.model_family = MODEL_FAMILY
                postprocess.video_input = video_input
                visualizer.addTopic(
                    "SpatialDetections",
                    postprocess.spatial_detections,
                )
                result_encoder = pipeline.create(dai.node.VideoEncoder)
                result_encoder.setDefaultProfilePreset(
                    FPS,
                    dai.VideoEncoderProperties.Profile.H264_MAIN,
                )
                postprocess.result.link(result_encoder.input)
                result_encoder.setKeyframeFrequency(FPS)
                result_encoder.setNumBFrames(0)
                visualizer.addTopic("YOLOPv2Result", result_encoder.out, "results")
                pipeline.create(StreamOutput).build(
                    result_encoder.out,
                    FPS,
                    8554,
                    "/yolopv2",
                )
                print(
                    "YOLOPv2 RTSP: rtsp://<device-ip>:8554/yolopv2",
                    flush=True,
                )

                bev_encoder = pipeline.create(dai.node.VideoEncoder)
                bev_encoder.setDefaultProfilePreset(
                    FPS,
                    dai.VideoEncoderProperties.Profile.H264_MAIN,
                )
                postprocess.bev.link(bev_encoder.input)
                bev_encoder.setKeyframeFrequency(FPS)
                bev_encoder.setNumBFrames(0)
                visualizer.addTopic("CenterlineBEV", bev_encoder.out, "bev")
                pipeline.create(StreamOutput).build(
                    bev_encoder.out,
                    FPS,
                    8553,
                    "/bev",
                )
                print(
                    "Virtual boundaries BEV RTSP: rtsp://<device-ip>:8553/bev",
                    flush=True,
                )

    print(f"YOLOPv2 {PIPELINE_STAGE} pipeline created", flush=True)
    pipeline.start()
    visualizer.registerPipeline(pipeline)
    print(f"YOLOPv2 {PIPELINE_STAGE} pipeline running", flush=True)

    while pipeline.isRunning():
        pipeline.processTasks()
        visualizer.waitKey(100)
