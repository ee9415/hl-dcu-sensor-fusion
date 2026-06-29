"""
YOLOv6n inference test on OAK-D Pro PoE (Myriad X VPU).
Runs 100 frames, prints detection count and FPS, then exits.
No display — terminal only.
"""
import time
from pathlib import Path
import depthai as dai

DEVICE_IP   = "192.168.200.31"
BLOB_PATH   = str(Path(__file__).parent.parent / "blobs" / "yolov6n_coco_416x416_openvino_2022.1_6shave.blob")
INPUT_W     = 416
INPUT_H     = 416
NUM_CLASSES = 80
NUM_FRAMES  = 100
CONF_THRESH = 0.5
IOU_THRESH  = 0.5

COCO_LABELS = [
    "person","bicycle","car","motorbike","aeroplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet",
    "tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave",
    "oven","toaster","sink","refrigerator","book","clock","vase","scissors",
    "teddy bear","hair drier","toothbrush",
]

pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(INPUT_W, INPUT_H)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setFps(30)

nn = pipeline.create(dai.node.YoloDetectionNetwork)
nn.setBlobPath(BLOB_PATH)
nn.setNumClasses(NUM_CLASSES)
nn.setCoordinateSize(4)
nn.setAnchors([])
nn.setAnchorMasks({})
nn.setIouThreshold(IOU_THRESH)
nn.setConfidenceThreshold(CONF_THRESH)
nn.setNumInferenceThreads(2)
nn.input.setBlocking(False)

cam.preview.link(nn.input)

det_out = pipeline.create(dai.node.XLinkOut)
det_out.setStreamName("detections")
nn.out.link(det_out.input)

device_info = dai.DeviceInfo(DEVICE_IP)

print(f"Connecting to {DEVICE_IP} ...")
with dai.Device(pipeline, device_info) as device:
    print(f"Connected. Device: {device.getDeviceName()}, MxId: {device.getMxId()}")
    q = device.getOutputQueue("detections", maxSize=4, blocking=False)

    frame_count = 0
    t_start = time.monotonic()

    print(f"Running {NUM_FRAMES} frames ...\n")
    while frame_count < NUM_FRAMES:
        dets = q.get()
        if dets is None:
            continue

        frame_count += 1
        detections = dets.detections
        elapsed = time.monotonic() - t_start
        fps = frame_count / elapsed if elapsed > 0 else 0

        if detections:
            labels = [COCO_LABELS[d.label] if d.label < len(COCO_LABELS) else str(d.label)
                      for d in detections]
            summary = ", ".join(f"{l}({d.confidence:.2f})" for l, d in zip(labels, detections))
        else:
            summary = "(none)"

        print(f"[{frame_count:>3}/{NUM_FRAMES}] fps={fps:.1f}  dets={len(detections)}  {summary}")

    elapsed = time.monotonic() - t_start
    print(f"\nDone. {NUM_FRAMES} frames in {elapsed:.1f}s  avg fps={NUM_FRAMES/elapsed:.1f}")
