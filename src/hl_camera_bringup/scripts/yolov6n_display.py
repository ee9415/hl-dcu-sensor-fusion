"""
YOLOv6n live inference display on OAK-D Pro PoE.
Shows RGB frame with bounding boxes. Press 'q' to quit.
"""
import cv2
import numpy as np
import depthai as dai
from pathlib import Path

DEVICE_IP   = "192.168.200.31"
BLOB_PATH   = str(Path(__file__).parent.parent / "blobs" / "yolov6n_coco_416x416_openvino_2022.1_6shave.blob")
INPUT_W     = 416
INPUT_H     = 416
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

np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(COCO_LABELS), 3), dtype=np.uint8)

pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(INPUT_W, INPUT_H)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setFps(30)

nn = pipeline.create(dai.node.YoloDetectionNetwork)
nn.setBlobPath(BLOB_PATH)
nn.setNumClasses(len(COCO_LABELS))
nn.setCoordinateSize(4)
nn.setAnchors([])
nn.setAnchorMasks({})
nn.setIouThreshold(IOU_THRESH)
nn.setConfidenceThreshold(CONF_THRESH)
nn.setNumInferenceThreads(2)
nn.input.setBlocking(False)

cam.preview.link(nn.input)

frame_out = pipeline.create(dai.node.XLinkOut)
frame_out.setStreamName("frame")
nn.passthrough.link(frame_out.input)

det_out = pipeline.create(dai.node.XLinkOut)
det_out.setStreamName("detections")
nn.out.link(det_out.input)

device_info = dai.DeviceInfo(DEVICE_IP)

print(f"Connecting to {DEVICE_IP} ...")
with dai.Device(pipeline, device_info) as device:
    print(f"Connected: {device.getDeviceName()}  MxId: {device.getMxId()}")
    print("Press 'q' in the window to quit.\n")

    q_frame = device.getOutputQueue("frame",      maxSize=4, blocking=False)
    q_det   = device.getOutputQueue("detections", maxSize=4, blocking=False)

    WIN_NAME = "OAK-D Pro PoE - YOLOv6n"
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, INPUT_W * 2, INPUT_H * 2)

    fps_alpha = 0.1
    fps_est   = 0.0
    import time
    t_prev  = time.monotonic()
    t_start = t_prev
    AUTO_QUIT_SEC = 60
    raised = False

    while True:
        in_frame = q_frame.get()
        in_dets  = q_det.get()
        if in_frame is None or in_dets is None:
            continue

        frame = in_frame.getCvFrame()
        dets  = in_dets.detections

        t_now  = time.monotonic()
        fps_est = fps_alpha * (1.0 / max(t_now - t_prev, 1e-6)) + (1 - fps_alpha) * fps_est
        t_prev = t_now

        for d in dets:
            x1 = int(d.xmin * INPUT_W)
            y1 = int(d.ymin * INPUT_H)
            x2 = int(d.xmax * INPUT_W)
            y2 = int(d.ymax * INPUT_H)
            label = COCO_LABELS[d.label] if d.label < len(COCO_LABELS) else str(d.label)
            color = COLORS[d.label % len(COLORS)].tolist()

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            text = f"{label} {d.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 4), (x1 + tw + 2, y1), color, -1)
            cv2.putText(frame, text, (x1 + 1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, f"YOLOv6n  {fps_est:.1f} fps  dets:{len(dets)}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(WIN_NAME, frame)

        if not raised:
            print("\n" + "="*50)
            print("  창이 열렸습니다! Alt+Tab 으로 전환하세요.")
            print(f"  제목: {WIN_NAME}")
            print("  종료: 창에서 'q' 키")
            print("="*50 + "\n", flush=True)
            raised = True
        elapsed = time.monotonic() - t_start
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or elapsed >= AUTO_QUIT_SEC:
            print(f"\nExiting after {elapsed:.0f}s.")
            break

    cv2.destroyAllWindows()
