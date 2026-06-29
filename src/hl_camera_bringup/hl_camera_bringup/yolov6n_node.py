import os
import threading

import cv2
import depthai as dai
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)

COCO_LABELS = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant",
    "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


class YoloV6nNode(Node):
    def __init__(self):
        super().__init__('yolov6n_node')

        default_blob = os.path.join(
            get_package_share_directory('hl_camera_bringup'),
            'blobs', 'yolov6n_coco_416x416_openvino_2022.1_6shave.blob'
        )
        self.declare_parameter('device_ip',  '192.168.200.31')
        self.declare_parameter('blob_path', default_blob)
        self.declare_parameter('input_width',  416)
        self.declare_parameter('input_height', 416)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold',  0.5)
        self.declare_parameter('frame_id', 'oak_rgb_camera_optical_frame')

        self._device_ip    = self.get_parameter('device_ip').value
        self._blob_path    = self.get_parameter('blob_path').value
        self._input_w      = self.get_parameter('input_width').value
        self._input_h      = self.get_parameter('input_height').value
        self._conf_thresh  = self.get_parameter('conf_threshold').value
        self._iou_thresh   = self.get_parameter('iou_threshold').value
        self._frame_id     = self.get_parameter('frame_id').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._img_pub = self.create_publisher(Image, '/oak/rgb/image_raw', sensor_qos)
        self._det_pub = self.create_publisher(Detection2DArray, '/oak/detections', 10)

        self._bridge  = CvBridge()
        self._running = True
        self._thread  = threading.Thread(target=self._run_pipeline, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Connecting to OAK-D at {self._device_ip} ...')

    # ------------------------------------------------------------------
    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(self._input_w, self._input_h)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(30)

        nn = pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(self._blob_path)
        nn.setNumClasses(len(COCO_LABELS))
        nn.setCoordinateSize(4)
        nn.setAnchors([])
        nn.setAnchorMasks({})
        nn.setIouThreshold(self._iou_thresh)
        nn.setConfidenceThreshold(self._conf_thresh)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        cam.preview.link(nn.input)

        xout_frame = pipeline.create(dai.node.XLinkOut)
        xout_frame.setStreamName('frame')
        nn.passthrough.link(xout_frame.input)

        xout_det = pipeline.create(dai.node.XLinkOut)
        xout_det.setStreamName('detections')
        nn.out.link(xout_det.input)

        return pipeline

    # ------------------------------------------------------------------
    def _run_pipeline(self):
        pipeline     = self._build_pipeline()
        device_info  = dai.DeviceInfo(self._device_ip)

        with dai.Device(pipeline, device_info) as device:
            self.get_logger().info(
                f'Connected: {device.getDeviceName()}  MxId: {device.getMxId()}'
            )
            q_frame = device.getOutputQueue('frame',      maxSize=4, blocking=False)
            q_det   = device.getOutputQueue('detections', maxSize=4, blocking=False)

            while self._running and rclpy.ok():
                in_frame = q_frame.get()
                in_dets  = q_det.get()
                if in_frame is None or in_dets is None:
                    continue

                stamp = self.get_clock().now().to_msg()
                self._publish_image(in_frame.getCvFrame(), stamp)
                self._publish_detections(in_dets.detections, stamp)

        self.get_logger().info('Pipeline stopped.')

    # ------------------------------------------------------------------
    def _publish_image(self, frame, stamp):
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        self._img_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish_detections(self, detections, stamp):
        arr = Detection2DArray()
        arr.header.stamp    = stamp
        arr.header.frame_id = self._frame_id

        for d in detections:
            det = Detection2D()
            det.header = arr.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = (
                COCO_LABELS[d.label] if d.label < len(COCO_LABELS) else str(d.label)
            )
            hyp.hypothesis.score = float(d.confidence)
            det.results.append(hyp)

            cx = (d.xmin + d.xmax) / 2.0 * self._input_w
            cy = (d.ymin + d.ymax) / 2.0 * self._input_h
            sx = (d.xmax - d.xmin) * self._input_w
            sy = (d.ymax - d.ymin) * self._input_h

            bbox = BoundingBox2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.size_x = sx
            bbox.size_y = sy
            det.bbox = bbox

            arr.detections.append(det)

        self._det_pub.publish(arr)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloV6nNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
