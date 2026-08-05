import os
import threading

import cv2
import depthai as dai
import numpy as np
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

# YOLOP (https://github.com/hustvl/YOLOP) detects a single class.
DETECTION_LABEL = 'vehicle'


class YoloPNode(Node):
    """Panoptic driving perception (vehicle detection + drivable-area /
    lane-line segmentation) using YOLOP on the OAK-D VPU.

    Runs the model as a generic dai.node.NeuralNetwork (not
    YoloDetectionNetwork) because YOLOP has three output heads
    (det_out, drive_area_seg, lane_line_seg) that the VPU's built-in
    YOLO decoder does not support; decoding/NMS for det_out is done
    on the host.
    """

    def __init__(self):
        super().__init__('yolop_node')

        default_blob = os.path.join(
            get_package_share_directory('hl_camera_bringup'),
            'blobs', 'yolop_bdd100k_320x320_openvino_2022.1_6shave.blob'
        )
        self.declare_parameter('device_ip',  '192.168.200.31')
        self.declare_parameter('blob_path', default_blob)
        self.declare_parameter('input_width',  320)
        self.declare_parameter('input_height', 320)
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('iou_threshold',  0.45)
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
        self._img_pub  = self.create_publisher(Image, '/oak/rgb/image_raw', sensor_qos)
        self._det_pub  = self.create_publisher(Detection2DArray, '/oak/detections', 10)
        self._area_pub = self.create_publisher(Image, '/oak/drivable_area', sensor_qos)
        self._lane_pub = self.create_publisher(Image, '/oak/lane_line', sensor_qos)

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

        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(self._blob_path)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        cam.preview.link(nn.input)

        xout_frame = pipeline.create(dai.node.XLinkOut)
        xout_frame.setStreamName('frame')
        nn.passthrough.link(xout_frame.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName('nn')
        nn.out.link(xout_nn.input)

        return pipeline

    # ------------------------------------------------------------------
    def _run_pipeline(self):
        pipeline     = self._build_pipeline()
        device_info  = dai.DeviceInfo(self._device_ip)

        with dai.Device(pipeline, device_info) as device:
            self.get_logger().info(
                f'Connected: {device.getDeviceName()}  MxId: {device.getMxId()}'
            )
            q_frame = device.getOutputQueue('frame', maxSize=4, blocking=False)
            q_nn    = device.getOutputQueue('nn',    maxSize=4, blocking=False)

            while self._running and rclpy.ok():
                in_frame = q_frame.get()
                in_nn    = q_nn.get()
                if in_frame is None or in_nn is None:
                    continue

                stamp = self.get_clock().now().to_msg()
                self._publish_image(in_frame.getCvFrame(), stamp)

                dets, drivable_mask, lane_mask = self._decode(in_nn)
                self._publish_detections(dets, stamp)
                self._publish_mask(self._area_pub, drivable_mask, stamp)
                self._publish_mask(self._lane_pub, lane_mask, stamp)

        self.get_logger().info('Pipeline stopped.')

    # ------------------------------------------------------------------
    def _decode(self, in_nn):
        det_out = np.array(in_nn.getLayerFp16('det_out')).reshape(6300, 6)
        da_seg  = np.array(in_nn.getLayerFp16('drive_area_seg')).reshape(2, self._input_h, self._input_w)
        ll_seg  = np.array(in_nn.getLayerFp16('lane_line_seg')).reshape(2, self._input_h, self._input_w)

        dets = self._decode_detections(det_out)
        drivable_mask = np.argmax(da_seg, axis=0).astype(np.uint8)
        lane_mask     = np.argmax(ll_seg, axis=0).astype(np.uint8)
        return dets, drivable_mask, lane_mask

    # ------------------------------------------------------------------
    def _decode_detections(self, preds):
        # preds columns: cx, cy, w, h, obj_conf, cls_conf (single class)
        scores = preds[:, 4] * preds[:, 5]
        keep = scores > self._conf_thresh
        preds  = preds[keep]
        scores = scores[keep]
        if preds.shape[0] == 0:
            return []

        cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        boxes = np.stack([x1, y1, w, h], axis=1).tolist()

        idxs = cv2.dnn.NMSBoxes(boxes, scores.tolist(), self._conf_thresh, self._iou_thresh)

        dets = []
        for i in np.array(idxs).flatten():
            bx, by, bw, bh = boxes[i]
            dets.append((bx + bw / 2.0, by + bh / 2.0, bw, bh, float(scores[i])))
        return dets

    # ------------------------------------------------------------------
    def _publish_image(self, frame, stamp):
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        self._img_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish_detections(self, dets, stamp):
        arr = Detection2DArray()
        arr.header.stamp    = stamp
        arr.header.frame_id = self._frame_id

        for cx, cy, w, h, conf in dets:
            det = Detection2D()
            det.header = arr.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = DETECTION_LABEL
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.size_x = w
            bbox.size_y = h
            det.bbox = bbox

            arr.detections.append(det)

        self._det_pub.publish(arr)

    # ------------------------------------------------------------------
    def _publish_mask(self, publisher, mask, stamp):
        msg = self._bridge.cv2_to_imgmsg(mask * 255, encoding='mono8')
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        publisher.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
