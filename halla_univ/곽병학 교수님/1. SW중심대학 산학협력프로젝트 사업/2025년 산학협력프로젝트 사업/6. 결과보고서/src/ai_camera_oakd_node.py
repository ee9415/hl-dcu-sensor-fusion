import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import depthai as dai
import cv2
import numpy as np

# COCO 클래스 ID 중 'traffic light'에 해당하는 값
#  - COCO 80 classes 기준: traffic light = 9
COCO_TRAFFIC_LIGHT_ID = 9


class AiCameraOAKD(Node):
    """
    OAK-D 카메라를 사용해 YOLO(v5 계열) 추론으로 신호등(traffic light) 존재 여부를 판별하고,
    ROS2 토픽으로 상태("DETECTED"/"NONE")를 주기적으로 퍼블리시하는 노드입니다.

    핵심 포인트
    1) DepthAI 파이프라인으로 ColorCamera(preview=640x352) → YoloDetectionNetwork 연결
    2) 호스트(OpenCV) 측으로 원본 프리뷰 프레임과 detection 결과를 각각 XLinkOut으로 송출
    3) detection 결과 중 label=9(traffic light)가 conf_thresh 이상이면 DETECTED 판단
    4) 시각화 옵션(show_window=True)이면 바운딩 박스/상태를 오버레이해 윈도우 출력
    """

    def __init__(self):
        # ROS2 노드 이름과 네임스페이스를 지정
        super().__init__('ai_camera_oakd_node', namespace='smart_vehicle')

        # -----------------------------
        # (1) ROS2 파라미터 선언 및 로딩
        # -----------------------------
        # blob_path: OAK-D 장치에서 실행할 OpenVINO blob 경로
        #   - 주의: 모델 입력 해상도(여기선 640x352)와 아래 카메라 preview 크기가 반드시 일치해야 합니다.
        self.declare_parameter('blob_path', '/home/qsentech/models/yolov5n_coco_640x352_openvino_2021.4_6shave.blob')
        # conf_thresh: detection confidence 임계값
        self.declare_parameter('conf_thresh', 0.5)
        # iou_thresh: NMS에 사용될 IoU 임계값
        self.declare_parameter('iou_thresh', 0.45)
        # cam_fps: 카메라 프리뷰 FPS (NN 입력도 같은 레이트로 들어감)
        self.declare_parameter('cam_fps', 8.0)
        # publish_hz: 상태 메시지 퍼블리시 빈도(스로틀용)
        self.declare_parameter('publish_hz', 4.0)
        # show_window: OpenCV 윈도우로 시각화할지 여부
        self.declare_parameter('show_window', True)

        # 퍼블리셔 준비 (문자열로 간단히 상태만 송신)
        # 전체 토픽명은 /smart_vehicle/vision/traffic_light_state 가 됩니다.
        self.pub = self.create_publisher(String, 'vision/traffic_light_state', 10)

        # 파라미터 값 실제 반영
        blob_path = self.get_parameter('blob_path').value
        self.conf = float(self.get_parameter('conf_thresh').value)
        iou = float(self.get_parameter('iou_thresh').value)
        fps = float(self.get_parameter('cam_fps').value)
        pub_hz = float(self.get_parameter('publish_hz').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        # 퍼블리시 스로틀(최소 간격) 계산
        #  - 예: publish_hz=4 → 최소 0.25초마다 1회 퍼블리시
        self.min_pub_period = 1.0 / max(0.1, pub_hz)
        self.last_pub = self.get_clock().now()

        # -----------------------------
        # (2) YOLO 앵커/마스크 설정 (입력: 640x352)
        # -----------------------------
        # ⚠️ 중요: DepthAI의 YoloDetectionNetwork는 "앵커 기반(anchor-based) YOLO"에 맞는 설정이 필요합니다.
        #          (YOLOv5/YOLOv3 등의 DetectionNetwork 호환 blob)
        #
        # 아래 anchors_640은 '640x640' 학습 기준의 YOLOv5 기본 앵커입니다.
        # 이 코드는 네트워크 입력을 640x352로 사용하므로, 세로(anchor height)만 352/640 비율로 스케일합니다.
        # → 이는 '실무적 타협'으로 성능이 완벽하다고 보장되진 않습니다.
        #    가장 바람직한 방법은 모델 자체를 640x352로 훈련/변환하거나 Letterbox(원본비 유지) 전략을 따르는 것입니다.
        anchors_640 = [
            (10, 13), (16, 30), (33, 23),
            (30, 61), (62, 45), (59, 119),
            (116, 90), (156, 198), (373, 326)
        ]
        scale_h = 352.0 / 640.0  # 세로 스케일 = 0.55
        anchors_640x352 = []
        for (aw, ah) in anchors_640:
            # DepthAI API는 플랫(flat) 리스트 [w0,h0,w1,h1,...] 를 요구합니다.
            # 가로(anchor width)는 640 기준 유지, 세로만 0.55배 적용
            anchors_640x352.append(int(round(aw)))
            anchors_640x352.append(int(round(ah * scale_h)))

        # 앵커 마스크: 각 출력 피처맵(스트라이드 8/16/32)에 대응할 앵커 인덱스
        # 키 이름 규칙: "side{가로셀수}" (예: 입력 640 → stride 8이면 640/8=80 → "side80")
        #  - 여기서는 입력이 640x352 이므로 stride 8일 때 피처맵 크기는 80x44, 가로 기준 80을 사용합니다.
        anchor_masks = {
            "side80": [0, 1, 2],   # stride 8
            "side40": [3, 4, 5],   # stride 16
            "side20": [6, 7, 8],   # stride 32
        }

        # -----------------------------
        # (3) DepthAI 파이프라인 정의
        # -----------------------------
        pipeline = dai.Pipeline()

        # (3-1) 컬러 카메라 노드
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setFps(fps)                         # 카메라 프레임률
        cam.setPreviewSize(640, 352)            # NN 입력과 동일해야 함 (왜? NN에 cam.preview를 직접 연결)
        cam.setInterleaved(False)               # CHW(Planar) 포맷 (BGR 채널 분리)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)  # OpenCV 기본 색 순서(BGR)

        # (3-2) 카메라 프리뷰 프레임을 호스트(PC)로 내보내는 XLinkOut
        #       - 시각화(오버레이용)로만 사용 (추론은 장치내 NN에서 수행)
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam.preview.link(xout_rgb.input)        # 카메라 프리뷰 → 호스트

        # (3-3) YOLO DetectionNetwork 노드
        #       - anchors/anchor_masks/numClasses/conf/iou 등을 올바르게 설정해야 올바른 디텍션이 나옵니다.
        nn = pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(blob_path)               # OpenVINO blob 경로
        nn.setNumClasses(80)                    # COCO 80 classes
        nn.setCoordinateSize(4)                 # bbox 좌표 (x,y,w,h 또는 x1,y1,x2,y2)
        nn.setConfidenceThreshold(self.conf)    # confidence 임계값
        nn.setIouThreshold(iou)                 # NMS용 IoU 임계값
        nn.setAnchors(anchors_640x352)          # 플랫 리스트 형태로 전달
        nn.setAnchorMasks(anchor_masks)         # 피처맵별 앵커 인덱스 매핑
        nn.input.setBlocking(False)             # 입력 큐 블로킹 방지 (프레임 드롭 허용)

        # 카메라 프리뷰를 NN 입력으로 연결 (이 경우 cam.preview 해상도는 NN 입력과 동일해야 함)
        cam.preview.link(nn.input)

        # (3-4) NN 결과(Detection 목록)를 호스트로 내보낼 XLinkOut
        xout_det = pipeline.create(dai.node.XLinkOut)
        xout_det.setStreamName("detections")
        nn.out.link(xout_det.input)

        # -----------------------------
        # (4) 장치 오픈 및 출력 큐 핸들 획득
        # -----------------------------
        #  - Device 생성 시 USB/CMD 링크가 열리며 파이프라인이 기기 내에서 실행됩니다.
        self.device = dai.Device(pipeline)
        # detections: NN 결과(Detection 목록), rgb: 프리뷰 프레임
        #  - blocking=False: 큐가 비어있으면 즉시 None 반환(폴링 방식에 적합)
        self.q_det = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)

        self.get_logger().info("[OAKD] pipeline started (depthai raw, input=640x352, overlay=ON)")

        # -----------------------------
        # (5) 시각화/판별에 사용할 상태 변수
        # -----------------------------
        self.last_frame = None      # 가장 최근의 프리뷰 프레임(OpenCV BGR)
        self.last_dets = []         # 가장 최근의 detection 결과 리스트

        # -----------------------------
        # (6) 주기 타이머 등록
        # -----------------------------
        #  - 카메라 FPS에 맞춰 tick()을 호출해 프레임/디텍션을 갱신하고 상태를 퍼블리시
        self.timer = self.create_timer(1.0 / max(1.0, fps), self.tick)

    def _get_frame(self):
        """
        최신 RGB 프레임 한 장을 가져옵니다. (없으면 None 반환)
        DepthAI의 XLinkOut("rgb") 큐에서 한 건을 non-blocking 방식으로 수신합니다.
        """
        img_msg = self.q_rgb.tryGet()
        if img_msg is None:
            return None
        # DepthAI 이미지 메시지를 OpenCV BGR Mat으로 변환
        frame = img_msg.getCvFrame()
        # frame.shape == (352, 640, 3)
        return frame

    def tick(self):
        """
        타이머 콜백:
          1) 최신 프레임 수신/보관
          2) 최신 detection 수신/보관
          3) 신호등(traffic light) 존재 여부 판정
          4) 퍼블리시 스로틀에 따라 상태 메시지 송신
          5) (옵션) 시각화 오버레이 출력
        """
        # (1) 프레임 갱신
        frame = self._get_frame()
        if frame is not None:
            self.last_frame = frame

        # (2) 디텍션 결과 갱신
        det_in = self.q_det.tryGet()
        if det_in is not None:
            self.last_dets = det_in.detections  # 리스트[Detection], 각 Detection에 xmin,ymin,xmax,ymax,label,confidence 등 포함

        # (3) 존재 여부 판단: label=9(traffic light) + conf >= conf_thresh
        detected = any(d.label == COCO_TRAFFIC_LIGHT_ID and d.confidence >= self.conf
                       for d in self.last_dets)

        # (4) 퍼블리시 스로틀 (최소 주기 경과 시에만 전송)
        now = self.get_clock().now()
        if (now - self.last_pub).nanoseconds >= self.min_pub_period * 1e9:
            msg = String()
            msg.data = "DETECTED" if detected else "NONE"
            self.pub.publish(msg)
            self.get_logger().info(f"[oakd] traffic_light: {msg.data}")
            self.last_pub = now

        # (5) 오버레이 출력 (옵션)
        if self.show_window and self.last_frame is not None:
            vis = self.last_frame.copy()
            H, W = vis.shape[:2]   # 예상: H=352, W=640

            # 디텍션 박스 그리기
            for d in self.last_dets:
                # DepthAI Detection 좌표는 [0,1] 정규화 범위 → 픽셀 좌표로 변환
                x1 = max(0, min(W - 1, int(d.xmin * W)))
                y1 = max(0, min(H - 1, int(d.ymin * H)))
                x2 = max(0, min(W - 1, int(d.xmax * W)))
                y2 = max(0, min(H - 1, int(d.ymax * H)))

                # 신호등은 노란색, 그 외 회색
                color = (0, 255, 255) if d.label == COCO_TRAFFIC_LIGHT_ID else (200, 200, 200)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

                # 라벨/신뢰도 텍스트
                label = f"id:{d.label} conf:{d.confidence:.2f}"
                cv2.putText(vis, label, (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            # 좌상단에 전체 상태 텍스트(녹색=DETECTED, 빨간색=NONE)
            state_txt = "DETECTED" if detected else "NONE"
            cv2.putText(vis, f"TL:{state_txt}", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if detected else (0, 0, 255), 2, cv2.LINE_AA)

            # 윈도우 표시
            cv2.imshow("OAK-D Traffic Light (640x352)", vis)

            # 'q' 입력 시 창만 닫습니다(노드는 계속 동작)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                # 노드까지 종료하려면 rclpy.shutdown() 호출을 고려하세요.
                # (현재는 GUI만 닫고 추론/퍼블리시는 계속됩니다.)


def main():
    # ROS2 초기화
    rclpy.init()
    node = AiCameraOAKD()
    try:
        # 콜백(타이머 포함) 처리 루프
        rclpy.spin(node)
    finally:
        # 종료/예외 시 정리
        node.destroy_node()
        cv2.destroyAllWindows()
        # self.device는 node 소멸 시 자동 close되지만, 명시적 종료가 필요하면 device.close() 호출 가능
        rclpy.shutdown()

