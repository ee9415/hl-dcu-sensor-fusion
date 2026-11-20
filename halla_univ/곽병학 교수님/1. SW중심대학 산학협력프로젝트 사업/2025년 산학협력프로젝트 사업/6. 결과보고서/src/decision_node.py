# smart_vehicle/decision_node.py
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ================================
# Decision 노드
# - 신호등 상태(RED/GREEN/YELLOW)를 받아서
# - 차량 제어 명령(BRAKE/GO/SLOW)으로 변환해서 퍼블리시
# ================================
class Decision(Node):
    def __init__(self):
        # 노드 이름은 'decision_node', 네임스페이스는 'smart_vehicle'
        super().__init__('decision_node', namespace='smart_vehicle')

        # Subscriber: 신호등 상태 토픽 구독
        #   - 토픽명: /smart_vehicle/vision/traffic_light_state
        #   - 메시지 타입: std_msgs/String
        #   - 콜백 함수: self.cb
        self.sub = self.create_subscription(String, 'vision/traffic_light_state', self.cb, 10)

        # Publisher: 차량 제어 명령 퍼블리시
        #   - 토픽명: /smart_vehicle/can/vehicle_command
        #   - 메시지 타입: std_msgs/String
        self.pub = self.create_publisher(String, 'can/control_cmd', 10)

    # 콜백 함수: 신호등 상태 메시지를 받으면 실행됨
    def cb(self, msg: String):
        cmd = String()  # 송신할 메시지 준비
        if msg.data == "RED":
            cmd.data = "BRAKE"   # 신호등 RED → 브레이크 명령
        elif msg.data == "GREEN":
            cmd.data = "GO"      # 신호등 GREEN → 진행 명령
        elif msg.data == "DETECTED":
            cmd.data = "BRAKE"   # 신호등 보이면 정지
        else:
            cmd.data = "SLOW"    # 나머지(YELLOW/UNKNOWN 등) → 감속 명령

        # 퍼블리시 (차량 명령을 /can/vehicle_command 로 전송)
        self.pub.publish(cmd)

        # 로그 출력 (디버깅용)
        self.get_logger().info(f"[decision] {msg.data} -> {cmd.data}")

# main 함수: 노드를 실행하는 엔트리 포인트
def main():
    rclpy.init()             # ROS2 초기화
    rclpy.spin(Decision())   # Decision 노드를 실행 (콜백 루프 대기)
    rclpy.shutdown()         # 종료 처리

