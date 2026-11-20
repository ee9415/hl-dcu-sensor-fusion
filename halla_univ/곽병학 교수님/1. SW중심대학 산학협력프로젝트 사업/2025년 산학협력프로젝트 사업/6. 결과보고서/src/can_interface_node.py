# smart_vehicle/can_interface_node.py
# -*- coding: utf-8 -*-
"""
can_interface_node
------------------
- ROS2 상위 레벨 명령을 CAN 프레임으로 매핑해 /to_can 토픽(can_msgs/Frame)으로 퍼블리시하는 노드
- 기본 입력:
    * /control_cmd : std_msgs/String  (예: "STOP", "GO", "LEFT", "RIGHT")
  선택 입력:
    * /twist_cmd   : geometry_msgs/Twist (linear.x, angular.z 사용)
- 출력:
    * /to_can      : can_msgs/Frame

핵심 포인트
-----------
1) 하드웨어를 모르는 계층에서 CAN 포맷(아이디, 바이트 데이터)을 결정하는 곳.
2) 하위 노드(예: canusb_sender_node)가 /to_can을 구독해 실제 버스로 내보냄.
3) ID/스케일/바이트 매핑은 프로젝트 규격에 맞춰 자유롭게 변경/확장 가능.
"""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from can_msgs.msg import Frame

# 선택: Twist를 쓰고 싶다면 메시지 의존성을 추가하세요 (package.xml에 geometry_msgs 의존성도 추가)
from geometry_msgs.msg import Twist


class CanInterfaceNode(Node):
    def __init__(self):
        super().__init__('can_interface_node', namespace='smart_vehicle')

        # ========= 파라미터(필요에 맞춰 Launch/CLI에서 오버라이드) =========
        # /control_cmd → 특정 ID로 1바이트 명령 전송
        self.declare_parameter('cmd_topic', 'can/control_cmd')
        self.declare_parameter('cmd_can_id', 0x123)        # 표준 ID(0x000~0x7FF), 확장 쓰면 아래 플래그도
        self.declare_parameter('cmd_is_extended', False)   # 확장 ID(29bit)면 True
        # /twist_cmd → 속도/조향을 packing
        self.declare_parameter('twist_topic', 'can/twist_cmd')
        self.declare_parameter('twist_can_id', 0x321)
        self.declare_parameter('twist_is_extended', False)

        # 명령 문자열 → 데이터 매핑 (자유롭게 수정/확장)
        # 여기서는 1바이트 코드만 사용 (STOP=0x01, GO=0x02, LEFT=0x03, RIGHT=0x04)
        self._cmd_map: Dict[str, int] = {
            'STOP': 0x00,
            'GO': 0x82,
            'LEFT': 0x03,
            'RIGHT': 0x04,
	    # 동의어 추가(Decision에서 오는 값들)
	    'BRAKE': 0x00,   # BRAKE == STOP 로 취급
	    'SLOW':  0x82,   # 감속 별도코드가 필요하면 0x05로 정의(ECU 규격에 맞게 조정)
        }

        # 파라미터 값 읽기
        self._cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self._cmd_can_id = self.get_parameter('cmd_can_id').get_parameter_value().integer_value
        self._cmd_is_ext = self.get_parameter('cmd_is_extended').get_parameter_value().bool_value

        self._twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        self._twist_can_id = self.get_parameter('twist_can_id').get_parameter_value().integer_value
        self._twist_is_ext = self.get_parameter('twist_is_extended').get_parameter_value().bool_value

        # 퍼블리셔: CAN 프레임을 하류 노드(canusb_sender)가 구독
        self.pub_can = self.create_publisher(Frame, '/to_can', 10)

        # 구독자: 상위 노드들 입력
        self.sub_cmd = self.create_subscription(String, self._cmd_topic, self._on_cmd, 10)
        # Twist 입력도 활성화(원치 않으면 주석 처리 가능)
        self.sub_twist = self.create_subscription(Twist, self._twist_topic, self._on_twist, 10)

        self.get_logger().info(
            f'CAN Interface ready: cmd_topic="{self._cmd_topic}" -> ID 0x{self._cmd_can_id:X}, '
            f'ext={self._cmd_is_ext}; twist_topic="{self._twist_topic}" -> ID 0x{self._twist_can_id:X}, '
            f'ext={self._twist_is_ext}'
        )

    # ========================= 콜백들 =========================

    def _on_cmd(self, msg: String) -> None:
        """
        /control_cmd(String) → /to_can(Frame)
        - 문자열 명령을 1바이트 코드로 변환해 전송
        - DLC=1, data[0] = 코드 (나머지는 0으로 padding)
        """
        key = msg.data.strip().upper()
        code = self._cmd_map.get(key, None)

        if code is None:
            self.get_logger().warn(f'Unknown control_cmd="{msg.data}", ignore')
            return

        frame = Frame()
        frame.id = int(self._cmd_can_id)
        frame.is_extended = bool(self._cmd_is_ext)
        frame.is_rtr = False
        frame.is_error = False
        frame.dlc = 1
        # CAN 데이터 배열은 8바이트 고정 → 필요한 길이만 채우고 나머지 0 패딩
        data: List[int] = [0] * 8
        data[0] = code & 0xFF
        frame.data = data

        self.pub_can.publish(frame)
        self.get_logger().info(f'CMD → CAN: "{key}" → ID=0x{frame.id:X}, data=[{data[0]:02X}]')

    def _on_twist(self, msg: Twist) -> None:
        """
        /twist_cmd(Twist) → /to_can(Frame)
        - 예시: linear.x(m/s), angular.z(rad/s)를 2바이트 정수로 스케일링해 전송
        - 실제 차량/ECU 규격에 맞춰 스케일/ID/패킹 방식을 바꾸세요.
        """
        # 스케일 선정(예시):
        #   linear.x  m/s → cm/s * 10  => int16
        #   angular.z rad/s → deg/s * 100 => int16
        linear_scaled = int(max(min(msg.linear.x * 1000 / 10, 32767), -32768))   # ~ int16 범위 클램프
        angular_deg_s = msg.angular.z * (180.0 / 3.1415926535)
        angular_scaled = int(max(min(angular_deg_s * 100, 32767), -32768))      # ~ int16 범위 클램프

        # int16을 리틀엔디언 2바이트씩 패킹(예시)
        def i16_to_le_bytes(val: int) -> List[int]:
            v = val & 0xFFFF
            return [v & 0xFF, (v >> 8) & 0xFF]

        b_lin = i16_to_le_bytes(linear_scaled)
        b_ang = i16_to_le_bytes(angular_scaled)

        frame = Frame()
        frame.id = int(self._twist_can_id)
        frame.is_extended = bool(self._twist_is_ext)
        frame.is_rtr = False
        frame.is_error = False
        frame.dlc = 4  # 두 필드만 전송(2+2)
        data: List[int] = [0] * 8
        data[0:2] = b_lin
        data[2:4] = b_ang
        frame.data = data

        self.pub_can.publish(frame)
        self.get_logger().info(
            f'TWIST → CAN: ID=0x{frame.id:X}, lin={msg.linear.x:.3f} m/s ({linear_scaled}), '
            f'ang={msg.angular.z:.3f} rad/s ({angular_scaled})'
        )


def main():
    rclpy.init()
    node = CanInterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

