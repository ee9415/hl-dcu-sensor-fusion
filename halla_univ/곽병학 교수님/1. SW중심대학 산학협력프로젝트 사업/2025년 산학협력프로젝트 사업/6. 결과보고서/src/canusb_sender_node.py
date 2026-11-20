# smart_vehicle/canusb_sender_node.py
# -*- coding: utf-8 -*-
"""
canusb_sender_node
------------------
- /to_can(can_msgs/Frame)를 구독해 Waveshare 'canusb' CLI를 호출하여 물리 CAN 버스로 송신하는 노드
- 핵심:
    * canusb가 0x접두 헥사를 싫어하는 빌드가 있으므로 ID는 '십진수'로 전달
    * 데이터는 "AA BB CC ..." 공백 구분 헥사 문자열로 전달
    * sudo/장치 경로/비트레이트/확장ID옵션은 파라미터로 설정 가능
"""

import shlex
import subprocess
from typing import List

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame


class CanusbSenderNode(Node):
    def __init__(self):
        super().__init__('canusb_sender_node', namespace='smart_vehicle')

        # ======== 파라미터 ========
        self.declare_parameter('canusb_path', '/home/qsentech/can_dir/USB-CAN-A/canusb')
        self.declare_parameter('dev', '/dev/ttyUSB0')
        self.declare_parameter('bitrate', 500000)            # CAN 버스 속도 (500k/1M 등)
        self.declare_parameter('use_sudo', True)             # sudo 필요 시 True
        self.declare_parameter('extended_flag_opt', '')      # canusb 확장ID 옵션(예: '-x'), 없으면 빈 문자열

        # /to_can 구독
        self.sub = self.create_subscription(Frame, '/to_can', self._on_can_frame, 50)
        self.get_logger().info('canusb_sender_node ready. Subscribing "/to_can"')

    # --------------------------- 콜백 ---------------------------
    def _on_can_frame(self, msg: Frame) -> None:
        """
        /to_can(Frame) → canusb CLI 호출
        - msg.id       : uint32 (표준/확장 모두 가능)
        - msg.is_extended: bool (확장 ID 여부)
        - msg.dlc      : 0~8
        - msg.data[0:dlc] : 데이터 바이트
        """
        # 파라미터 읽기
        canusb = self.get_parameter('canusb_path').get_parameter_value().string_value
        dev = self.get_parameter('dev').get_parameter_value().string_value
        br = self.get_parameter('bitrate').get_parameter_value().integer_value
        use_sudo = self.get_parameter('use_sudo').get_parameter_value().bool_value
        extopt = self.get_parameter('extended_flag_opt').get_parameter_value().string_value

        # 1) ID는 십진수로
        can_id_dec = int(msg.id)

        # 2) 데이터는 헥사 문자열 "AA BB CC ..." 로 변환 (dlc만큼)
        data_hex_list: List[str] = [f'{b:02X}' for b in list(msg.data)[:msg.dlc]]
        data_hex_str = ' '.join(data_hex_list) if data_hex_list else ''

        # 3) canusb 명령 빌드
        #    -s <bitrate> 는 CAN 버스 속도 (예: 500000)
        #    -i <id-dec>  는 십진수 ID
        #    -j "<hex bytes>" 는 데이터
        base_cmd = f'{canusb} -d {dev} -s {br} -t -i {can_id_dec} -j "{data_hex_str}"'
        if msg.is_extended and extopt:
            # canusb가 확장ID 전송을 위한 별도 플래그를 요구한다면 extopt에 넣어서 전달
            base_cmd = f'{canusb} -d {dev} -s {br} -t {extopt} -i {can_id_dec} -j "{data_hex_str}"'

        cmd = f'sudo {base_cmd}' if use_sudo else base_cmd

        # 4) 실행
        self.get_logger().info(
            f'SEND → canusb: ID=0x{msg.id:X} (dec={can_id_dec}), ext={msg.is_extended}, '
            f'dlc={msg.dlc}, data="{data_hex_str}"'
        )
        try:
            # canusb가 표준 출력에 내부 프로토콜(aa .. 55)이나 에러 메시지를 뿌릴 수 있음 → 로깅 용도
            out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT, timeout=3.0)
            if out:
                self.get_logger().debug(out.decode(errors='ignore').strip())
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f'canusb error: {e.output.decode(errors="ignore").strip()}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('canusb timeout (no response within 3s)')

# --------------------------- main ---------------------------
def main():
    rclpy.init()
    node = CanusbSenderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

