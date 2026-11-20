"""
aurix_can_bridge_node.py

/ackermann_cmd (ackermann_msgs/AckermannDriveStamped) 를 구독해서,
AURIX TC275 펌웨어가 이해하는 0x300 PC_COMMAND CAN 프레임으로 인코딩해
SocketCAN(can0)으로 주기적으로 내보내는 브릿지 노드.

펌웨어 쪽 디코드는 Can_SpeedTx.c: CanSpeedTx_DecodePcCommand() 참고.

페이로드 (8바이트, 리틀엔디안):
  byte0-1 : TargetSteeringDegX10  (int16, 0.1 deg/LSB)
  byte2-3 : TargetSpeedKphX100    (int16, 0.01 km/h/LSB, 음수=후진)
  byte4   : EnableFlag            (0=페일세이프, 1=명령 적용)
  byte5   : AliveCounter          (0~255 롤오버)
  byte6-7 : Reserved (0)

안전 설계:
  - /ackermann_cmd 가 command_timeout_ms 이상 안 들어오면 EnableFlag=0으로
    전환해서 보낸다 (AURIX가 조향중앙/속도0으로 페일세이프 하도록).
  - 이 노드가 죽거나 CAN 링크가 끊겨도, AURIX 쪽 g_canRxAlive 타임아웃
    (기본 500ms)이 있어서 이중으로 안전하게 멈춘다.
"""

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Float32

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = 16
PC_COMMAND_CAN_ID = 0x300
WHEEL_SPEED_CAN_ID = 0x120  # AURIX -> PC: 실측 차량속도 (Can_SpeedTx.c: CanSpeedTx_SendWheelSpeed 참고)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def encode_pc_command(steering_deg: float, speed_kph: float, enable: bool, alive: int) -> bytes:
    steer_x10 = int(round(clamp(steering_deg, -3276.8, 3276.7) * 10.0))
    speed_x100 = int(round(clamp(speed_kph, -327.68, 327.67) * 100.0))
    payload = struct.pack(
        "<hhBBH",
        steer_x10,
        speed_x100,
        1 if enable else 0,
        alive & 0xFF,
        0,
    )
    return payload


def make_can_frame(can_id: int, payload: bytes) -> bytes:
    data = payload[:8] + bytes(8 - len(payload[:8]))
    standard_id = int(can_id) & 0x7FF
    return struct.pack(CAN_FRAME_FORMAT, standard_id, 8, data)


def decode_wheel_speed(data8: bytes):
    """0x120 WheelSpeed 페이로드 디코드. Can_SpeedTx.c: CanSpeedTx_SendWheelSpeed 와 대칭.
    byte0-1: leftRpmX10, byte2-3: rightRpmX10, byte4-5: vehicleMpsX1000, byte6: valid"""
    left_rpm_x10, right_rpm_x10, vehicle_mps_x1000, valid = struct.unpack(
        "<hhhB", data8[:7]
    )
    return {
        "left_rpm": left_rpm_x10 / 10.0,
        "right_rpm": right_rpm_x10 / 10.0,
        "vehicle_mps": vehicle_mps_x1000 / 1000.0,
        "valid": bool(valid),
    }


class AurixCanBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("aurix_can_bridge_node")

        self.declare_parameter("interface", "can0")
        self.declare_parameter("drive_topic", "/ackermann_cmd")
        self.declare_parameter("send_period_ms", 20.0)       # 50 Hz
        self.declare_parameter("command_timeout_ms", 300.0)  # 이보다 오래 명령 없으면 fail-safe
        self.declare_parameter("reconnect_period_ms", 200.0)
        self.declare_parameter("status_period_ms", 1000.0)
        self.declare_parameter("wheelbase_unused", 0.0)  # (예약, 필요시 확장용)

        self.interface = str(self.get_parameter("interface").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.send_period_sec = max(float(self.get_parameter("send_period_ms").value), 1.0) / 1000.0
        self.command_timeout_sec = max(float(self.get_parameter("command_timeout_ms").value), 1.0) / 1000.0
        self.reconnect_period_sec = max(float(self.get_parameter("reconnect_period_ms").value), 1.0) / 1000.0
        self.status_period_sec = max(float(self.get_parameter("status_period_ms").value), 100.0) / 1000.0

        self.socket = None
        self.connected = False
        self.sent_count = 0
        self.error_count = 0
        self.reconnect_count = 0
        self.last_error = ""
        self.alive_counter = 0

        self.last_steering_deg = 0.0
        self.last_speed_kph = 0.0
        self.last_cmd_time = 0.0  # monotonic, 0 = 아직 명령 없음

        now = time.monotonic()
        self.next_send_time = now
        self.next_reconnect_time = now
        self.next_status_time = now

        self.sub = self.create_subscription(
            AckermannDriveStamped, self.drive_topic, self.on_drive_cmd, 10
        )
        self.status_pub = self.create_publisher(String, "/aurix/can_bridge_status", 10)
        self.speed_pub = self.create_publisher(Float32, "/aurix/vehicle_speed", 10)
        self.timer = self.create_timer(0.005, self.on_timer)

        self.get_logger().info(
            f"aurix_can_bridge_node ready. interface={self.interface} "
            f"drive_topic={self.drive_topic} "
            f"send={self.send_period_sec * 1000.0:.0f}ms "
            f"timeout={self.command_timeout_sec * 1000.0:.0f}ms"
        )

    def on_drive_cmd(self, msg: AckermannDriveStamped) -> None:
        # 부호 반전: 펌웨어 조향 서보 기준 +값이 오른쪽으로 나오는 게 실측으로 확인돼서
        # ROS2(AckermannDriveStamped, +값=왼쪽 관례)랑 맞춰주려고 여기서 뒤집음
        self.last_steering_deg = -(msg.drive.steering_angle * 180.0 / 3.141592653589793)
        self.last_speed_kph = msg.drive.speed * 3.6
        self.last_cmd_time = time.monotonic()

    def on_timer(self) -> None:
        now = time.monotonic()

        if self.connected:
            self.read_can_frames()
            if now >= self.next_send_time:
                self.send_command(now)
        else:
            if now >= self.next_reconnect_time:
                self.connect(now)

        if now >= self.next_status_time:
            self.publish_status()
            self.next_status_time = now + self.status_period_sec

    def read_can_frames(self) -> None:
        if self.socket is None:
            return
        # non-blocking 소켓이라 큐에 쌓인 만큼 한번에 다 비워줌
        for _ in range(32):
            try:
                frame = self.socket.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                break
            except OSError as exc:
                self.last_error = str(exc)
                break
            if len(frame) != CAN_FRAME_SIZE:
                continue
            can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame)
            can_id = can_id_raw & 0x1FFFFFFF  # EFF/RTR/ERR 플래그 비트 제거
            if can_id == WHEEL_SPEED_CAN_ID:
                decoded = decode_wheel_speed(data)
                msg = Float32()
                msg.data = decoded["vehicle_mps"]
                self.speed_pub.publish(msg)

    def connect(self, now: float) -> None:
        self.close_socket()
        try:
            can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            can_socket.bind((self.interface,))
            can_socket.setblocking(False)
            self.socket = can_socket
            self.connected = True
            self.reconnect_count += 1
            self.next_send_time = now
            self.last_error = ""
            self.get_logger().info(
                f"Connected to {self.interface}. reconnects={self.reconnect_count}"
            )
        except (AttributeError, OSError) as exc:
            self.connected = False
            self.error_count += 1
            self.last_error = str(exc)
            self.next_reconnect_time = now + self.reconnect_period_sec
            self.get_logger().warn(
                f"CAN reconnect failed on {self.interface}: {self.last_error}"
            )

    def send_command(self, now: float) -> None:
        if self.socket is None:
            self.connected = False
            self.next_reconnect_time = now + self.reconnect_period_sec
            return

        cmd_fresh = (
            self.last_cmd_time > 0.0
            and (now - self.last_cmd_time) <= self.command_timeout_sec
        )

        if cmd_fresh:
            steering_deg = self.last_steering_deg
            speed_kph = self.last_speed_kph
            enable = True
        else:
            # 명령이 끊겼음 -> 조향중앙/속도0 + EnableFlag=0 으로 명시적 페일세이프
            steering_deg = 0.0
            speed_kph = 0.0
            enable = False

        payload = encode_pc_command(steering_deg, speed_kph, enable, self.alive_counter)
        frame = make_can_frame(PC_COMMAND_CAN_ID, payload)
        self.alive_counter = (self.alive_counter + 1) & 0xFF

        try:
            sent_bytes = self.socket.send(frame)
            if sent_bytes != CAN_FRAME_SIZE:
                raise OSError(f"partial CAN frame sent: {sent_bytes}/{CAN_FRAME_SIZE}")
            self.sent_count += 1
            self.next_send_time = now + self.send_period_sec
        except OSError as exc:
            self.connected = False
            self.error_count += 1
            self.last_error = str(exc)
            self.next_reconnect_time = now + self.reconnect_period_sec
            self.close_socket()
            self.get_logger().warn(
                f"CAN send failed. retry in {self.reconnect_period_sec * 1000.0:.0f}ms: "
                f"{self.last_error}"
            )

    def publish_status(self) -> None:
        now = time.monotonic()
        cmd_fresh = (
            self.last_cmd_time > 0.0
            and (now - self.last_cmd_time) <= self.command_timeout_sec
        )
        status = String()
        status.data = (
            f"interface={self.interface}, connected={int(self.connected)}, "
            f"cmd_fresh={int(cmd_fresh)}, "
            f"steering_deg={self.last_steering_deg:.1f}, "
            f"speed_kph={self.last_speed_kph:.2f}, "
            f"sent={self.sent_count}, errors={self.error_count}, "
            f"reconnects={self.reconnect_count}, last_error={self.last_error}"
        )
        self.status_pub.publish(status)

    def close_socket(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None

    def destroy_node(self) -> bool:
        # 노드 종료 시 마지막으로 fail-safe(EnableFlag=0) 프레임 한번 쏘고 닫음
        if self.socket is not None:
            try:
                payload = encode_pc_command(0.0, 0.0, False, self.alive_counter)
                frame = make_can_frame(PC_COMMAND_CAN_ID, payload)
                self.socket.send(frame)
            except OSError:
                pass
        self.close_socket()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AurixCanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
