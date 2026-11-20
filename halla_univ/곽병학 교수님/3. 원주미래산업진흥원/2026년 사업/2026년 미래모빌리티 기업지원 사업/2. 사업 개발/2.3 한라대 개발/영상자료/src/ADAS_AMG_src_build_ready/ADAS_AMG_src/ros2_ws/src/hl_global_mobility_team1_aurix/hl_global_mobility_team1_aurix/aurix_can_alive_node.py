import socket
import struct
import time
from typing import Iterable, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = 16
DEFAULT_TEAM_NAME = "HL Global Mobility Team 1"


def clamp_byte(value: int) -> int:
    return max(0, min(255, int(value)))


def normalize_data(values: Iterable[int]) -> List[int]:
    data = [clamp_byte(value) for value in values]
    if len(data) > 8:
        data = data[:8]
    return data + ([0] * (8 - len(data)))


def make_can_frame(can_id: int, data: Iterable[int]) -> bytes:
    payload = bytes(normalize_data(data))
    standard_id = int(can_id) & 0x7FF
    return struct.pack(CAN_FRAME_FORMAT, standard_id, 8, payload)


class AurixCanAliveNode(Node):
    def __init__(self) -> None:
        super().__init__("aurix_can_alive_node")

        self.declare_parameter("team_name", DEFAULT_TEAM_NAME)
        self.declare_parameter("interface", "can0")
        self.declare_parameter("bitrate", 500000)
        self.declare_parameter("alive_can_id", 0x300)
        self.declare_parameter("alive_data", [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter("alive_period_ms", 50.0)
        self.declare_parameter("reconnect_period_ms", 200.0)
        self.declare_parameter("status_period_ms", 1000.0)

        self.team_name = str(self.get_parameter("team_name").value)
        self.interface = str(self.get_parameter("interface").value)
        self.bitrate = int(self.get_parameter("bitrate").value)
        self.alive_can_id = int(self.get_parameter("alive_can_id").value)
        self.alive_data = normalize_data(self.get_parameter("alive_data").value)
        self.alive_period_sec = max(float(self.get_parameter("alive_period_ms").value), 1.0) / 1000.0
        self.reconnect_period_sec = max(float(self.get_parameter("reconnect_period_ms").value), 1.0) / 1000.0
        self.status_period_sec = max(float(self.get_parameter("status_period_ms").value), 100.0) / 1000.0

        self.frame = make_can_frame(self.alive_can_id, self.alive_data)
        self.socket = None
        self.connected = False
        self.sent_count = 0
        self.error_count = 0
        self.reconnect_count = 0
        self.last_error = ""
        self.next_send_time = time.monotonic()
        self.next_reconnect_time = time.monotonic()
        self.next_status_time = time.monotonic()

        self.status_pub = self.create_publisher(String, "/aurix/can_alive_status", 10)
        self.timer = self.create_timer(0.01, self.on_timer)

        self.get_logger().info(
            f"{self.team_name}: AURIX CAN alive node ready. "
            f"interface={self.interface}, bitrate={self.bitrate}, "
            f"id=0x{self.alive_can_id:X}, cycle={self.alive_period_sec * 1000.0:.0f}ms, "
            f"reconnect={self.reconnect_period_sec * 1000.0:.0f}ms"
        )

    def on_timer(self) -> None:
        now = time.monotonic()

        if self.connected:
            if now >= self.next_send_time:
                self.send_alive(now)
        else:
            if now >= self.next_reconnect_time:
                self.connect(now)

        if now >= self.next_status_time:
            self.publish_status()
            self.next_status_time = now + self.status_period_sec

    def connect(self, now: float) -> None:
        self.close_socket()

        try:
            can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            can_socket.bind((self.interface,))
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

    def send_alive(self, now: float) -> None:
        if self.socket is None:
            self.connected = False
            self.next_reconnect_time = now + self.reconnect_period_sec
            return

        try:
            sent_bytes = self.socket.send(self.frame)
            if sent_bytes != CAN_FRAME_SIZE:
                raise OSError(f"partial CAN frame sent: {sent_bytes}/{CAN_FRAME_SIZE}")

            self.sent_count += 1
            self.next_send_time = now + self.alive_period_sec
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
        status = String()
        status.data = (
            f"team={self.team_name}, "
            f"interface={self.interface}, "
            f"connected={int(self.connected)}, "
            f"can_id=0x{self.alive_can_id:X}, "
            f"cycle_ms={self.alive_period_sec * 1000.0:.0f}, "
            f"reconnect_ms={self.reconnect_period_sec * 1000.0:.0f}, "
            f"sent={self.sent_count}, "
            f"errors={self.error_count}, "
            f"reconnects={self.reconnect_count}, "
            f"last_error={self.last_error}"
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
        self.close_socket()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AurixCanAliveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
