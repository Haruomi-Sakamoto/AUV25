#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct

from auv25_ros.msg import SensorPacket


class SerialSensorReader(Node):
    def __init__(self):
        super().__init__('serial_sensor_reader')

        # --- Serial設定 ---
        port = '/dev/ttyACM0'  # 必要に応じて変更
        baud = 115200

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to {port} at {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial open error: {e}")
            raise

        # Publisher
        self.pub = self.create_publisher(SensorPacket, 'sensor_packet', 10)

        # Timer (100ms)
        self.timer = self.create_timer(0.1, self.read_serial)

        # 1パケット = int32 ×6 = 24 バイト
        self.PACKET_SIZE = 24

    def read_serial(self):
        if self.ser.in_waiting < self.PACKET_SIZE:
            return  # データ不足

        data = self.ser.read(self.PACKET_SIZE)

        if len(data) != self.PACKET_SIZE:
            return

        # Little-endian Int32 ×6
        try:
            d1, c1, d2, c2, depth_mm, temp_x100 = struct.unpack('<6i', data)
        except struct.error as e:
            self.get_logger().warn(f"Unpack error: {e}")
            return

        msg = SensorPacket()
        msg.d1 = d1
        msg.c1 = c1
        msg.d2 = d2
        msg.c2 = c2
        msg.depth_mm = depth_mm
        msg.temp_x100 = temp_x100

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialSensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
