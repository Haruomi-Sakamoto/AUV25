#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import serial
import struct


class SensorsSerialNode(Node):
    def __init__(self):
        super().__init__('sensors_serial_node')

        # --- シリアル設定 ---
        self.ser = serial.Serial(
            port='/dev/ttyACM0',   # ← Arduino に合わせて変更
            baudrate=115200,
            timeout=1
        )

        # --- Publisher ---
        self.pub = self.create_publisher(Float64MultiArray, '/sensors/data', 10)

        # --- タイマー（100ms） ---
        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info('SensorsSerialNode started')

    def read_serial(self):
        """Arduino から 6×int32 のバイナリを読み Float64MultiArray で publish"""
        expected_bytes = 6 * 4  # int32 × 6 = 24 byte

        if self.ser.in_waiting < expected_bytes:
            return

        data = self.ser.read(expected_bytes)

        if len(data) != expected_bytes:
            self.get_logger().warn('Invalid packet size')
            return

        # --- バイナリ → int32 × 6 ---
        values_int = struct.unpack('<6i', data)  # little-endian
        values_float = [float(v) for v in values_int]

        # Publish
        msg = Float64MultiArray()
        msg.data = values_float
        self.pub.publish(msg)

        self.get_logger().debug(
            f"D1={values_float[0]} C1={values_float[1]}  "
            f"D2={values_float[2]} C2={values_float[3]}  "
            f"Depth(mm)={values_float[4]} Temp(x100)={values_float[5]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorsSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
