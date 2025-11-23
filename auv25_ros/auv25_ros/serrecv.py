#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Int32MultiArray

class SerialSensorReader(Node):
    HEADER = b'ST'         # Arduino側ヘッダ
    PACKET_SIZE = 24       # int32 ×6 = 24バイト

    def __init__(self):
        super().__init__('serial_sensor_reader')

        # --- Serial設定 ---
        port = '/dev/ttyACM0'
        baud = 115200
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to {port} at {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial open error: {e}")
            raise

        # Publisher
        self.pub = self.create_publisher(Int32MultiArray, 'sensor_packet', 10)

        # Timer (100ms)
        self.timer = self.create_timer(0.1, self.read_serial)

        # バッファ
        self.buffer = bytearray()

    def read_serial(self):
        # --- バッファに追加 ---
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)

        # --- ヘッダ同期ループ ---
        while True:
            if len(self.buffer) < 2 + self.PACKET_SIZE:
                # データ不足
                return

            # ヘッダ確認
            if self.buffer[0:2] != self.HEADER:
                # ヘッダがずれている場合、先頭バイトを捨てる
                self.buffer.pop(0)
                continue

            # ペイロード取得
            payload = self.buffer[2:2+self.PACKET_SIZE]

            # バッファから消去
            del self.buffer[0:2+self.PACKET_SIZE]

            try:
                # Little-endian int32 ×6
                values = list(struct.unpack('<6i', payload))
            except struct.error as e:
                self.get_logger().warn(f"Unpack error: {e}")
                continue

            # パブリッシュ
            msg = Int32MultiArray()
            msg.data = values
            self.pub.publish(msg)
            break  # 1パケットごとに処理


def main(args=None):
    rclpy.init(args=args)
    node = SerialSensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
