#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Int32MultiArray


class SerialSensorReader(Node):
    HEADER = b'ST'
    PACKET_SIZE = 24  # int32 × 6 = 24 bytes
    MAX_BUFFER = 4096  # バッファ暴走防止

    def __init__(self):
        super().__init__('serial_sensor_reader')

        # --- Serial設定 ---
        port = '/dev/ttyACM0'
        baud = 115200

        self.port = port
        self.baud = baud
        self.ser = None

        self.open_serial()

        # Publisher
        self.pub = self.create_publisher(Int32MultiArray, 'sensor_packet', 10)

        # Timer (100ms)
        self.timer = self.create_timer(0.1, self.read_serial)

        # バッファ
        self.buffer = bytearray()

    def open_serial(self):
        """シリアルを開く（失敗時は警告のみ）"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to {self.port} at {self.baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Serial open failed: {e}")

    def read_serial(self):
        """シリアル読み取りループ"""
        # 接続が落ちたら再接続を試す
        if self.ser is None or not self.ser.is_open:
            self.open_serial()
            return

        try:
            waiting = self.ser.in_waiting
        except Exception:
            self.get_logger().warn("Serial disconnected—retrying...")
            self.ser = None
            return

        # --- バッファ追加 ---
        if waiting > 0:
            try:
                self.buffer += self.ser.read(waiting)
            except Exception:
                self.get_logger().warn("Serial read error—closing port")
                self.ser.close()
                self.ser = None
                return

        # バッファサイズ制限
        if len(self.buffer) > self.MAX_BUFFER:
            self.buffer = self.buffer[-self.MAX_BUFFER:]
            self.get_logger().warn("Buffer truncated")

        # --- ヘッダ同期 ---
        while True:
            required = 2 + self.PACKET_SIZE
            if len(self.buffer) < required:
                return

            # ヘッダ探索（高速）
            if self.buffer[0:2] != self.HEADER:
                idx = self.buffer.find(self.HEADER)
                if idx == -1:
                    # 全部捨てて次回読む
                    self.buffer.clear()
                    return
                else:
                    # ヘッダまでスキップ
                    del self.buffer[:idx]
                    if len(self.buffer) < required:
                        return

            # パケット構築
            payload = self.buffer[2:required]
            del self.buffer[:required]

            try:
                values = list(struct.unpack('<6i', payload))
            except struct.error as e:
                self.get_logger().warn(f"Unpack error: {e}")
                continue

            # パブリッシュ
            msg = Int32MultiArray()
            msg.data = values
            msg.layout.dim = []  # 明示（ROS2では空でOK）
            msg.layout.data_offset = 0

            self.pub.publish(msg)

            # 1回で1パケット処理
            return


def main(args=None):
    rclpy.init(args=args)
    node = SerialSensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
