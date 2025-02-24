import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from brping import Ping1D

class PingsonnerNode(Node):
    def __init__(self):
        super().__init__('pingsonner_node')

        # Ping1Dのインスタンスをクラスメンバーにする
        self.ping = Ping1D()
        if not self.ping.connect_serial("/dev/ttyUSB0", 115200):
            self.get_logger().error('Failed to connect to Ping1D sonar.')
            raise RuntimeError('Ping1D connection failed')

        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        sensor_data = self.ping.get_distance()
        if sensor_data is not None:
            self.publisher_.publish(Float32(data=sensor_data))
            self.get_logger().info(f'Sensor data: {sensor_data}')
        else:
            self.get_logger().warn('Failed to get distance data. Retrying...')

    def destroy_node(self):
        self.ping.disconnect()  # シリアル接続を閉じる
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PingsonnerNode()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.logging.get_logger('pingsonner_main').error(str(e))
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
