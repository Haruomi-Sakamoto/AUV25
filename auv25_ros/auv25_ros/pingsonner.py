import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from brping import Ping1D

class PingsonnerNode(Node):
    def __init__(self):
        super().__init__('pingsonner_node')
        #self.myping = Ping1D()
        #self.myping.connect_serial("/dev/ttyUSB0", 115200)
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # センサーからデータを取得する処理をここに追加
        sensor_data = 10.5  # 仮のデータ

        self.publisher_.publish(Float32(data=sensor_data))
        self.get_logger().info(f'Sensor data: {sensor_data}')
        
    def Sensing(self):
        pass
        

def main(args=None):
    rclpy.init(args=args)
    node = PingsonnerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
