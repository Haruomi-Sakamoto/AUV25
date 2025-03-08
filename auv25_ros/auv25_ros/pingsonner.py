import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from brping import Ping1D

import random
#for gpio test

class PingsonnerNode(Node):
    def __init__(self):
        super().__init__('pingsonner_node')
        #self.myping = Ping1D()
        #self.myping.connect_serial("/dev/ttyUSB0", 115200)
        self.publisher_ = self.create_publisher(Float32, 'sonner_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
    
        #add get data
        sonner_data = float(random.randint(0, 20)) 
        #test data

        self.publisher_.publish(Float32(data=sonner_data))
        self.get_logger().info(f'Sonner data: {sonner_data}')
        
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
