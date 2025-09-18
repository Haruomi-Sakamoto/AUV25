import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

from auv25_ros.config import ThrusterConfig

class ThrusterControlNode(Node):
    def __init__(self):
        super().__init__('thruster_control_node')

        self.cfg = ThrusterConfig()

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray,'/thruster_output', 10)

    def twist_callback(self, msg: Twist):
        vel_vec = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ], dtype=float)

        allocation_matrix = np.array(self.cfg.allocation_matrix, dtype=float)
        thruster_output = self.cfg.scale * allocation_matrix @ vel_vec

        out = Float64MultiArray()
        out.data = thruster_output.tolist()
        self.pub.publish(out)

        self.get_logger().debug(f"Thruster output: {out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
