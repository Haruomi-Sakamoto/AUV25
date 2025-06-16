import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('j2tw_node')

        self.joy_subscriber = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.twist_publisher = self.create_publisher(Twist,'/cmd_vel',10)

    def joy_callback(self, joy_msg: Joy):
        twist = Twist()

        axes = joy_msg.axes
        get = lambda i: axes[i] if len(axes) > i else 0.0

        twist.linear.x  = get(4)
        twist.linear.y  = get(3)
        twist.linear.z  = get(5)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = get(0)

        self.twist_publisher.publish(twist)
        self.get_logger().info(f'Published Twist: {twist}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
