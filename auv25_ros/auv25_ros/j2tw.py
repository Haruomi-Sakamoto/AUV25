import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from auv25_ros.config import TwistConfig

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('j2tw_node')
        self.config = TwistConfig()

        self.joy_subscriber = self.create_subscription(Joy, '/remote_pc/joy', self.joy_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel_manual', 10)

        self.axes = []
        self.buttons = []
        self.linear = self.config.linear.copy()
        self.angular = self.config.angular.copy()
        self.armed = False
        self.prev_button7 = 0
        self.prev_button6 = 0

        self.timer = self.create_timer(0.1, self.publish_twist)

    def joy_callback(self, joy_msg: Joy):
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons

        if self.buttons[7] == 1 and self.prev_button7 == 0 and not self.armed :
            self.armed = True
            self.get_logger().info('ARMED')
        if self.buttons[6] == 1 and self.prev_button6 == 0 and self.armed:
            self.armed = False
            self.get_logger().info('DISARMED')
        
        self.prev_button7 = self.buttons[7] if len(self.buttons) > 7 else 0
        self.prev_button6 = self.buttons[6] if len(self.buttons) > 6 else 0

        if self.armed:
            self.linear[0] = -self.axes[2] * self.config.scale_linear if len(self.axes) > 2 else 0.0
            self.linear[1] = self.axes[3] * self.config.scale_linear if len(self.axes) > 3 else 0.0
            self.linear[2] = self.axes[1] * self.config.scale_linear if len(self.axes) > 1 else 0.0
            self.angular[0] = self.config.angular[0]
            self.angular[1] = self.config.angular[1]
            self.angular[2] = -self.axes[0] * self.config.scale_linear if len(self.axes) > 0 else 0.0
        else:
            self.linear[:] = [0.0, 0.0, 0.0]
            self.angular[:] = [0.0, 0.0, 0.0]

    def publish_twist(self):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self.linear
        twist.angular.x, twist.angular.y, twist.angular.z = self.angular
        self.twist_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
