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
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.axes = []
        self.buttons = []
        self.linear = self.config.linear.copy()
        self.angular = self.config.angular.copy()
        self.armed = False
        self.prev_button7 = 0
        self.prev_button6 = 0

    def joy_callback(self, joy_msg: Joy):
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons

        if self.buttons[7] == 1 and self.prev_button7 == 0:
            self.armed = True
            self.get_logger().info('ARMED')
        if self.buttons[6] == 1 and self.prev_button6 == 0:
            self.armed = False
            self.get_logger().info('DISARMED')
        
        self.prev_button7 = self.buttons[7] if len(self.buttons) > 7 else 0
        self.prev_button6 = self.buttons[6] if len(self.buttons) > 6 else 0

        if not self.armed:
            for i in range(3):
                self.linear[i] = 0.0
                self.angular[i] = 0.0
        else:
            self.linear[0] = self.axes[4] * self.config.twist.scale_linear
            self.linear[1] = self.axes[3] * self.config.twist.scale_linear
            self.linear[2] = ((-self.axes[5] + 1) / 2 - (self.axes[2] - 1) / 2) * self.config.twist.scale_linear
            self.angular[0] = self.config.twist.angular[0]
            self.angular[1] = self.config.twist.angular[1]
            self.angular[2] = self.axes[0] * self.config.twist.scale_angular

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self.linear
        twist.angular.x, twist.angular.y, twist.angular.z = self.angular

        self.twist_publisher.publish(twist)