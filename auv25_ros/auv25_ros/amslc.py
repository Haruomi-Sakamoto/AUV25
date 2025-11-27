#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelSelector(Node):
    def __init__(self):
        super().__init__('am_selector')

        self.mode = "manual"  # manual / auto

        # Subscribers
        self.create_subscription(Twist, '/auto_cmd_vel', self.auto_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_callback, 10)
        self.create_subscription(Joy, '/remote_pc/joy', self.joy_callback, 10)

        # Publisher to thruster controller
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_auto = Twist()
        self.last_manual = Twist()

        # 前回のボタン状態保存
        self.prev_button4 = 0
        self.prev_button5 = 0

        self.get_logger().info("CmdVelSelector started: default MANUAL mode")


    def joy_callback(self, msg: Joy):
        current_b4 = msg.buttons[4]
        current_b5 = msg.buttons[5]

        # ボタン4: MANUAL 切り替え (立ち上がり)
        if current_b4 == 1 and self.prev_button4 == 0:
            self.mode = "manual"
            self.get_logger().info("Mode switched to MANUAL")
            self.publish_current()

        # ボタン5: AUTO 切り替え (立ち上がり)
        if current_b5 == 1 and self.prev_button5 == 0:
            self.mode = "auto"
            self.get_logger().info("Mode switched to AUTO")
            self.publish_current()

            self.auto_start_pub.publish(Bool(data=True))

        # 今回のボタン状態を次回比較用に保存
        self.prev_button4 = current_b4
        self.prev_button5 = current_b5


    def auto_callback(self, msg: Twist):
        self.last_auto = msg
        if self.mode == "auto":
            self.publish_current()


    def manual_callback(self, msg: Twist):
        self.last_manual = msg
        if self.mode == "manual":
            self.publish_current()


    def publish_current(self):
        if self.mode == "manual":
            self.pub.publish(self.last_manual)
        else:
            self.pub.publish(self.last_auto)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
