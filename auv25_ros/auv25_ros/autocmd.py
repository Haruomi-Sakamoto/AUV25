#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class AutoCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('auto_cmd_vel_publisher')

        self.publisher = self.create_publisher(Twist, '/auto_cmd_vel', 10)
        self.start_pub = self.create_publisher(Bool, '/auto_cmd_start', 10)  # ★追加

        self.commands = [
            (0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 5),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 3),
            (0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 4),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10)
        ]

        self.index = 0
        self.start_time = None
        self.active = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.create_subscription(Bool, '/auto_cmd_start', self.start_callback, 10)

        self.get_logger().info("AutoCmdVelPublisher ready. Awaiting /auto_cmd_start")

    def start_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("AUTO MODE START: beginning command sequence")
            self.index = 0
            self.start_time = time.time()
            self.active = True

    def timer_callback(self):
        if not self.active:
            return

        if self.index >= len(self.commands):
            self.stop_robot()
            return

        vx, vy, vz, roll, pitch, yaw, duration = self.commands[self.index]

        if time.time() - self.start_time < duration:
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.linear.z = vz
            msg.angular.x = roll
            msg.angular.y = pitch
            msg.angular.z = yaw
            self.publisher.publish(msg)
        else:
            self.index += 1
            self.start_time = time.time()

    def stop_robot(self):
        # 停止コマンド送信
        msg = Twist()
        self.publisher.publish(msg)

        self.get_logger().info("Auto navigation completed. Sending FALSE")

        self.active = False

        # ★ 自分で False を返す
        self.start_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = AutoCmdVelPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
