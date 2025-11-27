#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import csv
import os
import time
from datetime import datetime
import requests


class CsvRecorder(Node):
    def __init__(self):
        super().__init__('csv_recorder')

        self.topic_names = [
            "/auv25/imu/odom_estimate",
            "/auv25/imu/raw",
            "/auv25/sensor_packet",
            "/cmd_vel",
            "/thruster_output"
        ]

        # 最新値保存
        self.data = {topic: None for topic in self.topic_names}

        # 保存ディレクトリ
        workspace = os.path.expanduser("~/colon_ws")
        save_dir = f"{workspace}/src/auv25_ros/csv"
        os.makedirs(save_dir, exist_ok=True)

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"{save_dir}/log_{timestamp_str}.csv"

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp"] + self.topic_names)

        # Subscribers
        self.create_subscription(Odometry, "/auv25/imu/odom_estimate",
                                lambda msg: self.generic_callback("/auv25/imu/odom_estimate", msg), 10)
        self.create_subscription(Imu, "/auv25/imu/raw",
                                lambda msg: self.generic_callback("/auv25/imu/raw", msg), 10)
        self.create_subscription(Int32MultiArray, "/auv25/sensor_packet",
                                lambda msg: self.generic_callback("/auv25/sensor_packet", msg), 10)
        self.create_subscription(Twist, "/cmd_vel",
                                lambda msg: self.generic_callback("/cmd_vel", msg), 10)
        self.create_subscription(Float32MultiArray, "/thruster_output",
                                lambda msg: self.generic_callback("/thruster_output", msg), 10)


        # Joyボタン制御
        self.recording = False
        self.button_prev = 0
        self.create_subscription(Joy, "/remote_pc/joy", self.joy_callback, 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("CSV Recorder Node Ready")

        # Discord
        self.webhook_url = "https://discord.com/api/webhooks/xxxxx"

    def generic_callback(self, topic, msg):
        # Odometry
        if isinstance(msg, Odometry):
            self.data[topic] = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        # Imu
        elif isinstance(msg, Imu):
            self.data[topic] = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
        # Twist
        elif isinstance(msg, Twist):
            self.data[topic] = [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ]
        # Float32MultiArray / Int32MultiArray
        elif hasattr(msg, "data"):
            self.data[topic] = list(msg.data)
        else:
            self.data[topic] = None


    def joy_callback(self, msg: Joy):
        button_val = msg.buttons[8]

        if self.button_prev == 0 and button_val == 1:
            self.recording = not self.recording
            state = "START" if self.recording else "STOP"
            self.get_logger().info(f"==== {state} CSV Recording ====")

            if not self.recording:
                self.csv_file.flush()
                self.send_to_discord()

        self.button_prev = button_val

    def timer_callback(self):
        if not self.recording:
            return

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        row = [timestamp]
        for t in self.topic_names:
            if self.data[t] is None:
                row.append("")  # 空欄
            else:
                row.append(self.data[t])

        self.writer.writerow(row)

    def send_to_discord(self):
        try:
            with open(self.csv_path, "rb") as f:
                files = {"file": (os.path.basename(self.csv_path), f, "text/csv")}
                payload = {"content": f"CSV Log uploaded: `{os.path.basename(self.csv_path)}`"}
                response = requests.post(self.webhook_url, data=payload, files=files)

            if response.status_code in [200, 204]:
                self.get_logger().info("CSV successfully sent to Discord.")
            else:
                self.get_logger().error(f"Discord upload failed: status={response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error sending to Discord: {e}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsvRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
