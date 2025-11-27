#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import csv
import os
import time
import requests   # ← Discord送信用

class CsvRecorder(Node):
    def __init__(self):
        super().__init__('csv_recorder')

        # ---- 記録対象トピック ----
       self.topic_names = [
            "/auv25/imu/odom_estimate",
            "/auv25/imu/raw",
            "/auv25/sensor_packet",
            "/cmd_vel",
            "/remote_pc/joy",
            "/thruster_output",
        ]

        self.data = {topic: None for topic in self.topic_names}

        # ---- 保存先（相対パス） ----
        workspace = os.path.expanduser("~/colon_ws")
        save_dir = f"{workspace}/src/auv25_ros/csv"
        os.makedirs(save_dir, exist_ok=True)

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"{save_dir}/log_{timestamp_str}.csv"

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp"] + self.topic_names)

        # ---- 購読 ---
        for topic in self.topic_names:
            self.create_subscription(Odometry, topic, self.generic_callback, 10)

        self.button_prev = 0
        self.recording = False
        self.create_subscription(Joy, "/remote_pc/joy", self.joy_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("CSV Recorder Node Ready")

        # ---- Discord Webhook URL ----
        self.webhook_url = "https://discord.com/api/webhooks/1443155754696835073/Nj0avUlzNQWlF6LJMPdSF-HbJD-b-PMoxYwpA8d8ZPhrP1eSWwvY6L91GBYps0jycrnb"  # ← 差し替えてください

    # ---- Odometry callback ----
    def generic_callback(self, msg):
        self.data["/odom"] = msg.pose.pose.position.x  # 例

    # ---- Joyボタン8トグル ----
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

    # ---- CSV書き込み ----
    def timer_callback(self):
        if not self.recording:
            return
        timestamp = int(time.time() * 1000)
        row = [timestamp] + [self.data[t] for t in self.topic_names]
        self.writer.writerow(row)

    # ---- Discord送信 ----
    def send_to_discord(self):
        try:
            with open(self.csv_path, "rb") as f:
                files = {"file": (os.path.basename(self.csv_path), f, "text/csv")}
                payload = {"content": f"CSV Log Uploaded: `{os.path.basename(self.csv_path)}`"}
                response = requests.post(self.webhook_url, data=payload, files=files)

            if response.status_code == 204:
                self.get_logger().info("CSV successfully sent to Discord.")
            else:
                self.get_logger().error(f"Discord upload failed. status={response.status_code}")

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
