#!/usr/bin/env python3
import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import requests

# 保存先（相対パス）
IMG_DIR = "src/auv25_ros/img"
os.makedirs(IMG_DIR, exist_ok=True)

# Discord Webhook URL
DISCORD_WEBHOOK_URL = "https://discord.com/api/webhooks/1443155754696835073/Nj0avUlzNQWlF6LJMPdSF-HbJD-b-PMoxYwpA8d8ZPhrP1eSWwvY6L91GBYps0jycrnb"  # ここは自分のWebhookに置き換え

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camsub_node')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.frame = None
        self.prev_button9 = 0  # 連射防止用

        # カメラ購読
        self.camera_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos
        )
        self.get_logger().info("cam viewer started (subscribing /camera/image_raw)")

        # ジョイスティック購読
        self.joy_sub = self.create_subscription(
            Joy,
            "/remote_pc/joy",
            self.joy_callback,
            qos
        )
        self.get_logger().info("joy started (subscribing /remote_pc/joy)")

        # Timerで表示
        self.timer = self.create_timer(0.01, self.display_frame)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def joy_callback(self, msg):
        button9 = msg.buttons[9] if len(msg.buttons) > 9 else 0

        # ボタン押下の立ち上がりで保存・送信
        if button9 == 1 and self.prev_button9 == 0:
            if self.frame is not None:
                # 180度回転
                frame_rot = cv2.rotate(self.frame, cv2.ROTATE_180)

                # 保存
                now = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(IMG_DIR, f"frame_{now}.png")
                cv2.imwrite(filename, frame_rot)
                self.get_logger().info(f"Saved frame to {filename}")

                # Discord送信
                try:
                    with open(filename, "rb") as f:
                        files = {"file": (os.path.basename(filename), f)}
                        response = requests.post(DISCORD_WEBHOOK_URL, files=files)
                        if response.status_code == 204:
                            self.get_logger().info("Image sent to Discord webhook successfully")
                        else:
                            self.get_logger().warn(f"Discord webhook response: {response.status_code}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send image to Discord: {e}")
            else:
                self.get_logger().warn("No frame to save yet.")

        self.prev_button9 = button9

    def display_frame(self):
        if self.frame is not None:
            # 180度回転して表示
            cv2.imshow("Camera Viewer", cv2.rotate(self.frame, cv2.ROTATE_180))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("viewer closed by user")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get
