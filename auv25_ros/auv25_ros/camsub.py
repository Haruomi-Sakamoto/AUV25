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

IMG_DIR = "/img"  # 保存先フォルダ

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

        # 前回のボタン状態（連射防止用）
        self.prev_button9 = 0

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
            "remotepc/joy/",
            self.joy_callback,
            qos
        )
        self.get_logger().info("joy started (subscribing remotepc/joy/)")

        self.timer = self.create_timer(0.01, self.display_frame)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def joy_callback(self, msg):

        # ボタンが押された瞬間だけ保存
        if msg.buttons[9] == 1 and self.prev_button9 == 0:
            if self.frame is not None:
                now = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(IMG_DIR, f"frame_{now}.png")
                cv2.imwrite(filename, self.frame)
                self.get_logger().info(f"Saved frame to {filename}")
            else:
                self.get_logger().warn("No frame to save yet.")

        self.prev_button9 = msg.buttons[9]

    def display_frame(self):
        if self.frame is not None:
            cv2.imshow("Camera Viewer", self.frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("viewer closed by user")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    os.makedirs(IMG_DIR, exist_ok=True)
    main()
