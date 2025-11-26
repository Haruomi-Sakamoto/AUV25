#!/usr/bin/env python3
import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camsub_node')

        # QoS設定（campubと一致させる）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos
        )

        self.get_logger().info("cam viewer started (subscribing /camera/image_raw)")
        self.frame = None

        # Timerで表示ループ（GUIを止めずにROSをspin）
        self.timer = self.create_timer(0.01, self.display_frame)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame = frame
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def display_frame(self):
        if self.frame is not None:
            cv2.imshow("Camera Viewer", self.frame)

        # qで終了
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
    main()
