#!/usr/bin/env python3
import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
import time

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('campub_node')

        # Camera retry
        self.connected = False
        self.cap = None

        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=10/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )

        self.pipeline = pipeline
        self.retry_timer = self.create_timer(1.0, self.try_connect)

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/image_raw", qos)

        # Joy subscribe
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.last_button_state = 0  # for edge detection

        self.get_logger().info("Waiting for camera connection...")

    def try_connect(self):
        if self.connected:
            return
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self.connected = True
            self.retry_timer.cancel()
            self.get_logger().info("📷 Camera connected — ready to capture via Joy")

    def joy_callback(self, msg: Joy):
        button = msg.buttons[9]  # Aボタン等、ボタン0をトリガー
        if button == 1 and self.last_button_state == 0:  # rising edge
            self.capture_frame()
        self.last_button_state = button

    def capture_frame(self):
        if not self.connected:
            self.get_logger().warn("Camera not connected yet")
            return

        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to capture frame")
            return

        # publish 1 frame
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

        # save file too
        filename = f"/home/haruomi/Pictures/capture_{int(time.time())}.jpg"
        cv2.imwrite(filename, frame)

        self.get_logger().info(f"📸 Captured & saved: {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
