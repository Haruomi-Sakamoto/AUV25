#!/usr/bin/env python3
import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('campub_node')

        self.connected = False
        self.cap = None

        self.pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=10/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )

        # retry every 1s
        self.retry_timer = self.create_timer(1.0, self.try_connect)

        # publish at 10Hz
        self.pub_timer = self.create_timer(0.1, self.publish_frame)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(Image, "/camera/image_raw", qos)
        self.bridge = CvBridge()

        self.get_logger().info("Waiting for camera connection...")

    def try_connect(self):
        """ Try connect camera until success """
        if self.connected:
            return

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self.connected = True
            self.get_logger().info("Camera connected and publishing")
        else:
            self.get_logger().warn("Retry camera connect...")

    def publish_frame(self):
        """ Publish frames continuously; if failure reconnect """
        if not self.connected:
            return

        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Camera frame lost — reconnecting...")
            self.connected = False
            self.cap.release()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

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
