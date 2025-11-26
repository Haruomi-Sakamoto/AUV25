#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('campub_node')

        # --- GStreamer パイプライン ---
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=10/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera (/dev/video0)")
            raise RuntimeError("Camera OPEN failed")

        # QoS (Sensor用)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info("USBCameraNode started (publishing /camera/image_raw)")

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("⚠ Failed to read frame — retrying")
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
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
