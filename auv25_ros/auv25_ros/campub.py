#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('campub_node')

        # --- GStreamer パイプライン（v4l2src） ---
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=640, height=480, framerate=10/1 ! "
            "jpegdec ! videoconvert ! appsink"
        )

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("camera open failed")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # タイマーでフレーム取得（10Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("USBCameraNode started")

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # OpenCV 画像を ROS Image に変換してパブリッシュ
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

        # 任意で保存も可能
        # cv2.imwrite("frame.jpg", frame)

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
