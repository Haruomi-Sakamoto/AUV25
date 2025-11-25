#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        # パラメータ宣言
        self.declare_parameter('device', 0)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('topic', '/camera/image_raw')

        # パラメータ取得
        self.device = self.get_parameter('device').value
        self.fps = float(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.topic = self.get_parameter('topic').value

        # Publisher
        self.image_publisher = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()

        # カメラ起動
        try:
            self.cap = cv2.VideoCapture(self.device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        except Exception as e:
            self.get_logger().error(f"Camera open failed: {e}")
            raise

        # タイマーでFPS制御
        period = 1.0 / max(self.fps, 0.1)
        self.timer = self.create_timer(period, self.capture_frame)

        self.get_logger().info(
            f"USB Camera Node started: dev={self.device}, fps={self.fps}, size={self.width}x{self.height}"
        )

    def capture_frame(self):
        if not self.cap.isOpened():
            self.get_logger().warn("Camera is not opened")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "usb_camera"
            self.image_publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Image publish error: {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()