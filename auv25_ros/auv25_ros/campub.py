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

        self.connected = False
        self.cap = None

        # QoS (Sensor用)
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', self.qos)

        # 1秒ごとリトライ
        self.retry_timer = self.create_timer(1.0, self.try_connect)

        self.get_logger().info("Waiting for camera connection...")


    def try_connect(self):
        if self.connected:
            return  # 既に接続済みなら何もしない

        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=10/1 ! "
            "jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if self.cap.isOpened():
            self.connected = True
            self.retry_timer.cancel()  # リトライ終了
            self.get_logger().info("📷 Camera connected (/dev/video0) — start publishing")

            # Publish開始 (10Hz)
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            # retry時はログを出さない
            pass


    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            return  # 読み取り失敗時は静かにスキップ

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
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
