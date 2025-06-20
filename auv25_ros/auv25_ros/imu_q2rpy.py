import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import transforms3d.euler as t3d_euler
import math

class QuaternionToRPYNode(Node):
    def __init__(self):
        super().__init__('imu_q2rpy_node')
        self.sub = self.create_subscription(Imu, 'imu/quaternion', self.callback, 10)
        self.pub = self.create_publisher(Vector3, 'imu/rpy', 10)

    def callback(self, msg: Imu):
        q = msg.orientation
        # transforms3dのquat2eulerは [w, x, y, z] 順
        quat = [q.w, q.x, q.y, q.z]

        # Roll, Pitch, Yaw（rad）を取得
        # 引数順はZYX (yaw-pitch-roll)で戻り値も yaw, pitch, roll の順なので注意
        yaw, pitch, roll = t3d_euler.quat2euler(quat, axes='szyx')

        # ログ（デバッグ用）
        self.get_logger().debug(f"RPY [rad]: roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")

        # Vector3 に格納 (x=roll, y=pitch, z=yaw)
        rpy_msg = Vector3()
        rpy_msg.x = roll
        rpy_msg.y = pitch
        rpy_msg.z = yaw

        self.pub.publish(rpy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToRPYNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
