import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
import math

def quaternion_from_euler(roll, pitch, yaw):
    """
    Roll, Pitch, Yaw → Quaternion 変換
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.sub_imu = self.create_subscription(Imu, 'imu/raw', self.imu_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'imu/odom_estimate', 10)

        self.last_time = None
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.get_logger().info("IMU Odometry + Attitude Node started.")

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now

        # --- 姿勢計算 ---
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Roll, Pitch: 重力ベクトルから計算
        self.roll = math.atan2(ay, az)
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Yaw: ジャイロ積分
        self.yaw += msg.angular_velocity.x * dt  # X軸回転が地面水平回転と仮定

        # --- 速度・位置計算 (YZ平面) ---
        self.vel_y += ay * dt
        self.vel_z += az * dt

        self.pos_y += self.vel_y * dt
        self.pos_z += self.vel_z * dt

        # Odometry メッセージ作成
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(x=0.0, y=self.pos_y, z=self.pos_z)
        odom.pose.pose.orientation = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        odom.twist.twist.linear.y = self.vel_y
        odom.twist.twist.linear.z = self.vel_z

        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
