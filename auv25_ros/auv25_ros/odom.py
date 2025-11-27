import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
import math
import numpy as np  # ベクトル計算用にnumpyを追加

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

def rotate_vector(v, q):
    """
    クォータニオン q を使ってベクトル v を回転させる (Body -> World)
    """
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # クォータニオン回転式
    rx = x * (1 - 2*(qy**2 + qz**2)) + y * (2*(qx*qy - qz*qw)) + z * (2*(qx*qz + qy*qw))
    ry = x * (2*(qx*qy + qz*qw)) + y * (1 - 2*(qx**2 + qz**2)) + z * (2*(qy*qz - qx*qw))
    rz = x * (2*(qx*qz - qy*qw)) + y * (2*(qy*qz + qx*qw)) + z * (1 - 2*(qx**2 + qy**2))
    return np.array([rx, ry, rz])

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.sub_imu = self.create_subscription(Imu, 'imu/raw', self.imu_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'imu/odom_estimate', 10)

        self.last_time = None
        
        # 状態量 (World Frame: X=East/Start, Y=North/Left, Z=Up)
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # パラメータ
        self.GRAVITY = 9.81
        self.ALPHA = 0.98       # 相補フィルター係数
        self.DEAD_ZONE = 0.05   # 加速度ノイズ除去閾値 (m/s^2)
        self.VELOCITY_THRESHOLD = 0.1 # 簡易ZUPT用速度閾値 (m/s)

        self.get_logger().info("IMU Odometry Node (Corrected Axes) started.")

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now

        # --- 1. 軸の入れ替え (Sensor Frame -> Robot Frame) ---
        # 設定: Sensor X=下, Y=後, Z=左
        # 目標: Robot  X=前, Y=左, Z=上
        
        # 加速度
        ax = -msg.linear_acceleration.y  # 前 = -(後)
        ay =  msg.linear_acceleration.z  # 左 = 左
        az = -msg.linear_acceleration.x  # 上 = -(下)

        # 角速度 (右ねじの法則に従い軸同様に変換)
        gx = -msg.angular_velocity.y
        gy =  msg.angular_velocity.z
        gz = -msg.angular_velocity.x

        # --- 2. 姿勢計算 (相補フィルター) ---
        acc_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        acc_roll  = math.atan2(ay, az)

        self.roll  = self.ALPHA * (self.roll  + gx * dt) + (1.0 - self.ALPHA) * acc_roll
        self.pitch = self.ALPHA * (self.pitch + gy * dt) + (1.0 - self.ALPHA) * acc_pitch
        self.yaw   += gz * dt

        # 現在の姿勢クォータニオン
        q_orientation = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        
        # クォータニオンの正規化 (Normalization)
        q_norm = math.sqrt(q_orientation.w**2 + q_orientation.x**2 + q_orientation.y**2 + q_orientation.z**2)
        if q_norm > 1e-6:
             q_orientation.w /= q_norm
             q_orientation.x /= q_norm
             q_orientation.y /= q_norm
             q_orientation.z /= q_norm


        # --- 3. 座標変換と重力除去 ---
        acc_robot = np.array([ax, ay, az])
        acc_world = rotate_vector(acc_robot, q_orientation)

        # 重力加速度 (World Z軸) を除去
        acc_world[2] -= self.GRAVITY

        # ノイズ対策 (微小な値は0にするデッドゾーン)
        if abs(acc_world[0]) < self.DEAD_ZONE: acc_world[0] = 0.0
        if abs(acc_world[1]) < self.DEAD_ZONE: acc_world[1] = 0.0
        if abs(acc_world[2]) < self.DEAD_ZONE: acc_world[2] = 0.0

        # --- 4. 速度・位置計算 (積分と簡易ZUPT) ---
        self.vel += acc_world * dt
        
        # 簡易ZUPT: 加速度が小さく速度も小さい場合、速度を強制的にゼロにする
        if np.linalg.norm(acc_world) < 0.1 and np.linalg.norm(self.vel) < self.VELOCITY_THRESHOLD:
            self.vel = np.array([0.0, 0.0, 0.0]) 

        self.pos += self.vel * dt

        # --- 5. Odometry メッセージ作成 ---
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 位置
        odom.pose.pose.position = Point(x=self.pos[0], y=self.pos[1], z=self.pos[2])
        # 姿勢
        odom.pose.pose.orientation = q_orientation
        
        # 共分散の設定 (上位システムに信頼度を伝える)
        odom.pose.covariance = [
            1e-1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x (位置の信頼度は低い)
            0.0, 1e-1, 0.0, 0.0, 0.0, 0.0,  # y
            0.0, 0.0, 1e-1, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,  # roll (姿勢は比較的信頼できる)
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2,  # yaw (ドリフトするため位置よりは悪い)
        ]

        # 速度
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]

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