#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2 as smbus
import math

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # I2C設定
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68
        self.bus.write_byte_data(self.device_address, 0x6B, 0)  # スリープ解除

        self.get_logger().info('MPU6050 initialized')

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        value = (high << 8) | low
        if value > 32767:  # ←修正: 32768 → 32767
            value -= 65536
        return value

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # 加速度 [m/s^2]
        accel_x = self.read_raw_data(0x3B) / 16384.0 * 9.80665
        accel_y = self.read_raw_data(0x3D) / 16384.0 * 9.80665
        accel_z = self.read_raw_data(0x3F) / 16384.0 * 9.80665

        # 角速度 [rad/s]
        gyro_x = math.radians(self.read_raw_data(0x43) / 131.0)
        gyro_y = math.radians(self.read_raw_data(0x45) / 131.0)
        gyro_z = math.radians(self.read_raw_data(0x47) / 131.0)

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        imu_msg.orientation_covariance[0] = -1.0  # orientation無効

        self.publisher_.publish(imu_msg)

        self.get_logger().info(
            f"Accel [m/s^2]: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} | "
            f"Gyro [rad/s]: x={gyro_x:.2f}, y={gyro_y:.2f}, z={gyro_z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
