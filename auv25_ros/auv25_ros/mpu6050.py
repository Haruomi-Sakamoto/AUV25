#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2 as smbus
import time
import math

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        timer_period = 0.1  # 0.1秒周期
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # I2C通信の初期化
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68
        self.bus.write_byte_data(self.device_address, 0x6B, 0)  # スリープ解除

        self.get_logger().info('MPU6050 node started')

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # 加速度 (単位：g)
        accel_x = self.read_raw_data(0x3B) / 16384.0
        accel_y = self.read_raw_data(0x3D) / 16384.0
        accel_z = self.read_raw_data(0x3F) / 16384.0

        # 角速度 (単位：deg/s → rad/s)
        gyro_x = math.radians(self.read_raw_data(0x43) / 131.0)
        gyro_y = math.radians(self.read_raw_data(0x45) / 131.0)
        gyro_z = math.radians(self.read_raw_data(0x47) / 131.0)

        imu_msg.linear_acceleration.x = accel_x * 9.80665
        imu_msg.linear_acceleration.y = accel_y * 9.80665
        imu_msg.linear_acceleration.z = accel_z * 9.80665

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.orientation_covariance[0] = -1.0  # 無効

        # トピックにパブリッシュ
        self.publisher_.publish(imu_msg)

        print(f"Accel: [{accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}] g, Gyro: [{gyro_x:.3f}, {gyro_y:.3f}, {gyro_z:.3f}] rad/s", flush=True)

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
