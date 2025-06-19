#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2 as smbus
import math
import time

from config.config import MPU6050Config

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.config = MPU6050Config()

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.bus = None
        self.i2c_available = False
        self.last_log_time = time.time()

        self.initialize_sensor()

    def initialize_sensor(self):
        try:
            self.bus = smbus.SMBus(self.config.i2c_bus_number)
            self.bus.write_byte_data(self.config.device_address, self.config.power_mgmt_1, 0x00)
            self.bus.write_byte_data(self.config.device_address, self.config.gyro_config, 0x00)
            self.bus.write_byte_data(self.config.device_address, self.config.accel_config, 0x00)
            self.i2c_available = True
            self.get_logger().info('MPU6050 initialized successfully.')
        except Exception as e:
            self.i2c_available = False
            self.get_logger().warning(f'MPU6050 initialization failed: {e}')

    def read_raw_data(self, addr):
        try:
            high = self.bus.read_byte_data(self.config.device_address, addr)
            low = self.bus.read_byte_data(self.config.device_address, addr + 1)
            value = (high << 8) | low
            if value > 32767:
                value -= 65536
            return value
        except Exception as e:
            self.get_logger().warning(f'I2C read failed at 0x{addr:02X}: {e}')
            self.i2c_available = False
            return 0

    def timer_callback(self):
        if not self.i2c_available:
            self.initialize_sensor()
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        accel_offsets = [0, 2, 4]
        gyro_offsets = [0, 2, 4]
        axes = ['x', 'y', 'z']

        accel_values = []
        gyro_values = []

        for offset in accel_offsets:
            raw = self.read_raw_data(self.config.accel_xout_h + offset)
            accel_values.append(raw / self.config.accel_scale_modifier * self.config.gravity)

        for offset in gyro_offsets:
            raw = self.read_raw_data(self.config.gyro_xout_h + offset)
            gyro_values.append(math.radians(raw / self.config.gyro_scale_modifier))

        imu_msg.linear_acceleration.x = accel_values[0]
        imu_msg.linear_acceleration.y = accel_values[1]
        imu_msg.linear_acceleration.z = accel_values[2]
        imu_msg.angular_velocity.x = gyro_values[0]
        imu_msg.angular_velocity.y = gyro_values[1]
        imu_msg.angular_velocity.z = gyro_values[2]

        imu_msg.linear_acceleration_covariance = [
        self.config.accel_cov_diag[0], 0.0, 0.0,
        0.0, self.config.accel_cov_diag[1], 0.0,
        0.0, 0.0, self.config.accel_cov_diag[2]
        ]

        imu_msg.angular_velocity_covariance = [
        self.config.gyro_cov_diag[0], 0.0, 0.0,
        0.0, self.config.gyro_cov_diag[1], 0.0,
        0.0, 0.0, self.config.gyro_cov_diag[2]
        ]

        imu_msg.orientation_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.publisher_.publish(imu_msg)

        now = time.time()
        if now - self.last_log_time > 1.0:
            self.get_logger().info(
                f"Accel [m/s²]: x={accel_values[0]:.2f}, y={accel_values[1]:.2f}, z={accel_values[2]:.2f} | "
                f"Gyro [rad/s]: x={gyro_values[0]:.2f}, y={gyro_values[1]:.2f}, z={gyro_values[2]:.2f}"
            )
            self.last_log_time = now


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
