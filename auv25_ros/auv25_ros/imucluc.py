#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import time


class ImuCulcNode(Node):

    def __init__(self):
        super().__init__('imu_pose_node')

        # Subscriber
        self.subscription = self.create_subscription(
            Imu,
            'imu/raw',
            self.imu_callback,
            10
        )

        # Publisher (姿勢 + 位置)
        self.pose_pub = self.create_publisher(Vector3, 'imu/position', 10)
        self.yaw_pub = self.create_publisher(Vector3, 'imu/orientation', 10)

        # 状態変数 -----------------------------------------
        self.last_time = None

        # 姿勢（ラジアン）
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # 加速度積分 → 速度
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # 速度積分 → 位置
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # ---------------------------------------------------

        self.get_logger().info("IMU Pose Node Started.")

    def imu_callback(self, msg: Imu):

        # 時間計算（積分に必要）
        now = time.time()
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now

        # ------------------------------
        # 1. ジャイロを積分して姿勢計算
        # ------------------------------
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        self.roll  += gx * dt
        self.pitch += gy * dt
        self.yaw   += gz * dt

        # ------------------------------
        # 2. 加速度から位置を計算
        # ------------------------------
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # 加速度を積分 → 速度
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        # 速度を積分 → 位置
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # ------------------------------
        # 3. Publish 姿勢
        # ------------------------------
        ori_msg = Vector3()
        ori_msg.x = self.roll
        ori_msg.y = self.pitch
        ori_msg.z = self.yaw
        self.yaw_pub.publish(ori_msg)

        # ------------------------------
        # 4. Publish 位置
        # ------------------------------
        pos_msg = Vector3()
        pos_msg.x = self.x
        pos_msg.y = self.y
        pos_msg.z = self.z
        self.pose_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCulcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
