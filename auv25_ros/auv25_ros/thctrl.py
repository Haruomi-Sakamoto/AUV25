#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

from auv25_ros.config import ThrusterConfig


class ThrusterControlNode(Node):
    def __init__(self):
        super().__init__('thruster_control_node')

        self.cfg = ThrusterConfig()

        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/thruster_output', 10)


        self.max_acc = 0.1
        self.max_dec = 0.1
        self.current_output = np.zeros(self.cfg.num_thrusters, dtype=float)

        self.get_logger().info('ThrusterControlNode + Trapezoid started')


    def trapezoid_filter(self, target_output):
        """
        target_output: ndarray (num_thrusters)
        return: filtered ndarray
        """
        filtered = np.copy(self.current_output)

        for i in range(len(target_output)):
            target = target_output[i]
            current = self.current_output[i]

            if target > current:
                current += min(self.max_acc, target - current)
            else:
                current -= min(self.max_dec, current - target)

            filtered[i] = current

        self.current_output = filtered
        return filtered

    # ------------------------------------------------------
    # cmd_vel → thruster の計算
    # ------------------------------------------------------
    def twist_callback(self, msg: Twist):
        # 入力ベクトル
        vel_vec = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ], dtype=float)

        allocation_matrix = np.array(self.cfg.allocation_matrix, dtype=float)
        target_output = self.cfg.scale * allocation_matrix @ vel_vec  # ndarray

        smoothed_output = self.trapezoid_filter(target_output)

        out_msg = Float64MultiArray()
        out_msg.data = smoothed_output.tolist()
        self.pub.publish(out_msg)

        self.get_logger().debug(
            f"Target: {target_output.tolist()}  →  Smoothed: {smoothed_output.tolist()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
