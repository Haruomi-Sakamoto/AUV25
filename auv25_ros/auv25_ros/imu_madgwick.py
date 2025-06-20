import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ahrs.filters import Madgwick
import numpy as np
from geometry_msgs.msg import Quaternion
from auv25_ros.config import MadgwickConfig

class ImuMadgwickNode(Node):
    def __init__(self):
        super().__init__('imu_madgwick_node')
        self.config = MadgwickConfig()
        self.sub = self.create_subscription(Imu, 'imu/data_raw', self.callback, 10)
        self.pub = self.create_publisher(Imu, 'imu/data_orientation', 10)
        self.filter = Madgwick(sampleperiod=1.0 / self.config.update_rate)
        self.q = np.array(self.config.initial_orientation)
        self.last_time = None

    def callback(self, msg: Imu):
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]) / self.config.gravity_value

        gyr = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        new_q = self.filter.updateIMU(self.q, acc=acc, gyr=gyr)
        if new_q is not None:
            self.q = new_q

        out_msg = Imu()
        out_msg.header = msg.header
        out_msg.linear_acceleration = msg.linear_acceleration
        out_msg.angular_velocity = msg.angular_velocity
        out_msg.orientation = Quaternion(x=self.q[1], y=self.q[2], z=self.q[3], w=self.q[0])

        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuMadgwickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
