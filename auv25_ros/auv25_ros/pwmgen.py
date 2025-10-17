import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import board
import busio
from adafruit_pca9685 import PCA9685

from auv25_ros.config import PCA9685Config


class PWMGenerateNode(Node):
    def __init__(self):
        super().__init__('pwmgen_node')
        self.cfg = PCA9685Config()

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pwm = PCA9685(i2c)
        self.pwm.frequency = self.cfg.pwmfreq

        self.create_subscription(Float64MultiArray, '/thruster_output', self.thruster_callback, 10)
        self.pub_pulse = self.create_publisher(Float64MultiArray, '/thruster_pulse_us', 10)

        self.get_logger().info('PWMThrusterNode started')

    def thruster_callback(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) < self.cfg.num_thrusters:
            self.get_logger().warn('thruster_output length is not sufficient')
            return

        pulse_list = []
        pulse_length_us = 1_000_000 / self.pwm.frequency / 4096

        for i in range(self.cfg.num_thrusters):
            pulse_us = self.cfg.pwm_neutral_us + data[i] * self.cfg.pwm_range_us * self.cfg.speed_scale + 90
            pulse_count = int(pulse_us / pulse_length_us)
            duty = int((pulse_count / 4096) * 65535)

            self.pwm.channels[self.cfg.thruster_channel[i]].duty_cycle = duty
            pulse_list.append(pulse_us)

        msg_pulse = Float64MultiArray()
        msg_pulse.data = pulse_list
        self.pub_pulse.publish(msg_pulse)

        self.get_logger().debug(f"Pulse_us: {[round(p, 1) for p in pulse_list]}")

def main(args=None):
    rclpy.init(args=args)
    node = PWMGenerateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
