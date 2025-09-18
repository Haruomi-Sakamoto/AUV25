import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import board
import busio
from adafruit_pca9685 import PCA9685

from auv25_ros.config import ThrusterConfig

class PWMGenerateNode(Node):
    def __init__(self):
        super().__init__('pwmgen_node')
        self.cfg = ThrusterConfig()

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pwm = PCA9685(i2c)
        self.pwm.frequency = 50  # 50Hz

        self.create_subscription(Float64MultiArray,'/thruster_output',self.thruster_callback,10)

        self.get_logger().info('PWMThrusterNode started')

    def thruster_callback(self, msg: Float64MultiArray):
        #-1.0 ~ 1.0 to PWM
        data = msg.data
        if len(data) < self.cfg.num_thrusters:
            self.get_logger().warn('thruster_output length is not 6')
            return

        for i in range(self.cfg.num_thrusters):
            cmd = max(min(data[i], 1.0), -1.0)

            pulse_us = self.cfg.pwm_neutral_us + cmd * self.cfg.pwm_range_us

            duty = int((pulse_us / 20000.0) * 0xFFFF)
            self.pwm.channels[i].duty_cycle = duty

        self.get_logger().debug(f"PWM output: {[round(x,2) for x in data]}")

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
