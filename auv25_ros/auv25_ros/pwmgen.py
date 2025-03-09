import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import board
import busio
from adafruit_pca9685 import PCA9685
import random  # for generating test data

class pwmgenNode(Node):
    def __init__(self):
        super().__init__('pwmgen_node')
        self.timer = self.create_timer(1.0, self.timer_callback)

        # I2Cの設定
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pwm = PCA9685(i2c)
        self.pwm.frequency = 50  # PWM周波数を50Hzに設定

    def timer_callback(self):
        # 204〜409のランダムな値を取得
        sonner_data = float(random.randint(2400, 4800)) 
        self.pwm.channels[15].duty_cycle = int(sonner_data)

def main(args=None):
    rclpy.init(args=args)
    node = pwmgenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

