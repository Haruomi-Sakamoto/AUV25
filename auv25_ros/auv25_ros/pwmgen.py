import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

import board
import busio
from adafruit_pca9685 import PCA9685

from auv25_ros.config import PCA9685Config


class PWMGenerateNode(Node):
    def __init__(self):
        super().__init__('pwmgen_node')
        self.cfg = PCA9685Config()

        # PCA9685 初期化
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pwm = PCA9685(i2c)
        self.pwm.frequency = self.cfg.pwmfreq

        # サーボ角度（初期値）
        self.camera_angle = 90.0

        # Thruster 購読
        self.create_subscription(Float64MultiArray, '/thruster_output',
                                 self.thruster_callback, 10)

        # Joy 購読
        self.create_subscription(Joy, '/remote_pc/joy',
                                 self.joy_callback, 10)

        # パルス監視用パブリッシャ
        self.pub_pulse = self.create_publisher(Float64MultiArray,
                                               '/thruster_pulse_us', 10)

        # サーボ更新タイマー（ゆっくり動かす）
        self.create_timer(0.02, self.update_camera_servo)  # 20ms周期

        # ボタン状態
        self.btn1 = False
        self.btn3 = False

        self.get_logger().info('PWMThrusterNode started')

    # -----------------------------
    #   Joy コールバック
    # -----------------------------
    def joy_callback(self, joy: Joy):
        # ボタン配列: joy.buttons[1], joy.buttons[3]
        try:
            self.btn1 = joy.buttons[1] == 1
            self.btn3 = joy.buttons[3] == 1
        except IndexError:
            return

    def update_camera_servo(self):
        delta = 1.0  # 1度ずつ動かす

        if self.btn3:      # 正転 → 角度を減らす
            self.camera_angle -= delta
        if self.btn1:      # 逆転 → 角度を増やす
            self.camera_angle += delta

        # 制限
        self.camera_angle = max(0, min(180, self.camera_angle))

        # パルスへ変換
        pulse = self.angle_to_us(self.camera_angle)

        # PCA9685へ書き込み
        self.write_servo_us(self.cfg.camera_channel, pulse)

    # -----------------------------
    #   角度 → パルス幅[us] 変換
    # -----------------------------
    def angle_to_us(self, angle_deg):
        # SG90 は 0°=500us / 180°=2500us 付近
        return 500 + (2000.0 * angle_deg / 180.0)

    # -----------------------------
    #   PCA9685 出力
    # -----------------------------
    def write_servo_us(self, channel, pulse_us):
        pulse_length_us = 1_000_000 / self.pwm.frequency / 4096
        pulse_count = int(pulse_us / pulse_length_us)
        duty = int((pulse_count / 4096) * 65535)
        self.pwm.channels[channel].duty_cycle = duty

    # -----------------------------
    #   Thruster コールバック
    # -----------------------------
    def thruster_callback(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) < self.cfg.num_thrusters:
            self.get_logger().warn('thruster_output length insufficient')
            return

        pulse_list = []
        pulse_length_us = 1_000_000 / self.pwm.frequency / 4096

        for i in range(self.cfg.num_thrusters):
            pulse_us = (
                self.cfg.pwm_neutral_us +
                data[i] * self.cfg.pwm_range_us * self.cfg.speed_scale +
                90
            )
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
