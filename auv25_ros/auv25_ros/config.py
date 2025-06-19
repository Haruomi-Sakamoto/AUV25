class TwistConfig:
    def __init__(self):
        self.linear = [0.0, 0.0, 0.0]
        self.angular = [0.0, 0.0, 0.0]

        self.scale_linear = 1.0
        self.scale_angular = 1.0

class MPU6050Config:
    def __init__(self):
        self.i2c_bus_number = 1
        self.device_address = 0x68

        self.accel_scale_modifier = 16384.0  # ±2g
        self.gyro_scale_modifier = 131.0     # ±250 °/s
        self.gravity = 9.80665               # m/s²

        # 加速度共分散の対角成分（m/s^2 の分散）
        self.accel_cov_diag = [0.1, 0.1, 0.1]

        # 角速度共分散の対角成分（rad/s の分散）
        self.gyro_cov_diag = [0.01, 0.01, 0.01]

        self.power_mgmt_1 = 0x6B
        self.gyro_config = 0x1B
        self.accel_config = 0x1C

        self.accel_xout_h = 0x3B
        self.gyro_xout_h = 0x43
