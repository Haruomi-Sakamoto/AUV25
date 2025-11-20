import math

class TwistConfig:
    def __init__(self):
        self.linear = [0.0, 0.0, 0.0]
        self.angular = [0.0, 0.0, 0.0]

        self.scale_linear = 1.0   # 最大0.5 m/s
        self.scale_angular = 1.0  # 最大1.0 rad/s

class MPU6050Config:
    def __init__(self):
        self.i2c_bus_number = 1
        self.device_address = 0x68

        self.accel_scale_modifier = 16384.0  # ±2g
        self.gyro_scale_modifier = 131.0     # ±250 °/s
        self.gravity = 9.80665               # m/s²

        # dispersion of m/s^2
        self.accel_cov_diag = [0.1, 0.1, 0.1]

        # dispersion of rad/s
        self.gyro_cov_diag = [0.01, 0.01, 0.01]

        self.power_mgmt_1 = 0x6B
        self.gyro_config = 0x1B
        self.accel_config = 0x1C

        self.accel_xout_h = 0x3B
        self.gyro_xout_h = 0x43

class ThrusterConfig:
    def __init__(self):
        # [ surge, sway, heave, roll, pitch, yaw ]^T → [ thr1..thr6 ]^T
        r2 = 1 / math.sqrt(2)
        self.allocation_matrix = [
            [  0,   0,  1,  1,  0,  0],  # Thruster1
            [  0,   0,  1, -1,  0,  0],  # Thruster2
            [ r2, -r2,  0,  0,  0,  1],  # Thruster3
            [-r2, -r2,  0,  0,  0,  1],  # Thruster4
            [-r2,  r2,  0,  0,  0, -1],  # Thruster5
            [ r2,  r2,  0,  0,  0, -1],  # Thruster6
        ]
        # speed scale
        self.scale = 1.0   

class PCA9685Config:
    def __init__(self):
        self.pwmfreq = 50

        self.camera_channel = 3
        self.thruster_channel = [5,7,11,13,9,15]

        self.pwm_neutral_us = 1500   # stop
        self.pwm_range_us   = 400

        self.speed_scale = 0.5

        self.num_thrusters  = 6


        """
        1 1
        15 1
        1 11
        """