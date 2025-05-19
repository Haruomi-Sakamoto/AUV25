import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque

class ImuPlotNode(Node):
    def __init__(self):
        super().__init__('imu_plot_node')
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.callback, 10)

        # 最新100サンプル保持
        self.ax = deque(maxlen=100)
        self.ay = deque(maxlen=100)
        self.az = deque(maxlen=100)

        plt.ion()
        self.fig, self.ax_plot = plt.subplots()
        self.line_x, = self.ax_plot.plot([], [], label='Accel X')
        self.line_y, = self.ax_plot.plot([], [], label='Accel Y')
        self.line_z, = self.ax_plot.plot([], [], label='Accel Z')
        self.ax_plot.legend()
        self.ax_plot.set_ylim(-20, 20)
        self.timer = self.create_timer(0.1, self.update_plot)

    def callback(self, msg):
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)

    def update_plot(self):
        x = range(len(self.ax))
        self.line_x.set_data(x, list(self.ax))
        self.line_y.set_data(x, list(self.ay))
        self.line_z.set_data(x, list(self.az))
        self.ax_plot.set_xlim(0, max(100, len(self.ax)))
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = ImuPlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
