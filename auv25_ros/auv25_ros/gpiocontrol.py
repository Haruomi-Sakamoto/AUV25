import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import gpiod

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpiocontrol_node')

        # GPIO 4 output
        self.gpio_pin = 4
        try:
            self.chip = gpiod.Chip('gpiochip0', gpiod.Chip.OPEN_BY_NAME)
            self.line = self.chip.get_line(self.gpio_pin)
            self.line.request(consumer="sensor_control", type=gpiod.LINE_REQ_DIR_OUT)
        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            raise

        # sensor_data subscription
        self.subscription = self.create_subscription(
            Float32,
            '/auv25/sensor_data',
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg):
        try:
            if msg.data >= 10:
                self.line.set_value(1)
                self.get_logger().info(f"GPIO {self.gpio_pin} HIGH (sensor_data: {msg.data})")
            else:
                self.line.set_value(0)
                self.get_logger().info(f"GPIO {self.gpio_pin} LOW (sensor_data: {msg.data})")
        except Exception as e:
            self.get_logger().error(f"Error in sensor callback: {e}")

    def destroy_node(self):
        try:
            self.line.release()
        except Exception as e:
            self.get_logger().error(f"Error releasing GPIO: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

