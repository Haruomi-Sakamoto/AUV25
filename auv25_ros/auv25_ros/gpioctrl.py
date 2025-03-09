import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import gpiod

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpioctrl_node')

        # GPIO 5 output
        self.gpio_pin = 4
        try:
            self.chip = gpiod.Chip('gpiochip4', gpiod.Chip.OPEN_BY_NAME)
            self.line = self.chip.get_line(self.gpio_pin)
            self.line.request(consumer="auv25", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            self.get_logger().info(f"GPIO {self.gpio_pin} initialized to LOW")

        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            raise

        # sonner_data subscription
        self.subscription = self.create_subscription(
            Float32,
            '/auv25/sonner_data',
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg):
        try:
            if msg.data >= 10:
                self.line.set_value(1)
            else:
                self.line.set_value(0)

            actual_value = self.line.get_value()
            state_str = "HIGH" if actual_value == 1 else "LOW"
            self.get_logger().info(f"GPIO {self.gpio_pin} {state_str} (sonner_data: {msg.data}, actual: {actual_value})")

        except Exception as e:
            self.get_logger().error(f"Error in sensor callback: {e}")

    def destroy_node(self):
        try:
            self.line.set_value(0)
            self.line.release()
            self.get_logger().info(f"GPIO {self.gpio_pin} released and set to LOW")
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

