from pca9685.models.config import PCA9685Config
from pca9685.services.pca9685 import PCA9685
import rclpy  # Import ROS 2 Python client library
from rclpy.node import Node  # Import the Node class
from std_msgs.msg import (
    Int32MultiArray,
)  # Import standard message type for two integers (channel, pulse)


class PCA9685SubscriberNode(Node):
    def __init__(self):
        super().__init__("pca9685_node")

        # Declare parameters with default values
        self.declare_parameter("bus", 1)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("frequency", 60)

        # Retrieve parameter values
        bus = self.get_parameter("bus").get_parameter_value().integer_value
        address = self.get_parameter("address").get_parameter_value().integer_value
        frequency = self.get_parameter("frequency").get_parameter_value().integer_value

        config = PCA9685Config(busnum=bus, address=address, frequency=frequency)
        self.pca9685 = PCA9685(config=config)

        # Create subscriber to listen on the `/pwm_command` topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/pwm_command",
            self.listener_callback,
            10,  # Queue size
        )

    def listener_callback(self, msg: Int32MultiArray):
        # Assuming msg.data contains [channel, pulse]
        if len(msg.data) == 2:
            channel = msg.data[0]
            pulse = msg.data[1]
            self.pca9685.set_pulse(channel, pulse)
        else:
            self.get_logger().error("Received message with unexpected length")


def main(args=None):
    rclpy.init(args=args)

    node = PCA9685SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
