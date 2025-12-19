import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParameterExampleNode(Node):

    def __init__(self):
        super().__init__('parameter_example_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('message_prefix', 'ParameterExample')
        self.declare_parameter('max_count', 10)
        self.declare_parameter('topic_name', 'parameter_topic')

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.message_prefix = self.get_parameter('message_prefix').value
        self.max_count = self.get_parameter('max_count').value
        self.topic_name = self.get_parameter('topic_name').value

        # Validate parameters
        if self.publish_rate <= 0:
            self.get_logger().fatal('Publish rate must be positive')
            raise ValueError('Invalid publish_rate parameter')

        if not self.topic_name or not isinstance(self.topic_name, str):
            self.get_logger().fatal('Invalid topic name')
            raise ValueError('Invalid topic_name parameter')

        # Create publisher with dynamic topic name
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        self.timer = self.create_timer(self.publish_rate, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        if self.count >= self.max_count:
            self.get_logger().info('Reached maximum count, stopping...')
            self.timer.cancel()
            return

        msg = String()
        msg.data = f'{self.message_prefix}: Count {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    parameter_example_node = ParameterExampleNode()
    try:
        rclpy.spin(parameter_example_node)
    except KeyboardInterrupt:
        parameter_example_node.get_logger().info('Interrupted by user')
    finally:
        parameter_example_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()