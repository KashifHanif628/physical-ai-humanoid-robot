import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):

    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_world',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    hello_world_subscriber = HelloWorldSubscriber()
    try:
        rclpy.spin(hello_world_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        hello_world_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()