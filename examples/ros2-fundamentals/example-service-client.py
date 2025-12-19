from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class ServiceClientExample(Node):

    def __init__(self):
        super().__init__('service_client_example')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future


def main(args=None):
    rclpy.init(args=args)
    service_client_example = ServiceClientExample()

    # Send a request
    future = service_client_example.send_request(4, 5)

    try:
        rclpy.spin_until_future_complete(service_client_example, future)
        response = future.result()
        if response is not None:
            service_client_example.get_logger().info(f'Result: {response.sum}')
        else:
            service_client_example.get_logger().error('Service call failed')
    except KeyboardInterrupt:
        service_client_example.get_logger().info('Interrupted by user')
    finally:
        service_client_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()