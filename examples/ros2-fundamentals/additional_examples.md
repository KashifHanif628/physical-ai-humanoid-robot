# Additional Python Node Examples

## Timer-Based Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimerNode(Node):

    def __init__(self):
        super().__init__('timer_node')
        self.publisher_ = self.create_publisher(String, 'timer_topic', 10)
        # Create a timer that triggers every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Timer message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    timer_node = TimerNode()
    try:
        rclpy.spin(timer_node)
    except KeyboardInterrupt:
        pass
    finally:
        timer_node.destroy_node()
        rclpy.shutdown()
```

## Multi-Publisher Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MultiPublisherNode(Node):

    def __init__(self):
        super().__init__('multi_publisher_node')
        self.string_publisher = self.create_publisher(String, 'string_topic', 10)
        self.int_publisher = self.create_publisher(Int32, 'int_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Publish string message
        str_msg = String()
        str_msg.data = f'Multi-publisher message {self.counter}'
        self.string_publisher.publish(str_msg)

        # Publish integer message
        int_msg = Int32()
        int_msg.data = self.counter
        self.int_publisher.publish(int_msg)

        self.get_logger().info(f'Published: "{str_msg.data}" and {int_msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    multi_publisher_node = MultiPublisherNode()
    try:
        rclpy.spin(multi_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        multi_publisher_node.destroy_node()
        rclpy.shutdown()
```

## Parameter-Based Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('message_prefix', 'ParameterNode')
        self.declare_parameter('max_count', 10)

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.message_prefix = self.get_parameter('message_prefix').value
        self.max_count = self.get_parameter('max_count').value

        self.publisher_ = self.create_publisher(String, 'parameter_topic', 10)
        self.timer = self.create_timer(self.publish_rate, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        if self.count >= self.max_count:
            self.get_logger().info('Reached maximum count, stopping...')
            self.timer.cancel()
            return

        msg = String()
        msg.data = f'{self.message_prefix}: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()
```