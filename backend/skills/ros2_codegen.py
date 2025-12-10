def generate_node(name: str, topic: str, node_type: str) -> str:
    """
    Goal: Secure Bonus Points for Reusable Intelligence.
    Function: generate_node(name: str, topic: str, node_type: str)
    Logic:
      - Generate a valid Python string containing a full ROS 2 Node using rclpy.
      - Include imports, class definition inheriting from Node, __init__, and main entry point.
      - If node_type is "publisher", create a timer and publisher.
      - If node_type is "subscriber", create a subscription callback.
    """
    if node_type.lower() == "publisher":
        node_code = f'''import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class {name.title()}Node(Node):

    def __init__(self):
        super().__init__('{name.lower()}_node')
        self.publisher_ = self.create_publisher(String, '{topic}', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {{self.i}}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{{msg.data}}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = {name.title()}Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    elif node_type.lower() == "subscriber":
        node_code = f'''import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class {name.title()}Node(Node):

    def __init__(self):
        super().__init__('{name.lower()}_node')
        self.subscription = self.create_subscription(
            String,
            '{topic}',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{{msg.data}}"')


def main(args=None):
    rclpy.init(args=args)
    node = {name.title()}Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    else:
        # Default to a basic node if type is not recognized
        node_code = f'''import rclpy
from rclpy.node import Node


class {name.title()}Node(Node):

    def __init__(self):
        super().__init__('{name.lower()}_node')
        self.get_logger().info('{name.title()} node has been started')


def main(args=None):
    rclpy.init(args=args)
    node = {name.title()}Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

    return node_code