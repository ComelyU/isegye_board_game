import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'display', 10)
        self.get_logger().info('Minimal Publisher Node Initialized')

    def publish_string(self, user_input):
        msg = String()
        msg.data = user_input
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    while rclpy.ok():
        user_input = input("Enter string to publish ('q' to quit): ")
        if user_input.lower() == 'q':
            break
        minimal_publisher.publish_string(user_input)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
