import rclpy
from rclpy.node import Node

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)

    example_subscriber = ExampleSubscriber()

    rclpy.spin(example_subscriber)

    example_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
