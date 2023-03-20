import rclpy
from rclpy.node import Node

from std_msgs.msg import String




def main(args=None):
    rclpy.init(args=args)

    example_publisher = ExamplePublisher()

    rclpy.spin(example_publisher)

    example_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
