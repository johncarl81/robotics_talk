from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    example_service = ExampleService()

    rclpy.spin(example_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
