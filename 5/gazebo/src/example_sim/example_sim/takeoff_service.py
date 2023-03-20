from example_interfaces.srv import Trigger
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSetV2

import rclpy
from rclpy.node import Node
import time


class TakeoffService(Node):

    def __init__(self):
        super().__init__('takeoff_service')
        self.id = "drone1"
        self.srv = self.create_service(Trigger, '/takeoff', self.trigger)
        self.setmode_service = self.create_client(SetMode, "/{}/mavros/set_mode".format(self.id))
        self.arm_service = self.create_client(CommandBool, "/{}/mavros/cmd/arming".format(self.id))
        self.takeoff_service = self.create_client(CommandTOL, "/{}/mavros/cmd/takeoff".format(self.id))
        self.land_service = self.create_client(CommandTOL, "/{}/mavros/cmd/land".format(self.id))

    def trigger(self, request, response):
        response.success = True
        response.message = "And we're off!"

        future = self.setmode_service.call_async(SetMode.Request(custom_mode='GUIDED'))

        def mode_finished(msg):
            result = future.result()
            if result.mode_sent:
                self.get_logger().info("Setmode success")
                self.arm()
            else:
                self.get_logger().info("Setmode failed")

        future.add_done_callback(mode_finished)

        return response

    def arm(self):
        self.get_logger().info("ARMING...")
        future = self.arm_service.call_async(CommandBool.Request(value=True))
        self.get_logger().info("ARMING finished...")

        def arm_finished(msg):
            result = future.result()
            if result and result.success:
                self.get_logger().info("Arming success")
                self.takeoff()

            else:
                self.get_logger().info("Arming failed")

        future.add_done_callback(arm_finished)

    def takeoff(self):
        future = self.takeoff_service.call_async(CommandTOL.Request(altitude=1.0))

        def takeoff_finished(msg):
            result = future.result()
            if result and result.success:
                time.sleep(10)
                self.land()
            else:
                self.get_logger().info("Takeoff failed")

        future.add_done_callback(takeoff_finished)

    def land(self):
        future = self.land_service.call_async(CommandTOL.Request(altitude=1.0))

        def landing_finished(msg):
            result = future.result()
            if result and result.success:
                self.get_logger().info("Landing success")
            else:
                self.get_logger().info("Landing failed")

        future.add_done_callback(landing_finished)


def main(args=None):
    rclpy.init(args=args)

    takeoff_service = TakeoffService()

    rclpy.spin(takeoff_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
