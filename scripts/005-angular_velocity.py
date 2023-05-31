#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import LoadController

class Mission(Node):

    def __init__(self):
        super().__init__('angular_velocity')
        self.velocity = 0.05
        self.angular = [1., 0., 0.]

        # Creating the client
        self.controller_manager_service = '/riptide_1/controller_manager'
        self.cli = self.create_client(LoadController, self.controller_manager_service)

        # Waiting for the controller_manager
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info(f'Controller manager not available at {self.controller_manager_service}')

        # Loading riptide_controller
        self.load_riptide_controller()

        # Twist publisher
        self.get_logger().info("Waiting for angular velocity on `~/riptide_controller/cmd_vel`")
        self.publisher_ = self.create_publisher(Twist, '/riptide_1//riptide_controller/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_riptide_controller(self):
        req = LoadController()
        req.name = "riptide_controller"
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result.ok:
            self.get_logger().fatal("Controller manager is not able to load `riptide_controller`")
            rclpy.shutdown()

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.x = self.angular[0]
        msg.angular.y = self.angular[1]
        msg.angular.z = self.angular[2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()