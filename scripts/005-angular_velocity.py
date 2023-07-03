#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController

class Mission(Node):

    def __init__(self):
        super().__init__('angular_velocity')
        self.velocity = 1.  
        self.angular = [0., 0., .1]

        self.get_logger().info(f"Control Twist {self.angular}")

        # Creating the controller loader
        self.controller_manager_service = '/riptide_1/controller_manager/load_controller'
        self.load_controller = self.create_client(LoadController, self.controller_manager_service)

        # Waiting for the controller_manager loader service
        while not self.load_controller.wait_for_service(timeout_sec = 10.0):
            self.get_logger().info(f'Controller manager not available at {self.controller_manager_service}')

        # Loading riptide_controller
        self.load_riptide_controller()

        # Creating the controller configurator
        self.controller_manager_service = '/riptide_1/controller_manager/configure_controller'
        self.configure_controller = self.create_client(ConfigureController, self.controller_manager_service)

        # Waiting for the controller_manager configure service
        while not self.configure_controller.wait_for_service(timeout_sec = 10.0):
            self.get_logger().info(f'Controller manager not available at {self.controller_manager_service}')

        # Configuring riptide_controller
        self.configure_riptide_controller()

        # Creating the controller activator
        self.controller_manager_service = '/riptide_1/controller_manager/switch_controller'
        self.switch_controller = self.create_client(SwitchController, self.controller_manager_service)

        # Waiting for the controller_manager configure service
        while not self.switch_controller.wait_for_service(timeout_sec = 10.0):
            self.get_logger().info(f'Controller manager not available at {self.controller_manager_service}')

        # Confiuring riptide_controller
        self.switch_riptide_controller()

        # Twist publisher
        self.get_logger().info("Publishing Twist on `/riptide_1/riptide_controller/cmd_vel`")
        self.publisher_ = self.create_publisher(Twist, '/riptide_1/riptide_controller/cmd_vel', 10)
        timer_period = 1.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_riptide_controller(self):
        req = LoadController.Request()
        req.name = "riptide_controller"
        self.future = self.load_controller.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result().ok:
            self.get_logger().fatal("Controller manager is not able to load `riptide_controller`")
            rclpy.shutdown()

    def configure_riptide_controller(self):
        req = ConfigureController.Request()
        req.name = "riptide_controller"
        self.future = self.configure_controller.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result().ok:
            self.get_logger().fatal("Controller manager is not able to load `riptide_controller`")
            rclpy.shutdown()
    
    def switch_riptide_controller(self):
        req = SwitchController.Request()
        req.activate_controllers = ["riptide_controller"]
        req.activate_asap = True
        req.strictness = SwitchController.Request().BEST_EFFORT
        self.future = self.switch_controller.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result().ok:
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
