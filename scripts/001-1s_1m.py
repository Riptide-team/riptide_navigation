#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from enum import Enum

from std_msgs.msg import String
from riptide_msgs.msg import Pressure
from riptide_msgs.action import Depth

from controller_manager_msgs.srv import SwitchController


class State(Enum):
    IDLE     = 0
    ACTION1M = 1
    ACTION0M = 2
    WAIT     = 3


class Mission(Node):

    def __init__(self):
        super().__init__('mission_1s_1m')
        self.state = State.IDLE

        self.t0 = time.time()
        self.duration_1m = 15

        # Pressure monitoring
        self.depth = 0
        self.d_min = 0.5
        self.d_max = 3
        self.subscription = self.create_subscription(
            Pressure,
            '/riptide_1/pressure_broadcaster/pressure_status',
            self.pressure_callback,
            10
        )

        # Action client
        self._action_client = ActionClient(self, Depth, '/riptide_1/depth')

        # Action flag
        self.flag = False

        # Controller manager
        self.switch_controller_srv = self.create_client(SwitchController, '/riptide_1/controller_manager')

        # Update loop
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop)

    def pressure_callback(self, msg):
        self.depth = msg.depth
        self.get_logger().info(f'Reading pressure {self.depth}')

    def send_goal(self, order):
        goal_msg = Depth.Goal()
        goal_msg.requested_depth = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Final depth: {0}'.format(result.depth))
        self.flag = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received error: {0}'.format(feedback.error))

    def reset_actuators(self):
        self.req = SwitchController()
        self.req.activate_controllers = ["actuators_reset"]
        self.req.deactivate_controllers = ["depth_controller"]
        self.req.strictness = SwitchController.BEST_EFFORT
        self.req.activate_asap = True
        self.future = self.switch_controller_srv.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        result = self.future.result()
        self.get_logger().info(f"Switching controllers {result.ok}")

    def loop(self):
        if self.d_min <= self.depth <= self.d_max:
            if self.state == State.IDLE:
                # Call 1 m action
                self.send_goal(1.)
                self.flag = False
                self.state = State.ACTION1M
                self.get_logger().info("Calling Action 1 m")
            elif self.state == State.ACTION1M and self.flag:
                self.flag = False
                self.t0 = time.time()
                self.state = State.WAIT
                self.get_logger().info("State Wait")
            elif self.state == State.WAIT and (time.time() - self.t0) > self.duration_1m:
                # Call 0 m action
                self.send_goal(0.)
                self.flag = False
                self.state = State.ACTION0M
                self.get_logger().info("Calling Action 0 m")
            elif self.state == State.ACTION0M and self.flag:
                self.state = State.IDLE
                self.reset_actuators()
                self.get_logger().info("State IDLE")
        else:
            self.state = State.IDLE
            self.reset_actuators()
            self.get_logger().info("State IDLE")


def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()