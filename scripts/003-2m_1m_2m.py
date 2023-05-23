#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
import time
from enum import Enum

from std_msgs.msg import String
from riptide_msgs.msg import Pressure
from riptide_msgs.msg import Multiplexer
from riptide_msgs.action import Depth


class State(Enum):
    IDLE      = 0
    ACTION2M1 = 1
    ACTION1M  = 2
    ACTION2M2 = 3
    ACTION0M  = 4
    END       = 5
    FAILSAFE  = 6


class Mission(Node):

    def __init__(self):
        super().__init__('mission_2m_1m_2m')
        self.state = State.IDLE

        # Duration of each state
        self.duration = 10.0

        # Pressure monitoring
        self.d_max = 4
        self.subscription = self.create_subscription(
            Pressure,
            '/riptide_1/pressure_broadcaster/pressure_status',
            self.pressure_callback,
            10
        )

        # State publisher
        self.state_publisher = self.create_publisher(String, "~/mission/state", 10)

        # Action client
        self._action_client = ActionClient(self, Depth, '/riptide_1/depth')
        
        self._send_goal_future = Future()
        self._get_result_future = Future()

        # Multiplexer subscriber
        self.multiplexer_subscriber = self.create_subscription(
            Multiplexer,
            '/riptide_1/tail_broadcaster/multiplexer_status',
            self.multiplexer_callback,
            10
        )

    def pressure_callback(self, msg):
        if self.state != State.FAILSAFE and msg.depth > self.d_max:
            self.get_logger().warn(f'Current pressure {msg.depth} > d_max = {self.d_max}: Aborting!')
            self.state = State.FAILSAFE
            self.execute_fsm()

    def multiplexer_callback(self, msg):
        if self.state == State.IDLE and msg.automatic and msg.remaining_time > 5:
            self.get_logger().info(f"Launching the mission! RH is the chef for the next {msg.remaining_time}")
            self.execute_fsm()

    def send_goal(self, depth, duration):
        goal_msg = Depth.Goal()
        goal_msg.requested_depth = depth
        goal_msg.duration = duration

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Checking if the goal is accepted or rejected
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return
        self.get_logger().info('Goal accepted!')

        # Waiting for the action's result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Getting the result
        result = future.result().result
        self.get_logger().info(f'Final depth: {result.depth}m, {result.elapsed_time}s')

        # Executing next fsm state
        self.execute_fsm()

    def feedback_callback(self, feedback_msg):
        # Getting feedback
        feedback = feedback_msg.feedback
        self.get_logger().info('Received error: {0}'.format(feedback.depth_error))

    def execute_fsm(self):
        msg = String()
        if self.state == State.FAILSAFE:
            # Canceling the current action
            self._get_result_future.cancel()

            # Calling action 0 m during 30 seconds
            self.send_goal(self, 0, 30)
            
            # Current state
            msg.data = "FailSafe"
            self.state = State.FAILSAFE

        elif self.state == State.IDLE:
            # Call 2 m action
            self.send_goal(2., self.duration)

            # Current state
            msg.data = "Action 2m"
            self.state = State.ACTION2M1
            self.get_logger().info("Calling Action 2m")

        elif self.state == State.ACTION2M1:
            # Call 1 m action
            self.send_goal(1., self.duration)

            # Current state
            msg.data = "Action 1m"
            self.state = State.ACTION1M
            self.get_logger().info("Calling Action 1m")

        elif self.state == State.ACTION1M:
            # Call 2 m action
            self.send_goal(2., self.duration)

            # Current state
            msg.data = "Action 2m"
            self.state = State.ACTION2M2
            self.get_logger().info("Calling Action 2m")

        elif self.state == State.ACTION2M2:
            # Call 0 m action
            self.send_goal(0., self.duration)

            # Current state
            msg.data = "Action 0m"
            self.state = State.ACTION0M
            self.get_logger().info("Calling Action 0m")

        elif self.state == State.ACTION0M:
            # Current state
            self.state = State.END
            msg.data = "END"
            self.get_logger().info("End of the mission")

        # Publishing the current state
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()