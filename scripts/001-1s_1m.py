#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from enum import Enum

from std_msgs.msg import String
from riptide_msgs.msg import Pressure
from riptide_msgs.action import Depth


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
        self.duration_1m = 5

        # Pressure monitoring
        self.pressure = 1013
        self.p_min = 1050
        self.p_max = 1350
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

        # Update loop
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.loop)

    def pressure_callback(self, msg):
        self.pressure = msg.pressure
        self.get_logger().info(f'Reading pressure {self.pressure}')

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

    def loop(self):
        if self.p_min <= self.pressure <= self.p_max:
            if self.state == State.IDLE and self.p_min <= self.pressure:
                # Call 1 m action
                self.get_logger().info("Calling Action 1 m")
                self.send_goal(1.)
                self.flag = False
                self.state = State.ACTION1M
            elif self.state == State.ACTION1M and self.flag:
                self.get_logger().info("State Wait")
                self.t0 = time.time()
                self.state = State.WAIT
                self.flag = False
            elif self.state == State.WAIT and time.time() - self.t0 > self.duration_1m:
                # Call 0 m action
                self.get_logger().info("Calling Action 0 m")
                self.send_goal(0.)
                self.state = State.ACTION0M
            elif self.state == State.ACTION0M and self.flag:
                self.get_logger().info("State IDLE")
                self.state == State.IDLE
            
        else:
            self.get_logger().info("State IDLE")
            self.send_goal(0.)
            self.state = State.IDLE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Mission()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()