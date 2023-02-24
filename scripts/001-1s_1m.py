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
    IDLE      = 0
    ACTION2M1 = 1
    ACTION1M  = 2
    ACTION2M2 = 3
    ACTION0M  = 4
    END       = 5


class Mission(Node):

    def __init__(self):
        super().__init__('mission_1s_1m')
        self.state = State.IDLE

        # self.t0 = time.time()
        self.duration = 20.0

        # Pressure monitoring
        self.depth = 0
        self.d_min = 0.25
        self.d_max = 3
        self.subscription = self.create_subscription(
            Pressure,
            '/riptide_1/pressure_broadcaster/pressure_status',
            self.pressure_callback,
            10
        )

        self.state_publisher = self.create_publisher(String, "~/mission/state", 10)

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
        # self.get_logger().info(f'Reading pressure {self.depth}')

    def send_goal(self, depth, duration):
        goal_msg = Depth.Goal()
        goal_msg.requested_depth = depth
        goal_msg.duration = duration

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
        self.get_logger().info(f'Final depth: {result.depth}m, {result.elapsed_time}s')

        self.get_logger().info("PUTTING FLAG TO TRUE")
        self.flag = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received error: {0}'.format(feedback.depth_error))

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
        msg = String()
        if self.d_min <= self.depth <= self.d_max and self.state!=State.END:
            if self.state == State.IDLE:
                # Call 2 m action
                self.send_goal(2., self.duration)
                self.flag = False
                self.state = State.ACTION2M1
                msg.data = "Action 2m"
                self.get_logger().info("Calling Action 2m")
            elif self.flag and self.state == State.ACTION2M1:
                # Call 1 m action
                self.send_goal(1., self.duration)
                self.flag = False
                self.state = State.ACTION1M
                msg.data = "Action 1m"
                self.get_logger().info("Calling Action 1m")
            elif self.flag and self.state == State.ACTION1M:
                # Call 2 m action
                self.send_goal(2., self.duration)
                self.flag = False
                self.state = State.ACTION2M2
                msg.data = "Action 2m"
                self.get_logger().info("Calling Action 2m")
            elif self.flag and self.state == State.ACTION2M2:
                # Call 0 m action
                self.send_goal(0., self.duration)
                self.flag = False
                self.state = State.ACTION0M
                msg.data = "Action 0m"
                self.get_logger().info("Calling Action 0m")
            elif self.flag and self.state == State.ACTION0M:
                self.state = State.END
                # self.reset_actuators()
                msg.data = "End"
                self.get_logger().info("State END")
                self.timer.cancel()
        else:
            self.state = State.IDLE
            # self.reset_actuators()
            msg.data = "Idle"
            self.get_logger().info("State IDLE")

        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()