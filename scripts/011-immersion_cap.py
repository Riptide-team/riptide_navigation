#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from enum import Enum
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from riptide_msgs.msg import Pressure
from riptide_msgs.msg import Multiplexer
from riptide_msgs.action import Immerse, Depth

from controller_manager_msgs.srv import SwitchController


class State(Enum):
    IDLE      = 0
    IMMERSE   = 1
    CAP       = 2
    END       = 3
    FAILSAFE  = 4


class Mission(Node):

    def __init__(self):
        super().__init__('mission_immersion')
        self.state = State.IDLE

        # Requested depth
        self.requested_depth = 3.
        self.timeout_depth_sec = 10

        # Messages timestamp failsafe
        self.failsafe_check_timeout = 3.0
        self.last_pressure_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()

        # Pressure monitoring
        self.d_max = 7.
        self.subscription = self.create_subscription(
            Pressure,
            '/pressure_broadcaster/pressure_status',
            self.pressure_callback,
            10
        )

        # Multiplexer subscriber
        self.multiplexer_subscriber = self.create_subscription(
            Multiplexer,
            '/tail_broadcaster/multiplexer_status',
            self.multiplexer_callback,
            10
        )

        # Imu callback
        self.imu_msg = Imu()
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',
            self.imu_callback,
            10
        )

        # State publisher
        self.state_publisher = self.create_publisher(String, "~/mission/state", 10)

        # Action client
        self.immerse_action_client = ActionClient(self, Immerse, '/immersion_controller/immerse')

        # Action client
        self.depth_action_client = ActionClient(self, Immerse, '/depth_controller/depth')

        # Creating the controller loader
        self.controller_manager_service = '/controller_manager/switch_controller'
        self.switch_controller = self.create_client(SwitchController, self.controller_manager_service)

        # Waiting for the controller_manager loader service
        while not self.switch_controller.wait_for_service(timeout_sec = 10.0):
            self.get_logger().info(f'Controller manager not available at {self.controller_manager_service}')

        # Loading immersion_controller
        self.call_switch_controller([], ["immersion_controller"])
        
        self.get_logger().info("Waiting for RC to give misison multiplexer time")

        # Launching failsafe check
        self.failsafe_timer = self.create_timer(self.failsafe_check_timeout, self.failsafe_check)

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.last_imu_time = Time.from_msg(msg.header.stamp)
        self.R = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_matrix()

    def pressure_callback(self, msg):
        if self.state != State.FAILSAFE and msg.depth > self.d_max:
            self.get_logger().warn(f'Current pressure {msg.depth} > d_max = {self.d_max}: Aborting!')
            self.state = State.FAILSAFE
            self.execute_fsm()
        self.current_depth = msg.depth
        self.last_pressure_time = Time.from_msg(msg.header.stamp)

    def multiplexer_callback(self, msg):
        if self.state == State.IDLE and msg.automatic and msg.remaining_time > 5:
            self.get_logger().info(f"Launching the mission! RH is the chef for the next {msg.remaining_time}")
            self.execute_fsm()

    def call_switch_controller(self, deactivate_controllers, activate_controllers):
        req = SwitchController.Request()
        req.activate_controllers = activate_controllers
        req.deactivate_controllers = deactivate_controllers
        req.activate_asap = True
        req.strictness = SwitchController.Request().BEST_EFFORT
        self.future = self.switch_controller.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if not self.future.result().ok:
            self.get_logger().fatal("Controller manager is not able to activate {activate_controllers} and to unactivate {deactivate_controllers}")
            rclpy.shutdown()

    def failsafe_check(self):
        if self.get_clock().now() - self.last_pressure_time > Duration(seconds=self.failsafe_check_timeout):
            self.state = State.FAILSAFE
            self.get_logger().fatal(f"Pressure timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.failsafe_timer.cancel()
            self.execute_fsm()
        elif self.get_clock().now() - self.last_imu_time > Duration(seconds=self.failsafe_check_timeout):
            self.state = State.FAILSAFE
            self.get_logger().fatal(f"IMU timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.failsafe_timer.cancel()
            self.execute_fsm()

    def execute_fsm(self):
        msg = String()
        if self.state == State.FAILSAFE:
            # Current state
            msg.data = "FailSafe"
            self.state = State.FAILSAFE
            self.get_logger().fatal("Failsafe")

        elif self.state == State.IDLE:
            # Current state
            msg.data = "Immerse"
            self.state = State.IMMERSE
            self.get_logger().info("State Immerse")
            self.send_immersion_goal()

        elif self.state == State.IMMERSE:
            msg.data = "Cap"
            self.state = State.CAP
            self.get_logger().info("State Cap")
            self.call_switch_controller(["immersion_controller"], ["riptide_controller", "log_controller", "depth_controller"])
            self.send_depth_goal()

        elif self.state == State.CAP:
            msg.data = "END"
            self.state = State.END
            self.get_logger().info("End of mission")

        # Publishing the current state
        self.state_publisher.publish(msg)

    def send_depth_goal(self):
        depth_msg = Depth.Goal()

        depth_msg.depth = self.requested_depth
        depth_msg.timeout.sec = int(self.timeout_depth_sec)

        self.depth_action_client.wait_for_server()
        self._send_goal_future = self.depth_action_client.send_goal_async(depth_msg, feedback_callback=self.depth_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_immersion_goal(self):
        immerse_msg = Immerse.Goal()

        self.immerse_action_client.wait_for_server()
        self._send_goal_future = self.immerse_action_client.send_goal_async(immerse_msg, feedback_callback=self.immerse_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def depth_feedback_callback(self, feedback_msg):
        # Getting feedback
        feedback = feedback_msg.feedback
        # TODO
        # self.get_logger().info('Remaining immersion time: {0}'.format(feedback.remaining_time))

    def immerse_feedback_callback(self, feedback_msg):
        # Getting feedback
        feedback = feedback_msg.feedback
        self.get_logger().info('Remaining immersion time: {0}'.format(feedback.remaining_time))

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
        # Executing next fsm state
        self.execute_fsm()


def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()