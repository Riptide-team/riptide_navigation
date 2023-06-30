#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from enum import Enum
from scipy.linalg import logm
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from riptide_msgs.msg import Pressure
from riptide_msgs.msg import Multiplexer
from riptide_msgs.action import FullDepth


class State(Enum):
    IDLE      = 0
    S1PING    = 1
    S1SOLID   = 2
    S2PING    = 3
    S2SOLID   = 4
    END       = 5
    FAILSAFE  = 6


def adjoint_inv(A):
    return np.array([[A[2,1]],[A[0,2]],[A[1,0]]])

def logw(R):
    return adjoint_inv(logm(R))


class Mission(Node):

    def __init__(self):
        super().__init__('mission_cycles')
        self.state = State.IDLE

        # General robot control
        self.velocity = 0.5
        self.roll = 0.
        self.n_cycles = 3
        self.depth = 1.0

        # State 1 configuration
        self.s1_yaw = 0
        self.s1_duration = 10.0
        self.s1_ping_max_duration = 30.0
        self.s1_ping_distance_trigger = 3.0

        # State 2 configuration
        self.s2_yaw = 270
        self.s2_duration = 10.0
        self.s2_ping_max_duration = 30.0
        self.s2_ping_distance_trigger = 3.0

        # Number of cycles
        self.counter = 0

        # Messages timestamp failsafe
        self.failsafe_check_timeout = 3.0
        self.last_echosounder_time = self.get_clock().now()
        self.last_pressure_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        self.timer = self.create_timer(self.failsafe_check_timeout, self.failsafe_check)

        # Robot orientation (initialized to identity and then will be update with imu sensor's)
        self.R = np.eye(3)

        # Robot depth
        self.depth = 0.

        # Pressure monitoring
        self.d_max = 4
        self.subscription = self.create_subscription(
            Pressure,
            '/riptide_1/pressure_broadcaster/pressure_status',
            self.pressure_callback,
            10
        )

        # Multiplexer subscriber
        self.multiplexer_subscriber = self.create_subscription(
            Multiplexer,
            '/riptide_1/tail_broadcaster/multiplexer_status',
            self.multiplexer_callback,
            10
        )

        # TODO add imu callback

        # State publisher
        self.state_publisher = self.create_publisher(String, "~/mission/state", 10)
        # self.state_id_publisher = self.create_publisher(Float64, "~/mission/state_id", 10)
        
        # Twits publisher
        self.twist_publisher = self.create_publisher(Twist, "~/riptide_controller/twist", 10)

        self.get_logger().info("Waiting for RC to give misison multiplexer time")

    def pressure_callback(self, msg):
        if self.state != State.FAILSAFE and msg.depth > self.d_max:
            self.get_logger().warn(f'Current pressure {msg.depth} > d_max = {self.d_max}: Aborting!')
            self.state = State.FAILSAFE
            self.execute_fsm()
        self.current_depth = msg.depth

    def multiplexer_callback(self, msg):
        if self.state == State.IDLE and msg.automatic and msg.remaining_time > 5:
            self.get_logger().info(f"Launching the mission! RH is the chef for the next {msg.remaining_time}")
            self.execute_fsm()

    
    def get_result_callback(self, future):
        # Getting the result
        result = future.result().result
        self.get_logger().info(f'Final depth: {result.final_depth}m, {result.final_duration}s')

        # Executing next fsm state
        self.execute_fsm()

    def feedback_callback(self, feedback_msg):
        # Getting feedback
        feedback = feedback_msg.feedback
        self.get_logger().info('Received error: {0}'.format(feedback.depth_error))

    def failsafe_check(self):
        if self.time.now() - self.last_echosounder_time > self.failsafe_check_timeout:
            msg = String()
            msg.data = "FailSafe"
            self.state = State.FAILSAFE
            self.get_logger().info(f"Echosounder timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.state_publisher.publish(msg)
        elif self.time.now() - self.last_pressure_time > self.failsafe_check_timeout:
            msg = String()
            msg.data = "FailSafe"
            self.state = State.FAILSAFE
            self.get_logger().info(f"Pressure timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.state_publisher.publish(msg)
        elif self.time.now() - self.last_imu_time > self.failsafe_check_timeout:
            msg = String()
            msg.data = "FailSafe"
            self.state = State.FAILSAFE
            self.get_logger().info(f"IMU timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.state_publisher.publish(msg)

    def control_callback(self):
        msg = Twist()
        if self.state == State.FAILSAFE or self.state == State.END or self.state == State.IDLE:
            # Filling the message
            msg.linear.x = 0.
            msg.angular.x = 0.
            msg.angular.y = 0.
            msg.angular.z = 0.
        else:
            # Depth error
            depth_error = self.current_depth = self.depth

            # Pitch error
            pitch_error = self.K_pitch * np.arctan(depth_error / self.r_pitch)

            # Wanted rotatio matrix computing
            if self.state == State.S1PING or self.state == State.S1SOLID:
                Rw = R.from_euler('zyx', [self.s1_yaw, pitch_error, self.roll]).as_matrix()
            else:
                Rw = R.from_euler('zyx', [self.s2_yaw, pitch_error, self.roll]).as_matrix()

            # Compute the command to put
            w = logw(self.R.T @ Rw)

            # Filling the message
            msg.linear.x = self.velocity
            msg.angular.x = w[0]
            msg.angular.y = w[1]
            msg.angular.z = w[2]
        
        self.twist_publisher.publish(msg)


    def execute_fsm(self):
        msg = String()
        if self.state == State.FAILSAFE:
            # Canceling the current action
            self._get_result_future.cancel()

            # Calling action 0 m during 30 seconds
            self.send_goal(self, 0, self.duration)
            
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