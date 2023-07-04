#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from enum import Enum
from scipy.linalg import logm
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
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
        self.velocity = 0.05
        self.roll = 0.
        self.n_cycles = 3
        self.depth = 1.0

        self.K_pitch = 2. / 3.
        self.r_pitch = 0.25

        # State 1 configuration
        self.s1_yaw = 0
        self.s1_duration = 10.0
        self.s1_ping_max_duration = 30.0
        self.s1_ping_distance_trigger = 3.

        # State 2 configuration
        self.s2_yaw = 270
        self.s2_duration = 10.0
        self.s2_ping_max_duration = 30.0
        self.s2_ping_distance_trigger = 3.

        # Number of cycles
        self.counter = 0

        # Messages timestamp failsafe
        self.failsafe_check_timeout = 3.0
        self.last_echosounder_time = self.get_clock().now()
        self.last_pressure_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()

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

        # Imu callback
        self.imu_msg = Imu()
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/riptide_1/imu_broadcaster/imu_status',
            self.imu_callback,
            10
        )

        # Echosounder callback
        self.range_msg = Range()
        self.echosounder_subscriber = self.create_subscription(
            Range,
            '/riptide_1/riptide_echosounder/processed_altitude',
            self.echosounder_callback,
            10
        )

        # State publisher
        self.state_publisher = self.create_publisher(String, "~/mission/state", 10)
        
        # Twits publisher
        self.control_time = 0.1
        self.twist_publisher = self.create_publisher(Twist, "/riptide_1/riptide_controller/twist", 10)
        self.pitch_error_publisher = self.create_publisher(Float64, "~/pitch_error", 10)
        self.depth_error_publisher = self.create_publisher(Float64, "~/depth_error", 10)
        self.control_timer = self.create_timer(self.control_time, self.control_callback)

        # Events checker
        self.last_time = self.get_clock().now()
        self.events = []
        self.event_timer = .25
        self.timer = self.create_timer(self.event_timer, self.events_check)

        self.get_logger().info("Waiting for RC to give misison multiplexer time")

        # Launching failsafe check
        self.failsafe_timer = self.create_timer(self.failsafe_check_timeout, self.failsafe_check)

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.last_imu_time = Time.from_msg(msg.header.stamp)

    def echosounder_callback(self, msg):
        self.range_msg = msg
        self.last_echosounder_time = Time.from_msg(msg.header.stamp)

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
        if self.get_clock().now() - self.last_echosounder_time > Duration(seconds=self.failsafe_check_timeout):
            self.state = State.FAILSAFE
            self.get_logger().fatal(f"Echosounder timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.failsafe_timer.cancel()
            self.execute_fsm()
        elif self.get_clock().now() - self.last_pressure_time > Duration(seconds=self.failsafe_check_timeout):
            self.state = State.FAILSAFE
            self.get_logger().fatal(f"Pressure timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.failsafe_timer.cancel()
            self.execute_fsm()
        elif self.get_clock().now() - self.last_imu_time > Duration(seconds=self.failsafe_check_timeout):
            self.state = State.FAILSAFE
            self.get_logger().fatal(f"IMU timestamp expired! Last message received more than {self.failsafe_check_timeout}s ago.")
            self.failsafe_timer.cancel()
            self.execute_fsm()

    def events_check(self):
        # Iterate over events
        for e in self.events:
            # If an event is True execute_fsm to go into the next state
            if e():
                self.execute_fsm()

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

            depth_error_msg = Float64()
            depth_error_msg.data = pitch_error
            self.depth_error_publisher.publish(depth_error_msg)

            # Pitch error
            pitch_error = self.K_pitch * np.arctan(depth_error / self.r_pitch)

            pitch_error_msg = Float64()
            pitch_error_msg.data = pitch_error
            self.pitch_error_publisher.publish(pitch_error_msg)

            # Wanted rotation matrix computing
            if self.state == State.S1PING or self.state == State.S1SOLID:
                Rw = R.from_euler('zyx', [self.s1_yaw, pitch_error, self.roll]).as_matrix()
            else:
                Rw = R.from_euler('zyx', [self.s2_yaw, pitch_error, self.roll]).as_matrix()

            # Compute the command to put
            w = logw(self.R.T @ Rw)

            # Filling the message
            msg.linear.x = self.velocity
            msg.angular.x = w[0, 0]
            msg.angular.y = w[1, 0]
            msg.angular.z = w[2, 0]
        
        self.twist_publisher.publish(msg)


    def execute_fsm(self):
        msg = String()
        if self.state == State.FAILSAFE:
            # Current state
            msg.data = "FailSafe"
            self.state = State.FAILSAFE
            self.get_logger().fatal("Failsafe")

        elif self.counter >= self.n_cycles and self.state != State.END:
            msg.data = "END"
            self.state = State.END
            self.get_logger().info("End of mission")

        elif self.state == State.IDLE:
            # Current state
            msg.data = "S1: Ping"
            self.state = State.S1PING
            self.last_time = self.get_clock().now()
            self.events = [lambda: (self.get_clock().now() > self.last_time + Duration(seconds=self.s1_ping_max_duration)), lambda: (self.range_msg.range < self.s1_ping_distance_trigger)]
            self.get_logger().info("State S1 Ping")

        elif self.state == State.S1PING:
            # Current state
            msg.data = "S1: Solid"
            self.state = State.S1SOLID
            self.last_time = self.get_clock().now()
            self.events = [lambda: (self.get_clock().now() > self.last_time + Duration(seconds=self.s1_duration))]
            self.get_logger().info("State S1 Solid")

        elif self.state == State.S1SOLID:
            # Current state
            msg.data = "S2: Ping"
            self.state = State.S2PING
            self.last_time = self.get_clock().now()
            self.events = [lambda: (self.get_clock().now() > self.last_time + Duration(seconds=self.s2_ping_max_duration)), lambda: (self.range_msg.range < self.s2_ping_distance_trigger)]
            self.get_logger().info("State S2 Ping")
        
        elif self.state == State.S2PING:
            # Current state
            msg.data = "S2: Solid"
            self.state = State.S2SOLID
            self.last_time = self.get_clock().now()
            self.events = [lambda: self.get_clock().now() > self.last_time + Duration(seconds=self.s2_duration)]
            self.get_logger().info("State S2 Solid")

        elif self.state == State.S2SOLID:
            self.counter += 1
            self.get_logger().info(f"Cycle counter: {self.counter}")
            # Current state
            self.state = State.S1PING
            self.last_time = self.get_clock().now()
            self.events = [lambda: self.get_clock().now() > self.last_time + Duration(seconds=self.s1_ping_max_duration), lambda: (self.range_msg.range < self.s1_ping_distance_trigger)]
            self.get_logger().info("State S1 Ping")

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