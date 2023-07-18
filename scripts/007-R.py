#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from enum import Enum
from scipy.linalg import logm, expm
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu

def adjoint(w):
    if isinstance(w, (float, int)):
        return np.array([[0,-w] , [w,0]])
    w = w.tolist()
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def expw(w):
    return expm(adjoint(w))

def adjoint_inv(A):
    return np.array([[A[2,1]],[A[0,2]],[A[1,0]]])

def logw(R):
    return adjoint_inv(logm(R))


class Mission(Node):

    def __init__(self):
        super().__init__('test_R')

        # Wanted orientation
        # self.R_wanted = R.from_euler("zyx", [90, 0, 45], degrees=True)
        self.R_wanted = expw(np.array([0, 0, np.pi/2])) @ expw(np.array([0, np.pi/4, 0])) @ expw(np.array([0, 0, 0]))

        # Robot orientation
        self.R_robot = np.eye(3)
        
        # Thruster velocity
        self.velocity = 0.05

        # Imu callback
        self.imu_msg = Imu()
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/riptide_1/imu_broadcaster/imu_status',
            self.imu_callback,
            10
        )

        # Twist publisher
        self.control_time = 0.1
        self.twist_publisher = self.create_publisher(TwistStamped, "/riptide_1/riptide_controller/cmd_vel", 10)

        self.control_timer = self.create_timer(self.control_time, self.control_callback)

        self.get_logger().info(f"Wanted rotation:\n{self.R_wanted}")

    def imu_callback(self, msg):
        self.imu_msg = msg
        # self.get_logger().info(f'Euler angles: {R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_euler("zyx", degrees=True)}')
        self.R_robot = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_matrix()

    def control_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        w = np.real(logw(self.R_robot.T @ self.R_wanted))

        # Filling the message
        msg.twist.linear.x = self.velocity
        msg.twist.angular.x = w[0, 0]
        msg.twist.angular.y = w[1, 0]
        msg.twist.angular.z = w[2, 0]
        
        self.twist_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()