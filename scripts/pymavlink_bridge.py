#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

from pymavlink import mavutil


class MavRosBridge(Node):

    def __init__(self):
        super().__init__('mavros_bridge')

        self.master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

        # Create a timer at 1Hz for hearthbeat
        self.timer = self.create_timer(1.0, self.hearthbeat_callback)

        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_system))

        self.timer_2 = self.create_timer(1.0, self.fake_gps)

        self.get_logger().info('Creating subscription to /ublox_gps/fix')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.listener_callback,
            10
        )

    def hearthbeat_callback(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,  # type
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # autopilot
            0,  # base_mode
            0,  # custom_mode
            0,  # system_status
            0  # mavlink_version
        )

    def fake_gps(self):
        self.master.mav.gps_raw_int_send(
            0,  # Timestamp (micros since boot or Unix epoch)
            0,  # GPS fix type.
            int(48*1e7),  # Latitude (WGS84), in degrees * 1E7
            int(4*1e7),  # Longitude (WGS84), in degrees * 1E7
            0,  # Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
            65535,  # GPS HDOP horizontal dilution of position in cm (m*100).
            65535,  # GPS VDOP vertical dilution of position in cm (m*100).
            0,  # GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
            0,  # GPS velocity in cm/s in EAST direction in earth-fixed NED frame
            0,  # GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
            65535,  # GPS speed accuracy in cm/s
            65535,  # GPS horizontal accuracy in cm
            65535,  # GPS vertical accuracy in cm
            65535,  # GPS velocity accuracy in cm/s
            65535,  # GPS heading accuracy in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        )

    def listener_callback(self, msg):
        # Send gps_raw_int to autopilot
        self.master.mav.gps_raw_int_send(
            0,  # Timestamp (micros since boot or Unix epoch)
            0,  # GPS fix type.
            int(msg.latitude*1e7),  # Latitude (WGS84), in degrees * 1E7
            int(msg.longitude*1e7),  # Longitude (WGS84), in degrees * 1E7
            0,  # Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
            65535,  # GPS HDOP horizontal dilution of position in cm (m*100).
            65535,  # GPS VDOP vertical dilution of position in cm (m*100).
            0,  # GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
            0,  # GPS velocity in cm/s in EAST direction in earth-fixed NED frame
            0,  # GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
            65535,  # GPS speed accuracy in cm/s
            65535,  # GPS horizontal accuracy in cm
            65535,  # GPS vertical accuracy in cm
            65535,  # GPS velocity accuracy in cm/s
            65535,  # GPS heading accuracy in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MavRosBridge()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()