#!/usr/bin/env python3

import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import (
    VehicleLocalPosition,
    LandingTargetPose,
)


class NoiseCharLogger(Node):

    def __init__(self):
        super().__init__('noise_char_logger')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.latest_ltp      = None
        self.latest_vlp      = None
        self.latest_cmd_vel  = None
        self.start_ltp_us    = None

        filename = "log_noise_char.csv"
        self.csv_file = open(filename, "w", newline="")
        self.writer   = csv.writer(self.csv_file)

        self.writer.writerow([
            "time_sec",
            "ltp_timestamp_us",
            "x_rel",
            "y_rel",
            "cmd_vx",
            "cmd_vy",
            "cmd_vz",
            "vlp_vx",
            "vlp_vy",
            "vlp_vz",
        ])

        self.create_subscription(
            LandingTargetPose,
            "/fmu/in/landing_target_pose",
            self.ltp_cb,
            px4_qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.vlp_cb,
            px4_qos
        )

        self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.cmd_vel_cb,
            ros_qos
        )

        self.get_logger().info(
            f"Logging to: {os.path.abspath(filename)}"
        )

    def ltp_cb(self, msg):
        self.latest_ltp = msg
        if self.start_ltp_us is None:
            self.start_ltp_us = msg.timestamp
        self.write_row()

    def vlp_cb(self, msg):
        self.latest_vlp = msg

    def cmd_vel_cb(self, msg):
        self.latest_cmd_vel = msg

    def write_row(self):
        if self.latest_ltp is None:
            return

        time_sec = (self.latest_ltp.timestamp - self.start_ltp_us) / 1e6

        if self.latest_cmd_vel is not None:
            cmd_vx = self.latest_cmd_vel.twist.linear.x
            cmd_vy = self.latest_cmd_vel.twist.linear.y
            cmd_vz = self.latest_cmd_vel.twist.linear.z
        else:
            cmd_vx = cmd_vy = cmd_vz = float('nan')

        if self.latest_vlp is not None:
            vlp_vx = self.latest_vlp.vx
            vlp_vy = self.latest_vlp.vy
            vlp_vz = self.latest_vlp.vz
        else:
            vlp_vx = vlp_vy = vlp_vz = float('nan')

        self.writer.writerow([
            time_sec,
            self.latest_ltp.timestamp,
            self.latest_ltp.x_rel,
            self.latest_ltp.y_rel,
            cmd_vx,
            cmd_vy,
            cmd_vz,
            vlp_vx,
            vlp_vy,
            vlp_vz,
        ])

        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NoiseCharLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Logger stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()