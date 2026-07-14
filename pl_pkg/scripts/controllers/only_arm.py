#!/usr/bin/env python3
"""
Arm / disarm the drone via /fmu/in/vehicle_command.

Usage:
  ros2 run <your_pkg> arm_drone          # arms
  ros2 run <your_pkg> arm_drone disarm   # disarms
  python3 arm_drone.py
  python3 arm_drone.py disarm
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand


class ArmDisarm(Node):

    def __init__(self, arm: bool):
        super().__init__('arm_disarm_node')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos
        )

        self._arm    = arm
        self._sent   = False

        # PX4 requires the offboard heartbeat to be running before it accepts
        # arm commands in some configurations. If arming via this script fails,
        # check that the drone is in a mode that allows external arming.
        # Small delay to let the publisher establish the DDS connection.
        self.create_timer(0.1, self._send)

    def _send(self):
        if self._sent:
            return

        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1           = 1.0 if self._arm else 0.0
        msg.param2           = 0.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)

        self._pub.publish(msg)
        self._sent = True

        action = 'ARM' if self._arm else 'DISARM'
        self.get_logger().info(f'Sent {action} command to /fmu/in/vehicle_command')

        # Give DDS a moment to deliver, then shut down
        self.create_timer(0.5, self._shutdown)

    def _shutdown(self):
        rclpy.shutdown()


def main():
    arm = True
    if len(sys.argv) > 1 and sys.argv[1].lower() == 'disarm':
        arm = False

    rclpy.init()
    node = ArmDisarm(arm)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()