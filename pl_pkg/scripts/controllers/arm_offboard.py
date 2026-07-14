#!/usr/bin/env python3
"""
Offboard Mode Setup Script
==========================
Arms the drone and switches to offboard flight mode.

Sequence (all at 20 Hz):
  Ticks  0-19  : publish OffboardControlMode + TrajectorySetpoint heartbeat
                 (PX4 requires this BEFORE it accepts the mode switch)
  Tick   20    : send ARM command
  Ticks 20-39  : keep publishing heartbeat, wait for arm confirmation
  Tick   40    : send DO_SET_MODE → OFFBOARD
  Ticks  40+   : keep publishing heartbeat indefinitely
                 (stops PX4 from falling back out of offboard mode)

Once this script reports OFFBOARD ACTIVE, launch your precision landing
controller. It will start publishing its own setpoints and take over.
You can then Ctrl+C this script — the controller maintains the heartbeat.

Usage:
  python3 offboard_setup.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


LOOP_RATE_HZ        = 20

TICKS_ARM           = 20    # send arm after 1s of heartbeat
TICKS_MODE_SWITCH   = 40   
# send offboard switch after 2s of heartbeat

NAV_STATE_OFFBOARD  = 14
ARMING_STATE_ARMED  = 2


class OffboardSetup(Node):

    def __init__(self):
        super().__init__('offboard_setup')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status_cb,
            px4_qos
        )
        # NOTE: topic is vehicle_local_position_v1 on this system
        # (confirmed from ros2 topic list — not vehicle_local_position)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb,
            px4_qos
        )

        # ── Publishers ───────────────────────────────────────────
        self._offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos
        )
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos
        )

        # ── State ────────────────────────────────────────────────
        self._ticks        = 0
        self._arming_state = 0
        self._nav_state    = 0

        # Hold position: captured once the drone is armed and we know where it is
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_z = -1.0   # fallback if no position received yet

        self._arm_sent         = False
        self._mode_switch_sent = False
        self._offboard_active  = False

        self.create_timer(1.0 / LOOP_RATE_HZ, self._loop)
        self.get_logger().info('Offboard setup node started')

    # ── Callbacks ────────────────────────────────────────────────
    def _status_cb(self, msg: VehicleStatus):
        self._arming_state = msg.arming_state
        self._nav_state    = msg.nav_state

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._hold_x = msg.x
        self._hold_y = msg.y
        # Capture hold altitude from current position once, don't keep updating
        # after we've sent the mode switch — avoids chasing a drifting altitude
        if not self._mode_switch_sent:
            self._hold_z = msg.z

    # ── Heartbeat publishers ─────────────────────────────────────
    def _publish_heartbeat(self):
        """
        Must be published at >2 Hz continuously.
        PX4 exits offboard mode if this stops for more than 500ms.
        Using position=True so the TrajectorySetpoint position fields are used.
        """
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_pub.publish(msg)

    def _publish_hold_setpoint(self):
        """
        Hover at captured hold position.
        PX4 requires a valid setpoint alongside OffboardControlMode.
        Without this, the mode switch is rejected.
        """
        msg = TrajectorySetpoint()
        msg.position     = [self._hold_x, self._hold_y, self._hold_z]
        msg.velocity     = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw          = float('nan')
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self._setpoint_pub.publish(msg)

    # ── Commands ─────────────────────────────────────────────────
    def _send_arm(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1           = 1.0   # 1.0 = arm, 0.0 = disarm
        msg.param2           = 0.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self._cmd_pub.publish(msg)
        self.get_logger().info('ARM command sent')

    def _send_offboard_mode(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0
        msg.param2           = 6.0   # 6 = OFFBOARD
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self._cmd_pub.publish(msg)
        self.get_logger().info('DO_SET_MODE → OFFBOARD sent')

    # ── Main loop ────────────────────────────────────────────────
    def _loop(self):
        self._ticks += 1

        # Always publish heartbeat + setpoint first, every tick
        self._publish_heartbeat()
        self._publish_hold_setpoint()

        # ── Step 1: arm after warmup ticks ───────────────────────
        if self._ticks == TICKS_ARM:
            self._send_arm()
            self._arm_sent = True

        # ── Step 2: switch mode after arm ticks ──────────────────
        if self._ticks == TICKS_MODE_SWITCH:
            if self._arming_state != ARMING_STATE_ARMED:
                # Arm didn't go through — retry arm and delay mode switch
                self.get_logger().warn(
                    'Not armed yet at mode switch tick — retrying arm, '
                    'delaying mode switch by 20 ticks'
                )
                self._send_arm()
                # Push mode switch back
                global TICKS_MODE_SWITCH 
                TICKS_MODE_SWITCH += 20
            else:
                self._send_offboard_mode()
                self._mode_switch_sent = True

        # ── Status logging ───────────────────────────────────────
        if self._ticks % 20 == 0:
            armed = (self._arming_state == ARMING_STATE_ARMED)
            offboard = (self._nav_state == NAV_STATE_OFFBOARD)

            if offboard and not self._offboard_active:
                self._offboard_active = True
                self.get_logger().info(
                    'OFFBOARD ACTIVE — drone is armed and in offboard mode. '
                    'You can now launch the precision landing controller. '
                    'Keep this node running until the controller is up.'
                )

            self.get_logger().info(
                f'tick={self._ticks:4d}  '
                f'armed={armed}  '
                f'offboard={offboard}  '
                f'hold=({self._hold_x:.2f}, {self._hold_y:.2f}, {self._hold_z:.2f})',
                throttle_duration_sec=1
            )


# ── Entry point ──────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = OffboardSetup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Offboard setup node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()