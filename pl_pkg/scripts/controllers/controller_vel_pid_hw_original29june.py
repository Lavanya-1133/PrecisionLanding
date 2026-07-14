#!/usr/bin/env python3
"""
Precision Landing Controller — Velocity Setpoint + PI
======================================================
Modified from controller_ESD.py.

Key change: ALIGN, DESCEND_FAST, DESCEND_SLOW now use velocity setpoints
with a PI controller for XY. TAKEOFF and RECOVERY still use position
setpoints (OffboardControlMode switches accordingly).

State machine:
  IDLE → TAKEOFF → SEARCH → ALIGN → DESCEND_FAST → DESCEND_SLOW → LAND
                                          ↑               ↓
                                      RECOVERY ←←← (marker lost)
  Any state → ABORT (pilot takes over)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    LandingTargetPose,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from geometry_msgs.msg import TwistStamped


# ──────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────
SEARCH_ALT          = -1.0    # m NED — lowered for hardware test
DESCEND_FAST_ALT    = -0.5    # m NED — altitude to switch to slow descent
LAND_ALT            = -0.2    # m NED — trigger LAND at 0.2m above ground

ALIGN_THRESHOLD     = 0.5
DESCENT_PAUSE_THR   = 0.4

LOOP_RATE_HZ        = 20.0
MARKER_TIMEOUT_SEC  = 5.0

# Descent velocities — both capped below MAX_VELOCITY_Z
DESCENT_FAST_VEL    =  0.4    # m/s downward during fast descent
DESCENT_SLOW_VEL    =  0.2    # m/s downward during slow descent — keep soft near ground

KP_XY               = 0.2
KI_XY               = 0.00
KD_XY               = 0.00
MAX_VELOCITY_XY     = 1.0     # m/s — hardware safety limit
MAX_VELOCITY_Z      = 0.5     # m/s — hardware safety limit, applied to all Z commands
MAX_INTEGRAL        = 1.0

# PX4 nav_state values
NAV_STATE_OFFBOARD  = 14
ARMING_STATE_ARMED  = 2


# ──────────────────────────────────────────────────────────────────
# State enum
# ──────────────────────────────────────────────────────────────────
class State:
    IDLE          = "IDLE"
    TAKEOFF       = "TAKEOFF"
    SEARCH        = "SEARCH"
    ALIGN         = "ALIGN"
    DESCEND_FAST  = "DESCEND_FAST"
    DESCEND_SLOW  = "DESCEND_SLOW"
    LAND          = "LAND"
    RECOVERY      = "RECOVERY"
    ABORT         = "ABORT"
    DONE          = "DONE"


# ──────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────
class PrecisionLandController(Node):

    def __init__(self):
        super().__init__('precision_land_controller')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_cb,
            px4_qos
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status_cb,
            px4_qos
        )
        self.create_subscription(
            LandingTargetPose,
            '/fmu/in/landing_target_pose',
            self._target_cb,
            px4_qos
        )

        # ── Publishers ───────────────────────────────────────────
        self.traj_pub     = self.create_publisher(TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',   px4_qos)
        self.cmd_pub      = self.create_publisher(VehicleCommand,        '/fmu/in/vehicle_command',        px4_qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode,  '/fmu/in/offboard_control_mode',  px4_qos)

        # Monitoring topic — PID velocity output, subscribe with:
        #   ros2 topic echo /cmd_vel
        #   ros2 run plotjuggler plotjuggler (for live plot)
        self.cmd_vel_pub  = self.create_publisher(
            TwistStamped, '/cmd_vel',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # ── Internal state ───────────────────────────────────────
        self.state          = State.IDLE
        self.offboard_ticks = 0

        # Drone position (NED)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.arming_state = 0
        self.nav_state    = 0

        # Marker data — relative offset of marker from drone in NED
        self.correction_north = 0.0
        self.correction_east  = 0.0
        self.marker_visible   = False
        self.last_seen_time   = None

        # Hold position for SEARCH / RECOVERY states
        self.hold_x = 0.0
        self.hold_y = 0.0

        # ── PI controller state ───────────────────────────────────
        # NOTE: integrals are reset explicitly on entering velocity-controlled states.
        # Think about: should they also be reset on marker loss and re-acquisition?
        self._int_x     = 0.0
        self._int_y     = 0.0
        self._prev_err_x = 0.0   # for D term — reset on state entry
        self._prev_err_y = 0.0
        self._last_time = None   # for computing dt each tick

        # ── Main loop ────────────────────────────────────────────
        self.create_timer(1.0 / LOOP_RATE_HZ, self._loop)

        self.get_logger().info('Precision Land Controller (velocity+PI) started')

    # ──────────────────────────────────────────────────────────────
    # Callbacks — unchanged from original
    # ──────────────────────────────────────────────────────────────
    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z

    def _status_cb(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state    = msg.nav_state

    def _target_cb(self, msg: LandingTargetPose):
        if not msg.rel_pos_valid:
            return

        # Frame conversion — unchanged from original
        # landing_target_pose: x_rel=East, y_rel=North, z_rel=Up
        # NED: x=North, y=East, z=Down
        self.correction_north = -msg.x_rel   # North error
        self.correction_east  = -msg.y_rel   # East error

        self.marker_visible   = True
        self.last_seen_time   = time.time()

    # ──────────────────────────────────────────────────────────────
    # Offboard control mode publisher
    # ──────────────────────────────────────────────────────────────
    def _publish_offboard_control_mode(self, use_velocity: bool = False):
        """
        CAUTION: switching between position=True and velocity=True mid-flight.
        PX4 should handle this — it re-reads the flags each cycle — but watch
        for any unexpected behaviour in your tests when the state transitions
        between TAKEOFF (position) and ALIGN (velocity).

        If you see jitter on that transition, one alternative is to always
        publish velocity=True and implement position hold as a P controller
        in the SEARCH/TAKEOFF/RECOVERY states too.
        """
        msg = OffboardControlMode()
        msg.position     = not use_velocity
        msg.velocity     = use_velocity
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────
    # Setpoint publishers
    # ──────────────────────────────────────────────────────────────
    def _publish_position_setpoint(self, x: float, y: float, z: float):
        """Position setpoint — used for TAKEOFF, SEARCH, RECOVERY."""
        msg = TrajectorySetpoint()
        msg.position     = [x, y, z]
        msg.velocity     = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw          = float('nan')
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def _publish_velocity_setpoint(self, vx: float, vy: float, vz: float):
        """
        Velocity setpoint — used for ALIGN, DESCEND_FAST, DESCEND_SLOW.
        Position fields must be NaN when OffboardControlMode.velocity=True.
        vz is clamped to MAX_VELOCITY_Z regardless of what the caller passes.
        """
        vz = self._clamp(vz, -MAX_VELOCITY_Z, MAX_VELOCITY_Z)

        msg = TrajectorySetpoint()
        msg.position     = [float('nan'), float('nan'), float('nan')]
        msg.velocity     = [vx, vy, vz]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw          = float('nan')
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

        # Publish to monitoring topic
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = vx
        cmd.twist.linear.y  = vy
        cmd.twist.linear.z  = vz
        self.cmd_vel_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────────
    # PI controller
    # ──────────────────────────────────────────────────────────────
    def _calculate_velocity_xy(self, dt: float):
        """
        PI controller for XY velocity.

        Error convention (matches original code):
          correction_north > 0  →  marker is north of drone  →  fly north (+vx)
          correction_east  > 0  →  marker is east  of drone  →  fly east  (+vy)

        Compare with C++ reference where error = drone_pos - tag_pos,
        requiring a sign flip. Here the error is already in the right sign
        because correction_* is marker-relative-to-drone.

        Think about:
        - dt is computed from wall time. What happens if a loop tick is late?
          Should you clamp dt to avoid integral blow-up on a stall?
        - The integral here accumulates even when the drone is moving toward
          the target. Is that always correct?
        """

        err_x = self.correction_north
        err_y = self.correction_east

        # Proportional
        vx_p = KP_XY * self.correction_north
        vy_p = KP_XY * self.correction_east

        # Integral (with dt for proper units: m/s not m/tick)
        self._int_x += self.correction_north * dt
        self._int_y += self.correction_east  * dt

        # Anti-windup clamp
        self._int_x = self._clamp(self._int_x, -MAX_INTEGRAL, MAX_INTEGRAL)
        self._int_y = self._clamp(self._int_y, -MAX_INTEGRAL, MAX_INTEGRAL)

        vx_i = KI_XY * self._int_x
        vy_i = KI_XY * self._int_y


        vx_d = KD_XY * (err_x - self._prev_err_x) / dt
        vy_d = KD_XY * (err_y - self._prev_err_y) / dt
 
        self._prev_err_x = err_x
        self._prev_err_y = err_y

        #print('vx_d: %.2f' % vx_d, 'vy_d: %.2f' % vy_d)
 
        vx = self._clamp(vx_p + vx_i + vx_d, -MAX_VELOCITY_XY, MAX_VELOCITY_XY)
        vy = self._clamp(vy_p + vy_i + vy_d, -MAX_VELOCITY_XY, MAX_VELOCITY_XY)

        return vx, vy

    def _reset_pid(self):
        """Call this whenever entering a velocity-controlled state."""
        self._int_x = 0.0
        self._int_y = 0.0
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0

    # ──────────────────────────────────────────────────────────────
    # Helpers — unchanged from original
    # ──────────────────────────────────────────────────────────────
    def _send_offboard_mode_command(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0
        msg.param2           = 6.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)
        self.get_logger().info('Sent OFFBOARD mode command to PX4')

    def _send_land_command(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.param1           = 0.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        # Reset PI integrals whenever entering a velocity-controlled state.
        # Think about: is this always the right call? What if the drone is
        # mid-correction and the integral has built up useful information?
        if new_state in (State.ALIGN, State.DESCEND_FAST, State.DESCEND_SLOW):
            self._reset_pid()
        self.state = new_state

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(value, max_val))

    def _check_marker_timeout(self):
        if self.last_seen_time is not None:
            if time.time() - self.last_seen_time > MARKER_TIMEOUT_SEC:
                if self.marker_visible:
                    self.get_logger().warn('Marker timeout — marker lost')
                self.marker_visible = False

    def _horizontal_error_ok(self, threshold: float) -> bool:
        return (abs(self.correction_north) < threshold and
                abs(self.correction_east)  < threshold)

    # ──────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────
    def _loop(self):

        # ── Compute dt ───────────────────────────────────────────
        now = time.time()
        dt  = (now - self._last_time) if self._last_time is not None else (1.0 / LOOP_RATE_HZ)
        # Clamp dt: if a tick was delayed (e.g. scheduler lag), don't let the
        # integral accumulate as if 2 seconds passed.
        dt  = self._clamp(dt, 0.001, 0.2)
        self._last_time = now

        # ── STEP 0: ABORT CHECK ──────────────────────────────────
        if self.nav_state != NAV_STATE_OFFBOARD:
            if self.state not in (State.IDLE, State.ABORT, State.DONE):
                self.get_logger().warn(
                    f'nav_state changed to {self.nav_state} — aborting. Pilot has control.')
                self.state = State.ABORT

        # ── STEP 1: OFFBOARD HEARTBEAT ───────────────────────────
        # Velocity-controlled states need velocity=True in OffboardControlMode.
        # All other states use position=True (same as original).
        velocity_states = (State.ALIGN, State.DESCEND_FAST, State.DESCEND_SLOW)
        self._publish_offboard_control_mode(use_velocity=(self.state in velocity_states))

        # ── STEP 2: MARKER TIMEOUT CHECK ─────────────────────────
        self._check_marker_timeout()

        # ── STEP 3: STATE MACHINE ─────────────────────────────────

        # ── IDLE ─────────────────────────────────────────────────
        if self.state == State.IDLE:
            if self.arming_state != ARMING_STATE_ARMED:
                self.get_logger().info('Waiting for arm...', throttle_duration_sec=3)
                return
            self.offboard_ticks += 1
            if self.offboard_ticks == 10:
                self._send_offboard_mode_command()
            if self.nav_state != NAV_STATE_OFFBOARD:
                self.get_logger().info(
                    f'Waiting for OFFBOARD mode... ({self.offboard_ticks} ticks)',
                    throttle_duration_sec=2)
                return
            self.get_logger().info('Armed and OFFBOARD — starting takeoff')
            self._transition(State.TAKEOFF)

        # ── TAKEOFF — position setpoint, unchanged ───────────────
        elif self.state == State.TAKEOFF:
            self._publish_position_setpoint(self.pos_x, self.pos_y, SEARCH_ALT)
            if self.pos_z <= SEARCH_ALT + 0.3:
                self.hold_x = self.pos_x
                self.hold_y = self.pos_y
                self.get_logger().info(f'Reached search altitude ({SEARCH_ALT}m NED)')
                self._transition(State.SEARCH)

        # ── SEARCH — position setpoint, unchanged ────────────────
        elif self.state == State.SEARCH:
            self._publish_position_setpoint(self.hold_x, self.hold_y, SEARCH_ALT)
            if self.marker_visible:
                self.get_logger().info(
                    f'Marker found — N:{self.correction_north:.2f} E:{self.correction_east:.2f}')
                self._transition(State.ALIGN)

        # ── ALIGN — velocity setpoint + PI ───────────────────────
        elif self.state == State.ALIGN:
            """
            Hold altitude (vz=0), drive XY toward marker via PI.
            Transition to DESCEND_FAST once XY error is within ALIGN_THRESHOLD.

            Think about: the original ALIGN used target_x = pos_x + correction_north
            (position setpoint). Now you're commanding velocity instead.
            The PI controller will drive error to zero, but it won't "hold" a
            position when correction=0. Is that a problem here?
            """
            if not self.marker_visible:
                self.get_logger().warn('Marker lost during ALIGN — returning to SEARCH')
                self._transition(State.SEARCH)
                return

            vx, vy = self._calculate_velocity_xy(dt)
            self._publish_velocity_setpoint(vx, vy, 0.0)  # vz=0: hold altitude

            if self._horizontal_error_ok(ALIGN_THRESHOLD):
                self.get_logger().info(
                    f'Aligned (error < {ALIGN_THRESHOLD}m) — starting fast descent')
                self._transition(State.DESCEND_FAST)

        # ── DESCEND_FAST — velocity setpoint + PI ────────────────
        elif self.state == State.DESCEND_FAST:
            if not self.marker_visible:
                self.get_logger().warn('Marker lost during DESCEND_FAST — RECOVERY')
                self._transition(State.RECOVERY)
                return

            vx, vy = self._calculate_velocity_xy(dt)

            # Only descend if XY error is acceptable, otherwise hover and correct
            if self._horizontal_error_ok(DESCENT_PAUSE_THR):
                vz = DESCENT_FAST_VEL   # positive NED = downward
            else:
                vz = 0.0
                self.get_logger().info(
                    f'Descent paused — re-aligning (N:{self.correction_north:.2f} E:{self.correction_east:.2f})',
                    throttle_duration_sec=1)

            self._publish_velocity_setpoint(vx, vy, vz)

            # Altitude check to transition to slow descent
            # NOTE: in velocity mode you no longer have a position ratchet.
            # The drone descends at DESCENT_FAST_VEL until this altitude check triggers.
            # Think about: what happens if the drone overshoots DESCEND_FAST_ALT
            # slightly before the check fires? Is that acceptable?
            if self.pos_z >= DESCEND_FAST_ALT - 0.2:
                self.get_logger().info('Reached fast-descent altitude — switching to slow')
                self._transition(State.DESCEND_SLOW)

        # ── DESCEND_SLOW — velocity setpoint + PI ────────────────
        elif self.state == State.DESCEND_SLOW:
            if not self.marker_visible:
                self.get_logger().warn('Marker lost during DESCEND_SLOW — RECOVERY')
                self._transition(State.RECOVERY)
                return

            vx, vy = self._calculate_velocity_xy(dt)

            if self._horizontal_error_ok(DESCENT_PAUSE_THR):
                vz = DESCENT_SLOW_VEL
            else:
                vz = 0.0
                self.get_logger().info(
                    f'Descent paused — re-aligning (N:{self.correction_north:.2f} E:{self.correction_east:.2f})',
                    throttle_duration_sec=1)

            self._publish_velocity_setpoint(vx, vy, vz)

            if self.pos_z >= LAND_ALT - 0.05:
                self.get_logger().info(f'Within {abs(LAND_ALT)}m of ground — sending LAND command')
                self._transition(State.LAND)

        # ── LAND — unchanged ─────────────────────────────────────
        elif self.state == State.LAND:
            self._send_land_command()
            self.get_logger().info('LAND command sent — PX4 taking over final touchdown')
            self._transition(State.DONE)

        # ── RECOVERY — position setpoint, unchanged ──────────────
        elif self.state == State.RECOVERY:
            self._publish_position_setpoint(self.pos_x, self.pos_y, SEARCH_ALT)
            if self.pos_z <= SEARCH_ALT + 0.3:
                self.hold_x = self.pos_x
                self.hold_y = self.pos_y
                self.get_logger().info('Back at search altitude — waiting for marker')
                self._transition(State.SEARCH)

        # ── ABORT ────────────────────────────────────────────────
        elif self.state == State.ABORT:
            self.get_logger().info(
                f'Controller aborted — pilot in control (nav_state={self.nav_state})',
                throttle_duration_sec=5)

        # ── DONE ─────────────────────────────────────────────────
        elif self.state == State.DONE:
            pass


# ──────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
