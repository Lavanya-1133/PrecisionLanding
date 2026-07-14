"""
aruco_detect.py
───────────────────────────────────────────────────────────────────────────────
Detects ArUco markers using a calibrated camera, estimates 6-DOF pose
(x, y, z in metres + rotation vector), draws 3-D axes on the live frame,
and streams LANDING_TARGET MAVLink messages to a Pixhawk 6C.

Hardware assumptions
  • Hiwonder icSpring USB camera on /dev/video0  (640 × 480 @ 30 fps)
  • camera_calibration.npz  produced by calibrate.py (same directory)
  • Pixhawk 6C connected via USB-Serial → /dev/ttyACM0

Dependencies
  pip install opencv-contrib-python pymavlink numpy

Usage
  python aruco_detect.py [--port /dev/ttyACM0] [--baud 115200]
                         [--marker-id 0] [--marker-size 0.15]
───────────────────────────────────────────────────────────────────────────────
"""

import argparse
import math
import sys
import time

import cv2
import numpy as np
from pymavlink import mavutil

# ── Defaults ────────────────────────────────────────────────────────────────
DEVICE_INDEX    = 8
WIDTH, HEIGHT   = 640, 480
FPS             = 30
CALIB_FILE      = "camera_calibration.npz"

ARUCO_DICT      = cv2.aruco.DICT_6X6_100

DEFAULT_MARKER_ID   = 0
DEFAULT_MARKER_SIZE = 0.15

DEFAULT_PORT    = "/dev/ttyACM0"
DEFAULT_BAUD    = 115200

MAVLINK_HZ      = 20
# ────────────────────────────────────────────────────────────────────────────


def rvec_to_rpy_deg(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy   = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2( R[2, 1],  R[2, 2])
        pitch = math.atan2(-R[2, 0],  sy)
        yaw   = math.atan2( R[1, 0],  R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2],  R[1, 1])
        pitch = math.atan2(-R[2, 0],  sy)
        yaw   = 0.0
    return (math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw))


def send_landing_target(mav, x, y, z, roll_r, pitch_r, yaw_r, marker_size):
    now_us = int(time.time() * 1e6)

    # Convert x,y offsets to angles (radians)
    angle_x = math.atan2(y, z)   # pitch angle (down)
    angle_y = math.atan2(x, z)   # yaw angle (right)

    mav.mav.landing_target_send(
        now_us,      # time_usec
        0,           # target_num
        8,           # frame MAV_FRAME_BODY_NED
        angle_x,     # angle_x
        angle_y,     # angle_y
        z,           # distance
        marker_size, # size_x
        marker_size, # size_y
    )


def main():
    parser = argparse.ArgumentParser(description="ArUco → MAVLink landing target")
    parser.add_argument("--port",        default=DEFAULT_PORT)
    parser.add_argument("--baud",        type=int, default=DEFAULT_BAUD)
    parser.add_argument("--marker-id",   type=int, default=DEFAULT_MARKER_ID)
    parser.add_argument("--marker-size", type=float, default=DEFAULT_MARKER_SIZE)
    parser.add_argument("--no-mavlink",  action="store_true")
    parser.add_argument("--calib",       default=CALIB_FILE)
    args = parser.parse_args()

    # ── Load calibration ───────────────────────────────────────────────────
    try:
        calib = np.load(args.calib)
        camera_matrix = calib["camera_matrix"]
        dist_coeffs   = calib["dist_coeffs"]
        print(f"✅ Calibration loaded: {args.calib}")
    except FileNotFoundError:
        print(f"❌ Calibration file not found: {args.calib}")
        sys.exit(1)

    # ── MAVLink connection ─────────────────────────────────────────────────
    mav = None
    if not args.no_mavlink:
        try:
            mav = mavutil.mavlink_connection(args.port, baud=args.baud)
            mav.wait_heartbeat(timeout=5)

            # Set source system and component for PX4 to accept messages
            mav.mav.srcSystem = 1
            mav.mav.srcComponent = 195  # MAV_COMP_ID_ONBOARD_COMPUTER

            print(f"✅ MAVLink connected → {args.port} @ {args.baud}")
            print(f"   Heartbeat from system {mav.target_system}, "
                  f"component {mav.target_component}")
        except Exception as e:
            print(f"⚠️  MAVLink connection failed: {e}")
            mav = None
    else:
        print("ℹ️  --no-mavlink flag set: running without MAVLink")

    # ── Camera ─────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    if not cap.isOpened():
        print("❌ Cannot open camera /dev/video0")
        sys.exit(1)

    print(f"✅ Camera opened: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
          f"@ {cap.get(cv2.CAP_PROP_FPS):.0f} fps")

    # ── ArUco setup ────────────────────────────────────────────────────────
    aruco_dict   = cv2.aruco.Dictionary_get(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters_create()

    half = args.marker_size / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

    print("="*55)
    print("  ArUco Detect + MAVLink Landing Target")
    print("="*55)
    print(f"  Dictionary  : DICT_6X6_100")
    print(f"  Target ID   : {args.marker_id if args.marker_id >= 0 else 'any'}")
    print(f"  Marker size : {args.marker_size*100:.1f} cm")
    print(f"  MAVLink     : {'disabled' if mav is None else args.port}")
    print("-"*55)
    print("  Q = quit    S = save snapshot")
    print("="*55)

    mav_interval  = 1.0 / MAVLINK_HZ
    t_last_mav    = 0.0
    frame_count   = 0
    fps_display   = 0.0
    t_fps         = time.time()
    snapshot_n    = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Frame grab failed")
            break

        frame_count += 1
        if frame_count % 30 == 0:
            fps_display = 30.0 / (time.time() - t_fps)
            t_fps = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=aruco_params)

        target_found = False
        tx = ty = tz = 0.0
        roll_d = pitch_d = yaw_d = 0.0

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if args.marker_id >= 0 and marker_id != args.marker_id:
                    continue

                ok, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corners[i].reshape(4, 2),
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  rvec, tvec, half)

                tx, ty, tz = tvec.flatten()
                roll_d, pitch_d, yaw_d = rvec_to_rpy_deg(rvec)
                target_found = True

                cx = int(corners[i][0][:, 0].mean())
                cy = int(corners[i][0][:, 1].mean())

                label = (f"ID:{marker_id}  "
                         f"x={tx:+.3f}m  y={ty:+.3f}m  z={tz:.3f}m")
                cv2.putText(frame, label,
                            (cx - 120, cy - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 255), 1, cv2.LINE_AA)

                rpy_label = (f"R={roll_d:+.1f}°  "
                             f"P={pitch_d:+.1f}°  "
                             f"Y={yaw_d:+.1f}°")
                cv2.putText(frame, rpy_label,
                            (cx - 120, cy + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 200, 255), 1, cv2.LINE_AA)

                break

        # ── MAVLink send ───────────────────────────────────────────────────
        now = time.time()
        if target_found and mav is not None and (now - t_last_mav) >= mav_interval:
            send_landing_target(
                mav,
                tx, ty, tz,
                math.radians(roll_d),
                math.radians(pitch_d),
                math.radians(yaw_d),
                args.marker_size,
            )
            t_last_mav = now

        # ── HUD overlay ───────────────────────────────────────────────────
        status_color = (0, 255, 0) if target_found else (0, 80, 255)
        status_text  = (f"TARGET  x={tx:+.3f} y={ty:+.3f} z={tz:.3f} m"
                        if target_found else "NO TARGET")
        cv2.putText(frame, status_text,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, status_color, 2, cv2.LINE_AA)

        mav_status = ("MAVLink ✓" if mav else "MAVLink ✗")
        cv2.putText(frame,
                    f"FPS:{fps_display:.1f}  {mav_status}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.putText(frame, "Q=quit  S=snapshot",
                    (10, HEIGHT - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (180, 180, 180), 1, cv2.LINE_AA)

        cv2.imshow("ArUco Detect → MAVLink", frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q')):
            print("\n✅ Quit by user")
            break

        if key in (ord('s'), ord('S')):
            snapshot_n += 1
            fname = f"aruco_snap_{snapshot_n:03d}.jpg"
            cv2.imwrite(fname, frame)
            print(f"📸 Snapshot saved: {fname}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n📊 Frames processed: {frame_count}")
    print("Done.")


if __name__ == "__main__":
    main()

# # Basic use
# python aruco_detect.py

# # Custom port/marker
# python aruco_detect.py --port /dev/ttyUSB0 --baud 57600 --marker-id 7 --marker-size 0.20

# # Camera-only, no MAVLink (useful for tuning/debug)
# python aruco_detect.py --no-mavlink


#(venv) cpri@cpri:~/aruco_drone$ mavproxy.py   --master /dev/ttyACM0   --baudrate 115200   --out udp:127.0.0.1:14550   --out udp:127.0.0.1:14551   --daemon
