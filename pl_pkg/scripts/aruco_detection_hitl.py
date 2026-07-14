#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import LandingTargetPose

import cv2
import numpy as np
from cv_bridge import CvBridge
from cv2 import aruco


class ArucoLandingDetector(Node):
    def __init__(self):
        super().__init__('aruco_landing_detector')

        # --- QoS for PX4 topics ---
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Hardcoded intrinsics (HITL hardware cam) ---
        # These are the primary values. info_callback will override
        # only if v4l2_camera publishes a valid non-zero camera_info.
        self.camera_matrix = np.array([
            [556.30763089,   0.0,          316.91915551],
            [  0.0,        551.86603228,  205.66767332],
            [  0.0,          0.0,           1.0        ]
        ], dtype=np.float64)

        self.dist_coeffs = np.array(
            [[-0.47040191, -0.16859327, -0.01375266, -0.00279181, 1.15570743]],
            dtype=np.float64
        )

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',           # v4l2_camera default topic
            self.image_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',         # verify this with: ros2 topic list
            self.info_callback,
            10
        )

        # --- Publisher to PX4 ---
        self.target_pub = self.create_publisher(
            LandingTargetPose,
            '/fmu/in/landing_target_pose',
            px4_qos
        )

        self.debug_pub = self.create_publisher(Image, '/aruco_debug', 10)

        # --- ArUco setup ---
        self.bridge = CvBridge()
        self.aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
        self.aruco_params = aruco.DetectorParameters()
        self.detector     = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.marker_size = 0.15  # metres, physical marker size

        self.get_logger().info('ArUco Landing Detector started — HITL mode')

    # ── Camera intrinsics override ─────────────────────────────────
    def info_callback(self, msg: CameraInfo):
        # Only override hardcoded values if published data is non-zero
        # (yaml may not load correctly from v4l2_camera, hardcoded is fallback)
        if msg.k[0] != 0.0:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)

    # ── Main detection loop ────────────────────────────────────────
    def image_callback(self, msg: Image):
        # v4l2_camera outputs yuv422 converted to rgb8,
        # requesting bgr8 so OpenCV handles it correctly
        #self.get_logger().info('image_callback triggered')
        frame    = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #self.get_logger().info(f'Frame shape: {frame.shape}, mean: {frame.mean():.2f}')
        gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # --- TEMP: undistort test ---
        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        annotated = undistorted.copy()
            #test_corners, test_ids, _ = self.detector.detectMarkers(gray)
            #self.get_logger().info(f'Raw detection result: {test_ids}')
        #annotated = frame.copy()

        h, w      = frame.shape[:2]
        frame_cx  = w // 2
        frame_cy  = h // 2

        # Green crosshair at image center = drone body frame reference point
        cv2.drawMarker(annotated, (frame_cx, frame_cy),
                       (0, 255, 0), cv2.MARKER_CROSS, 30, 2)

        corners, ids, _ = self.detector.detectMarkers(gray)
        #self.get_logger().info(f'all detected ids: {ids}')
        #cv2.imwrite('/home/cpri/Pictures/Screenshots/frame1.png', gray)

        if ids is not None and 0 in ids.flatten():
            idx            = list(ids.flatten()).index(0)
            target_corners = corners[idx]

            # Draw marker outline and ID
            aruco.drawDetectedMarkers(annotated, corners, ids)

            obj_points = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                target_corners[0],
                self.camera_matrix,
                self.dist_coeffs
            )

            if success:
                x, y, z = tvec.flatten()

                # Marker center in pixel coords
                marker_center = np.mean(target_corners[0], axis=0).astype(int)
                marker_cx, marker_cy = marker_center

                # Red line: drone frame center → marker center (the error vector)
                cv2.line(annotated,
                         (frame_cx, frame_cy),
                         (marker_cx, marker_cy),
                         (0, 0, 255), 2)
                cv2.circle(annotated, (marker_cx, marker_cy), 6, (0, 0, 255), -1)

                # Overlay x_rel, y_rel, z_rel
                cv2.putText(annotated, f'x_rel: {x:.3f} m', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(annotated, f'y_rel: {y:.3f} m', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(annotated, f'z_rel (alt): {z:.3f} m', (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                self.get_logger().info(
                    f'Marker detected → x:{x:.3f}  y:{y:.3f}  z:{z:.3f}'
                )
                self.publish_landing_target(x, y, z, msg.header.stamp)

        else:
            cv2.putText(annotated, 'No marker detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.get_logger().info('No marker detected', throttle_duration_sec=1)

        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.debug_pub.publish(debug_msg)
        

    # ── Publish LandingTargetPose to PX4 ──────────────────────────
    def publish_landing_target(self, x, y, z, stamp):
        # NOTE: This assumes camera frame axes align with NED body frame.
        # Verify this after first test — wrong frame transform = wrong landing direction.
        msg = LandingTargetPose()

        msg.x_rel = float(x)   # forward (NED x)
        msg.y_rel = float(y)   # right   (NED y)
        msg.z_rel = float(z)   # down    (NED z)

        msg.vx_rel = 0.0
        msg.vy_rel = 0.0

        msg.rel_pos_valid = True
        msg.rel_vel_valid = False
        msg.abs_pos_valid = False
        msg.is_static     = True

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
