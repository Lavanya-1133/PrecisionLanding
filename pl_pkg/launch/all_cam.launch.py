"""
Launch file: camera + ArUco detection + dual rqt image viewers.

Starts:
  - v4l2_camera_node   (raw camera stream)
  - aruco_detection_hitl (ArUco marker detection + PX4 target publisher)
  - rqt_image_view x2  (/image_raw and /aruco_debug)

Override camera settings at runtime:
    ros2 launch pl_pkg all_cam.launch.py video_device:=/dev/video2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments ---
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0',
        description='Path to the camera device (prefer /dev/v4l/by-id/... for stability)'
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file:///home/cpri/lv_precision_landing_ws/src/pl_pkg/include/cam_calib.yaml',
        description='URL to the camera calibration YAML file'
    )

    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_link',
        description='TF frame id to stamp on published images'
    )

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='YUYV',
        description='Pixel format reported by the camera (check with v4l2-ctl --list-formats-ext)'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )

    # --- Nodes ---
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'image_size': [int(640), int(480)],
        }],
    )

    aruco_detection_node = Node(
        package='pl_pkg',
        executable='aruco_detection_hitl.py',
        name='aruco_landing_detector',
        output='screen',
    )

    rqt_image_raw = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_raw',
        arguments=['/image_raw'],
        output='screen',
    )

    rqt_aruco_debug = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_aruco_debug',
        arguments=['/aruco_debug'],
        output='screen',
    )

    return LaunchDescription([
        video_device_arg,
        camera_info_url_arg,
        camera_frame_id_arg,
        pixel_format_arg,
        image_width_arg,
        image_height_arg,
        v4l2_camera_node,
        aruco_detection_node,
        rqt_image_raw,
        rqt_aruco_debug,
    ])
