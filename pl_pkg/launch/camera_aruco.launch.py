"""
Launch file for v4l2_camera_node.
 
Uses a persistent /dev/v4l/by-id/... symlink by default so the camera
device path stays stable across replugs / re-enumeration, instead of
relying on /dev/videoX which can shift.
 
Usage:
    ros2 launch <your_package> v4l2_camera_launch.py
 
Override device or calibration file at runtime:
    ros2 launch <your_package> v4l2_camera_launch.py \
        video_device:=/dev/video2 \
        camera_info_url:=file:///home/cpri/lv_precision_landing_ws/src/pl_pkg/include/cam_calib.yaml
"""
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def generate_launch_description():
    # --- Launch arguments (override from CLI with name:=value) ---
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        # TODO: replace with your actual by-id symlink from
        # `ls -l /dev/v4l/by-id/` on the Jetson, e.g.:
        # /dev/v4l/by-id/usb-Intel_R__RealSense_D435I-video-index0
        default_value='/dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0',
        #usb-icSpring_icspring_camera-video-index0
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
 
    image_size_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )
 
    # --- Node ---
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
            'image_size': [
                int(640),
                int(480),
            ],
        }],
    )
 
    return LaunchDescription([
        video_device_arg,
        camera_info_url_arg,
        camera_frame_id_arg,
        pixel_format_arg,
        image_size_arg,
        image_height_arg,
        v4l2_camera_node,
    ])