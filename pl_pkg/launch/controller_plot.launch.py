"""
Launch file: precision landing controller + PlotJuggler.

Starts:
  - controller_vel_pid_hw  (velocity PID precision landing controller)
  - plotjuggler            (subscribe to /cmd_vel to monitor PID output live)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    controller_node = Node(
        package='pl_pkg',
        executable='controller_vel_pid_hw.py',
        name='precision_land_controller',
        output='screen',
    )

    # plotjuggler_node = Node(
    #     package='plotjuggler',
    #     executable='plotjuggler',
    #     name='plotjuggler',
    #     output='screen',
    # )

    return LaunchDescription([
        controller_node,
        #plotjuggler_node,
    ])
