from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auv_sim',
            executable='motion_sim',
            name='motion_sim',
            output='screen',
        ),
        Node(
            package='auv_sim',
            executable='depth_sim',
            name='depth_sim',
            output='screen',
        ),
        Node(
            package='auv_sim',
            executable='servo_sim',
            name='servo_sim',
            output='screen',
        ),
        Node(
            package='auv_sim',
            executable='battery_system_sim',
            name='battery_system_sim',
            output='screen',
        ),
    ])
