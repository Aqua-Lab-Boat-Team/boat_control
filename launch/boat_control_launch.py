from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boat_control',
            executable='gcs_interface',
        ),
        Node(
            package='boat_control',
            executable='mission_manager',
        ),        Node(
            package='boat_control',
            executable='vehicle_supervisor',
        ),
    ])

