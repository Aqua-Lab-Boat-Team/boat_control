from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boat_control',
            namespace='b_ctrl1',
            executable='gcs_interface',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
            ros_arguments=['--log-level', 'warn']
        )
    ])

