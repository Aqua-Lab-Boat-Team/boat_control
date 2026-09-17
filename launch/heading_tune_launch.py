from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boat_control',
            executable='gps',
        ),
        Node(
            package='boat_control',
            executable='heading_tuner',
        ),
        Node(
            package='boat_control',
            executable='low_level_motor_controller'
        )
    ])
