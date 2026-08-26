from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_gps = LaunchConfiguration('use_sim_gps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_gps',
            default_value='false',
            description='Use simulated GPS instead of real GPS'
        ),
        Node(
            package='boat_control',
            executable='gcs_interface',
        ),
        Node(
            package='boat_control',
            executable='mission_manager',
            parameters=[
                {
                    'use_sim_gps': use_sim_gps
                }
            ],
        ),
        Node(
            package='boat_control',
            executable='vehicle_supervisor',
        ),
        Node(
            package='boat_control',
            executable='vehicle_controller',
        ),
        Node(
            package='boat_control',
            executable='sim_gps',
        )
    ])
