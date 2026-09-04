from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_gps = LaunchConfiguration('use_sim_gps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_gps',
            default_value='false',
            description='Use simulated GPS instead of real GPS'
        ),
        DeclareLaunchArgument(
            'use_sim_boat',
            default_value='false',
            description='Use simulated boat'
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
            condition=IfCondition(use_sim_gps)
        )
    ])
