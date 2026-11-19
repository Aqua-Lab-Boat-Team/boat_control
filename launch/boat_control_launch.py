from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    OrSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_gps = LaunchConfiguration('use_sim_gps')
    use_sim_boat = LaunchConfiguration('use_sim_boat')
    use_waypoint_sim = AndSubstitution(use_sim_gps, NotSubstitution(use_sim_boat))
    use_simulation = OrSubstitution(use_sim_gps, use_sim_boat)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_gps',
            default_value='false',
            description='Use simulated GPS instead of real GPS'
        ),
        DeclareLaunchArgument(
            'use_boat_sim',
            default_value='false',
            description='Alias for use_sim_boat'
        ),
        DeclareLaunchArgument(
            'use_sim_boat',
            default_value=LaunchConfiguration('use_boat_sim'),
            description='Use the thruster-driven boat simulator'
        ),
        DeclareLaunchArgument(
            'initialize_from_mission',
            default_value='true',
            description='Use the first uploaded waypoint as the origin before moving'
        ),
        DeclareLaunchArgument(
            'initial_latitude',
            default_value='0.0',
            description='Starting latitude of the simulated boat in degrees'
        ),
        DeclareLaunchArgument(
            'initial_longitude',
            default_value='0.0',
            description='Starting longitude of the simulated boat in degrees'
        ),
        DeclareLaunchArgument(
            'initial_heading_deg',
            default_value='0.0',
            description='Starting compass heading of the simulated boat in degrees'
        ),
        DeclareLaunchArgument(
            'full_power_speed_mps',
            default_value='1.0',
            description='Simulated speed at 100 percent thruster output before scaling'
        ),
        DeclareLaunchArgument(
            'thruster_spacing_m',
            default_value='1.0',
            description='Distance between the simulated thrusters in meters'
        ),
        Node(
            package='boat_control',
            executable='gcs_interface',
        ),
        Node(
            package='boat_control',
            executable='mission_manager',
            parameters=[{'use_sim_gps': ParameterValue(use_simulation, value_type=bool)}],
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
            executable='low_level_motor_controller',
            condition=UnlessCondition(use_simulation)
        ),
        Node(
            package='boat_control',
            executable='gps',
            condition=UnlessCondition(use_simulation)
        ),
        Node(
            package='boat_control',
            executable='sim_gps',
            condition=IfCondition(use_waypoint_sim)
        ),
        Node(
            package='boat_control',
            executable='sim_boat',
            condition=IfCondition(use_sim_boat),
            parameters=[{
                'respect_supervisor': True,
                'initialize_from_mission': ParameterValue(
                    LaunchConfiguration('initialize_from_mission'), value_type=bool
                ),
                'initial_latitude': ParameterValue(
                    LaunchConfiguration('initial_latitude'), value_type=float
                ),
                'initial_longitude': ParameterValue(
                    LaunchConfiguration('initial_longitude'), value_type=float
                ),
                'initial_heading_deg': ParameterValue(
                    LaunchConfiguration('initial_heading_deg'), value_type=float
                ),
                'full_power_speed_mps': ParameterValue(
                    LaunchConfiguration('full_power_speed_mps'), value_type=float
                ),
                'thruster_spacing_m': ParameterValue(
                    LaunchConfiguration('thruster_spacing_m'), value_type=float
                ),
            }],
        )
    ])
