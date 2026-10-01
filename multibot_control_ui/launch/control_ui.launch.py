"""Launch the control UI in the domain-bridge control domain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the UI node with ROS_DOMAIN_ID 22."""
    zone_config = PathJoinSubstitution([
        FindPackageShare('multibot_control_ui'),
        'config',
        'bottleneck_zones.yaml',
    ])
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '22'),
        DeclareLaunchArgument(
            'runtime_zone_robot_radius_m',
            default_value='0.09',
            description='Robot footprint radius used by UI-drawn zones.',
        ),
        DeclareLaunchArgument(
            'runtime_zone_clearance_margin_m',
            default_value='0.01',
            description='Extra clearance outside the robot radius.',
        ),
        Node(
            package='multibot_control_ui',
            executable='control_ui',
            name='multibot_control_ui',
            output='screen',
            parameters=[{
                'input_cmd_vel_topic': '/cmd_vel',
                'lane_exit_position_variance': 0.0025,
                'lane_exit_yaw_variance': 0.0012184696791468343,
                'command_timeout_sec': 0.5,
                'permit_ttl_sec': 0.5,
                'permit_publish_rate_hz': 10.0,
                'heartbeat_stale_sec': 1.5,
                'pose_stale_sec': 3.0,
                'zone_config_file': zone_config,
                'runtime_zone_robot_radius_m': ParameterValue(
                    LaunchConfiguration('runtime_zone_robot_radius_m'),
                    value_type=float,
                ),
                'runtime_zone_clearance_margin_m': ParameterValue(
                    LaunchConfiguration('runtime_zone_clearance_margin_m'),
                    value_type=float,
                ),
            }],
        ),
    ])
