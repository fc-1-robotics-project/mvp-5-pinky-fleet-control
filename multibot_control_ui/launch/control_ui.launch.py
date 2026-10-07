"""Launch the control UI in the domain-bridge control domain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
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
    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', '22'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        Node(package='domain_bridge', executable='domain_bridge',
             name='pinky_fleet_bridge', output='screen',
             arguments=[PathJoinSubstitution([FindPackageShare('multibot_control_ui'),
                                              'config', 'domain_bridge.yaml'])],
             condition=IfCondition(LaunchConfiguration('start_bridge'))),
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
    ]
    ui = actions[-1]
    bridge = actions[2]
    # Stop the launch if either of its essential processes exits.
    actions[0:0] = [RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason=reason))],
    )) for process, reason in ((ui, 'Control UI exited'), (bridge, 'Fleet bridge exited'))]
    return LaunchDescription(actions)
