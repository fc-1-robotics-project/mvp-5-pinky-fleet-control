"""ROS interfaces for command routing and fleet localisation monitoring."""

import math
import time
from typing import Dict, Optional
import uuid

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from pinky_interfaces.msg import FleetPermit, RobotHeartbeat
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy

from .map_math import yaw_degrees_to_quaternion
from .robot_config import ROBOT_BY_NAME, ROBOTS


class FleetControlNode(Node):
    """Route a common velocity input and collect map/localisation data."""

    def __init__(self) -> None:
        super().__init__('multibot_control_ui')

        # Launch defaults are repeated here so direct ``ros2 run`` execution
        # remains usable. Keep both locations aligned when adding parameters.
        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('permit_ttl_sec', 0.5)
        self.declare_parameter('permit_publish_rate_hz', 10.0)
        self.declare_parameter('heartbeat_stale_sec', 1.5)
        self.declare_parameter('pose_stale_sec', 3.0)
        self.declare_parameter('zone_config_file', '')
        self.declare_parameter('runtime_zone_robot_radius_m', 0.09)
        self.declare_parameter('runtime_zone_clearance_margin_m', 0.01)
        input_topic = self.get_parameter(
            'input_cmd_vel_topic',
        ).get_parameter_value().string_value
        self.command_timeout = self.get_parameter(
            'command_timeout_sec',
        ).get_parameter_value().double_value
        if self.command_timeout <= 0.0:
            raise ValueError('command_timeout_sec must be greater than zero')
        self.permit_ttl_sec = self._positive_parameter('permit_ttl_sec')
        permit_rate = self._positive_parameter('permit_publish_rate_hz')
        self.heartbeat_stale_sec = self._positive_parameter(
            'heartbeat_stale_sec',
        )
        self.pose_stale_sec = self._positive_parameter('pose_stale_sec')
        self.zone_config_file = self.get_parameter('zone_config_file').value
        self.runtime_zone_robot_radius_m = self._positive_parameter(
            'runtime_zone_robot_radius_m',
        )
        self.runtime_zone_clearance_margin_m = self._nonnegative_parameter(
            'runtime_zone_clearance_margin_m',
        )

        # ROS callbacks only update these caches. The coordinator reads them
        # later from the Tk refresh loop and owns all motion-policy decisions.
        self.selected_robot: Optional[str] = None
        self.last_command_robot: Optional[str] = None
        self.last_command_time: Optional[float] = None
        self.command_active = False
        self.dropped_command_count = 0

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_map_source: Optional[str] = None
        self.map_generation = 0
        self.poses: Dict[str, PoseWithCovarianceStamped] = {}
        self.pose_received_at: Dict[str, float] = {}
        self.controller_id = f'control-ui-{uuid.uuid4().hex}'
        self.gate_modes = {
            robot.name: FleetPermit.MODE_HOLD
            for robot in ROBOTS
        }
        self.gate_reasons = {
            robot.name: 'STARTUP'
            for robot in ROBOTS
        }
        self.gate_lease_ids = {robot.name: '' for robot in ROBOTS}
        self.gate_allowed_zones = {robot.name: [] for robot in ROBOTS}
        self.permit_sequences = {robot.name: 0 for robot in ROBOTS}
        self.heartbeats: Dict[str, RobotHeartbeat] = {}
        self.heartbeat_received_at: Dict[str, float] = {}

        # All bridged endpoints are namespaced on domain 22. Their robot-local
        # names and domains are defined separately in domain_bridge.yaml.
        self.command_publishers = {
            robot.name: self.create_publisher(
                Twist,
                robot.command_topic,
                10,
            )
            for robot in ROBOTS
        }
        self.initial_pose_publishers = {
            robot.name: self.create_publisher(
                PoseWithCovarianceStamped,
                robot.initial_pose_topic,
                10,
            )
            for robot in ROBOTS
        }
        permit_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.permit_publishers = {
            robot.name: self.create_publisher(
                FleetPermit,
                f'/{robot.name}/fleet/permit',
                permit_qos,
            )
            for robot in ROBOTS
        }
        self.command_subscription = self.create_subscription(
            Twist,
            input_topic,
            self._route_command,
            10,
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                robot.map_topic,
                lambda message, name=robot.name: self._map_callback(
                    name,
                    message,
                ),
                map_qos,
            )
            for robot in ROBOTS
        ]
        self.pose_subscriptions = [
            self.create_subscription(
                PoseWithCovarianceStamped,
                robot.pose_topic,
                lambda message, name=robot.name: self._pose_callback(
                    name,
                    message,
                ),
                10,
            )
            for robot in ROBOTS
        ]
        self.heartbeat_subscriptions = [
            self.create_subscription(
                RobotHeartbeat,
                f'/{robot.name}/fleet/heartbeat',
                lambda message, name=robot.name: self._heartbeat_callback(
                    name,
                    message,
                ),
                10,
            )
            for robot in ROBOTS
        ]

        # The command watchdog stops stale teleoperation. Permit publication
        # is independent and keeps each short-lived gate lease refreshed.
        self.watchdog_timer = self.create_timer(0.1, self._watchdog)
        self.permit_timer = self.create_timer(
            1.0 / permit_rate,
            self.publish_permits_now,
        )
        self.get_logger().info(
            f'Listening on {input_topic}; select a robot in the UI to route '
            'velocity commands.',
        )

    def select_robot(self, robot_name: str) -> None:
        """Select the command target, stopping the previous target first."""
        if robot_name not in ROBOT_BY_NAME:
            raise ValueError(f'Unknown robot: {robot_name}')
        if robot_name == self.selected_robot:
            return
        if self.selected_robot is not None:
            self.publish_stop(self.selected_robot)
        self.selected_robot = robot_name
        self.command_active = False
        self.last_command_time = None
        self.get_logger().info(f'Command target selected: {robot_name}')

    def clear_selection(self) -> None:
        """Stop the selected robot and disable routing."""
        if self.selected_robot is not None:
            self.publish_stop(self.selected_robot)
        self.selected_robot = None
        self.command_active = False
        self.last_command_time = None

    def publish_stop(self, robot_name: str) -> None:
        """Publish a zero velocity command to one robot."""
        self.command_publishers[robot_name].publish(Twist())

    def set_gate_mode(
        self,
        robot_name: str,
        mode: int,
        *,
        lease_id: str = '',
        allowed_zone_ids=None,
        reason: str = '',
    ) -> None:
        """Set the short-lived permit state repeatedly sent to one robot."""
        if robot_name not in ROBOT_BY_NAME:
            raise ValueError(f'Unknown robot: {robot_name}')
        valid_modes = (
            FleetPermit.MODE_HOLD,
            FleetPermit.MODE_RUN,
            FleetPermit.MODE_ESTOP,
        )
        if mode not in valid_modes:
            raise ValueError(f'Unknown gate mode: {mode}')
        zones = list(allowed_zone_ids or [])
        changed = (
            self.gate_modes[robot_name] != mode
            or self.gate_lease_ids[robot_name] != lease_id
            or self.gate_allowed_zones[robot_name] != zones
        )
        self.gate_modes[robot_name] = mode
        self.gate_lease_ids[robot_name] = lease_id
        self.gate_allowed_zones[robot_name] = zones
        self.gate_reasons[robot_name] = reason
        if changed:
            self.permit_sequences[robot_name] += 1

    def publish_permits_now(self) -> None:
        """Refresh every robot's fail-closed permit heartbeat."""
        ttl_nanoseconds = int(self.permit_ttl_sec * 1e9)
        for robot in ROBOTS:
            message = FleetPermit()
            message.header.stamp = self.get_clock().now().to_msg()
            message.robot_id = robot.name
            message.controller_id = self.controller_id
            message.sequence = self.permit_sequences[robot.name]
            message.mode = self.gate_modes[robot.name]
            message.ttl.sec = ttl_nanoseconds // 1_000_000_000
            message.ttl.nanosec = ttl_nanoseconds % 1_000_000_000
            message.lease_id = self.gate_lease_ids[robot.name]
            message.allowed_zone_ids = self.gate_allowed_zones[robot.name]
            self.permit_publishers[robot.name].publish(message)

    def heartbeat_age(self, robot_name: str) -> Optional[float]:
        """Return seconds since the robot-local gate heartbeat arrived."""
        received_at = self.heartbeat_received_at.get(robot_name)
        if received_at is None:
            return None
        return time.monotonic() - received_at

    def heartbeat_is_fresh(self, robot_name: str) -> bool:
        """Return whether gate heartbeat freshness permits fleet operation."""
        age = self.heartbeat_age(robot_name)
        return age is not None and age <= self.heartbeat_stale_sec

    def pose_is_fresh(self, robot_name: str) -> bool:
        """Return whether a recent localization pose exists."""
        age = self.pose_age(robot_name)
        return age is not None and age <= self.pose_stale_sec

    def robot_position(self, robot_name: str):
        """Return the current map-frame x/y position, when available."""
        message = self.poses.get(robot_name)
        if message is None:
            return None
        position = message.pose.pose.position
        return position.x, position.y

    def stop_all(self) -> None:
        """Publish a zero velocity command to every robot."""
        for robot in ROBOTS:
            self.publish_stop(robot.name)
        self.command_active = False

    def publish_initial_pose(
        self,
        robot_name: str,
        x: float,
        y: float,
        yaw_degrees: float,
    ) -> PoseWithCovarianceStamped:
        """Publish an editable map-frame initial pose for one robot."""
        if robot_name not in ROBOT_BY_NAME:
            raise ValueError(f'Unknown robot: {robot_name}')
        if not all(math.isfinite(value) for value in (x, y, yaw_degrees)):
            raise ValueError('Initial pose values must be finite numbers.')

        orientation_z, orientation_w = yaw_degrees_to_quaternion(yaw_degrees)
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = orientation_z
        message.pose.pose.orientation.w = orientation_w
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = math.radians(15.0) ** 2
        self.initial_pose_publishers[robot_name].publish(message)
        self.get_logger().info(
            f'Initial pose sent to {robot_name}: x={x:.3f}, y={y:.3f}, '
            f'yaw={yaw_degrees:.1f} deg',
        )
        return message

    def pose_age(self, robot_name: str) -> Optional[float]:
        """Return seconds since the last pose arrived."""
        received_at = self.pose_received_at.get(robot_name)
        if received_at is None:
            return None
        return time.monotonic() - received_at

    def _route_command(self, message: Twist) -> None:
        if self.selected_robot is None:
            self.dropped_command_count += 1
            return
        if self.gate_modes[self.selected_robot] != FleetPermit.MODE_RUN:
            self.dropped_command_count += 1
            return

        self.command_publishers[self.selected_robot].publish(message)
        self.last_command_robot = self.selected_robot
        self.last_command_time = time.monotonic()
        self.command_active = not self._is_zero_command(message)

    def _map_callback(self, robot_name: str, message: OccupancyGrid) -> None:
        # Both robots use the same map. Keep robot1 as the canonical source,
        # while allowing robot2 to provide the map when robot1 is unavailable.
        if self.latest_map_source == 'robot1' and robot_name != 'robot1':
            return
        self.latest_map = message
        self.latest_map_source = robot_name
        self.map_generation += 1

    def _pose_callback(
        self,
        robot_name: str,
        message: PoseWithCovarianceStamped,
    ) -> None:
        self.poses[robot_name] = message
        self.pose_received_at[robot_name] = time.monotonic()

    def _heartbeat_callback(
        self,
        robot_name: str,
        message: RobotHeartbeat,
    ) -> None:
        if message.robot_id != robot_name:
            self.get_logger().warning(
                f'Ignored mismatched heartbeat on {robot_name}: '
                f'{message.robot_id}',
            )
            return
        self.heartbeats[robot_name] = message
        self.heartbeat_received_at[robot_name] = time.monotonic()

    def _watchdog(self) -> None:
        if not self.command_active or self.last_command_time is None:
            return
        if time.monotonic() - self.last_command_time < self.command_timeout:
            return

        if self.last_command_robot is not None:
            self.publish_stop(self.last_command_robot)
            self.get_logger().warning(
                f'Command timeout: stopped {self.last_command_robot}',
            )
        self.command_active = False

    @staticmethod
    def _is_zero_command(message: Twist) -> bool:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        return all(abs(value) < 1e-9 for value in values)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be zero or greater')
        return value


def create_node(args=None) -> FleetControlNode:
    """Initialise rclpy and return the fleet control node."""
    rclpy.init(args=args)
    return FleetControlNode()
