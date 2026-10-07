"""Static robot registry used by the control node and UI."""

from dataclasses import dataclass
import os
from typing import Tuple


@dataclass(frozen=True)
class RobotConfig:
    """Topics and presentation settings for one robot."""

    name: str
    domain_id: int
    command_topic: str
    pose_topic: str
    map_topic: str
    initial_pose_topic: str
    navigation_action: str
    drive_mode_request_topic: str
    drive_mode_status_topic: str
    lane_finish_topic: str
    default_initial_pose: Tuple[float, float, float]
    color: str


# This registry is shared by the UI, control node, coordinator, and Nav2
# clients. Changes to names/domains/topics must also be reflected in the
# domain bridge configuration and robot-side IDs.
ROBOTS = (
    RobotConfig(
        name='robot1',
        domain_id=21,
        command_topic='/robot1/cmd_vel',
        pose_topic='/robot1/amcl_pose',
        map_topic='/robot1/map',
        initial_pose_topic='/robot1/initialpose',
        navigation_action='/navigate_to_pose',
        drive_mode_request_topic='/robot1/drive/mode_request',
        drive_mode_status_topic='/robot1/drive/mode_status',
        lane_finish_topic='/robot1/lane/finish',
        default_initial_pose=(
            1.3,
            1.0,
            -176.0,
        ),
        color='#1976d2',
    ),
    RobotConfig(
        name='robot2',
        domain_id=19,
        command_topic='/robot2/cmd_vel',
        pose_topic='/robot2/amcl_pose',
        map_topic='/robot2/map',
        initial_pose_topic='/robot2/initialpose',
        navigation_action='/navigate_to_pose',
        drive_mode_request_topic='/robot2/drive/mode_request',
        drive_mode_status_topic='/robot2/drive/mode_status',
        lane_finish_topic='/robot2/lane/finish',
        default_initial_pose=(
            -0.2,
            0.6,
            -90.0,
        ),
        color='#e65100',
    ),
)

# Optional: PINKY_FLEET_ROBOTS=robot2 limits this control PC to the named
# robots, so it sends no permit or mode request to a robot someone else runs.
_active = {name.strip() for name in os.environ.get('PINKY_FLEET_ROBOTS', '').split(',')
           if name.strip()}
if _active - {robot.name for robot in ROBOTS}:
    raise ValueError(f'PINKY_FLEET_ROBOTS has unknown robots: {sorted(_active)}')
if _active:
    ROBOTS = tuple(robot for robot in ROBOTS if robot.name in _active)

ROBOT_BY_NAME ={robot.name: robot for robot in ROBOTS}
