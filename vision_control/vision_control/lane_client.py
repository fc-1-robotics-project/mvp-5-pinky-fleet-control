"""Per-domain FollowLane action clients for the central control PC."""

import math
import json
import time
from threading import Lock, Thread
from typing import Dict

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from pinky_interfaces.action import FollowLane
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


# NTP-synchronized hosts can still differ by a few tens of milliseconds.
SOURCE_FUTURE_TOLERANCE_S = .1
SOURCE_MAX_AGE_S = 1.5


class RobotLaneClient:
    """Run one FollowLane client directly in a robot's ROS domain."""

    def __init__(self, robot) -> None:
        self.robot = robot
        self.context = Context()
        rclpy.init(context=self.context, domain_id=robot.domain_id)
        self.node = Node(
            f'{robot.name}_lane_client',
            context=self.context,
            use_global_arguments=False,
        )
        self.action_client = ActionClient(
            self.node, FollowLane, '/follow_lane',
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = Thread(
            target=self.executor.spin,
            name=f'{robot.name}-lane-action',
            daemon=True,
        )
        self.lock = Lock()
        self._status = '차선 액션 검색 중'
        self._state = 'SEARCHING'
        self._goal_handle = None
        self._generation = 0
        self._mission_status = {}
        self._mission_received = 0.
        self._localization = None
        self._fleet_localization = None
        self._localization_epoch = (0., 0.)
        self.node.create_subscription(String, '/lane/mission_status', self._mission_callback, 10)
        self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._localization_callback, qos_profile_sensor_data)
        self.node.create_subscription(PoseWithCovarianceStamped, '/fleet/pose', self._fleet_localization_callback, qos_profile_sensor_data)
        self.thread.start()

    def send_goal(
        self,
        mission_id: str,
        route_id: str,
        detection_timeout_sec: float,
        max_duration_sec: float,
    ) -> bool:
        """Send one lane segment without blocking the fleet UI."""
        if self.state() in {'PENDING', 'WAITING_FOR_LANE', 'FOLLOWING', 'SAFETY_HOLD', 'STOPPING', 'CANCELLING'}:
            return False
        if (not mission_id or not route_id
                or not all(math.isfinite(value) and value > 0.0 for value in (
                    detection_timeout_sec, max_duration_sec,
                ))):
            self._set_result('잘못된 차선 임무', 'ERROR')
            return False
        if not self.action_client.server_is_ready():
            self._set_result('차선 액션 서버 연결 안 됨', 'UNAVAILABLE')
            return False
        goal = FollowLane.Goal()
        goal.mission_id = mission_id
        goal.route_id = route_id
        goal.detection_timeout_sec = float(detection_timeout_sec)
        goal.max_duration_sec = float(max_duration_sec)
        with self.lock:
            self._generation += 1
            generation = self._generation
            self._goal_handle = None
            self._state = 'PENDING'
            self._status = '차선 임무 승인 대기 중'
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda message: self._feedback_callback(
                message, generation,
            ),
        )
        future.add_done_callback(
            lambda result: self._goal_response(result, generation),
        )
        return True

    def cancel_goal(self) -> bool:
        with self.lock:
            handle = self._goal_handle
            pending = self._state in {'PENDING', 'WAITING_FOR_LANE', 'FOLLOWING', 'SAFETY_HOLD', 'STOPPING'}
            if not pending:
                return False
            self._generation += 1
            generation = self._generation
            self._state, self._status = 'CANCELLING', '차선 중단 · 허가 해제 확인 중'
        if handle is not None:
            self._cancel_handle(handle, generation)
        return True

    def _cancel_handle(self, handle, generation):
        handle.cancel_goal_async()
        future = handle.get_result_async()
        future.add_done_callback(lambda _: self._set_if_current(
            generation, '차선 임무 중단됨', 'CANCELED'))

    def _mission_callback(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self._mission_status = data
            self._mission_received = time.monotonic()

    def telemetry(self):
        with self.lock:
            result = dict(self._mission_status)
            result['received_age_s'] = time.monotonic() - self._mission_received
            result['fresh'] = 0 <= result['received_age_s'] <= SOURCE_MAX_AGE_S
            exit_status = result.get('exit_status')
            if isinstance(exit_status, dict) and exit_status.get('version') == 1:
                stamp = result.get('status_time_s')
                now = self.node.get_clock().now().nanoseconds / 1e9
                age = now - stamp if type(stamp) in (int, float) else math.inf
                result['fresh'] = (result['fresh'] and math.isfinite(age)
                                   and -SOURCE_FUTURE_TOLERANCE_S <= age <= SOURCE_MAX_AGE_S)
                result['received_age_s'] = max(result['received_age_s'], age)
        return result

    def _localization_callback(self, message):
        with self.lock:
            self._localization = (time.monotonic(), message)

    def _fleet_localization_callback(self, message):
        with self.lock:
            self._fleet_localization = (time.monotonic(), message)

    def current_localization(self, *, position_variance_limit=.04, yaw_variance_limit=.07):
        """Read a fresh map pose using the caller's covariance limits."""
        return self.localization_status(position_variance_limit=position_variance_limit,
                                        yaw_variance_limit=yaw_variance_limit)['pose']

    def localization_status(self, *, position_variance_limit=.04, yaw_variance_limit=.07):
        """Explain missing/stale/invalid localization without changing AMCL data."""
        if not all(math.isfinite(v) and v > 0 for v in (position_variance_limit, yaw_variance_limit)):
            raise ValueError('Localization variance limits must be finite and positive')
        result = dict(pose=None, reason='위치 정보 수신 대기')
        with self.lock:
            sample = self._fleet_localization
        if sample is None:
            return result
        received_age = time.monotonic() - sample[0]
        if not 0 <= received_age <= SOURCE_MAX_AGE_S:
            result['reason'] = f'위치 정보 수신 지연 ({received_age:.2f}초 / 최대 1.5초)'
            return result
        message = sample[1]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if (not math.isfinite(age)
                or not -SOURCE_FUTURE_TOLERANCE_S <= age <= SOURCE_MAX_AGE_S):
            result['reason'] = (f'위치 원본 시각 불일치/지연 ({age:.2f}초 / '
                                f'허용 -{SOURCE_FUTURE_TOLERANCE_S:.1f}~{SOURCE_MAX_AGE_S:.1f}초)')
            return result
        if message.header.frame_id != 'map':
            result['reason'] = f'위치 좌표계 확인 필요 ({message.header.frame_id} / map 필요)'
            return result
        pose, covariance = message.pose.pose, message.pose.covariance
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  covariance[0], covariance[7], covariance[35])
        if not all(math.isfinite(v) for v in values):
            result['reason'] = '위치/방향/분산에 유효하지 않은 값이 있음'
            return result
        if not .99 <= sum(v*v for v in (q.x, q.y, q.z, q.w)) <= 1.01:
            result['reason'] = '위치 방향 quaternion 확인 필요'
            return result
        result['covariance'] = (covariance[0], covariance[7], covariance[35])
        if any(covariance[i] < 0 for i in (0, 7, 35)):
            result['reason'] = '위치/방향 분산이 음수임'
            return result
        if max(covariance[0], covariance[7]) > position_variance_limit:
            result['reason'] = (f'위치 분산 범위 초과 (x={covariance[0]:.3f}, '
                                f'y={covariance[7]:.3f} / 최대 {position_variance_limit:.3f}m²)')
            return result
        if covariance[35] > yaw_variance_limit:
            result['reason'] = (f'방향 분산 범위 초과 (표준편차 '
                                f'{math.degrees(math.sqrt(covariance[35])):.1f}° / 최대 '
                                f'{math.degrees(math.sqrt(yaw_variance_limit)):.1f}°)')
            return result
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        result.update(pose=(pose.position.x, pose.position.y, math.degrees(yaw)), reason='위치 수신 정상')
        return result

    def mark_localization_reset(self):
        with self.lock:
            self._localization_epoch = (time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9)

    def localization_ready(self, after, target):
        with self.lock:
            sample = self._localization
            epoch = self._localization_epoch
        if sample is None or sample[0] <= max(after, epoch[0]) or time.monotonic() - sample[0] > 1.5:
            return False
        msg = sample[1]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if msg.header.frame_id != 'map' or stamp <= epoch[1]:
            return False
        pose = msg.pose.pose
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35])
        if not all(math.isfinite(v) for v in values):
            return False
        if not .99 <= q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w <= 1.01:
            return False
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        error = math.atan2(math.sin(yaw-math.radians(target[2])), math.cos(yaw-math.radians(target[2])))
        return (math.hypot(pose.position.x-target[0], pose.position.y-target[1]) <= .15
                and abs(error) <= math.radians(15)
                and all(0 <= msg.pose.covariance[i] <= limit for i, limit in ((0,.04),(7,.04),(35,.07))))

    def state(self) -> str:
        with self.lock:
            state = self._state
        if state == 'SEARCHING' and self.action_client.server_is_ready():
            self._set_result('차선 액션 준비됨', 'READY')
            return 'READY'
        return state

    def status(self) -> str:
        self.state()
        with self.lock:
            return self._status

    def shutdown(self) -> None:
        if self.context.ok():
            self.cancel_goal()
        deadline = time.monotonic() + 2.
        while self.context.ok() and self.state() == 'CANCELLING' and time.monotonic() < deadline:
            time.sleep(.02)
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=1.0)
        self.action_client.destroy()
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)

    def _goal_response(self, future, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            with self.lock:
                if self._generation == generation + 1 and self._state == 'CANCELLING':
                    self._state, self._status = 'CANCELED', '차선 요청 취소됨'
            self._set_if_current(generation, f'차선 목표 전송 실패: {error}', 'ERROR')
            return
        if not goal_handle.accepted:
            with self.lock:
                if generation + 1 == self._generation and self._state == 'CANCELLING':
                    self._state, self._status = 'CANCELED', '차선 시작 취소됨'
            self._set_if_current(generation, '차선 임무가 거부됨', 'REJECTED')
            return
        with self.lock:
            obsolete = generation != self._generation
            if not obsolete:
                self._goal_handle = goal_handle
                self._state = 'WAITING_FOR_LANE'
                self._status = '차선 준비 대기 중'
        # An already completed future can invoke its callback immediately.
        # Register outside the state lock so late cancellation cannot deadlock.
        if obsolete:
            self._cancel_handle(goal_handle, generation + 1)
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(
            lambda result: self._result_callback(result, generation),
        )

    def _feedback_callback(self, message, generation: int) -> None:
        feedback = message.feedback
        with self.lock:
            if generation != self._generation:
                return
            self._state = feedback.state
            self._status = f'{feedback.state} · {feedback.detail}'

    def _result_callback(self, future, generation: int) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
        except Exception as error:
            self._set_if_current(generation, f'차선 결과 수신 실패: {error}', 'ERROR')
            return
        if (status == GoalStatus.STATUS_SUCCEEDED
                and result.code == FollowLane.Result.RESULT_SUCCESS):
            state = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            state = 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            state = 'ABORTED'
        else:
            state = 'ERROR'
        with self.lock:
            if generation != self._generation:
                return
            self._goal_handle = None
            self._state = state
            self._status = result.message

    def _set_result(self, status: str, state: str) -> None:
        with self.lock:
            self._status = status
            self._state = state

    def _set_if_current(self, generation: int, status: str, state: str) -> None:
        with self.lock:
            if generation != self._generation:
                return
            self._status = status
            self._state = state


class FleetLaneClients:
    """Own one FollowLane client for every configured robot domain."""

    def __init__(self, robots) -> None:
        self.clients: Dict[str, RobotLaneClient] = {
            robot.name: RobotLaneClient(robot) for robot in robots
        }

    def send_goal(
        self,
        robot_name: str,
        mission_id: str,
        route_id: str = 'right_lane',
        detection_timeout_sec: float = 20.0,
        max_duration_sec: float = 300.0,
    ) -> bool:
        return self.clients[robot_name].send_goal(
            mission_id,
            route_id,
            detection_timeout_sec,
            max_duration_sec,
        )

    def cancel_goal(self, robot_name: str) -> bool:
        return self.clients[robot_name].cancel_goal()

    def state(self, robot_name: str) -> str:
        return self.clients[robot_name].state()

    def status(self, robot_name: str) -> str:
        return self.clients[robot_name].status()

    def telemetry(self, robot_name):
        return self.clients[robot_name].telemetry()

    def current_localization(self, robot_name, **limits):
        return self.clients[robot_name].current_localization(**limits)

    def localization_status(self, robot_name, **limits):
        return self.clients[robot_name].localization_status(**limits)

    def mark_localization_reset(self, robot_name):
        self.clients[robot_name].mark_localization_reset()

    def localization_ready(self, robot_name, after, target):
        return self.clients[robot_name].localization_ready(after, target)

    def shutdown(self) -> None:
        for client in self.clients.values():
            client.shutdown()
