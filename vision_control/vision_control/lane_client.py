"""Per-domain FollowLane action clients for the central control PC."""

import math
from threading import Lock, Thread
from typing import Dict

from action_msgs.msg import GoalStatus
from pinky_interfaces.action import FollowLane
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


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
        self.thread.start()

    def send_goal(
        self,
        mission_id: str,
        route_id: str,
        detection_timeout_sec: float,
        max_duration_sec: float,
    ) -> bool:
        """Send one lane segment without blocking the fleet UI."""
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
        if handle is None:
            return False
        self._set_result('차선 임무 취소 요청 중', 'CANCELLING')
        handle.cancel_goal_async()
        return True

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
            self._set_if_current(generation, f'차선 목표 전송 실패: {error}', 'ERROR')
            return
        if not goal_handle.accepted:
            self._set_if_current(generation, '차선 임무가 거부됨', 'REJECTED')
            return
        with self.lock:
            if generation != self._generation:
                goal_handle.cancel_goal_async()
                return
            self._goal_handle = goal_handle
            self._state = 'WAITING_FOR_LANE'
            self._status = '차선 준비 대기 중'
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
        detection_timeout_sec: float = 5.0,
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

    def shutdown(self) -> None:
        for client in self.clients.values():
            client.shutdown()
