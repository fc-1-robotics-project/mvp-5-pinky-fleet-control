"""Per-domain Nav2 action clients used by the fleet control UI."""

import math
from threading import Lock, Thread
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .map_math import yaw_degrees_to_quaternion
from .robot_config import RobotConfig, ROBOTS


class RobotNavigationClient:
    """Run a NavigateToPose client directly in one robot's ROS domain."""

    def __init__(self, robot: RobotConfig) -> None:
        self.robot = robot

        # Nav2 actions are not bridged. Each robot therefore needs its own
        # ROS context and executor connected directly to that robot's domain.
        self.context = Context()
        rclpy.init(context=self.context, domain_id=robot.domain_id)
        self.node = Node(
            f'{robot.name}_navigation_client',
            context=self.context,
            use_global_arguments=False,
        )
        self.action_client = ActionClient(
            self.node,
            NavigateToPose,
            robot.navigation_action,
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = Thread(
            target=self.executor.spin,
            name=f'{robot.name}-nav2-action',
            daemon=True,
        )
        self.lock = Lock()
        self._status = 'Nav2 검색 중'
        self._state = 'SEARCHING'
        self._goal_handle = None
        self._cancel_requested = False
        self._last_goal: Optional[Tuple[float, float, float]] = None

        # A generation token prevents callbacks from a superseded goal from
        # overwriting the status of the newest request.
        self._goal_generation = 0
        self.thread.start()

    def send_goal(self, x: float, y: float, yaw_degrees: float) -> bool:
        """Send a map-frame goal without blocking the Tkinter event loop."""
        with self.lock:
            if self._state in {'PENDING', 'ACTIVE', 'CANCELLING'} or self._goal_handle is not None:
                return False  # A cancel acknowledgement is not a terminal result.
        if not all(math.isfinite(value) for value in (x, y, yaw_degrees)):
            self._set_status('잘못된 목표 좌표')
            self._set_state('ERROR')
            return False
        if not self.action_client.server_is_ready():
            self._set_status('Nav2 액션 서버 연결 안 됨')
            self._set_state('UNAVAILABLE')
            return False

        orientation_z, orientation_w = yaw_degrees_to_quaternion(yaw_degrees)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = orientation_z
        goal.pose.pose.orientation.w = orientation_w

        with self.lock:
            if self._state in {'PENDING', 'ACTIVE', 'CANCELLING'} or self._goal_handle is not None:
                return False
            self._goal_generation += 1
            generation = self._goal_generation
            self._last_goal = (x, y, yaw_degrees)
            self._status = '목표 승인 대기 중'
            self._state = 'PENDING'
            self._cancel_requested = False
        try:
            future = self.action_client.send_goal_async(
                goal,
                feedback_callback=lambda message: self._feedback_callback(message, generation),
            )
        except Exception as error:
            self._set_result_if_current(generation, f'목표 전송 확인 실패: {error}', 'CANCELLING')
            return False  # Unknown remote acceptance requires STOP confirmation.
        future.add_done_callback(
            lambda result: self._goal_response_callback(
                result,
                generation,
            ),
        )
        return True

    def cancel_goal(self) -> bool:
        """Request cancellation of this client's active goal."""
        with self.lock:
            goal_handle = self._goal_handle
            generation = self._goal_generation
            if self._cancel_requested:
                return True
            if goal_handle is None and self._state != 'PENDING':
                return False
            self._cancel_requested = True
            self._status = '목표 취소 요청 중'
            self._state = 'CANCELLING'
        if goal_handle is None:
            return True  # Cancel immediately when a pending acceptance arrives.
        self._cancel_handle(goal_handle, generation)
        return True

    def _cancel_handle(self, goal_handle, generation):
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda result: self._cancel_done_callback(result, generation),
            )
        except Exception as error:
            with self.lock:
                if generation == self._goal_generation and self._cancel_requested:
                    self._status = f'취소 전송 실패: {error}'
                    self._cancel_requested = False  # Permit another RPC; retain state and handle.

    def status(self) -> str:
        """Return a thread-safe navigation status for presentation."""
        with self.lock:
            status = self._status
        if status == 'Nav2 검색 중' and self.action_client.server_is_ready():
            self._set_status('Nav2 준비됨')
            self._set_state('READY')
            return 'Nav2 준비됨'
        return status

    def last_goal(self) -> Optional[Tuple[float, float, float]]:
        """Return the last accepted-for-sending goal request."""
        with self.lock:
            return self._last_goal

    def state(self) -> str:
        """Return a stable machine-readable navigation state."""
        with self.lock:
            return self._state

    def has_active_goal(self) -> bool:
        """Report whether a goal handle remains without a terminal result."""
        with self.lock:
            return self._goal_handle is not None

    def shutdown(self) -> None:
        """Stop the domain-specific executor and release its ROS context."""
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=1.0)
        self.action_client.destroy()
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)

    def _goal_response_callback(self, future, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # ROS future exceptions are implementation-specific.
            self._set_result_if_current(
                generation,
                f'목표 전송 실패: {error}',
                'CANCELLING',
            )
            return
        if not goal_handle.accepted:
            self._set_result_if_current(
                generation,
                'Nav2가 목표를 거부함',
                'REJECTED',
            )
            return

        with self.lock:
            if generation != self._goal_generation:
                goal_handle.cancel_goal_async()
                return
            self._goal_handle = goal_handle
            cancel_requested = self._cancel_requested
            self._status = '목표 취소 요청 중' if cancel_requested else '주행 중'
            self._state = 'CANCELLING' if cancel_requested else 'ACTIVE'
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._result_callback(result, generation),
            )
        except Exception as error:
            self._set_result_if_current(generation, f'결과 요청 실패: {error}', 'CANCELLING')
            cancel_requested = True
            with self.lock:
                self._cancel_requested = True
        if cancel_requested:
            self._cancel_handle(goal_handle, generation)

    def _feedback_callback(self, feedback_message, generation: int) -> None:
        remaining = feedback_message.feedback.distance_remaining
        with self.lock:
            if generation == self._goal_generation and not self._cancel_requested:
                self._status = f'주행 중 · 남은 거리 {remaining:.2f} m'

    def _result_callback(self, future, generation: int) -> None:
        try:
            result = future.result()
            status = result.status
            error_code = result.result.error_code
        except Exception as error:  # ROS future exceptions are implementation-specific.
            self._set_result_if_current(
                generation,
                f'결과 수신 실패: {error}',
                'CANCELLING',
            )
            return

        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and error_code == NavigateToPose.Result.NONE
        ):
            text = '목표 도착 완료'
            state = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            text = '목표 취소됨'
            state = 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            text = f'주행 실패 · 오류 {error_code}'
            state = 'ABORTED'
        else:
            text = f'주행 종료 · 상태 {status} · 오류 {error_code}'
            state = 'ERROR'
        with self.lock:
            if generation != self._goal_generation:
                return
            self._goal_handle = None
            self._cancel_requested = False
            self._status = text
            self._state = state

    def _cancel_done_callback(self, future, generation: int) -> None:
        try:
            response = future.result()
            cancelled = bool(response.goals_canceling)
            text = '목표 취소 처리 중' if cancelled else '목표 취소 거부됨'
        except Exception as error:  # ROS future exceptions are implementation-specific.
            text = f'취소 실패: {error}'
            cancelled = False
        with self.lock:
            if generation != self._goal_generation or not self._cancel_requested:
                return
            self._status = text  # Keep waiting for the old goal's terminal result.
            if not cancelled:
                self._cancel_requested = False  # Retain the handle but allow cancellation retry.

    def _set_status(self, text: str) -> None:
        with self.lock:
            self._status = text

    def _set_state(self, state: str) -> None:
        with self.lock:
            self._state = state

    def _set_result_if_current(
        self,
        generation: int,
        status: str,
        state: str,
    ) -> None:
        """Update shared action state only for the newest submitted goal."""
        with self.lock:
            if generation != self._goal_generation:
                return
            self._status = status
            self._state = state


class FleetNavigationClients:
    """Own one action client and ROS context for every robot domain."""

    def __init__(self) -> None:
        self.clients: Dict[str, RobotNavigationClient] = {
            robot.name: RobotNavigationClient(robot)
            for robot in ROBOTS
        }

    def send_goal(
        self,
        robot_name: str,
        x: float,
        y: float,
        yaw_degrees: float,
    ) -> bool:
        """Send a goal to a named robot."""
        return self.clients[robot_name].send_goal(x, y, yaw_degrees)

    def cancel_goal(self, robot_name: str) -> bool:
        """Cancel a named robot's active goal."""
        return self.clients[robot_name].cancel_goal()

    def status(self, robot_name: str) -> str:
        """Get a named robot's navigation status."""
        return self.clients[robot_name].status()

    def state(self, robot_name: str) -> str:
        """Get a machine-readable navigation state for one robot."""
        return self.clients[robot_name].state()

    def has_active_goal(self, robot_name: str) -> bool:
        """Report whether the named robot still has an accepted goal handle."""
        return self.clients[robot_name].has_active_goal()

    def shutdown(self) -> None:
        """Shut down every domain-specific action client."""
        for client in self.clients.values():
            client.shutdown()
