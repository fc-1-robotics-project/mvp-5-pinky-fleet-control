"""Tkinter fleet UI for command selection and map monitoring."""

import math
import tkinter as tk
from tkinter import ttk
from typing import Optional, Tuple

import rclpy
from std_srvs.srv import SetBool
from vision_control.lane_client import FleetLaneClients

from .control_node import create_node, FleetControlNode
from .fleet_coordinator import FleetCoordinator
from .map_math import grid_to_world, occupancy_color, quaternion_to_yaw
from .map_math import world_to_grid
from .navigation_client import FleetNavigationClients
from .robot_config import ROBOT_BY_NAME, ROBOTS
from .zone_config import load_zones


ROS_POLL_INTERVAL_MS = 20
UI_REFRESH_INTERVAL_MS = 100


class MultiBotControlUI:
    """
    Coordinate Tk widgets with the ROS node and fleet state machine.

    Fleet policy stays in ``FleetCoordinator`` and ROS I/O stays in
    ``FleetControlNode``. This class only handles user events, periodic
    refreshes, and drawing the latest shared state.
    """

    def __init__(
        self,
        node: FleetControlNode,
        navigation: FleetNavigationClients,
        coordinator: FleetCoordinator,
        lane_navigation: FleetLaneClients,
    ) -> None:
        self.node = node
        self.navigation = navigation
        self.coordinator = coordinator
        self.lane_navigation = lane_navigation
        self.root = tk.Tk()
        self.root.title('Pinky Pro Multi-Robot Control')
        self.root.minsize(1180, 740)
        self.root.configure(background='#0b1220')
        self._configure_styles()

        self.selected_robot = tk.StringVar(value='')
        self.routing_status = tk.StringVar(
            value='/cmd_vel 대기 중 - 먼저 로봇을 선택하세요.',
        )
        self.map_status = tk.StringVar(value='맵 수신 대기 중')
        self.robot_status = {
            robot.name: tk.StringVar(value='위치 수신 대기 중')
            for robot in ROBOTS
        }
        self.initial_pose_values = {
            robot.name: {
                'x': tk.StringVar(value=f'{robot.default_initial_pose[0]:.6f}'),
                'y': tk.StringVar(value=f'{robot.default_initial_pose[1]:.6f}'),
                'yaw': tk.StringVar(
                    value=f'{robot.default_initial_pose[2]:.3f}',
                ),
            }
            for robot in ROBOTS
        }
        self.initial_pose_status = tk.StringVar(
            value='좌표를 수정한 뒤 로봇별 설정 버튼을 누르세요.',
        )
        self.goal_status = tk.StringVar(
            value='로봇 선택 후 지도에서 클릭하고 진행 방향으로 드래그하세요.',
        )
        self.lane_after_goal = tk.BooleanVar(value=False)
        self.fleet_status = tk.StringVar(value=coordinator.summary)
        self.gate_status = tk.StringVar(value='로봇 gate heartbeat 수신 대기 중')
        self.navigation_status = {
            robot.name: tk.StringVar(value='Nav2 검색 중')
            for robot in ROBOTS
        }
        self.robot_buttons = {}
        self.default_button_colours = {}
        self.goal_markers = {}
        self.goal_drag_start = None
        self.zone_edit_mode = False
        self.zone_drag_start = None
        self.map_photo: Optional[tk.PhotoImage] = None
        self.scaled_map_photo: Optional[tk.PhotoImage] = None
        self.rendered_map_generation = -1
        self.map_geometry: Optional[Tuple[float, float, float]] = None
        self.map_display = (0.0, 0.0, 1.0)
        self.closing = False

        self._build_ui()
        self.lane_test_services = [
            node.create_service(
                SetBool, f'/{robot.name}/lane/test',
                lambda request, response, name=robot.name:
                    self._lane_test_request(name, request, response),
            )
            for robot in ROBOTS
        ]
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(ROS_POLL_INTERVAL_MS, self._poll_ros)
        self.root.after(UI_REFRESH_INTERVAL_MS, self._refresh_ui)

    def _configure_styles(self) -> None:
        """Configure a compact dark theme for the desktop dashboard."""
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('TFrame', background='#0b1220')
        style.configure(
            'TLabelframe',
            background='#111c2e',
            foreground='#e8eef8',
            borderwidth=1,
            relief='solid',
        )
        style.configure(
            'TLabelframe.Label',
            background='#0b1220',
            foreground='#93c5fd',
            font=('DejaVu Sans', 11, 'bold'),
        )
        style.configure('TLabel', background='#111c2e', foreground='#dbe7f5')
        style.configure(
            'Title.TLabel',
            background='#0b1220',
            foreground='#f8fafc',
            font=('DejaVu Sans', 21, 'bold'),
        )
        style.configure(
            'Subtitle.TLabel',
            background='#0b1220',
            foreground='#8293aa',
            font=('DejaVu Sans', 10),
        )
        style.configure(
            'TEntry',
            fieldbackground='#0c1628',
            foreground='#f8fafc',
            insertcolor='#f8fafc',
            bordercolor='#334155',
            lightcolor='#334155',
            darkcolor='#334155',
            padding=5,
        )
        style.configure(
            'TButton',
            background='#24334d',
            foreground='#e8eef8',
            borderwidth=0,
            padding=(10, 7),
        )
        style.map(
            'TButton',
            background=[('active', '#334a70'), ('pressed', '#1e3a5f')],
        )
        style.configure('TSeparator', background='#334155')

    def _build_ui(self) -> None:
        """Assemble the window from responsibility-based panels."""
        container = ttk.Frame(self.root, padding=12)
        container.grid(row=0, column=0, sticky='nsew')
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        container.rowconfigure(1, weight=1)
        container.columnconfigure(1, weight=1)

        heading = ttk.Label(
            container,
            text='Pinky Pro 멀티로봇 관제',
            style='Title.TLabel',
        )
        heading.grid(row=0, column=0, sticky='w', pady=(0, 12))
        ttk.Label(
            container,
            text='LIVE FLEET CONTROL  ·  DOMAIN 22',
            style='Subtitle.TLabel',
        ).grid(row=0, column=1, sticky='e', pady=(0, 12))

        controls_shell = ttk.Frame(container)
        controls_shell.grid(row=1, column=0, sticky='nsew', padx=(0, 12))
        controls_shell.rowconfigure(0, weight=1)
        controls_shell.columnconfigure(0, weight=1)

        self.controls_canvas = tk.Canvas(
            controls_shell,
            width=430,
            background='#0b1220',
            highlightthickness=0,
        )
        controls_scrollbar = ttk.Scrollbar(
            controls_shell,
            orient='vertical',
            command=self.controls_canvas.yview,
        )
        self.controls_canvas.configure(
            yscrollcommand=controls_scrollbar.set,
        )
        self.controls_canvas.grid(row=0, column=0, sticky='nsew')
        controls_scrollbar.grid(row=0, column=1, sticky='ns')

        controls = ttk.LabelFrame(
            self.controls_canvas,
            text='명령 대상',
            padding=12,
        )
        self.controls_window = self.controls_canvas.create_window(
            (0, 0),
            window=controls,
            anchor='nw',
        )
        controls.bind('<Configure>', self._on_controls_content_resize)
        self.controls_canvas.bind(
            '<Configure>',
            self._on_controls_canvas_resize,
        )
        self.root.bind_all('<MouseWheel>', self._on_controls_mousewheel)
        self.root.bind_all('<Button-4>', self._on_controls_mousewheel)
        self.root.bind_all('<Button-5>', self._on_controls_mousewheel)

        next_row = self._build_robot_selection_panel(controls)
        next_row = self._build_initial_pose_panel(controls, next_row)
        next_row = self._build_navigation_panel(controls, next_row)
        self._build_fleet_panel(controls, next_row)
        self._build_map_panel(container)

    def _on_controls_content_resize(self, _event: tk.Event) -> None:
        """Keep the control-column scrollbar aligned with its contents."""
        self.controls_canvas.configure(
            scrollregion=self.controls_canvas.bbox('all'),
        )

    def _on_controls_canvas_resize(self, event: tk.Event) -> None:
        """Fill the available control-column width without clipping widgets."""
        self.controls_canvas.itemconfigure(
            self.controls_window,
            width=event.width,
        )

    def _on_controls_mousewheel(self, event: tk.Event) -> Optional[str]:
        """Scroll the command column when the pointer is over that column."""
        pointer_x = self.root.winfo_pointerx()
        pointer_y = self.root.winfo_pointery()
        canvas_x = self.controls_canvas.winfo_rootx()
        canvas_y = self.controls_canvas.winfo_rooty()
        canvas_right = canvas_x + self.controls_canvas.winfo_width()
        canvas_bottom = canvas_y + self.controls_canvas.winfo_height()
        inside_controls = (
            canvas_x <= pointer_x < canvas_right
            and canvas_y <= pointer_y < canvas_bottom
        )
        if not inside_controls:
            return None

        if (
            getattr(event, 'num', None) == 4
            or getattr(event, 'delta', 0) > 0
        ):
            direction = -1
        else:
            direction = 1
        self.controls_canvas.yview_scroll(direction, 'units')
        return 'break'

    def _build_robot_selection_panel(self, controls: ttk.LabelFrame) -> int:
        """Build command-target buttons and return the next grid row."""
        ttk.Label(
            controls,
            text='관제 PC의 /cmd_vel을\n전달할 로봇을 선택하세요.',
            justify='left',
        ).grid(row=0, column=0, sticky='w', pady=(0, 12))

        for item_row, robot in enumerate(ROBOTS, start=1):
            button = tk.Button(
                controls,
                text=f'{robot.name}\nDomain {robot.domain_id}',
                width=18,
                height=3,
                command=lambda name=robot.name: self._select_robot(name),
                relief='flat',
                font=('', 11, 'bold'),
                bg='#1e293b',
                fg='#dbeafe',
                activebackground='#334155',
                activeforeground='#ffffff',
                borderwidth=0,
                cursor='hand2',
            )
            button.grid(row=item_row, column=0, sticky='ew', pady=4)
            self.robot_buttons[robot.name] = button
            self.default_button_colours[robot.name] = (
                button.cget('background'),
                button.cget('foreground'),
                button.cget('activebackground'),
                button.cget('activeforeground'),
            )

            ttk.Label(
                controls,
                textvariable=self.robot_status[robot.name],
                foreground=robot.color,
            ).grid(row=item_row, column=1, sticky='w', padx=(8, 0))

        emergency_button = tk.Button(
            controls,
            text='전체 정지 / 선택 해제',
            command=self._emergency_stop,
            bg='#c62828',
            fg='white',
            activebackground='#8e0000',
            activeforeground='white',
            font=('', 11, 'bold'),
            height=2,
            relief='flat',
            borderwidth=0,
            cursor='hand2',
        )
        emergency_button.grid(
            row=len(ROBOTS) + 1,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(18, 8),
        )

        ttk.Separator(controls).grid(
            row=len(ROBOTS) + 2,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=8,
        )
        ttk.Label(
            controls,
            textvariable=self.routing_status,
            wraplength=280,
            justify='left',
        ).grid(
            row=len(ROBOTS) + 3,
            column=0,
            columnspan=2,
            sticky='w',
        )

        return len(ROBOTS) + 4

    def _build_initial_pose_panel(
        self,
        controls: ttk.LabelFrame,
        row: int,
    ) -> int:
        """Build editable AMCL initial-pose controls."""
        initial_pose_panel = ttk.LabelFrame(
            controls,
            text='AMCL 초기 위치 / 차선 출구 pose',
            padding=8,
        )
        initial_pose_panel.grid(
            row=row,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(18, 0),
        )
        for column, label in enumerate(('로봇', 'x (m)', 'y (m)', 'yaw (°)')):
            ttk.Label(initial_pose_panel, text=label).grid(
                row=0,
                column=column,
                padx=3,
                pady=(0, 4),
            )

        for item_row, robot in enumerate(ROBOTS, start=1):
            values = self.initial_pose_values[robot.name]
            ttk.Label(
                initial_pose_panel,
                text=robot.name,
                foreground=robot.color,
            ).grid(row=item_row, column=0, padx=3, pady=3)
            ttk.Entry(
                initial_pose_panel,
                textvariable=values['x'],
                width=9,
            ).grid(row=item_row, column=1, padx=3, pady=3)
            ttk.Entry(
                initial_pose_panel,
                textvariable=values['y'],
                width=9,
            ).grid(row=item_row, column=2, padx=3, pady=3)
            ttk.Entry(
                initial_pose_panel,
                textvariable=values['yaw'],
                width=8,
            ).grid(row=item_row, column=3, padx=3, pady=3)
            ttk.Button(
                initial_pose_panel,
                text='설정',
                command=lambda name=robot.name: self._set_initial_pose(name),
            ).grid(row=item_row, column=4, padx=(6, 3), pady=3)

        ttk.Label(
            initial_pose_panel,
            textvariable=self.initial_pose_status,
            wraplength=360,
            justify='left',
        ).grid(
            row=len(ROBOTS) + 1,
            column=0,
            columnspan=5,
            sticky='w',
            pady=(6, 0),
        )

        return row + 1

    def _build_navigation_panel(
        self,
        controls: ttk.LabelFrame,
        row: int,
    ) -> int:
        """Build per-robot Nav2 status and cancellation controls."""
        navigation_panel = ttk.LabelFrame(
            controls,
            text='Nav2 주행',
            padding=8,
        )
        navigation_panel.grid(
            row=row,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(14, 0),
        )
        for item_row, robot in enumerate(ROBOTS):
            ttk.Label(
                navigation_panel,
                text=f'{robot.name}  ●',
                foreground=robot.color,
            ).grid(
                row=item_row,
                column=0,
                sticky='w',
                padx=(0, 8),
                pady=2,
            )
            ttk.Label(
                navigation_panel,
                textvariable=self.navigation_status[robot.name],
            ).grid(row=item_row, column=1, sticky='w', pady=2)
        ttk.Button(
            navigation_panel,
            text='선택 로봇 목표 취소',
            command=self._cancel_selected_goal,
        ).grid(
            row=len(ROBOTS),
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0),
        )
        ttk.Checkbutton(
            navigation_panel,
            text='다음 지도 목표 도착 후 우측 차선 주행',
            variable=self.lane_after_goal,
        ).grid(
            row=len(ROBOTS) + 1,
            column=0,
            columnspan=2,
            sticky='w',
            pady=(8, 0),
        )
        ttk.Button(
            navigation_panel,
            text='선택 로봇 수동 / 자율 전환',
            command=self._toggle_manual,
        ).grid(
            row=len(ROBOTS) + 2,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(6, 0),
        )
        ttk.Button(
            navigation_panel,
            text='선택 로봇 차선 단독 테스트 시작',
            command=self._start_lane_test,
        ).grid(
            row=len(ROBOTS) + 4,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(6, 0),
        )
        ttk.Button(
            navigation_panel,
            text='선택 로봇 차선 종료 Bool 전송',
            command=self._publish_lane_finish,
        ).grid(
            row=len(ROBOTS) + 3,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(6, 0),
        )

        return row + 1

    def _build_fleet_panel(
        self,
        controls: ttk.LabelFrame,
        row: int,
    ) -> None:
        """Build supervision and runtime bottleneck controls."""
        fleet_panel = ttk.LabelFrame(
            controls,
            text='병목 관제',
            padding=8,
        )
        fleet_panel.grid(
            row=row,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(14, 0),
        )
        ttk.Button(
            fleet_panel,
            text='관제 시작 / 상태 재확인',
            command=self._start_fleet,
        ).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(
            fleet_panel,
            text='관제 일시정지',
            command=self._pause_fleet,
        ).grid(row=0, column=1, sticky='ew', padx=(4, 0))
        ttk.Button(
            fleet_panel,
            text='맵에서 병목 사각형 지정',
            command=self._begin_zone_edit,
        ).grid(row=1, column=0, sticky='ew', padx=(0, 4), pady=(6, 0))
        ttk.Button(
            fleet_panel,
            text='병목 구역 삭제',
            command=self._clear_zones,
        ).grid(row=1, column=1, sticky='ew', padx=(4, 0), pady=(6, 0))
        ttk.Label(
            fleet_panel,
            textvariable=self.fleet_status,
            wraplength=360,
            justify='left',
        ).grid(row=2, column=0, columnspan=2, sticky='w', pady=(8, 0))
        ttk.Label(
            fleet_panel,
            textvariable=self.gate_status,
            wraplength=360,
            justify='left',
        ).grid(row=3, column=0, columnspan=2, sticky='w', pady=(4, 0))

    def _build_map_panel(self, container: ttk.Frame) -> None:
        """Build the shared-map canvas and bind pointer interactions."""
        map_panel = ttk.LabelFrame(container, text='공유 맵 / 로봇 위치', padding=6)
        map_panel.grid(row=1, column=1, sticky='nsew')
        map_panel.rowconfigure(1, weight=1)
        map_panel.columnconfigure(0, weight=1)

        ttk.Label(
            map_panel,
            textvariable=self.goal_status,
        ).grid(row=0, column=0, sticky='w', padx=6, pady=(3, 7))

        self.canvas = tk.Canvas(
            map_panel,
            background='#07111f',
            highlightthickness=0,
            cursor='crosshair',
        )
        self.canvas.grid(row=1, column=0, sticky='nsew')
        self.canvas.bind('<Configure>', self._on_canvas_resize)
        self.canvas.bind('<ButtonPress-1>', self._on_goal_press)
        self.canvas.bind('<B1-Motion>', self._on_goal_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_goal_release)
        ttk.Label(map_panel, textvariable=self.map_status).grid(
            row=2,
            column=0,
            sticky='w',
            padx=6,
            pady=(7, 3),
        )

    # User actions ---------------------------------------------------------

    def _select_robot(self, robot_name: str) -> None:
        self.node.select_robot(robot_name)
        self.selected_robot.set(robot_name)
        self.routing_status.set(
            f'{robot_name} 선택됨 · 수동 전환 버튼을 눌러 조작하세요.',
        )
        self._update_button_styles()

    def _toggle_manual(self) -> None:
        robot_name = self.selected_robot.get()
        if not robot_name:
            self.routing_status.set('먼저 로봇을 선택하세요.')
            return
        allowed, detail = self.coordinator.toggle_manual(robot_name)
        self.routing_status.set(detail if allowed else f'전환 실패 · {detail}')

    def _publish_lane_finish(self) -> None:
        robot_name = self.selected_robot.get()
        if not robot_name:
            self.goal_status.set('차선 종료 신호를 보낼 로봇을 선택하세요.')
            return
        self.node.publish_lane_finish(robot_name)
        self.goal_status.set(f'{robot_name} 차선 종료 Bool 상승 신호 전송')

    def _start_lane_test(self) -> None:
        """Start lane following directly from the operator-selected lane."""
        robot_name = self.selected_robot.get()
        if not robot_name:
            self.goal_status.set('먼저 로봇을 선택하세요.')
            return
        allowed, detail = self.coordinator.submit_lane_test(robot_name)
        self.goal_status.set(detail if allowed else f'차선 테스트 실패 · {detail}')
        if allowed:
            self.node.clear_selection()
            self.routing_status.set(f'{robot_name} 차선 단독 테스트 · 수동 라우팅 정지')

    def _lane_test_request(self, robot_name, request, response):
        """Expose the same supervised test controls to a commissioning terminal."""
        if request.data:
            response.success, response.message = self.coordinator.submit_lane_test(robot_name)
        else:
            self.coordinator.cancel_robot(robot_name)
            response.success, response.message = True, '차선 테스트 취소 · STOP'
        self.goal_status.set(response.message)
        return response

    def _emergency_stop(self) -> None:
        self.coordinator.emergency_stop()
        self.node.stop_all()
        self.node.clear_selection()
        self.selected_robot.set('')
        self.routing_status.set(
            '로봇 정지 / Nav2 목표 일시 취소 - 저장 목표는 재시작 시 복구됩니다.',
        )
        self._update_button_styles()

    def _set_initial_pose(self, robot_name: str) -> None:
        values = self.initial_pose_values[robot_name]
        try:
            x = float(values['x'].get())
            y = float(values['y'].get())
            yaw_degrees = float(values['yaw'].get())
            self.node.publish_initial_pose(robot_name, x, y, yaw_degrees)
        except ValueError as error:
            self.initial_pose_status.set(f'입력 오류: {error}')
            return

        self.initial_pose_status.set(
            f'{robot_name} 초기 위치 전송 완료: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw_degrees:.1f}°',
        )

    def _cancel_selected_goal(self) -> None:
        robot_name = self.selected_robot.get()
        if not robot_name:
            self.goal_status.set('목표를 취소할 로봇을 먼저 선택하세요.')
            return
        self.coordinator.cancel_robot(robot_name)
        self.goal_status.set(f'{robot_name} 목표 취소를 요청했습니다.')

    def _start_fleet(self) -> None:
        success, detail = self.coordinator.start()
        self.fleet_status.set(detail)
        if not success:
            self.goal_status.set(detail)

    def _pause_fleet(self) -> None:
        self.coordinator.pause()
        self.fleet_status.set(self.coordinator.summary)

    def _begin_zone_edit(self) -> None:
        if self.coordinator.enabled:
            self.goal_status.set('관제를 일시정지한 뒤 구역을 변경하세요.')
            return
        self.zone_edit_mode = True
        self.zone_drag_start = None
        self.goal_status.set(
            '지도에서 병목 구역의 한쪽 모서리부터 반대쪽 모서리까지 드래그하세요.',
        )

    def _clear_zones(self) -> None:
        try:
            self.coordinator.clear_zones()
        except ValueError as error:
            self.goal_status.set(str(error))
            return
        self.zone_edit_mode = False
        self.canvas.delete('zone')
        self.goal_status.set(self.coordinator.summary)

    def _on_goal_press(self, event: tk.Event) -> None:
        if self.zone_edit_mode:
            world_point = self._canvas_to_world(event.x, event.y)
            if world_point is None:
                return
            self.zone_drag_start = (
                float(event.x),
                float(event.y),
                world_point,
            )
            self.canvas.delete('zone_preview')
            return
        robot_name = self.selected_robot.get()
        if not robot_name:
            self.goal_status.set('왼쪽에서 목표를 보낼 로봇을 먼저 선택하세요.')
            return
        world_point = self._canvas_to_world(event.x, event.y)
        if world_point is None:
            self.goal_status.set('맵 내부에서 드래그를 시작하세요.')
            return

        self.goal_drag_start = (
            float(event.x),
            float(event.y),
            world_point[0],
            world_point[1],
        )
        self.canvas.delete('goal_preview')
        self.goal_status.set(
            f'{robot_name} 목표 방향을 정하려면 마우스를 드래그하세요.',
        )

    def _on_goal_drag(self, event: tk.Event) -> None:
        if self.zone_edit_mode and self.zone_drag_start is not None:
            start_x, start_y, _ = self.zone_drag_start
            self.canvas.delete('zone_preview')
            self.canvas.create_rectangle(
                start_x,
                start_y,
                event.x,
                event.y,
                outline='#fbbf24',
                width=3,
                dash=(6, 4),
                tags='zone_preview',
            )
            return
        if self.goal_drag_start is None:
            return
        start_x, start_y, _, _ = self.goal_drag_start
        robot_name = self.selected_robot.get()
        color = ROBOT_BY_NAME[robot_name].color
        self.canvas.delete('goal_preview')
        self.canvas.create_oval(
            start_x - 5,
            start_y - 5,
            start_x + 5,
            start_y + 5,
            fill=color,
            outline='#ffffff',
            width=2,
            tags='goal_preview',
        )
        self.canvas.create_line(
            start_x,
            start_y,
            event.x,
            event.y,
            fill=color,
            width=4,
            arrow='last',
            arrowshape=(12, 14, 5),
            tags='goal_preview',
        )

    def _on_goal_release(self, event: tk.Event) -> None:
        if self.zone_edit_mode and self.zone_drag_start is not None:
            _, _, first_world = self.zone_drag_start
            self.zone_drag_start = None
            second_world = self._canvas_to_world(event.x, event.y)
            self.canvas.delete('zone_preview')
            if second_world is None:
                return
            try:
                self.coordinator.set_runtime_rectangle(
                    first_world,
                    second_world,
                )
            except ValueError as error:
                self.goal_status.set(str(error))
                return
            self.zone_edit_mode = False
            self.goal_status.set(self.coordinator.summary)
            self._draw_robot_markers()
            return
        if self.goal_drag_start is None:
            return
        start_x, start_y, world_x, world_y = self.goal_drag_start
        self.goal_drag_start = None
        self.canvas.delete('goal_preview')
        delta_x = float(event.x) - start_x
        delta_y = float(event.y) - start_y
        if math.hypot(delta_x, delta_y) < 10.0:
            self.goal_status.set('방향을 알 수 있도록 10px 이상 드래그하세요.')
            return

        robot_name = self.selected_robot.get()
        if not robot_name or self.map_geometry is None:
            return
        yaw_degrees = math.degrees(
            math.atan2(-delta_y, delta_x) + self.map_geometry[2],
        )
        yaw_degrees = (yaw_degrees + 180.0) % 360.0 - 180.0
        if self.lane_after_goal.get():
            values = self.initial_pose_values[robot_name]
            try:
                exit_pose = (
                    float(values['x'].get()),
                    float(values['y'].get()),
                    float(values['yaw'].get()),
                )
            except ValueError:
                self.goal_status.set('차선 출구 AMCL pose 입력값을 확인하세요.')
                return
            sent, detail = self.coordinator.submit_lane_entry_goal(
                robot_name,
                world_x,
                world_y,
                yaw_degrees,
                exit_pose,
            )
            if sent:
                self.lane_after_goal.set(False)
        else:
            sent, detail = self.coordinator.submit_goal(
                robot_name,
                world_x,
                world_y,
                yaw_degrees,
            )
        if not sent:
            self.goal_status.set(f'{robot_name}: {detail}')
            return

        self.node.clear_selection()
        self.routing_status.set(
            f'{robot_name} Nav2 주행 중 · 수동 /cmd_vel 라우팅 정지',
        )
        self.goal_markers[robot_name] = (world_x, world_y, yaw_degrees)
        self.goal_status.set(
            f'{detail} · x={world_x:.2f}, y={world_y:.2f}, '
            f'yaw={yaw_degrees:.1f}°',
        )
        self._draw_robot_markers()

    def _canvas_to_world(
        self,
        canvas_x: float,
        canvas_y: float,
    ) -> Optional[Tuple[float, float]]:
        message = self.node.latest_map
        if message is None or self.map_geometry is None:
            self.goal_status.set('목표를 지정하려면 먼저 맵을 수신해야 합니다.')
            return None

        offset_x, offset_y, scale = self.map_display
        grid_x = (canvas_x - offset_x) / scale
        grid_y = message.info.height - (canvas_y - offset_y) / scale
        if not (
            0.0 <= grid_x < message.info.width
            and 0.0 <= grid_y < message.info.height
        ):
            return None
        origin_x, origin_y, origin_yaw = self.map_geometry
        return grid_to_world(
            grid_x,
            grid_y,
            origin_x,
            origin_y,
            origin_yaw,
            message.info.resolution,
        )

    def _update_button_styles(self) -> None:
        selected = self.selected_robot.get()
        for robot in ROBOTS:
            button = self.robot_buttons[robot.name]
            if robot.name == selected:
                button.configure(
                    relief='sunken',
                    bg=robot.color,
                    fg='white',
                    activebackground=robot.color,
                    activeforeground='white',
                )
            else:
                background, foreground, active_bg, active_fg = (
                    self.default_button_colours[robot.name]
                )
                button.configure(
                    relief='flat',
                    bg=background,
                    fg=foreground,
                    activebackground=active_bg,
                    activeforeground=active_fg,
                )

    # Periodic ROS/Tk integration -----------------------------------------

    def _poll_ros(self) -> None:
        if self.closing:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(ROS_POLL_INTERVAL_MS, self._poll_ros)

    def _refresh_ui(self) -> None:
        if self.closing:
            return

        self.coordinator.tick()
        if self.node.map_generation != self.rendered_map_generation:
            self._render_map()
        self._draw_robot_markers()
        self._refresh_robot_status()
        self._refresh_navigation_status()
        self._refresh_fleet_status()
        self.root.after(UI_REFRESH_INTERVAL_MS, self._refresh_ui)

    def _render_map(self) -> None:
        message = self.node.latest_map
        if message is None:
            return

        width = message.info.width
        height = message.info.height
        if width <= 0 or height <= 0 or len(message.data) != width * height:
            self.map_status.set('잘못된 맵 데이터를 수신했습니다.')
            return

        photo = tk.PhotoImage(width=width, height=height)
        rows = []
        for display_row in range(height):
            grid_row = height - 1 - display_row
            offset = grid_row * width
            colours = [
                occupancy_color(value)
                for value in message.data[offset:offset + width]
            ]
            rows.append('{' + ' '.join(colours) + '}')
        photo.put(' '.join(rows))
        self.map_photo = photo
        self.rendered_map_generation = self.node.map_generation
        self._scale_map_to_canvas()

        origin = message.info.origin
        origin_yaw = quaternion_to_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        self.map_geometry = (
            origin.position.x,
            origin.position.y,
            origin_yaw,
        )
        self.map_status.set(
            f'{self.node.latest_map_source} 맵 | {width} x {height} | '
            f'{message.info.resolution:.3f} m/cell',
        )

    # Map overlays ---------------------------------------------------------

    def _scale_map_to_canvas(self) -> None:
        message = self.node.latest_map
        if self.map_photo is None or message is None:
            return

        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        fit = min(
            canvas_width / message.info.width,
            canvas_height / message.info.height,
        )
        if fit >= 1.0:
            zoom = max(1, int(fit))
            subsample = 1
        else:
            zoom = 1
            subsample = max(1, math.ceil(1.0 / fit))

        self.scaled_map_photo = self.map_photo.zoom(zoom).subsample(subsample)
        display_width = self.scaled_map_photo.width()
        display_height = self.scaled_map_photo.height()
        offset_x = (canvas_width - display_width) / 2.0
        offset_y = (canvas_height - display_height) / 2.0
        scale = zoom / subsample
        self.map_display = (offset_x, offset_y, scale)

        self.canvas.delete('all')
        self.canvas.create_image(
            offset_x,
            offset_y,
            image=self.scaled_map_photo,
            anchor='nw',
            tags='map',
        )

    def _draw_robot_markers(self) -> None:
        self.canvas.delete('robot')
        self.canvas.delete('goal')
        self.canvas.delete('zone')
        message = self.node.latest_map
        if message is None or self.map_geometry is None:
            return

        offset_x, offset_y, scale = self.map_display
        origin_x, origin_y, origin_yaw = self.map_geometry
        zone_colours = {
            'FREE': '#22c55e',
            'OCCUPIED': '#f97316',
            'CLEARING': '#eab308',
            'CONFLICT': '#a855f7',
            'LOCKED': '#ef4444',
        }
        for runtime in self.coordinator.zone_list():
            coordinates = []
            for point_x, point_y in runtime.config.polygon:
                grid_x, grid_y = world_to_grid(
                    point_x,
                    point_y,
                    origin_x,
                    origin_y,
                    origin_yaw,
                    message.info.resolution,
                )
                coordinates.extend((
                    offset_x + grid_x * scale,
                    offset_y + (message.info.height - grid_y) * scale,
                ))
            colour = zone_colours.get(runtime.state, '#fbbf24')
            self.canvas.create_polygon(
                *coordinates,
                fill='',
                outline=colour,
                width=3,
                tags='zone',
            )
            if coordinates:
                min_x, min_y, max_x, max_y = runtime.config.bounds
                self.canvas.create_text(
                    coordinates[0],
                    coordinates[1] - 28,
                    text=(
                        f'{runtime.config.zone_id} · {runtime.state}\n'
                        f'x[{min_x:.2f},{max_x:.2f}] '
                        f'y[{min_y:.2f},{max_y:.2f}] · '
                        f'R {runtime.config.robot_radius_m:.2f} + '
                        f'M {runtime.config.clearance_margin_m:.2f} = '
                        f'{runtime.config.effective_clearance_m:.2f} m'
                    ),
                    fill=colour,
                    anchor='w',
                    font=('', 9, 'bold'),
                    tags='zone',
                )
        for robot in ROBOTS:
            pose_message = self.node.poses.get(robot.name)
            if pose_message is None:
                continue

            pose = pose_message.pose.pose
            grid_x, grid_y = world_to_grid(
                pose.position.x,
                pose.position.y,
                origin_x,
                origin_y,
                origin_yaw,
                message.info.resolution,
            )
            canvas_x = offset_x + grid_x * scale
            canvas_y = offset_y + (message.info.height - grid_y) * scale
            yaw = quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) - origin_yaw
            marker_size = 12.0
            tip = (
                canvas_x + marker_size * math.cos(yaw),
                canvas_y - marker_size * math.sin(yaw),
            )
            left = (
                canvas_x + marker_size * 0.7 * math.cos(yaw + 2.5),
                canvas_y - marker_size * 0.7 * math.sin(yaw + 2.5),
            )
            right = (
                canvas_x + marker_size * 0.7 * math.cos(yaw - 2.5),
                canvas_y - marker_size * 0.7 * math.sin(yaw - 2.5),
            )
            self.canvas.create_polygon(
                *tip,
                *left,
                *right,
                fill=robot.color,
                outline='white',
                width=2,
                tags='robot',
            )
            self.canvas.create_text(
                canvas_x,
                canvas_y + 20,
                text=robot.name,
                fill=robot.color,
                font=('', 10, 'bold'),
                tags='robot',
            )

        for robot in ROBOTS:
            goal = self.goal_markers.get(robot.name)
            if goal is None:
                continue
            goal_x, goal_y, yaw_degrees = goal
            grid_x, grid_y = world_to_grid(
                goal_x,
                goal_y,
                origin_x,
                origin_y,
                origin_yaw,
                message.info.resolution,
            )
            canvas_x = offset_x + grid_x * scale
            canvas_y = offset_y + (message.info.height - grid_y) * scale
            relative_yaw = math.radians(yaw_degrees) - origin_yaw
            end_x = canvas_x + 25.0 * math.cos(relative_yaw)
            end_y = canvas_y - 25.0 * math.sin(relative_yaw)
            self.canvas.create_oval(
                canvas_x - 6,
                canvas_y - 6,
                canvas_x + 6,
                canvas_y + 6,
                fill='#0b1220',
                outline=robot.color,
                width=3,
                tags='goal',
            )

            self.canvas.create_line(
                canvas_x,
                canvas_y,
                end_x,
                end_y,
                fill=robot.color,
                width=3,
                arrow='last',
                tags='goal',
            )
            self.canvas.create_text(
                canvas_x,
                canvas_y - 18,
                text=f'{robot.name} GOAL',
                fill=robot.color,
                font=('', 9, 'bold'),
                tags='goal',
            )

    # Status presentation and shutdown ------------------------------------

    def _refresh_robot_status(self) -> None:
        for robot in ROBOTS:
            age = self.node.pose_age(robot.name)
            if age is None:
                status = '위치 수신 대기 중'
            elif not self.node.pose_is_fresh(robot.name):
                status = f'연결 지연 ({age:.1f}s)'
            else:
                pose = self.node.poses[robot.name].pose.pose.position
                status = f'x {pose.x:.2f}  y {pose.y:.2f}'
            self.robot_status[robot.name].set(status)

    def _refresh_navigation_status(self) -> None:
        for robot in ROBOTS:
            self.navigation_status[robot.name].set(
                f'{self.coordinator.robot_details[robot.name]} · '
                f'{self.navigation.status(robot.name)} · '
                f'{self.lane_navigation.status(robot.name)}',
            )

    def _refresh_fleet_status(self) -> None:
        self.fleet_status.set(self.coordinator.summary)
        selected = self.selected_robot.get()
        if selected and self.coordinator.manual_robot == selected:
            if selected in self.coordinator.blocked_robots:
                self.routing_status.set(
                    f'{selected} 수동 명령 HOLD · '
                    f'{self.coordinator.robot_details[selected]}',
                )
            else:
                self.routing_status.set(
                    f'/cmd_vel -> /{selected}/cmd_vel_manual_candidate · RUN',
                )
        details = []
        for robot in ROBOTS:
            age = self.node.heartbeat_age(robot.name)
            if age is None:
                details.append(f'{robot.name}: heartbeat 없음')
                continue
            heartbeat = self.node.heartbeats[robot.name]
            drive_mode = self.node.drive_mode_status.get(robot.name, {})
            mode_text = drive_mode.get('mode', 'mode?')
            details.append(
                f'{robot.name}: {heartbeat.status} · {mode_text} '
                f'({age:.1f}s)',
            )
        self.gate_status.set(' | '.join(details))

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        if self.map_photo is not None:
            self._scale_map_to_canvas()

    def _on_close(self) -> None:
        self.closing = True
        self.coordinator.emergency_stop('UI_CLOSED')
        self.node.stop_all()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    """Run the ROS node and Tkinter UI on the control domain."""
    node = create_node(args=args)
    navigation = FleetNavigationClients()
    lane_navigation = FleetLaneClients(ROBOTS)
    try:
        zones = load_zones(node.zone_config_file)
    except ValueError as error:
        node.get_logger().error(str(error))
        zones = []
    coordinator = FleetCoordinator(
        node, navigation, zones, lane_navigation=lane_navigation,
    )
    ui = None
    try:
        ui = MultiBotControlUI(
            node, navigation, coordinator, lane_navigation,
        )
        ui.run()
    except KeyboardInterrupt:
        if ui is not None:
            node.stop_all()
    finally:
        navigation.shutdown()
        lane_navigation.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
