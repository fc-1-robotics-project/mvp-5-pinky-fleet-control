"""Tkinter fleet UI for command selection and map monitoring."""

import math
import hashlib
import signal
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox
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
from .lane_routes import DIRECTIONS, POSES, load_routes, save_routes, validate_pose, validate_route


ROS_POLL_INTERVAL_MS = 20
UI_REFRESH_INTERVAL_MS = 100
POSE_LABELS = {'entry': '차선 입구', 'exit': '차선 출구', 'next': '이후 Nav2 목표'}


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
            value='지도 선택 → 위치·방향 드래그 → 로봇별 설정. 직접 입력도 가능합니다.',
        )
        self.goal_status = tk.StringVar(
            value='로봇 선택 후 지도에서 클릭하고 진행 방향으로 드래그하세요.',
        )
        self.route_path = Path.home() / '.config/pinky_fleet_control/lane_routes.json'
        self.route_load_error = ''
        try:
            self.routes = load_routes(self.route_path)
        except (OSError, ValueError, TypeError) as error:
            self.routes = {}
            self.route_load_error = str(error)
        self.lane_direction = tk.StringVar(value=DIRECTIONS[0])
        self.route_values = {field: [tk.StringVar() for _ in range(3)] for field in POSES}
        self.route_status = tk.StringVar(value=self.route_load_error or '입구·출구·다음 목표를 지도에서 지정하세요.')
        self.lane_status = tk.StringVar(value='차선 임무 서버 대기 중')
        self.field_ready = tk.BooleanVar(value=False)
        self.map_capture = None
        self.lane_buttons = {}
        self._load_route_fields()
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
        self.goal_drag_target = None
        self.zone_edit_mode = False
        self.zone_drag_start = None
        self.map_photo: Optional[tk.PhotoImage] = None
        self.scaled_map_photo: Optional[tk.PhotoImage] = None
        self.rendered_map_generation = -1
        self.map_geometry: Optional[Tuple[float, float, float]] = None
        self.map_display = (0.0, 0.0, 1.0)
        self.closing = False
        self.shutdown_requested = False

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

    def _load_route_fields(self, _event=None):
        route = self.routes.get(self.lane_direction.get(), {})
        for field, variables in self.route_values.items():
            values = route.get(field, ('', '', ''))
            for variable, value in zip(variables, values):
                variable.set(str(value))
        self._cancel_map_capture()
        self.route_status.set('저장된 경로 불러옴' if route else '이 방향의 경로를 설정하세요.')

    def _map_key(self):
        message = self.node.latest_map
        if message is None:
            raise ValueError('지도를 먼저 수신해야 합니다.')
        geometry = (message.header.frame_id, message.info.width, message.info.height,
                    message.info.resolution, str(message.info.origin))
        digest = hashlib.sha256(repr(geometry).encode())
        digest.update(bytes(value + 1 for value in message.data))
        return digest.hexdigest()

    def _capture_route_pose(self, field):
        self._begin_pose_capture('route', field)

    def _capture_initial_pose(self, robot_name):
        self._begin_pose_capture('initial', robot_name)

    def _discard_map_drag(self):
        self.goal_drag_start = self.goal_drag_target = None
        self.zone_drag_start = None
        if hasattr(self, 'canvas'):
            self.canvas.delete('goal_preview', 'zone_preview')

    def _cancel_map_capture(self, _event=None):
        self.map_capture = None
        self.zone_edit_mode = False
        self._discard_map_drag()
        if hasattr(self, 'capture_cancel_button'):
            self.capture_cancel_button.configure(state='disabled')
        self.goal_status.set('로봇 선택 후 지도에서 클릭하고 진행 방향으로 드래그하세요.')
        return 'break'

    def _begin_pose_capture(self, kind, target):
        self._cancel_map_capture()
        if self.node.latest_map is None or self.map_geometry is None:
            self.goal_status.set('지도를 수신한 뒤 지도 선택을 눌러주세요.')
            return
        self.map_capture = (kind, target)
        label = POSE_LABELS[target] if kind == 'route' else f'{target} 초기 위치'
        self.capture_cancel_button.configure(state='normal')
        self.goal_status.set(f'{label} 입력: 위치를 누르고 방향으로 드래그하세요. Esc로 취소')

    def _apply_route_pose(self, field):
        try:
            pose = validate_pose([v.get() for v in self.route_values[field]], field)
            key = self._map_key()
            direction = self.lane_direction.get()
            route = dict(self.routes.get(direction, {}))
            if route and route.get('map_key') != key:
                raise ValueError('지도가 변경됐습니다. 세 좌표를 확인한 뒤 방향별 경로 저장을 누르세요.')
            route.update({field: pose, 'map_key': key})
            updated = dict(self.routes, **{direction: route})
            save_routes(self.route_path, updated)
            self.routes = updated
        except (OSError, ValueError) as error:
            self.route_status.set(str(error))
            return
        self._cancel_map_capture()
        missing = [POSE_LABELS[f] for f in POSES if f not in route]
        suffix = ' · 남은 항목: ' + ', '.join(missing) if missing else ' · 경로 준비 완료'
        self.route_status.set(f'{direction} {POSE_LABELS[field]} 지정·저장 완료{suffix}')

    def _save_route(self):
        try:
            route = validate_route({field: [v.get() for v in values]
                                    for field, values in self.route_values.items()})
            route['map_key'] = self._map_key()
            updated = dict(self.routes, **{self.lane_direction.get(): route})
            save_routes(self.route_path, updated)
            self.routes = updated
        except (OSError, ValueError) as error:
            self.route_status.set(str(error))
            return
        self.route_status.set(f'{self.lane_direction.get()} 입구·출구·다음 목표 저장 완료')
        self._cancel_map_capture()

    def _start_continuous(self):
        robot = self.selected_robot.get()
        if not robot or not self.field_ready.get():
            self.goal_status.set('로봇 선택과 현장 준비 확인이 필요합니다.')
            return
        try:
            route = self.routes.get(self.lane_direction.get())
            if route is None:
                raise ValueError('이 방향의 경로를 먼저 저장하세요.')
            edited = validate_route({field: [v.get() for v in values]
                                     for field, values in self.route_values.items()})
            if edited != validate_route(route):
                raise ValueError('좌표를 수정했습니다. 방향별 저장 후 시작하세요.')
            if route.get('map_key') != self._map_key():
                raise ValueError('저장 당시와 지도가 다릅니다. 좌표를 확인하고 다시 저장하세요.')
            allowed, detail = self.coordinator.submit_lane_entry_goal(
                robot, *route['entry'], route['exit'],
                route_id=self.lane_direction.get(), next_goal=route['next'])
        except ValueError as error:
            self.goal_status.set(str(error))
            return
        self.goal_status.set(detail if allowed else f'시작 실패 · {detail}')
        if allowed:
            self.field_ready.set(False)
            self.node.clear_selection()

    def _pause_selected_mission(self):
        robot = self.selected_robot.get()
        request = self.coordinator.requests.get(robot)
        if request is None:
            self.goal_status.set('진행 중인 임무를 선택하세요.')
            return
        _, detail = self.coordinator.pause_robot(robot, not request.paused)
        self.goal_status.set(detail)

    def _build_lane_panel(self, controls, row):
        panel = ttk.LabelFrame(controls, text='차선 / Nav2 연속 임무', padding=8)
        panel.grid(row=row, column=0, columnspan=2, sticky='ew', pady=8)
        panel.columnconfigure(0, weight=1)
        picker = ttk.Combobox(panel, textvariable=self.lane_direction,
                              values=DIRECTIONS, state='readonly', width=12)
        picker.grid(row=0, column=0, sticky='w')
        picker.bind('<<ComboboxSelected>>', self._load_route_fields)
        ttk.Label(panel, text='양방향 모두 전진 주행').grid(row=0, column=1, sticky='w')
        table = ttk.Frame(panel)
        table.grid(row=1, column=0, columnspan=2, sticky='ew', pady=4)
        for column, label in enumerate(('좌표', 'x (m)', 'y (m)', '방향 (°)')):
            ttk.Label(table, text=label).grid(row=0, column=column, padx=2)
        for index, (field, label) in enumerate(zip(POSES, ('입구', '출구', '다음 목표')), 1):
            ttk.Label(table, text=label).grid(row=index, column=0, sticky='w')
            for column, variable in enumerate(self.route_values[field], 1):
                ttk.Entry(table, textvariable=variable, width=7).grid(row=index, column=column, padx=1)
            ttk.Button(table, text='지도 선택', width=0,
                       command=lambda f=field: self._capture_route_pose(f)).grid(row=index, column=4, padx=3)
            ttk.Button(table, text='지정', width=0,
                       command=lambda f=field: self._apply_route_pose(f)).grid(row=index, column=5, padx=3)
        ttk.Button(panel, text='방향별 경로 저장', command=self._save_route).grid(row=2, column=0, columnspan=2, sticky='ew', pady=4)
        ttk.Label(panel, textvariable=self.route_status, wraplength=360).grid(row=3, column=0, columnspan=2, sticky='w')
        ttk.Checkbutton(panel, text='현장 감시·즉시 정지 준비 확인', variable=self.field_ready).grid(row=4, column=0, columnspan=2, sticky='w', pady=6)
        for key, label, handler, grid_row, column in (
                ('continuous', '연속 임무 시작', self._start_continuous, 5, 0),
                ('test', '차선 단독 테스트', self._start_lane_test, 5, 1),
                ('finish', '차선 구간 완료', self._publish_lane_finish, 6, 0),
                ('pause', '일시정지 / 재개', self._pause_selected_mission, 6, 1),
                ('cancel', '선택 임무 중단', self._cancel_selected_goal, 7, 0),
                ('test_all', '전체 로봇 차선 테스트', self._start_lane_test_all, 7, 1)):
            button = ttk.Button(panel, text=label, command=handler)
            button.grid(row=grid_row, column=column, sticky='ew', padx=2, pady=3)
            self.lane_buttons[key] = button
        ttk.Label(panel, textvariable=self.lane_status, wraplength=360, justify='left').grid(row=8, column=0, columnspan=2, sticky='w', pady=4)
        return row + 1

    def _lane_start_available(self, robot):
        """Return whether one robot can accept a new lane mission now."""
        data = self.lane_navigation.telemetry(robot) if robot else {}
        return bool(robot and data.get('fresh', False) and not data.get('active')
                    and self.coordinator.requests.get(robot) is None
                    and not self.coordinator.emergency
                    and self.lane_navigation.state(robot) != 'CANCELLING')

    def _refresh_lane_panel(self):
        robot = self.selected_robot.get()
        data = self.lane_navigation.telemetry(robot) if robot else {}
        request = self.coordinator.requests.get(robot)
        fresh = data.get('fresh', False)
        available = self._lane_start_available(robot)
        for key in ('continuous', 'test'):
            self.lane_buttons[key].configure(state='normal' if available and self.field_ready.get() else 'disabled')
        all_available = all(self._lane_start_available(item.name) for item in ROBOTS)
        self.lane_buttons['test_all'].configure(
            state='normal' if all_available and self.field_ready.get() else 'disabled')
        active_lane = request is not None and request.phase in {'LANE_ACTIVE', 'SAFETY_HOLD'}
        self.lane_buttons['finish'].configure(state='normal' if fresh and active_lane else 'disabled')
        for key in ('pause', 'cancel'):
            self.lane_buttons[key].configure(state='normal' if request is not None else 'disabled')
        if not robot:
            self.lane_status.set('로봇을 선택하세요.')
        elif not fresh:
            self.lane_status.set(f'{robot} · 차선 임무 서버 연결 대기')
        else:
            phase = request.phase if request else data.get('state', 'IDLE')
            reason = data.get('detail') if request else data.get('readiness_reason')
            self.lane_status.set(f"{robot} · {phase} · 실제 모드 {data.get('mode')}\n"
                                 f"차선 거리 {data.get('distance_m', 0)} m · {data.get('lane_reason') or '-'}\n"
                                 f"{reason} · 로컬 허가 {'켜짐' if data.get('permission_enabled') else '해제'}")

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

        self.panes = tk.PanedWindow(
            container, orient='horizontal', sashwidth=10, sashrelief='raised',
            sashcursor='sb_h_double_arrow', background='#334155',
            borderwidth=0, opaqueresize=True,
        )
        self.panes.grid(row=1, column=0, columnspan=2, sticky='nsew')
        controls_shell = ttk.Frame(self.panes)
        self.controls_shell = controls_shell
        controls_shell.rowconfigure(0, weight=1)
        controls_shell.columnconfigure(0, weight=1)

        self.controls_canvas = tk.Canvas(
            controls_shell,
            width=620,
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
        self.controls_scrollbar = controls_scrollbar

        controls = ttk.LabelFrame(
            self.controls_canvas,
            text='명령 대상',
            padding=12,
        )
        self.controls = controls
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)
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
        self.root.bind('<Escape>', self._cancel_map_capture)

        next_row = self._build_robot_selection_panel(controls)
        next_row = self._build_lane_panel(controls, next_row)
        next_row = self._build_initial_pose_panel(controls, next_row)
        next_row = self._build_navigation_panel(controls, next_row)
        self._build_fleet_panel(controls, next_row)
        map_panel = self._build_map_panel(self.panes)
        self.panes.add(controls_shell, minsize=620, width=640, stretch='never')
        self.panes.add(map_panel, minsize=320, stretch='always')
        self.root.after_idle(self._fit_controls_pane)

    def _fit_controls_pane(self):
        """Keep every input/button visible using the actual font/widget sizes."""
        required = self.controls.winfo_reqwidth() + self.controls_scrollbar.winfo_reqwidth() + 4
        minimum = max(540, required)
        self.panes.paneconfigure(self.controls_shell, minsize=minimum)
        self.root.minsize(max(1180, minimum + 360), 740)
        self.panes.sash_place(0, max(minimum, 640), 0)

    def _on_controls_content_resize(self, _event: tk.Event) -> None:
        """Keep the control-column scrollbar aligned with its contents."""
        self.controls_canvas.configure(
            scrollregion=self.controls_canvas.bbox('all'),
        )

    def _on_controls_canvas_resize(self, event: tk.Event) -> None:
        """Fill the available control-column width without clipping widgets."""
        self.controls_canvas.itemconfigure(
            self.controls_window,
            width=max(event.width, self.controls.winfo_reqwidth()),
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
                text='지도 선택',
                width=0,
                command=lambda name=robot.name: self._capture_initial_pose(name),
            ).grid(row=item_row, column=4, padx=3, pady=3)
            ttk.Button(
                initial_pose_panel,
                text='설정',
                width=0,
                command=lambda name=robot.name: self._set_initial_pose(name),
            ).grid(row=item_row, column=5, padx=3, pady=3)

        ttk.Label(
            initial_pose_panel,
            textvariable=self.initial_pose_status,
            wraplength=360,
            justify='left',
        ).grid(
            row=len(ROBOTS) + 1,
            column=0,
            columnspan=6,
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
                wraplength=360,
                justify='left',
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
        ttk.Button(navigation_panel, text='선택 로봇 수동 / 자율 전환',
                   command=self._toggle_manual).grid(row=len(ROBOTS)+1, column=0,
                                                    columnspan=2, sticky='ew', pady=6)

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

    def _build_map_panel(self, container):
        """Build the shared-map canvas and bind pointer interactions."""
        map_panel = ttk.LabelFrame(container, text='공유 맵 / 로봇 위치', padding=6)
        map_panel.rowconfigure(1, weight=1)
        map_panel.columnconfigure(0, weight=1)

        toolbar = ttk.Frame(map_panel)
        toolbar.grid(row=0, column=0, sticky='ew', padx=6, pady=(3, 7))
        toolbar.columnconfigure(0, weight=1)
        hint = ttk.Label(
            toolbar,
            textvariable=self.goal_status,
            wraplength=360,
        )
        hint.grid(row=0, column=0, sticky='ew')
        hint.bind('<Configure>', lambda event: hint.configure(wraplength=max(100, event.width)))
        self.capture_cancel_button = ttk.Button(
            toolbar, text='선택 취소 (Esc)', width=0, state='disabled',
            command=self._cancel_map_capture,
        )
        self.capture_cancel_button.grid(row=0, column=1, padx=(6, 0))

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
        return map_panel

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
        robot = self.selected_robot.get()
        request = self.coordinator.requests.get(robot)
        if request is None or request.phase not in {'LANE_ACTIVE', 'SAFETY_HOLD'}:
            self.goal_status.set('차선 주행 중인 로봇을 선택하세요.')
            return
        if not request.lane_only and not messagebox.askyesno(
                '출구 도착 확인', f'{robot}이 지정된 출구 {request.lane_exit_pose}에 도착했나요?\n확인 후 위치 검증과 다음 Nav2 목표를 진행합니다.'):
            return
        self.node.publish_lane_finish(robot)
        self.goal_status.set(f'{robot} 차선 구간 완료 요청 · 정지/권한 해제 확인 중')

    def _start_lane_test(self) -> None:
        robot = self.selected_robot.get()
        if not robot or not self.field_ready.get():
            self.goal_status.set('로봇 선택과 현장 준비 확인이 필요합니다.')
            return
        allowed, detail = self.coordinator.submit_lane_test(robot, self.lane_direction.get())
        self.goal_status.set(detail if allowed else f'시작 실패 · {detail}')
        if allowed:
            self.field_ready.set(False)
            self.node.clear_selection()

    def _start_lane_test_all(self) -> None:
        """Start the lane-only test on every robot, or on none of them."""
        if not self.field_ready.get():
            self.goal_status.set('현장 준비 확인이 필요합니다.')
            return
        waiting = [robot.name for robot in ROBOTS
                   if not self._lane_start_available(robot.name)]
        if waiting:
            self.goal_status.set(f"시작 실패 · 준비 안 된 로봇: {', '.join(waiting)}")
            return
        results = [(robot.name, *self.coordinator.submit_lane_test(
            robot.name, self.lane_direction.get())) for robot in ROBOTS]
        failed = [f'{name}: {detail}' for name, allowed, detail in results if not allowed]
        if failed:
            # A late refusal must not leave the others driving unattended.
            for name, allowed, _ in results:
                if allowed:
                    self.coordinator.cancel_robot(name)
            self.goal_status.set('시작 실패 · 전체 취소 · ' + ' / '.join(failed))
            return
        self.goal_status.set(f'전체 {len(results)}대 차선 단독 테스트 시작 · 로봇별로 차선 구간 완료')
        self.field_ready.set(False)
        self.node.clear_selection()

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
        self._cancel_map_capture()

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
        self._cancel_map_capture()
        self.zone_edit_mode = True
        self.capture_cancel_button.configure(state='normal')
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
        self._cancel_map_capture()
        self.canvas.delete('zone')
        self.goal_status.set(self.coordinator.summary)

    def _on_goal_press(self, event: tk.Event) -> None:
        self._discard_map_drag()
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
        target = self.map_capture or ('goal', self.selected_robot.get())
        kind, name = target
        if kind == 'goal' and name not in ROBOT_BY_NAME:
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
        self.goal_drag_target = target
        self.canvas.delete('goal_preview')

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
        kind, name = self.goal_drag_target
        color = '#fbbf24' if kind == 'route' else ROBOT_BY_NAME[name].color
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
            self.capture_cancel_button.configure(state='disabled')
            self.goal_status.set(self.coordinator.summary)
            self._draw_robot_markers()
            return
        if self.goal_drag_start is None:
            return
        start_x, start_y, world_x, world_y = self.goal_drag_start
        target = self.goal_drag_target
        self._discard_map_drag()
        delta_x = float(event.x) - start_x
        delta_y = float(event.y) - start_y
        if math.hypot(delta_x, delta_y) < 10.0:
            self.goal_status.set('방향을 알 수 있도록 10px 이상 드래그하세요.')
            return

        if target is None or self.map_geometry is None:
            return
        yaw_degrees = math.degrees(
            math.atan2(-delta_y, delta_x) + self.map_geometry[2],
        )
        yaw_degrees = (yaw_degrees + 180.0) % 360.0 - 180.0
        kind, name = target
        if kind in {'route', 'initial'}:
            if kind == 'route':
                variables = self.route_values[name]
                label, action = POSE_LABELS[name], '지정'
            else:
                variables = [self.initial_pose_values[name][key] for key in ('x', 'y', 'yaw')]
                label, action = f'{name} 초기 위치', '설정'
            for variable, value in zip(variables, (world_x, world_y, yaw_degrees)):
                variable.set(f'{value:.3f}')
            # Keep capture armed for adjustments until explicit apply or cancel.
            self.goal_status.set(f'{label} 입력됨 · 해당 행의 {action} 버튼으로 확정하세요. 다시 드래그해 수정 · Esc로 선택 종료')
            return
        robot_name = name
        sent, detail = self.coordinator.submit_goal(robot_name, world_x, world_y, yaw_degrees)
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
        if self.shutdown_requested or not rclpy.ok():
            self._on_close()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(ROS_POLL_INTERVAL_MS, self._poll_ros)

    def _refresh_ui(self) -> None:
        if self.closing:
            return
        if self.shutdown_requested or not rclpy.ok():
            self._on_close()
            return

        self.coordinator.tick()
        self._refresh_lane_panel()
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

        self._discard_map_drag()
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
        self._discard_map_drag()
        if self.map_photo is not None:
            self._scale_map_to_canvas()

    def _on_close(self) -> None:
        if self.closing:
            return
        self.closing = True
        if rclpy.ok():
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
    previous_signals = {}
    try:
        ui = MultiBotControlUI(
            node, navigation, coordinator, lane_navigation,
        )
        # Let Tk's next poll stop the robots before shutting down ROS contexts.
        # A signal handler must not publish or acquire client locks directly.
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signals[signum] = signal.signal(
                signum, lambda *_: setattr(ui, 'shutdown_requested', True))
        ui.run()
    except KeyboardInterrupt:
        if ui is not None and rclpy.ok():
            node.stop_all()
    finally:
        if rclpy.ok():
            coordinator.emergency_stop('UI_SHUTDOWN')
        navigation.shutdown()
        lane_navigation.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
