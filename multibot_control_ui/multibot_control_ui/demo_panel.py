"""Optional Tk editor for the demo; coordinates are applied explicitly."""

from copy import deepcopy
import json
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from .demo_mission import POSES, save_plan, validate_plan
from .lane_routes import validate_pose
from .robot_config import ROBOTS

LABELS = {'a_exit': 'A 차선 끝 / Nav2 합류', 'a_final': 'A Nav2 최종 목적지',
          'b_entry': 'B 차선 입구 (방향 포함)', 'b_exit': 'B 차선 최종 끝',
          'waypoint': '대기 waypoint'}


class DemoPanel:
    def __init__(self, ui):
        self.ui, self.demo = ui, ui.demo
        self.path = Path.home() / '.config/pinky_fleet_control/two_robot_demo.json'
        self.window = tk.Toplevel(ui.root)
        self.window.title('두 로봇 통합 시연')
        self.window.minsize(760, 620)
        body = ttk.Frame(self.window, padding=10)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=1)
        self.plan = dict(version=1, map_key='', waypoints={'A': [], 'B': []},
                         closed={'A': False, 'B': False})
        self.status = tk.StringVar(value='지도 좌표를 입력하고 지정하세요. 종료 기준: 7cm / 실제 정지 3초')
        roles = ttk.Frame(body)
        roles.grid(row=0, column=0, sticky='ew')
        names = [robot.name for robot in ROBOTS]
        self.robots, self.routes = {}, {}
        for index, role in enumerate(('A', 'B')):
            self.robots[role] = tk.StringVar(value=names[index] if index < len(names) else '')
            self.routes[role] = tk.StringVar(value='A_to_B' if role == 'A' else 'B_to_A')
            ttk.Label(roles, text=role + (' (차선 출발)' if role == 'A' else ' (Nav2 출발)')).grid(row=index, column=0)
            ttk.Combobox(roles, textvariable=self.robots[role], values=names,
                         state='readonly', width=10).grid(row=index, column=1, padx=5)
            ttk.Label(roles, text='차선 route_id').grid(row=index, column=2)
            ttk.Entry(roles, textvariable=self.routes[role], width=14).grid(row=index, column=3)
        coordinates = ttk.LabelFrame(body, text='고정 좌표 (m / yaw °)', padding=5)
        coordinates.grid(row=1, column=0, sticky='ew', pady=8)
        self.values = {field: [tk.StringVar() for _ in range(3)] for field in (*POSES, 'waypoint')}
        for row, field in enumerate(POSES):
            ttk.Label(coordinates, text=LABELS[field]).grid(row=row, column=0, sticky='w')
            self._pose_row(coordinates, row, field)
            ttk.Button(coordinates, text='지정', command=lambda f=field: self._run(lambda: self._apply(f))).grid(row=row, column=5)
        queue = ttk.LabelFrame(body, text='Nav2 대기 목록 · 각 점 도착 후 다음 점 전송', padding=5)
        queue.grid(row=2, column=0, sticky='nsew')
        body.rowconfigure(2, weight=1)
        queue.columnconfigure(0, weight=1)
        queue.rowconfigure(1, weight=1)
        controls = ttk.Frame(queue)
        controls.grid(row=0, column=0, sticky='ew')
        self.role = tk.StringVar(value='A')
        picker = ttk.Combobox(controls, textvariable=self.role, values=('A', 'B'), state='readonly', width=4)
        picker.pack(side='left')
        picker.bind('<<ComboboxSelected>>', lambda _: self.refresh(force=True))
        self.closed = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text='이 로봇 Nav2 목록 확정 (빈 목록만으로는 전환 안 함)',
                        variable=self.closed, command=lambda: self._run(self._close_queue)).pack(side='left', padx=8)
        self.tree = ttk.Treeview(queue, columns=('state', 'x', 'y', 'yaw'), show='headings', height=6)
        for key in ('state', 'x', 'y', 'yaw'):
            self.tree.heading(key, text=key)
            self.tree.column(key, width=110, stretch=True)
        self.tree.grid(row=1, column=0, sticky='nsew')
        scrollbar = ttk.Scrollbar(queue, orient='vertical', command=self.tree.yview)
        scrollbar.grid(row=1, column=1, sticky='ns')
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.bind('<<TreeviewSelect>>', self._select_waypoint)
        edit = ttk.Frame(queue)
        edit.grid(row=2, column=0, sticky='ew', pady=5)
        ttk.Label(edit, text='x / y / yaw').grid(row=0, column=0)
        self._pose_row(edit, 0, 'waypoint')
        actions = ttk.Frame(queue)
        actions.grid(row=3, column=0, sticky='ew')
        for title, action in [('추가', 'add'), ('선택 수정', 'edit'), ('삭제', 'delete'), ('위', 'up'), ('아래', 'down')]:
            ttk.Button(actions, text=title, command=lambda a=action: self._run(lambda: self._edit_queue(a))).pack(side='left', padx=2)
        buttons = ttk.Frame(body)
        buttons.grid(row=3, column=0, sticky='ew', pady=8)
        for title, action in [('설정 불러오기', self._load), ('설정 저장', self._save),
                              ('통합 시연 시작', self._start), ('시연 중단', self.demo.cancel)]:
            ttk.Button(buttons, text=title, command=lambda a=action: self._run(a)).pack(side='left', padx=3)
        self.ready = tk.BooleanVar(value=False)
        ttk.Checkbutton(body, text='두 로봇 배치·코스 확인·현장 감시·즉시 정지 가능', variable=self.ready).grid(row=4, column=0, sticky='w')
        ttk.Label(body, text='일시정지/재개·긴급 정지·수동 차선 완료는 관제 본창의 기존 버튼을 사용합니다.').grid(row=5, column=0, sticky='w')
        ttk.Label(body, textvariable=self.status, wraplength=720).grid(row=6, column=0, sticky='w', pady=6)
        self.progress = tk.StringVar()
        ttk.Label(body, textvariable=self.progress, wraplength=720).grid(row=7, column=0, sticky='w')
        self._signature = None
        if self.demo.active:
            self.plan = deepcopy(self.demo.plan)
            self._fill_configuration()
        elif self.path.exists():
            self._run(self._load)
        self.refresh(force=True)

    def _pose_row(self, parent, row, field):
        for column, variable in enumerate(self.values[field], 1):
            ttk.Entry(parent, textvariable=variable, width=9).grid(row=row, column=column, padx=2)
        ttk.Button(parent, text='지도 선택', command=lambda: self.ui._begin_pose_capture('demo', field)).grid(row=row, column=4, padx=3)

    def capture(self, field, pose):
        for variable, value in zip(self.values[field], pose):
            variable.set(f'{value:.3f}')
        self.status.set(f'{LABELS[field]} 입력됨 · 지정 또는 waypoint 추가/선택 수정으로 확정하세요.')

    def _run(self, action):
        try:
            action()
        except (ValueError, OSError, TypeError, KeyError) as error:
            self.status.set(str(error))
        self.refresh(force=True)

    def _apply(self, field):
        if self.demo.active:
            raise ValueError('시연 중 고정 좌표 변경은 중단 후 적용하세요.')
        key = self.ui._map_key()
        if self.plan['map_key'] and self.plan['map_key'] != key:
            raise ValueError('지도가 변경됐습니다. 설정을 다시 불러오거나 창을 닫고 새로 지정하세요.')
        self.plan[field] = validate_pose([v.get() for v in self.values[field]], field)
        self.plan['map_key'] = key
        self.ui._cancel_map_capture()
        self.status.set(LABELS[field] + ' 지정 완료')

    def _queues(self):
        return (self.demo.pending, self.demo.closed) if self.demo.active else (self.plan['waypoints'], self.plan['closed'])

    def _set_pending(self, points, closed):
        role = self.role.get()
        if self.demo.active:
            self.demo.replace_pending(role, points, closed)
        else:
            key = self.ui._map_key()
            if self.plan['map_key'] and self.plan['map_key'] != key:
                raise ValueError('지도가 변경됐습니다. 좌표를 다시 지정하세요.')
            self.plan['map_key'] = key
            self.plan['waypoints'][role], self.plan['closed'][role] = points, closed

    def _close_queue(self):
        points = list(self._queues()[0][self.role.get()])
        self._set_pending(points, self.closed.get())

    def _edit_queue(self, action):
        role = self.role.get()
        points, closed = self._queues()
        updated = list(points[role])
        index = None
        if action != 'add':
            selected = self.tree.selection()
            if not selected or not selected[0].startswith('pending:'):
                raise ValueError('대기 중인 waypoint를 선택하세요. 현재 목표와 완료된 점은 변경하지 않습니다.')
            index = int(selected[0].split(':')[1])
        if action in ('add', 'edit'):
            pose = validate_pose([v.get() for v in self.values['waypoint']], 'waypoint')
            if action == 'add':
                updated.append(pose)
            else:
                updated[index] = pose
        elif action == 'delete':
            updated.pop(index)
        else:
            other = index + (-1 if action == 'up' else 1)
            if 0 <= other < len(updated):
                updated[index], updated[other] = updated[other], updated[index]
        self._set_pending(updated, closed[role])
        self.ui._cancel_map_capture()
        self.status.set(f'{role} 대기 목록 변경 완료 · 현재 목표 유지')

    def _select_waypoint(self, _event):
        selected = self.tree.selection()
        if selected and selected[0].startswith('pending:'):
            pose = self._queues()[0][self.role.get()][int(selected[0].split(':')[1])]
            self.capture('waypoint', pose)

    def _configuration(self):
        plan = deepcopy(self.plan)
        for role in ('A', 'B'):
            plan[role.lower()] = self.robots[role].get()
            plan[role.lower() + '_route'] = self.routes[role].get()
        return validate_plan(plan)

    def _save(self):
        if self.demo.active:
            raise ValueError('시연 중 설정 저장은 중단 후 가능합니다.')
        save_plan(self.path, self._configuration())
        self.status.set(f'설정 저장: {self.path}')

    def _load(self):
        if self.demo.active:
            raise ValueError('시연 중에는 설정을 불러올 수 없습니다.')
        self.plan = validate_plan(json.loads(self.path.read_text()))
        self._fill_configuration()
        self.status.set('설정 불러옴 · 현재 지도/실제 위치를 확인하세요.')

    def _fill_configuration(self):
        for field in POSES:
            self.capture(field, self.plan[field])
        for role in ('A', 'B'):
            self.robots[role].set(self.plan[role.lower()])
            self.routes[role].set(self.plan[role.lower() + '_route'])

    def _start(self):
        if not self.ready.get():
            raise ValueError('두 로봇 현장 확인 체크 후 시작하세요.')
        self.demo.start(self._configuration(), self.ui._map_key())
        self.ready.set(False)
        self.status.set(self.demo.detail)

    def refresh(self, force=False):
        role = self.role.get()
        queues, closed = self._queues()
        rows = []
        if self.demo.active:
            rows += [(f'done:{i}', '완료', p) for i, p in enumerate(self.demo.done[role])]
            task = self.demo.tasks.get(role)
            if task and task['kind'] == 'NAV':
                rows.append(('current', task['request'].phase, task['goal']))
        rows += [(f'pending:{i}', '대기', p) for i, p in enumerate(queues[role])]
        signature = repr((role, rows, closed[role]))
        if force or signature != self._signature:
            selected = self.tree.selection()
            selected_values = self.tree.item(selected[0], 'values') if selected else None
            self.tree.delete(*self.tree.get_children())
            for key, state, pose in rows:
                self.tree.insert('', 'end', iid=key, values=(state, *(f'{v:.3f}' for v in pose)))
            if (selected and self.tree.exists(selected[0])
                    and self.tree.item(selected[0], 'values') == selected_values):
                self.tree.selection_set(selected[0])
            self.closed.set(closed[role])
            self._signature = signature
        self.progress.set(f'{self.demo.stage} · {self.demo.detail}')
