#!/usr/bin/env python3
"""Supervise one Pinky lane trial and release both motion permissions.

Start this recorder before requesting the lane test.  The operator must press
the UI's lane-finish Bool button at the course exit; this script never invents
that finish signal.  It can request a STOP and revoke both test permissions.
"""

import argparse
from collections import Counter
import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait-active-s', type=float, default=90.0)
    parser.add_argument('--max-active-s', type=float, default=900.0)
    parser.add_argument('--fault-hold-s', type=float, default=2.0)
    parser.add_argument('--lane-fault-hold-s', type=float, default=15.0)
    parser.add_argument('--stop-hold-s', type=float, default=2.0)
    parser.add_argument('--no-progress-s', type=float, default=3.0)
    parser.add_argument('--stuck-s', type=float, default=6.0)
    parser.add_argument('--telemetry-stale-s', type=float, default=2.5)
    parser.add_argument('--progress-m', type=float, default=0.003)
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0 for value in vars(args).values()):
        parser.error('all limits must be finite and positive')
    return args


class Domain:
    def __init__(self, domain_id):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = Node(f'pinky_trial_monitor_{domain_id}', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def spin_once(self, timeout_sec=0.01):
        self.executor.spin_once(timeout_sec=timeout_sec)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


class TrialMonitor:
    def __init__(self, args):
        self.args = args
        self.robot = Domain(21)
        try:
            self.central = Domain(22)
        except Exception:
            self.robot.close()
            raise
        self.enable = self.robot.node.create_client(SetBool, '/lane/set_enabled')
        self.test = self.central.node.create_client(SetBool, '/robot1/lane/test')
        self.mode_stop = self.robot.node.create_publisher(String, '/drive/mode_request', 10)
        self.latest = {}
        self.seen = {}
        self.diagnostic_counts = Counter()
        self.lane_reasons = Counter()
        self.faults = Counter()
        self.command_reasons = Counter()
        self.distance_m = 0.0
        self.peak_cmd_mps = 0.0
        self.previous_pose = None
        self.start_pose = None
        self.last_pose = None
        self.progress_anchor = None
        self.last_progress = None
        self.active_since = None
        self.fault_since = None
        self.fault_kind = None
        self.stop_since = None
        self.zero_cmd_since = None
        self.completed = False
        self.completed_since = None
        self.finish_seen = False
        self.finish_since = None
        self.signal_name = None
        self.lane_fault_samples = []
        node = self.robot.node
        node.create_subscription(String, '/lane/diagnostics', self.on_diagnostic,
                                 qos_profile_sensor_data)
        node.create_subscription(String, '/lane/observation', self.on_observation,
                                 qos_profile_sensor_data)
        node.create_subscription(String, '/lane/command', self.on_command,
                                 qos_profile_sensor_data)
        node.create_subscription(String, '/drive/mode_status', self.on_mode,
                                 qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odom', self.on_odom,
                                 qos_profile_sensor_data)
        node.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel,
                                 qos_profile_sensor_data)
        node.create_subscription(Bool, '/lane/completed', self.on_completed, 10)
        node.create_subscription(Bool, '/lane/finish', self.on_finish, 10)

    def now(self):
        return time.monotonic()

    def mark(self, topic):
        self.seen[topic] = self.now()

    @staticmethod
    def payload(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def on_diagnostic(self, message):
        item = self.payload(message)
        self.mark('diagnostic')
        self.latest['diagnostic'] = item
        if self.active_since is not None:
            self.diagnostic_counts[item.get('reason', 'malformed')] += 1
            self.lane_reasons[item.get('lane_reason', 'missing')] += 1
            if item.get('fault'):
                self.faults[item['fault']] += 1
            if (item.get('lane_reason') == 'no_current_lane'
                    and len(self.lane_fault_samples) < 40):
                points = item.get('path_points_m', ())
                path_length = None
                try:
                    path_length = round(sum(math.dist(a, b)
                                            for a, b in zip(points, points[1:])), 4)
                except (TypeError, ValueError):
                    pass
                self.lane_fault_samples.append({
                    'elapsed_s': round(self.now() - self.active_since, 2),
                    'control_reason': item.get('reason'),
                    'path_length_m': path_length,
                    'observation': self.latest.get('observation'),
                })

    def on_observation(self, message):
        item = self.payload(message)
        self.mark('observation')
        detections = item.get('detections', ())
        self.latest['observation'] = [
            {'class': detection.get('class_id'),
             'boundary_px_count': len(detection.get('boundary_px', ())),
             'first_px': detection.get('boundary_px', ())[0]
             if detection.get('boundary_px') else None,
             'last_px': detection.get('boundary_px', ())[-1]
             if detection.get('boundary_px') else None}
            for detection in detections if isinstance(detection, dict)
        ]

    def on_command(self, message):
        item = self.payload(message)
        self.mark('command')
        if self.active_since is not None:
            self.command_reasons[item.get('reason', 'malformed')] += 1

    def on_mode(self, message):
        item = self.payload(message)
        self.mark('mode')
        self.latest['mode'] = item.get('mode')

    def on_odom(self, message):
        self.mark('odom')
        point = (message.pose.pose.position.x, message.pose.pose.position.y)
        if not all(math.isfinite(value) for value in point):
            return
        if self.active_since is not None:
            if self.start_pose is None:
                self.start_pose = point
                self.progress_anchor = point
                self.last_progress = self.now()
            if self.previous_pose is not None:
                step = math.dist(self.previous_pose, point)
                if step < 0.05:  # Reject odometry resets, not ordinary travel.
                    self.distance_m += step
            if (self.progress_anchor is not None
                    and math.dist(self.progress_anchor, point) >= self.args.progress_m):
                self.progress_anchor = point
                self.last_progress = self.now()
            self.last_pose = point
        self.previous_pose = point

    def on_cmd_vel(self, message):
        self.mark('cmd_vel')
        speed = float(message.linear.x)
        omega = float(message.angular.z)
        self.latest['velocity'] = (speed, omega)
        if self.active_since is not None and math.isfinite(speed):
            self.peak_cmd_mps = max(self.peak_cmd_mps, abs(speed))

    def on_completed(self, message):
        self.mark('completed')
        if message.data is True and self.active_since is not None:
            self.completed = True
            self.completed_since = self.now()

    def on_finish(self, message):
        self.mark('finish')
        if message.data is True and self.active_since is not None:
            self.finish_seen = True
            self.finish_since = self.now()

    def spin(self, timeout_sec=0.02):
        self.robot.spin_once(timeout_sec / 2)
        self.central.spin_once(timeout_sec / 2)

    def call_false(self, client, domain, name):
        for attempt in range(1, 4):
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(SetBool.Request(data=False))
            deadline = self.now() + 3.0
            while not future.done() and self.now() < deadline:
                domain.spin_once(0.02)
            if not future.done():
                continue
            try:
                response = future.result()
                if response is not None and response.success:
                    return {'ok': True, 'attempt': attempt,
                            'message': response.message}
                error = response.message if response is not None else 'no response'
            except Exception as exc:
                error = repr(exc)
        return {'ok': False, 'attempt': 3, 'message': locals().get('error',
                                                                  name + ' unavailable')}

    def release_permissions(self):
        # STOP is an extra path to the robot-local mux if a service is delayed.
        results = {}
        try:
            for _ in range(3):
                self.mode_stop.publish(String(data='STOP'))
                self.robot.spin_once(0.02)
            results['mode_stop'] = {'ok': True}
        except Exception as exc:
            results['mode_stop'] = {'ok': False, 'message': repr(exc)}
        for key, client, domain, name in (
            ('local_enabled_false', self.enable, self.robot, 'local lane latch'),
            ('central_test_false', self.test, self.central, 'central lane test'),
        ):
            try:
                results[key] = self.call_false(client, domain, name)
            except Exception as exc:
                results[key] = {'ok': False, 'message': repr(exc)}
        try:
            for _ in range(3):
                self.mode_stop.publish(String(data='STOP'))
                self.robot.spin_once(0.02)
        except Exception as exc:
            results['mode_stop_after'] = {'ok': False, 'message': repr(exc)}
        return results

    def summary(self, reason, cleanup):
        net = (math.dist(self.start_pose, self.last_pose)
               if self.start_pose is not None and self.last_pose is not None
               else None)
        return {
            'stop_reason': reason,
            'completed_bool': self.completed,
            'finish_bool_seen': self.finish_seen,
            'active_duration_s': (round(self.now() - self.active_since, 2)
                                  if self.active_since is not None else None),
            'odom_path_length_m': round(self.distance_m, 4),
            'odom_net_displacement_m': round(net, 4) if net is not None else None,
            'peak_cmd_mps': round(self.peak_cmd_mps, 4),
            'last_cmd_vel': self.latest.get('velocity'),
            'last_mode': self.latest.get('mode'),
            'last_diagnostic': self.latest.get('diagnostic'),
            'diagnostic_reasons': dict(self.diagnostic_counts),
            'lane_reasons': dict(self.lane_reasons),
            'faults': dict(self.faults),
            'command_reasons': dict(self.command_reasons),
            'focus_counts': {
                'single_boundary': self.lane_reasons['single_boundary'],
                'recent_path_no_boundaries': self.lane_reasons['recent_path_no_boundaries'],
                'no_current_lane': self.lane_reasons['no_current_lane'],
                'lane_observation_stale': self.faults['lane_observation_stale'],
            },
            'lane_fault_samples': self.lane_fault_samples,
            'cleanup': cleanup,
        }

    def healthy_lane_fault(self, diagnostic):
        def fresh(key, limit):
            value = diagnostic.get(key)
            return isinstance(value, (int, float)) and math.isfinite(value) and 0 <= value <= limit
        return (diagnostic.get('fault') == 'lane:no_current_lane'
                and diagnostic.get('lane_reason') == 'no_current_lane'
                and self.now() - self.seen.get('observation', 0.) <= 1.1
                and fresh('scan_age_s', .3)
                and fresh('odom_age_s', .3)
                and diagnostic.get('observation_error') is None
                and diagnostic.get('sensor_error') is None
                and diagnostic.get('scan_coverage_ok') is True
                and diagnostic.get('emergency') is False)

    def run(self):
        started = self.now()
        last_print = 0.0
        reason = 'unknown'
        cleanup = None
        try:
            while True:
                self.spin()
                now = self.now()
                mode = self.latest.get('mode')
                diagnostic = self.latest.get('diagnostic', {})
                if self.signal_name is not None:
                    reason = 'signal_' + self.signal_name
                    break
                if self.active_since is None:
                    if mode == 'LANE':
                        self.active_since = now
                        self.previous_pose = None
                        self.diagnostic_counts.clear()
                        self.lane_reasons.clear()
                        self.faults.clear()
                        self.command_reasons.clear()
                        print('LANE_ACTIVE', flush=True)
                    elif now - started >= self.args.wait_active_s:
                        reason = 'lane_mode_not_reached'
                        break
                    continue  # The expected initial STOP is not a fault.
                if now - self.active_since >= self.args.max_active_s:
                    reason = 'active_deadline'
                    break
                if self.completed and (self.finish_seen
                                       or now - self.completed_since >= 1.0):
                    reason = ('operator_finish_bool' if self.finish_seen
                              else 'lane_completed_bool')
                    break
                if mode == 'STOP':
                    if self.stop_since is None:
                        self.stop_since = now
                    elif now - self.stop_since >= self.args.stop_hold_s:
                        reason = ('operator_finish_then_stop' if self.finish_seen
                                  else 'sustained_stop_mode')
                        break
                else:
                    self.stop_since = None
                if diagnostic.get('reason') == 'sensor_failure':
                    kind = ('lane_no_current' if self.healthy_lane_fault(diagnostic)
                            else 'other_sensor_failure')
                    if self.fault_kind != kind:
                        self.fault_kind = kind
                        self.fault_since = now
                    hold = (self.args.lane_fault_hold_s if kind == 'lane_no_current'
                            else self.args.fault_hold_s)
                    if now - self.fault_since >= hold:
                        reason = 'persistent_sensor_failure'
                        break
                else:
                    self.fault_since = None
                    self.fault_kind = None
                if self.finish_since is not None and now - self.finish_since >= 5.0:
                    reason = 'finish_signal_without_stop'
                    break
                if (self.last_progress is not None
                        and now - self.active_since >= 2.0):
                    stopped_for = now - self.last_progress
                    speed = abs(self.latest.get('velocity', (0.0, 0.0))[0])
                    if speed >= 0.002 and stopped_for >= self.args.stuck_s:
                        reason = 'no_odom_progress'
                        break
                velocity = self.latest.get('velocity')
                if (mode == 'LANE' and velocity is not None
                        and abs(velocity[0]) < 0.002
                        and abs(velocity[1]) < 0.01):
                    if self.zero_cmd_since is None:
                        self.zero_cmd_since = now
                    elif (now - self.active_since >= 2.0
                          and now - self.zero_cmd_since >= self.args.no_progress_s):
                        reason = 'persistent_stop'
                        break
                else:
                    self.zero_cmd_since = None
                if any(now - self.seen.get(key, 0.0) >= self.args.telemetry_stale_s
                       for key in ('diagnostic', 'command', 'mode', 'odom', 'cmd_vel')):
                    reason = 'telemetry_stale'
                    break
                if velocity and (not all(math.isfinite(v) for v in velocity)
                                 or abs(velocity[0]) > 0.0305
                                 or abs(velocity[1]) > 0.605):
                    reason = 'cmd_vel_limit_exceeded'
                    break
                if now - last_print >= 5.0:
                    print(json.dumps({
                        'elapsed_s': round(now - self.active_since, 1),
                        'odom_path_length_m': round(self.distance_m, 4),
                        'mode': mode,
                        'cmd_vel': velocity,
                        'lane_reason': diagnostic.get('lane_reason'),
                        'control_reason': diagnostic.get('reason'),
                        'fault': diagnostic.get('fault'),
                        'completed_bool': self.completed,
                        'finish_bool_seen': self.finish_seen,
                    }), flush=True)
                    last_print = now
        except KeyboardInterrupt:
            reason = 'operator_keyboard_interrupt'
        except Exception as exc:
            reason = 'monitor_exception:' + repr(exc)
        finally:
            try:
                cleanup = self.release_permissions()
            except Exception as exc:
                cleanup = {'error': repr(exc)}
            print('TRIAL_SUMMARY ' + json.dumps(self.summary(reason, cleanup),
                                                allow_nan=False), flush=True)
            self.central.close()
            self.robot.close()
        return 0 if (reason in {'operator_finish_bool', 'operator_finish_then_stop'}
                     and isinstance(cleanup, dict)
                     and all(isinstance(item, dict) and item.get('ok')
                             for item in cleanup.values())) else 2


def main():
    args = parse_args()
    monitor = TrialMonitor(args)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda number, _frame: setattr(
            monitor, 'signal_name', signal.Signals(number).name))
    return monitor.run()


if __name__ == '__main__':
    raise SystemExit(main())
