#!/usr/bin/env python3
"""Arm one supervised Pinky lane trial after fresh stationary ROS checks.

Run on the PC after the robot launch, domain bridge, UI, and independent trial
monitor are ready.  The monitor owns trial termination and permission cleanup.
This script fails closed if either enabling service does not acknowledge.
"""

import argparse
import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class Domain:
    def __init__(self, domain_id):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = Node(f'pinky_trial_start_{domain_id}', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def spin_once(self, timeout=0.02):
        self.executor.spin_once(timeout_sec=timeout)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


class Starter:
    PARAMS = ('dry_run', 'hardware_watchdog_confirmed',
              'command_timeout_s', 'max_source_age_s', 'max_speed_mps',
              'max_omega_radps', 'output_topic')

    def __init__(self):
        self.robot = Domain(21)
        try:
            self.central = Domain(22)
        except Exception:
            self.robot.close()
            raise
        self.diagnostic = None
        self.diagnostic_count = 0
        self.last = {}
        self.local = self.robot.node.create_client(SetBool, '/lane/set_enabled')
        self.test = self.central.node.create_client(SetBool, '/robot1/lane/test')
        self.watchdog = self.robot.node.create_client(
            GetParameters, '/lane_watchdog/get_parameters')
        self.safety = self.robot.node.create_client(
            GetParameters, '/lane_safety/get_parameters')
        self.stop_pub = self.robot.node.create_publisher(
            String, '/drive/mode_request', 10)
        node = self.robot.node
        node.create_subscription(String, '/lane/diagnostics', self.on_diag,
                                 qos_profile_sensor_data)
        node.create_subscription(String, '/drive/mode_status', self.on_mode,
                                 qos_profile_sensor_data)
        node.create_subscription(Twist, '/cmd_vel', self.on_cmd,
                                 qos_profile_sensor_data)
        node.create_subscription(Bool, '/lane/estop', self.on_estop,
                                 qos_profile_sensor_data)

    def close(self):
        self.central.close()
        self.robot.close()

    @staticmethod
    def decoded(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def on_diag(self, message):
        self.diagnostic = self.decoded(message)
        self.diagnostic_count += 1
        self.last['diagnostic'] = time.monotonic()

    def on_mode(self, message):
        self.last['mode'] = (time.monotonic(), self.decoded(message).get('mode'))

    def on_cmd(self, message):
        self.last['cmd'] = (time.monotonic(),
                            (message.linear.x, message.angular.z))

    def on_estop(self, message):
        self.last['estop'] = (time.monotonic(), message.data)

    def spin(self, timeout=0.02):
        self.robot.spin_once(timeout / 2)
        self.central.spin_once(timeout / 2)

    def await_future(self, future, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.spin()
        if not future.done():
            raise TimeoutError('ROS service did not answer within timeout')
        return future.result()

    def get_parameters(self, client, names, label):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{label} parameter service unavailable')
        response = self.await_future(client.call_async(
            GetParameters.Request(names=list(names))))
        if response is None or len(response.values) != len(names):
            raise RuntimeError(f'{label} parameters unavailable')
        values = {}
        for name, item in zip(names, response.values):
            if item.type == 1:
                values[name] = item.bool_value
            elif item.type == 3:
                values[name] = item.double_value
            elif item.type == 4:
                values[name] = item.string_value
            else:
                raise RuntimeError(f'{label}.{name} has unexpected type {item.type}')
        return values

    def verify_parameters(self):
        actual = self.get_parameters(self.watchdog, self.PARAMS, 'watchdog')
        expected = {
            'dry_run': False,
            'hardware_watchdog_confirmed': True,
            'command_timeout_s': .2,
            'max_source_age_s': 1.1,
            'max_speed_mps': .03,
            'max_omega_radps': .6,
            'output_topic': 'cmd_vel_lane_candidate',
        }
        for key, want in expected.items():
            got = actual[key]
            equal = (math.isclose(got, want, abs_tol=1e-6, rel_tol=0)
                     if isinstance(want, float) else got == want)
            if not equal:
                raise RuntimeError(f'watchdog.{key}: expected {want!r}, got {got!r}')
        safety = self.get_parameters(self.safety, ('start_enabled',), 'safety')
        if safety['start_enabled'] is not False:
            raise RuntimeError('lane_safety did not start disabled')
        return actual

    def ready(self):
        now = time.monotonic()
        for key in ('diagnostic', 'mode', 'cmd', 'estop'):
            stamp = (self.last[key] if key == 'diagnostic'
                     else self.last[key][0]) if key in self.last else None
            if stamp is None or now - stamp > .75:
                return False, f'{key} telemetry absent/stale'
        diagnostic = self.diagnostic
        if diagnostic.get('lane_valid') is not True:
            return False, f"lane invalid: {diagnostic.get('lane_reason')}"
        if diagnostic.get('observation_error') is not None:
            return False, 'lane observation error'
        if diagnostic.get('sensor_error') is not None:
            return False, 'sensor/TF error'
        if diagnostic.get('scan_coverage_ok') is not True:
            return False, 'scan coverage not validated'
        if diagnostic.get('scan_hit') is not False:
            return False, 'scan obstacle or unknown collision result'
        for field, limit in (('capture_age_s', 1.1), ('scan_age_s', .5),
                             ('odom_age_s', .5)):
            age = diagnostic.get(field)
            if not isinstance(age, (int, float)) or not math.isfinite(age) \
                    or age < 0 or age > limit:
                return False, f'{field} invalid/stale: {age!r}'
        if self.last['mode'][1] != 'STOP':
            return False, f"mode is {self.last['mode'][1]!r}, expected STOP"
        if self.last['estop'][1] is not True:
            return False, 'local lane E-stop is not asserted'
        linear, angular = self.last['cmd'][1]
        if (not math.isfinite(linear) or not math.isfinite(angular)
                or abs(linear) > 1e-4 or abs(angular) > 1e-4):
            return False, f'cmd_vel is nonzero: {(linear, angular)!r}'
        return True, 'ready'

    def await_ready(self, timeout):
        deadline = time.monotonic() + timeout
        stable = 0
        last_checked_count = 0
        last_reason = 'telemetry not received'
        while time.monotonic() < deadline:
            self.spin()
            if self.diagnostic_count == last_checked_count:
                continue
            last_checked_count = self.diagnostic_count
            good, last_reason = self.ready()
            stable = stable + 1 if good else 0
            if stable >= 3:  # Three distinct fresh diagnostics.
                return self.diagnostic
        raise RuntimeError(f'stationary preflight failed: {last_reason}')

    def set_bool(self, client, data, label):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{label} service unavailable')
        response = self.await_future(client.call_async(SetBool.Request(data=data)))
        if response is None or response.success is not True:
            message = response.message if response is not None else 'no response'
            raise RuntimeError(f'{label} rejected request: {message}')
        return response.message

    def rollback(self, central_attempted):
        failures = []
        if central_attempted:
            try:
                self.set_bool(self.test, False, 'central lane test rollback')
            except Exception as exc:
                failures.append(str(exc))
        try:
            self.set_bool(self.local, False, 'local lane latch rollback')
        except Exception as exc:
            failures.append(str(exc))
        for _ in range(3):
            self.stop_pub.publish(String(data='STOP'))
            self.robot.spin_once(.02)
        return failures

    def run(self, ready_timeout):
        local_attempted = False
        central_attempted = False
        try:
            parameters = self.verify_parameters()
            diagnostic = self.await_ready(ready_timeout)
            if not self.local.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('local lane latch unavailable')
            if not self.test.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('central lane test unavailable')
            good, reason = self.ready()
            if not good:
                raise RuntimeError('preflight changed before arming: ' + reason)
            local_attempted = True
            local_message = self.set_bool(self.local, True, 'local lane latch')
            central_attempted = True
            central_message = self.set_bool(self.test, True, 'central lane test')
            print('LANE_START_SUCCESS ' + json.dumps({
                'local': local_message, 'central': central_message,
                'lane_reason': diagnostic.get('lane_reason'),
                'watchdog': parameters,
            }, ensure_ascii=False), flush=True)
            return 0
        except Exception as exc:
            rollback_errors = (self.rollback(central_attempted)
                               if local_attempted else [])
            print('LANE_START_FAILED ' + json.dumps({
                'error': str(exc), 'rollback_errors': rollback_errors,
            }, ensure_ascii=False), flush=True)
            return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ready-timeout-s', type=float, default=20.0)
    args = parser.parse_args()
    if not math.isfinite(args.ready_timeout_s) or args.ready_timeout_s <= 0:
        parser.error('ready timeout must be finite and positive')
    starter = Starter()
    try:
        return starter.run(args.ready_timeout_s)
    finally:
        starter.close()


if __name__ == '__main__':
    raise SystemExit(main())
