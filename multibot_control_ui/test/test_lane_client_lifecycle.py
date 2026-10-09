from concurrent.futures import Future
from threading import Lock
from types import SimpleNamespace
import pytest
from vision_control.lane_client import RobotLaneClient


class Handle:
    accepted = True
    def __init__(self):
        self.cancelled = False
        self.result = Future()
    def cancel_goal_async(self):
        self.cancelled = True
        return Future()
    def get_result_async(self):
        return self.result


def client():
    c = RobotLaneClient.__new__(RobotLaneClient)
    c.lock = Lock()
    c._goal_handle = None
    c._generation = 1
    c._state = 'PENDING'
    c._status = ''
    return c


def test_cancel_pending_goal_cancels_it_when_acceptance_arrives():
    c = client()
    assert c.cancel_goal()
    assert c._state == 'CANCELLING'
    handle = Handle()
    accepted = Future()
    accepted.set_result(handle)
    c._goal_response(accepted, 1)
    assert handle.cancelled
    handle.result.set_result(None)
    assert c._state == 'CANCELED'


def test_old_acceptance_cannot_cancel_new_ui_state():
    c = client()
    c._generation, c._state = 3, 'FOLLOWING'
    handle = Handle()
    accepted = Future()
    accepted.set_result(handle)
    c._goal_response(accepted, 1)
    handle.result.set_result(None)
    assert handle.cancelled
    assert c._state == 'FOLLOWING'


def test_localization_requires_post_reset_sample_and_consistent_pose():
    import time
    c = client()
    now = time.monotonic()
    c._localization_epoch = (now - .2, 100.)
    covariance = [0.] * 36
    covariance[0] = covariance[7] = .0025
    covariance[35] = .0012
    message = SimpleNamespace(header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=101,nanosec=0)),
        pose=SimpleNamespace(covariance=covariance,pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.),
             orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
    c._localization = (now, message)
    assert c.localization_ready(now-.1, (1.,2.,0.))
    assert not c.localization_ready(now+.1, (1.,2.,0.))
    assert not c.localization_ready(now-.1, (3.,2.,0.))
    message.header.stamp.sec = 99
    assert not c.localization_ready(now-.1, (1.,2.,0.))


def test_cancel_late_completed_goal_does_not_deadlock():
    from threading import Thread
    c = client()
    assert c.cancel_goal()
    handle = Handle()
    handle.result.set_result(None)
    accepted = Future()
    accepted.set_result(handle)
    worker = Thread(target=c._goal_response, args=(accepted, 1), daemon=True)
    worker.start()
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert c._state == 'CANCELED'


def test_stopping_rejects_replacement_and_can_be_cancelled():
    c = client()
    c._state = 'STOPPING'
    assert not c.send_goal('new', 'A_to_B', 20., 300.)
    assert c._state == 'STOPPING'
    assert c.cancel_goal()
    assert c._state == 'CANCELLING'


def test_current_localization_requires_fresh_source_stamp_frame_and_covariance():
    import time
    c = client()
    covariance = [0.] * 36
    message = SimpleNamespace(header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=101, nanosec=0)),
        pose=SimpleNamespace(covariance=covariance, pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2.),
             orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    c.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=101_000_000_000)))
    c._fleet_localization = (time.monotonic(), message)
    assert c.current_localization() == (1., 2., 0.)
    message.header.stamp.sec = 99
    assert c.current_localization() is None
    message.header.stamp.sec = 101
    message.pose.covariance[0] = 1.
    assert c.current_localization() is None
    message.pose.covariance[0] = 0.
    message.header.frame_id = 'odom'
    assert c.current_localization() is None


def test_delayed_new_exit_status_cannot_be_freshened_by_network_receipt():
    import time
    c = client()
    c.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=101_000_000_000)))
    c._mission_received = time.monotonic()
    c._mission_status = dict(status_time_s=99., exit_status=dict(version=1))
    assert not c.telemetry()['fresh']
    c._mission_status['status_time_s'] = 100.5
    assert c.telemetry()['fresh']
    assert c.telemetry()['received_age_s'] == .5


def pose_client(position_variance=.25, yaw_variance=None):
    import math
    import time
    c = client()
    covariance = [0.] * 36
    covariance[0] = covariance[7] = position_variance
    covariance[35] = math.radians(15.) ** 2 if yaw_variance is None else yaw_variance
    message = SimpleNamespace(header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=101, nanosec=0)),
        pose=SimpleNamespace(covariance=covariance, pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2.),
             orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    c.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=101_000_000_000)))
    c._fleet_localization = (time.monotonic(), message)
    return c


def demo_limits():
    from multibot_control_ui.demo_mission import POSE_POSITION_VARIANCE_LIMIT, POSE_YAW_VARIANCE_LIMIT
    return dict(position_variance_limit=POSE_POSITION_VARIANCE_LIMIT,
                yaw_variance_limit=POSE_YAW_VARIANCE_LIMIT)


def test_manual_initial_pose_variance_is_accepted_by_demo_without_rewriting_covariance():
    from vision_control.lane_client import FleetLaneClients
    c = pose_client()
    original = list(c._fleet_localization[1].pose.covariance)
    assert c.current_localization() is None  # Legacy callers retain the stricter default.
    fleet = FleetLaneClients.__new__(FleetLaneClients)
    fleet.clients = {'robot1': c}
    assert fleet.current_localization('robot1', **demo_limits()) == (1., 2., 0.)
    assert fleet.localization_status('robot1', **demo_limits())['reason'] == '위치 수신 정상'
    assert c._fleet_localization[1].pose.covariance == original


def test_demo_covariance_bounds_have_specific_reasons_and_remain_bounded():
    import math
    c = pose_client(.30, math.radians(20.) ** 2)
    assert c.current_localization(**demo_limits()) is not None
    covariance = c._fleet_localization[1].pose.covariance
    covariance[7] = .30001
    assert c.current_localization(**demo_limits()) is None
    assert '위치 분산 범위 초과' in c.localization_status(**demo_limits())['reason']
    assert '0.300m²' in c.localization_status(**demo_limits())['reason']
    covariance[7] = .25
    covariance[35] = math.radians(20.01) ** 2
    assert c.current_localization(**demo_limits()) is None
    assert '방향 분산 범위 초과' in c.localization_status(**demo_limits())['reason']


def test_relaxed_demo_limits_do_not_accept_stale_invalid_or_unknown_localization():
    import math
    import time
    for fault in ('missing', 'received_stale', 'source_stale', 'future', 'frame',
                  'nan_position', 'bad_quaternion', 'negative_variance', 'nan_variance'):
        c = pose_client()
        message = c._fleet_localization[1]
        if fault == 'missing':
            c._fleet_localization = None
        elif fault == 'received_stale':
            c._fleet_localization = (time.monotonic()-1.6, message)
        elif fault == 'source_stale':
            message.header.stamp.sec = 99
        elif fault == 'future':
            message.header.stamp.sec = 102
        elif fault == 'frame':
            message.header.frame_id = 'odom'
        elif fault == 'nan_position':
            message.pose.pose.position.x = math.nan
        elif fault == 'bad_quaternion':
            message.pose.pose.orientation.w = 0.
        elif fault == 'negative_variance':
            message.pose.covariance[0] = -.1
        elif fault == 'nan_variance':
            message.pose.covariance[35] = math.nan
        status = c.localization_status(**demo_limits())
        assert status['pose'] is None, fault
        assert status['reason'], fault


def timed_pose_client(monkeypatch, source_age_ms, received_age_s=0.):
    monkeypatch.setattr('vision_control.lane_client.time.monotonic', lambda: 1000.)
    c = pose_client()
    message = c._fleet_localization[1]
    source_ns = 101_000_000_000 - source_age_ms * 1_000_000
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(source_ns, 1_000_000_000)
    c._fleet_localization = (1000. - received_age_s, message)
    c._mission_received = 1000. - received_age_s
    c._mission_status = dict(status_time_s=source_ns / 1e9, exit_status=dict(version=1))
    return c


@pytest.mark.parametrize('source_age_ms, expected_fresh', [
    (-15, True), (-60, True), (-100, True), (-101, False),
    (1500, True), (1501, False),
])
def test_demo_pose_and_status_share_bounded_clock_tolerance(monkeypatch, source_age_ms, expected_fresh):
    c = timed_pose_client(monkeypatch, source_age_ms)
    assert (c.current_localization(**demo_limits()) is not None) is expected_fresh
    assert c.telemetry()['fresh'] is expected_fresh


@pytest.mark.parametrize('received_age_s, expected_fresh', [
    (-.001, False), (1.5, True), (1.6, False),
])
def test_small_source_clock_lead_does_not_bypass_receive_age(monkeypatch, received_age_s, expected_fresh):
    c = timed_pose_client(monkeypatch, -60, received_age_s)
    assert (c.current_localization(**demo_limits()) is not None) is expected_fresh
    assert c.telemetry()['fresh'] is expected_fresh


@pytest.mark.parametrize('stamp', ['missing', None, float('nan'), float('inf'), float('-inf'), True, False])
def test_clock_tolerance_does_not_accept_invalid_status_timestamps(monkeypatch, stamp):
    c = timed_pose_client(monkeypatch, -60)
    if stamp == 'missing':
        c._mission_status.pop('status_time_s')
    else:
        c._mission_status['status_time_s'] = stamp
    assert not c.telemetry()['fresh']
