from concurrent.futures import Future
from threading import Lock
from types import SimpleNamespace
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
