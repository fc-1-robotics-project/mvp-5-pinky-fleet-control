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
