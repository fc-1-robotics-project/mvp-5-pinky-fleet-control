"""Fleet pose freshness uses both original source and receive time."""

from types import SimpleNamespace

import pytest

from multibot_control_ui.control_node import FleetControlNode


@pytest.mark.parametrize('stamp', [0., 95., 101.])
def test_receiving_old_zero_or_future_pose_does_not_make_it_fresh(stamp, monkeypatch):
    monkeypatch.setattr('multibot_control_ui.control_node.time.monotonic', lambda: 100.)
    pose = SimpleNamespace(header=SimpleNamespace(frame_id='map',
        stamp=SimpleNamespace(sec=int(stamp), nanosec=0)))
    fake = SimpleNamespace(poses={'robot1': pose}, pose_received_at={'robot1': 100.},
        pose_stale_sec=3., get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    fake.pose_age = lambda name: FleetControlNode.pose_age(fake, name)
    assert not FleetControlNode.pose_is_fresh(fake, 'robot1')


def test_pose_age_keeps_source_delay_after_transport_receives_again(monkeypatch):
    monkeypatch.setattr('multibot_control_ui.control_node.time.monotonic', lambda: 100.)
    pose = SimpleNamespace(header=SimpleNamespace(frame_id='map',
        stamp=SimpleNamespace(sec=98, nanosec=500_000_000)))
    fake = SimpleNamespace(poses={'robot1': pose}, pose_received_at={'robot1': 99.9},
        pose_stale_sec=3., get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    fake.pose_age = lambda name: FleetControlNode.pose_age(fake, name)
    assert fake.pose_age('robot1') == 1.5
    assert FleetControlNode.pose_is_fresh(fake, 'robot1')
    pose.header.frame_id = 'odom'
    assert not FleetControlNode.pose_is_fresh(fake, 'robot1')


@pytest.mark.parametrize('ahead,expected', [(50_000_000, True), (100_000_000, True), (110_000_000, False)])
def test_source_clock_tolerance_is_preserved_in_fleet_pose_age(ahead, expected, monkeypatch):
    monkeypatch.setattr('multibot_control_ui.control_node.time.monotonic', lambda: 100.)
    pose = SimpleNamespace(header=SimpleNamespace(frame_id='map',
        stamp=SimpleNamespace(sec=100, nanosec=ahead)))
    fake = SimpleNamespace(poses={'robot1': pose}, pose_received_at={'robot1': 100.},
        pose_stale_sec=3., get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    fake.pose_age = lambda name: FleetControlNode.pose_age(fake, name)
    assert FleetControlNode.pose_is_fresh(fake, 'robot1') is expected
