from pathlib import Path

import pytest

from t870_mcu.arbitration import InputManager
from t870_mcu.mode_policy import ModeArbitrator, VALID_MODES


SOURCES = ("lidar", "camera", "gps", "parking", "manual")


@pytest.fixture
def rig():
    inputs = InputManager(SOURCES)
    policy = ModeArbitrator(inputs, mode_timeout_s=2.0)
    return inputs, policy


def mode(rig, value, now=100.0):
    inputs, policy = rig
    assert policy.set_mode(value, now)
    return inputs, policy, now


def command(inputs, source, drive, wheel, now=100.0):
    inputs.update_drive(source, drive, now, True, "ok")
    inputs.update_wheel(source, wheel, now, True, "ok")


def signal(policy, name, value, now=100.0):
    policy.update_signal(name, value, now)


def camera_ready(policy, now=100.0, source="CAMERA_PATH"):
    signal(policy, "camera_authority", True, now)
    signal(policy, "camera_source", source, now)


def general_clear(policy, now=100.0):
    signal(policy, "front_obstacle", False, now)


def lidar9_ready(policy, now=100.0, stop=False):
    signal(policy, "lidar_stop", stop, now)
    signal(policy, "lidar_status_valid", True, now)


def test_all_production_mode_numbers_are_accepted_and_echoable(rig):
    _, policy = rig
    assert VALID_MODES == {str(value) for value in range(1, 12)}
    for value in range(1, 12):
        assert policy.set_mode(str(value), 100.0 + value)
        assert policy.mode == str(value)


@pytest.mark.parametrize("bad", ("", "0", "12", "NORMAL", "  ", "2.0"))
def test_invalid_mode_fails_closed(rig, bad):
    _, policy = rig
    assert not policy.set_mode(bad, 100.0)
    assert not policy.evaluate(100.0).ready
    assert policy.evaluate(100.0).drive == 0.0


def test_mode_stale_fails_closed(rig):
    _, policy, now = mode(rig, "1")
    assert policy.evaluate(now + 2.1).safety_state == "MODE_INVALID_OR_STALE"


def test_mode_change_invalidates_old_commands_and_signals(rig):
    inputs, policy, now = mode(rig, "5")
    command(inputs, "lidar", 2.0, 10, now)
    signal(policy, "avoidance_active", True, now)
    assert policy.evaluate(now).drive_source == "lidar"
    assert policy.set_mode("1", now + 0.1)
    assert not policy.evaluate(now + 0.1).ready


def test_mode1_ignores_continuous_inactive_lidar_zero(rig):
    inputs, policy, now = mode(rig, "1")
    command(inputs, "lidar", 0.0, 20, now)
    command(inputs, "camera", 2.0, -7, now)
    camera_ready(policy, now)
    general_clear(policy, now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel) == (2.0, -7)
    assert result.drive_source == result.wheel_source == "camera"


def test_general_obstacle_safe_gps_dr_fallback(rig):
    inputs, policy, now = mode(rig, "3")
    command(inputs, "camera", 2.0, 5, now)
    command(inputs, "gps", 1.0, -9, now)
    signal(policy, "front_obstacle", True, now)
    signal(policy, "gps_path_safe", True, now)
    signal(policy, "path_safety_status", "GPS_DR_SAFE", now)
    result = policy.evaluate(now)
    assert (result.drive_source, result.wheel_source) == ("gps", "gps")
    assert (result.drive, result.wheel) == (1.0, -9)


@pytest.mark.parametrize("safe,status", ((False, "UNSAFE"), (True, "UNSAFE")))
def test_general_obstacle_unsafe_stops(rig, safe, status):
    inputs, policy, now = mode(rig, "8")
    command(inputs, "gps", 2.0, 0, now)
    signal(policy, "front_obstacle", True, now)
    signal(policy, "gps_path_safe", safe, now)
    signal(policy, "path_safety_status", status, now)
    assert policy.evaluate(now).drive == 0.0


def test_general_gps_safety_stale_stops(rig):
    inputs, policy, now = mode(rig, "1")
    command(inputs, "gps", 2.0, 0, now)
    signal(policy, "front_obstacle", True, now)
    assert policy.evaluate(now).safety_state == "GPS_DR_PATH_SAFETY_STALE"


def test_obstacle_removed_returns_to_camera(rig):
    inputs, policy, now = mode(rig, "1")
    command(inputs, "camera", 2.0, 4, now)
    command(inputs, "gps", 1.0, -4, now)
    camera_ready(policy, now)
    signal(policy, "front_obstacle", True, now)
    signal(policy, "gps_path_safe", True, now)
    signal(policy, "path_safety_status", "SAFE", now)
    assert policy.evaluate(now).drive_source == "gps"
    signal(policy, "front_obstacle", False, now + 0.1)
    assert policy.evaluate(now + 0.1).drive_source == "camera"


def test_mode2_fresh_camera_zero_is_valid_and_immediate(rig):
    inputs, policy, now = mode(rig, "2")
    command(inputs, "camera", 0.0, 8, now)
    command(inputs, "gps", 2.0, -8, now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel, result.drive_source) == (0.0, 8, "camera")


def test_mode2_camera_stale_falls_back_to_gps(rig):
    inputs, policy, now = mode(rig, "2")
    command(inputs, "camera", 0.0, 8, now - 1.0)
    command(inputs, "gps", 2.0, -8, now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel, result.drive_source) == (2.0, -8, "gps")


@pytest.mark.parametrize("route_mode", ("4", "6"))
def test_intersection_stop_uses_camera_zero_and_gps_wheel(rig, route_mode):
    inputs, policy, now = mode(rig, route_mode)
    command(inputs, "camera", 0.0, 21, now)
    command(inputs, "gps", 2.0, -11, now)
    signal(policy, "camera_authority", True, now)
    signal(policy, "camera_source", "MISSION", now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel) == (0.0, -11)
    assert (result.drive_source, result.wheel_source) == ("camera", "gps")


@pytest.mark.parametrize("route_mode", ("4", "6"))
def test_intersection_go_uses_gps_drive_and_wheel(rig, route_mode):
    inputs, policy, now = mode(rig, route_mode)
    command(inputs, "gps", 2.0, 13, now)
    signal(policy, "camera_authority", False, now)
    signal(policy, "camera_source", "GPS_DR", now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel) == (2.0, 13)
    assert result.drive_source == result.wheel_source == "gps"


def test_intersection_unknown_or_stale_stops(rig):
    inputs, policy, now = mode(rig, "4")
    command(inputs, "gps", 2.0, 0, now)
    assert not policy.evaluate(now).ready
    signal(policy, "camera_authority", False, now)
    signal(policy, "camera_source", "INACTIVE", now)
    assert policy.evaluate(now).drive == 0.0


def test_mode5_false_uses_camera(rig):
    inputs, policy, now = mode(rig, "5")
    command(inputs, "camera", 2.0, 6, now)
    command(inputs, "lidar", 1.0, -15, now)
    signal(policy, "avoidance_active", False, now)
    camera_ready(policy, now)
    result = policy.evaluate(now)
    assert (result.drive_source, result.wheel_source) == ("camera", "camera")


def test_mode5_true_uses_lidar(rig):
    inputs, policy, now = mode(rig, "5")
    command(inputs, "lidar", 1.0, -15, now)
    signal(policy, "avoidance_active", True, now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel) == (1.0, -15)
    assert result.drive_source == result.wheel_source == "lidar"


def test_mode5_true_to_false_discards_lidar_ownership_immediately(rig):
    inputs, policy, now = mode(rig, "5")
    command(inputs, "lidar", 1.0, -15, now)
    command(inputs, "camera", 2.0, 7, now)
    signal(policy, "avoidance_active", True, now)
    assert policy.evaluate(now).wheel_source == "lidar"
    signal(policy, "avoidance_active", False, now + 0.01)
    camera_ready(policy, now + 0.01)
    result = policy.evaluate(now + 0.01)
    assert (result.drive, result.wheel) == (2.0, 7)
    assert result.wheel_source == "camera"


def test_mode5_stale_gate_falls_back_not_old_lidar(rig):
    inputs, policy, now = mode(rig, "5")
    command(inputs, "lidar", 1.0, -15, now)
    command(inputs, "gps", 2.0, 3, now + 0.6)
    signal(policy, "avoidance_active", True, now)
    assert policy.evaluate(now).drive_source == "lidar"
    assert policy.evaluate(now + 0.6).drive_source == "gps"


@pytest.mark.parametrize("route_mode", ("7", "10"))
def test_parking_requires_separate_active_fresh_commands(rig, route_mode):
    inputs, policy, now = mode(rig, route_mode)
    command(inputs, "lidar", 2.0, 15, now)
    assert not policy.evaluate(now).ready
    command(inputs, "parking", -1.0, 27, now)
    signal(policy, "parking_active", True, now)
    result = policy.evaluate(now)
    assert (result.drive, result.wheel) == (-1.0, 27)
    assert result.drive_source == result.wheel_source == "parking"


@pytest.mark.parametrize("level", (0.0, 1.0, 2.0))
def test_mode9_lidar_speed_levels_have_priority(rig, level):
    inputs, policy, now = mode(rig, "9")
    inputs.update_drive("lidar", level, now, True, "ok")
    command(inputs, "camera", 3.0, 4, now)
    lidar9_ready(policy, now)
    result = policy.evaluate(now)
    assert result.drive == level
    assert (result.wheel, result.wheel_source) == (4, "camera")


def test_mode9_lidar_stop_is_hard_stop(rig):
    inputs, policy, now = mode(rig, "9")
    inputs.update_drive("lidar", 2.0, now, True, "ok")
    command(inputs, "camera", 3.0, 0, now)
    lidar9_ready(policy, now, stop=True)
    result = policy.evaluate(now)
    assert result.drive == 0.0 and result.hard_stop


def test_mode9_stale_or_invalid_status_stops(rig):
    inputs, policy, now = mode(rig, "9")
    inputs.update_drive("lidar", 2.0, now, True, "ok")
    signal(policy, "lidar_stop", False, now)
    assert policy.evaluate(now).hard_stop


def test_mode11_ignores_intersection_signal_and_drives_camera(rig):
    inputs, policy, now = mode(rig, "11")
    command(inputs, "camera", 2.0, 5, now)
    camera_ready(policy, now)
    general_clear(policy, now)
    signal(policy, "camera_source", "MISSION", now)
    assert policy.evaluate(now).drive_source == "camera"


def test_final_wheel_is_always_clamped_to_vehicle_limit(rig):
    _, policy = rig
    assert policy._result(1.0, 99, "x", "x", "OK").wheel == 27
    assert policy._result(1.0, -99, "x", "x", "OK").wheel == -27


def test_removed_camera_stop_and_single_final_publishers():
    root = Path(__file__).resolve().parents[3]
    manager = (root / "src/t870_mcu/t870_mcu/manager_node.py").read_text()
    config = (root / "src/t870_mcu/config/t870_mcu.yaml").read_text()
    combined = manager + config
    assert "/camera_stop" not in combined
    assert manager.count("create_publisher(Float32, self.output_drive_topic") == 1
    assert manager.count("create_publisher(Int32, self.output_wheel_topic") == 1
