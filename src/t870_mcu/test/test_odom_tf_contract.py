from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from t870_mcu.frame_contract import validate_enabled_tree
from t870_mcu.odom_tf_contract import (
    copy_odom_pose_to_transform, odom_transform_matches,
    odom_ownership_fault, validate_frame_contract)


ROOT = Path(__file__).resolve().parents[3]


def point(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    return NS(x=x, y=y, z=z, w=w)


def fake_odom():
    return NS(
        header=NS(stamp=NS(sec=123, nanosec=456), frame_id="odom"),
        child_frame_id="base_link",
        pose=NS(pose=NS(position=point(1.2, -0.4, 0.0),
                        orientation=quaternion(0.0, 0.0, 0.2, 0.98))))


def fake_tf():
    return NS(
        header=NS(stamp=NS(sec=0, nanosec=0), frame_id=""),
        child_frame_id="",
        transform=NS(translation=point(), rotation=quaternion()))


def load_frames():
    path = ROOT / "src/t870_mcu/config/t870_frames.yaml"
    return yaml.safe_load(path.read_text())


def test_odom_and_tf_use_identical_stamp_frames_pose_and_quaternion():
    odom = fake_odom()
    tf = copy_odom_pose_to_transform(odom, fake_tf())
    assert odom_transform_matches(odom, tf) == (True, "ok")
    assert tf.header.stamp.sec == 123 and tf.header.stamp.nanosec == 456
    assert tf.header.frame_id == "odom" and tf.child_frame_id == "base_link"


def test_timestamp_mismatch_is_detected():
    odom = fake_odom()
    tf = copy_odom_pose_to_transform(odom, fake_tf())
    tf.header.stamp = NS(sec=123, nanosec=457)
    assert odom_transform_matches(odom, tf)[1] == "timestamp_mismatch"


def test_position_or_quaternion_mismatch_is_detected():
    odom = fake_odom()
    tf = copy_odom_pose_to_transform(odom, fake_tf())
    tf.transform.rotation.w = 0.5
    assert odom_transform_matches(odom, tf)[1] == "pose_mismatch"


def test_duplicate_odom_topic_or_tf_owner_fails_closed():
    assert odom_ownership_fault([], False) is None
    assert odom_ownership_fault(["wheel_odom"], False) == (
        "FAIL_DUPLICATE_TF: existing odom->base_link or /odom owner "
        "['wheel_odom']")
    assert odom_ownership_fault([], True).startswith("FAIL_DUPLICATE_TF:")


@pytest.mark.parametrize("parent,child,reason", (
    ("", "base_link", "frame_id_empty"),
    ("/odom", "base_link", "frame_id_must_not_start_with_slash"),
    ("odom", "odom", "parent_equals_child"),
))
def test_wrong_frame_ids_are_rejected(parent, child, reason):
    assert validate_frame_contract(parent, child) == (False, reason)


def test_static_sensor_mounts_match_vehicle_request():
    data = load_frames()
    frames = data["frames"]
    camera = frames["camera_link"]
    assert camera["parent"] == "base_link"
    assert (camera["x"], camera["y"], camera["z"]) == (0.32, 0.0, 0.85)
    assert camera["pitch"] == pytest.approx(-0.0872664626)
    laser = frames["front_laser"]
    assert laser["parent"] == "base_link"
    assert (laser["x"], laser["y"], laser["z"]) == (0.65, 0.0, 0.20)
    assert "laser" not in frames
    assert frames["rear_laser"]["enabled"] is False
    assert camera["measurement_status"] == "remeasure_required"
    assert laser["measurement_status"] == "remeasure_required"


def test_vehicle_geometry_fails_closed_instead_of_selecting_guess():
    geometry = load_frames()["vehicle_geometry"]
    assert geometry["confirmed"] is False
    assert geometry["wheelbase_m"] is None
    assert geometry["track_width_m"] is None
    assert geometry["diagnostic"] == "FAIL_VEHICLE_GEOMETRY_UNCONFIRMED"


def test_enabled_tf_tree_is_connected_to_base_link():
    assert validate_enabled_tree(load_frames()["frames"]) == []


def test_disconnected_and_cyclic_tf_trees_are_detected():
    assert validate_enabled_tree({"sensor": {"parent": "missing"}}) == [
        "orphan:sensor->missing"]
    errors = validate_enabled_tree({
        "a": {"parent": "b"}, "b": {"parent": "a"}})
    assert any(error.startswith("cycle:") for error in errors)


def test_bridge_is_only_production_odom_owner_and_mirror_is_diagnostic():
    config = yaml.safe_load(
        (ROOT / "src/t870_mcu/config/t870_mcu.yaml").read_text())
    params = config["mcu_bridge"]["ros__parameters"]
    assert params["pub_topic_odom"] == "/odom"
    assert params["pub_topic_odom_diagnostic"] == "/mcu/odom"
    assert params["publish_tf"] is True
    assert params["vehicle_geometry_confirmed"] is False
    assert params["wheelbase_m"] == 0.0
    assert params["track_width_m"] == 0.0


def test_duplicate_odom_tf_and_front_lidar_owners_are_guarded():
    bridge = (ROOT / "src/t870_mcu/t870_mcu/bridge_node.py").read_text()
    assert "odom_ownership_fault" in bridge
    assert "get_publishers_info_by_topic(\"/odom\")" in bridge
    assert "front_lidar_static_tf" in bridge
    assert "existing_tf" in bridge


def test_front_steering_wheels_are_not_static_frames():
    config = (ROOT / "src/t870_mcu/config/t870_frames.yaml").read_text()
    launch = (ROOT / "src/t870_mcu/launch/t870_frames.launch.py").read_text()
    assert "front_left_wheel" not in config + launch
    assert "front_right_wheel" not in config + launch
    assert "/mcu/steer_deg" in config


def test_avoidance_mount_tf_is_disabled_by_default():
    path = Path("/home/qor/avoidance_sim/src/avoidance_lidar/launch/lidar_driver.launch.py")
    if not path.exists():
        pytest.skip("avoidance_sim checkout unavailable")
    source = path.read_text()
    assert "DeclareLaunchArgument('publish_mount_tf', default_value='false')" in source
    assert "condition=IfCondition(publish_mount_tf)" in source
