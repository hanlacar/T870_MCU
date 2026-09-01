"""t870_frames.launch.py — 센서 장착 위치 static TF 발행.

    ros2 launch t870_mcu t870_frames.launch.py

odom 과는 다른 것이다.
    odom       차가 어디까지 왔나 (t870_mcu.launch.py 의 브릿지가 발행)
    이 런치     센서가 차의 어디에 붙어 있나 (고정값)

값은 config/t870_frames.yaml 에서 읽는다. 실측해서 그 파일을 고칠 것.

다른 파일을 쓰려면:
    ros2 launch t870_mcu t870_frames.launch.py frames:=/path/to/my.yaml

확인:
    ros2 run tf2_tools view_frames        # 트리 그림을 PDF 로
    ros2 run tf2_ros tf2_echo base_link front_laser
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from t870_mcu.frame_contract import validate_enabled_tree


def _make_nodes(context, *args, **kwargs):
    path = LaunchConfiguration("frames").perform(context)
    with open(path, "r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp) or {}

    frames = dict(data.get("frames") or {})
    if not frames:
        raise RuntimeError("%s 에 frames 항목이 없다" % path)
    tree_errors = validate_enabled_tree(frames)
    if tree_errors:
        raise RuntimeError("invalid static TF tree: %s" % tree_errors)

    nodes = []
    geometry = data.get("vehicle_geometry") or {}
    confirmed = bool(geometry.get("confirmed", False))
    wheelbase = geometry.get("wheelbase_m")
    track_width = geometry.get("track_width_m")
    if confirmed:
        try:
            wheelbase = float(wheelbase)
            track_width = float(track_width)
        except (TypeError, ValueError):
            raise RuntimeError(
                "confirmed vehicle geometry requires numeric wheelbase/track_width")
        if wheelbase <= 0.0 or track_width <= 0.0:
            raise RuntimeError(
                "confirmed vehicle geometry requires positive wheelbase/track_width")
        frames["front_axle"] = {
            "enabled": True, "parent": "base_link", "x": wheelbase,
            "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        frames["rear_left_wheel"] = {
            "enabled": True, "parent": "rear_axle", "x": 0.0,
            "y": track_width / 2.0, "z": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        frames["rear_right_wheel"] = {
            "enabled": True, "parent": "rear_axle", "x": 0.0,
            "y": -track_width / 2.0, "z": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    else:
        nodes.append(LogInfo(msg=(
            "FAIL_VEHICLE_GEOMETRY_UNCONFIRMED: front_axle and wheel TFs "
            "disabled until measured wheelbase_m and track_width_m are set")))

    tree_errors = validate_enabled_tree(frames)
    if tree_errors:
        raise RuntimeError("invalid generated TF tree: %s" % tree_errors)

    for child, cfg in frames.items():
        if not bool(cfg.get("enabled", True)):
            nodes.append(LogInfo(msg="TF disabled (unmeasured): %s" % child))
            continue
        parent = str(cfg.get("parent", "base_link"))
        if cfg.get("measurement_status") == "remeasure_required":
            nodes.append(LogInfo(msg=(
                "TF_REMEASURE_REQUIRED: %s -> %s mount must be verified on vehicle"
                % (parent, child))))
        # static_transform_publisher 인자 순서:
        #   --x --y --z --roll --pitch --yaw --frame-id --child-frame-id
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_%s" % child,
            output="log",
            arguments=[
                "--x", str(float(cfg.get("x", 0.0))),
                "--y", str(float(cfg.get("y", 0.0))),
                "--z", str(float(cfg.get("z", 0.0))),
                "--roll", str(float(cfg.get("roll", 0.0))),
                "--pitch", str(float(cfg.get("pitch", 0.0))),
                "--yaw", str(float(cfg.get("yaw", 0.0))),
                "--frame-id", parent,
                "--child-frame-id", str(child),
            ],
        ))
    return nodes


def generate_launch_description():
    default_frames = str(
        Path(get_package_share_directory("t870_mcu")) / "config" / "t870_frames.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("frames", default_value=default_frames,
                              description="센서 장착 위치 yaml"),
        OpaqueFunction(function=_make_nodes),
    ])
