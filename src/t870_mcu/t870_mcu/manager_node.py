#!/usr/bin/env python3
"""Production MCU manager: freshness checks and explicit mode arbitration."""

import json
import math
import time
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from .arbitration import ArbitrationStatus, InputManager, SafetyManager
from .diagnostics import check_subscriptions
from .mode_policy import ModeArbitrator


STAGE_MPS = {1: 0.229, 2: 0.526, 3: 0.823}


class ManagerNode(Node):
    """Sole owner of /mcu_drive and /mcu_wheel."""

    def __init__(self):
        super().__init__("mcu_manager")
        defaults = {
            "source_names": ["lidar", "camera", "gps", "parking", "manual"],
            "publish_hz": 20.0, "drive_timeout_s": 0.5,
            "wheel_timeout_s": 0.5, "signal_timeout_s": 0.5,
            "mode_timeout_s": 1.0, "stop_timeout_s": 0.5,
            "manual_override_enabled": True, "manual_source_name": "manual",
            "drive_validation_mode": "allowed_values",
            "drive_allowed_values": [-1.0, 0.0, 1.0, 2.0, 3.0],
            "drive_min": -1.0, "drive_max": 3.0,
            "wheel_min": -27, "wheel_max": 27,
            "max_steer_deg": 27.0, "wheelbase_m": 0.0,
            "mps_deadband": 0.05, "mode_topic": "/drive_mode",
            "estop_topic": "/estop_lock",
            "output_drive_topic": "/mcu_drive",
            "output_wheel_topic": "/mcu_wheel",
            "output_stop_topic": "/mcu_stop",
            "status_mode_topic": "/mcu/current_mode",
            "status_drive_source_topic": "/mcu/active_drive_source",
            "status_wheel_source_topic": "/mcu/active_wheel_source",
            "status_safety_topic": "/mcu/safety_state",
            "status_ready_topic": "/mcu/manager_ready",
            "avoidance_active_topic": "/avoidance/active",
            "camera_authority_topic": "/camera/control_authority",
            "camera_source_topic": "/camera/command_source",
            "front_obstacle_topic": "/lidar/front_obstacle_0_5m",
            "gps_path_safe_topic": "/lidar/gps_dr_path_safe",
            "path_safety_status_topic": "/lidar/path_safety_status",
            "lidar_stop_topic": "/lidar_stop",
            "lidar_status_topic": "/avoidance/safety/status",
            "parking_active_topic": "/parking/active",
            "parking_drive_topic": "/parking_drive",
            "parking_wheel_topic": "/parking_wheel",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        gp = lambda name: self.get_parameter(name).value

        self.source_names = [str(v).strip().lower() for v in gp("source_names")]
        self.drive_timeout = float(gp("drive_timeout_s"))
        self.wheel_timeout = float(gp("wheel_timeout_s"))
        self.signal_timeout = float(gp("signal_timeout_s"))
        self.stop_timeout = float(gp("stop_timeout_s"))
        self.manual_override = bool(gp("manual_override_enabled"))
        self.manual_name = str(gp("manual_source_name")).strip().lower()
        self.max_steer_deg = float(gp("max_steer_deg"))
        self.wheelbase = float(gp("wheelbase_m"))
        self.mps_deadband = float(gp("mps_deadband"))
        if not 0.0 < self.max_steer_deg <= 27.0:
            raise ValueError("max_steer_deg must be in (0, 27]")
        if self.manual_name not in self.source_names:
            raise ValueError("manual source missing from source_names")

        self.safety = SafetyManager(
            gp("drive_validation_mode"), gp("drive_allowed_values"),
            gp("drive_min"), gp("drive_max"), gp("wheel_min"), gp("wheel_max"))
        self.inputs = InputManager(self.source_names)
        self.policy = ModeArbitrator(
            self.inputs, self.drive_timeout, self.wheel_timeout,
            self.signal_timeout, float(gp("mode_timeout_s")),
            int(self.max_steer_deg))
        self.estop_asserted = False
        self.last_status = ArbitrationStatus()
        self._diag_seen, self._sub_specs = set(), []
        self._publisher_conflicts = []

        self.src_cfg = {}
        for source in self.source_names:
            drive_default = (str(gp("parking_drive_topic")) if source == "parking"
                             else "/%s_drive" % source)
            wheel_default = (str(gp("parking_wheel_topic")) if source == "parking"
                             else "/%s_wheel" % source)
            for name, default in (
                    ("%s_drive_topic" % source, drive_default),
                    ("%s_wheel_topic" % source, wheel_default),
                    ("%s_drive_unit" % source, "stage"),
                    ("%s_wheel_type" % source, "int"),
                    ("%s_cmd_vel_topic" % source, "")):
                if not self.has_parameter(name):
                    self.declare_parameter(name, default)
            self._subscribe_source(source)

        self._sub(String, gp("mode_topic"), self._cb_mode, "route mode")
        self._sub(Bool, gp("estop_topic"), self._cb_estop, "emergency stop")
        signal_specs = (
            ("avoidance_active", Bool, "avoidance_active_topic"),
            ("camera_authority", Bool, "camera_authority_topic"),
            ("camera_source", String, "camera_source_topic"),
            ("front_obstacle", Bool, "front_obstacle_topic"),
            ("gps_path_safe", Bool, "gps_path_safe_topic"),
            ("path_safety_status", String, "path_safety_status_topic"),
            ("lidar_stop", Bool, "lidar_stop_topic"),
            ("parking_active", Bool, "parking_active_topic"),
        )
        for signal, msg_type, parameter in signal_specs:
            self._sub(msg_type, gp(parameter), partial(self._cb_signal, signal), signal)
        self._sub(String, gp("lidar_status_topic"), self._cb_lidar_status,
                  "LiDAR safety status")
        self._sub(Bool, "/manual_stop", self._cb_manual_stop,
                  "manual emergency stop")

        self.output_drive_topic = str(gp("output_drive_topic"))
        self.output_wheel_topic = str(gp("output_wheel_topic"))
        self.pub_drive = self.create_publisher(Float32, self.output_drive_topic, 10)
        self.pub_wheel = self.create_publisher(Int32, self.output_wheel_topic, 10)
        self.pub_stop = self.create_publisher(Bool, str(gp("output_stop_topic")), 10)
        self.pub_mode = self.create_publisher(String, str(gp("status_mode_topic")), 10)
        self.pub_dsrc = self.create_publisher(
            String, str(gp("status_drive_source_topic")), 10)
        self.pub_wsrc = self.create_publisher(
            String, str(gp("status_wheel_source_topic")), 10)
        self.pub_safety = self.create_publisher(
            String, str(gp("status_safety_topic")), 10)
        self.pub_ready = self.create_publisher(
            Bool, str(gp("status_ready_topic")), 10)

        hz = float(gp("publish_hz"))
        if hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(3.0, self._diagnose)
        self.get_logger().info(
            "mode arbitration ready; final owner of %s and %s" %
            (self.output_drive_topic, self.output_wheel_topic))

    def _sub(self, msg_type, topic, callback, label):
        topic = str(topic)
        self.create_subscription(msg_type, topic, callback, 10)
        type_name = {Bool: "std_msgs/msg/Bool", Float32: "std_msgs/msg/Float32",
                     Int32: "std_msgs/msg/Int32", String: "std_msgs/msg/String"}[msg_type]
        self._sub_specs.append((topic, type_name, label))

    def _subscribe_source(self, source):
        du = str(self.get_parameter("%s_drive_unit" % source).value).lower()
        wt = str(self.get_parameter("%s_wheel_type" % source).value).lower()
        cv = str(self.get_parameter("%s_cmd_vel_topic" % source).value).strip()
        if du not in ("stage", "mps") or wt not in ("int", "float", "norm"):
            raise ValueError("invalid source unit/type for %s" % source)
        self.src_cfg[source] = {"drive_unit": du, "wheel_type": wt}
        if cv:
            if self.wheelbase <= 0.0:
                raise ValueError(
                    "Twist conversion requires confirmed wheelbase_m")
            self.create_subscription(Twist, cv, partial(self._cb_cmdvel, source), 10)
            self._sub_specs.append((cv, "geometry_msgs/msg/Twist", source + " Twist"))
            return
        drive_topic = self.get_parameter("%s_drive_topic" % source).value
        wheel_topic = self.get_parameter("%s_wheel_topic" % source).value
        self._sub(Float32, drive_topic, partial(self._cb_drive, source), source + " drive")
        wheel_msg = Int32 if wt == "int" else Float32
        callback = self._cb_wheel if wt == "int" else self._cb_wheel_float
        self._sub(wheel_msg, wheel_topic, partial(callback, source), source + " wheel")

    def _mps_to_stage(self, value):
        if abs(value) < self.mps_deadband:
            return 0.0
        if value < 0.0:
            return -1.0
        return float(min(STAGE_MPS, key=lambda s: abs(abs(value) - STAGE_MPS[s])))

    def _cb_drive(self, source, msg):
        value = float(msg.data)
        if self.src_cfg[source]["drive_unit"] == "mps":
            value = self._mps_to_stage(value)
        valid, reason = self.safety.validate_drive(value)
        self.inputs.update_drive(source, value, time.monotonic(), valid, reason)

    def _cb_wheel(self, source, msg):
        self._apply_wheel(source, int(msg.data))

    def _cb_wheel_float(self, source, msg):
        value = float(msg.data)
        if self.src_cfg[source]["wheel_type"] == "norm":
            value *= self.max_steer_deg
        self._apply_wheel(source, int(round(value)))

    def _apply_wheel(self, source, value):
        valid, reason = self.safety.validate_wheel(value)
        self.inputs.update_wheel(source, value, time.monotonic(), valid, reason)

    def _cb_cmdvel(self, source, msg):
        velocity = float(msg.linear.x)
        stage = self._mps_to_stage(velocity)
        valid, reason = self.safety.validate_drive(stage)
        self.inputs.update_drive(source, stage, time.monotonic(), valid, reason)
        wheel = 0 if abs(velocity) < self.mps_deadband else int(round(
            -math.degrees(math.atan(self.wheelbase * float(msg.angular.z) / velocity))))
        self._apply_wheel(source, max(-27, min(27, wheel)))

    def _cb_mode(self, msg):
        if not self.policy.set_mode(msg.data, time.monotonic()):
            self.get_logger().error(
                "invalid mode %r; production contract is String '1'..'11'" % msg.data)

    def _cb_signal(self, name, msg):
        self.policy.update_signal(name, msg.data, time.monotonic())

    def _cb_lidar_status(self, msg):
        valid = False
        try:
            status = json.loads(msg.data)
            state = str(status.get("state", "")).upper()
            valid = (state not in ("", "WAIT_SCAN", "INVALID_SCAN", "LIDAR_TIMEOUT")
                     and not status.get("publisher_conflicts", []))
        except (TypeError, ValueError):
            pass
        self.policy.update_signal("lidar_status_valid", valid, time.monotonic())

    def _cb_manual_stop(self, msg):
        self.inputs.update_stop("manual", bool(msg.data), time.monotonic())

    def _cb_estop(self, msg):
        self.estop_asserted = bool(msg.data)

    def _publisher_names(self, topic):
        result = []
        for info in self.get_publishers_info_by_topic(topic):
            name = getattr(info, "node_name", "?")
            if name != self.get_name():
                ns = (getattr(info, "node_namespace", "") or "").rstrip("/")
                result.append((ns + "/" + name) if ns else "/" + name)
        return sorted(set(result))

    def _diagnose(self):
        check_subscriptions(self, self._sub_specs, self._diag_seen)
        conflicts = []
        for topic in (self.output_drive_topic, self.output_wheel_topic):
            conflicts.extend("%s:%s" % (topic, name)
                             for name in self._publisher_names(topic))
        self._publisher_conflicts = sorted(set(conflicts))
        if conflicts:
            self.get_logger().error(
                "duplicate final publisher; forcing stop: %s" % conflicts)

    def _tick(self):
        now = time.monotonic()
        if self.estop_asserted:
            self._publish(0.0, 0, True, "stop", "stop", "ESTOP", False)
            return
        if self.inputs.stop_asserted("manual", now, self.stop_timeout):
            self._publish(0.0, 0, True, "manual_stop", "stop",
                          "MANUAL_EMERGENCY_STOP", False)
            return
        if self._publisher_conflicts:
            self._publish(0.0, 0, True, "stop", "stop",
                          "DUPLICATE_FINAL_PUBLISHER", False)
            return
        if self.manual_override:
            drive_ok, _, drive = self.inputs.drive_status(
                self.manual_name, now, self.drive_timeout)
            wheel_ok, _, wheel = self.inputs.wheel_status(
                self.manual_name, now, self.wheel_timeout)
            if drive_ok and wheel_ok:
                self._publish(drive, wheel, False, "manual", "manual",
                              "MANUAL_OVERRIDE", True)
                return
        result = self.policy.evaluate(now)
        self._publish(result.drive, result.wheel, result.hard_stop,
                      result.drive_source, result.wheel_source,
                      result.safety_state, result.ready)

    def _publish(self, drive, wheel, stop, drive_source, wheel_source, state, ready):
        wheel = max(-27, min(27, int(wheel)))
        self.pub_drive.publish(Float32(data=float(drive)))
        self.pub_wheel.publish(Int32(data=wheel))
        self.pub_stop.publish(Bool(data=bool(stop)))
        mode = self.policy.mode or "INVALID"
        self.pub_mode.publish(String(data=mode))
        self.pub_dsrc.publish(String(data=drive_source))
        self.pub_wsrc.publish(String(data=wheel_source))
        self.pub_safety.publish(String(data=state))
        self.pub_ready.publish(Bool(data=bool(ready)))
        status = ArbitrationStatus(mode, drive_source, wheel_source, state, bool(ready))
        if status != self.last_status:
            self.get_logger().info("mode=%s drive=%s wheel=%s safety=%s" %
                                   (mode, drive_source, wheel_source, state))
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
