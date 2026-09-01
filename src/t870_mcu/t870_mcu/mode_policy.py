"""Mode-specific MCU arbitration policy with no ROS dependency.

Perception stays in the camera/LiDAR/GPS nodes.  This module only checks
freshness and selects the command owner for the current route mode.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from .arbitration import InputManager


VALID_MODES = frozenset(str(value) for value in range(1, 12))
GENERAL_CAMERA_MODES = frozenset(("1", "3", "8", "11"))
INTERSECTION_MODES = frozenset(("4", "6"))
PARKING_MODES = frozenset(("7", "10"))


@dataclass
class TimedSignal:
    value: Any = None
    received_at: Optional[float] = None

    def update(self, value: Any, now: float) -> None:
        self.value = value
        self.received_at = float(now)

    def fresh(self, now: float, timeout_s: float) -> bool:
        return (self.received_at is not None and
                0.0 <= float(now) - self.received_at <= timeout_s)


@dataclass(frozen=True)
class ModeDecision:
    drive: float = 0.0
    wheel: int = 0
    drive_source: str = "stop"
    wheel_source: str = "stop"
    safety_state: str = "SAFE_STOP"
    ready: bool = False
    hard_stop: bool = False


@dataclass
class ModeArbitrator:
    inputs: InputManager
    drive_timeout_s: float = 0.5
    wheel_timeout_s: float = 0.5
    signal_timeout_s: float = 0.5
    mode_timeout_s: float = 1.0
    max_wheel_deg: int = 27
    mode: Optional[str] = None
    mode_received_at: Optional[float] = None
    signals: Dict[str, TimedSignal] = field(default_factory=dict)

    SIGNAL_NAMES = (
        "avoidance_active", "camera_authority", "camera_source",
        "front_obstacle", "gps_path_safe", "path_safety_status",
        "lidar_stop", "lidar_status_valid", "parking_active",
    )

    def __post_init__(self) -> None:
        self.signals = {name: TimedSignal() for name in self.SIGNAL_NAMES}

    def update_signal(self, name: str, value: Any, now: float) -> None:
        if name not in self.signals:
            raise KeyError(name)
        self.signals[name].update(value, now)

    def set_mode(self, value: Any, now: float) -> bool:
        """Accept only production route numbers and reset old-mode state."""
        new_mode = str(value).strip()
        valid = new_mode in VALID_MODES
        if not valid:
            self.mode = None
            self.mode_received_at = float(now)
            self._reset_transition_state()
            return False
        if new_mode != self.mode:
            self._reset_transition_state()
            # Commands received for the previous mode must not cross ownership.
            self.inputs.invalidate_commands("mode_changed")
        self.mode = new_mode
        self.mode_received_at = float(now)
        return True

    def _reset_transition_state(self) -> None:
        for signal in self.signals.values():
            signal.value = None
            signal.received_at = None

    def _signal(self, name: str, now: float) -> Tuple[bool, Any]:
        signal = self.signals[name]
        return signal.fresh(now, self.signal_timeout_s), signal.value

    def _drive(self, source: str, now: float):
        return self.inputs.drive_status(source, now, self.drive_timeout_s)

    def _wheel(self, source: str, now: float):
        return self.inputs.wheel_status(source, now, self.wheel_timeout_s)

    def _pair(self, source: str, now: float, reason: str = "OK") -> ModeDecision:
        drive_ok, drive_reason, drive = self._drive(source, now)
        wheel_ok, wheel_reason, wheel = self._wheel(source, now)
        if not drive_ok or not wheel_ok:
            return self._stop(
                "%s_STALE(%s:%s,%s:%s)" %
                (source.upper(), source, drive_reason, source, wheel_reason))
        return self._result(drive, wheel, source, source, reason)

    def _camera_then_gps(self, now: float, reason: str) -> ModeDecision:
        camera_authority_fresh, camera_authority = self._signal(
            "camera_authority", now)
        if camera_authority_fresh and bool(camera_authority):
            drive_ok, _, _ = self._drive("camera", now)
            wheel_ok, _, _ = self._wheel("camera", now)
            if drive_ok and wheel_ok:
                return self._pair("camera", now, reason)
        gps = self._pair("gps", now, reason + "_GPS_FALLBACK")
        return gps if gps.ready else self._stop(reason + "_NO_FRESH_PATH")

    def _general(self, now: float) -> ModeDecision:
        obstacle_fresh, obstacle = self._signal("front_obstacle", now)
        if not obstacle_fresh:
            return self._stop("FRONT_OBSTACLE_STATUS_STALE")
        if bool(obstacle):
            safe_fresh, safe = self._signal("gps_path_safe", now)
            status_fresh, status = self._signal("path_safety_status", now)
            if not safe_fresh or not status_fresh:
                return self._stop("GPS_DR_PATH_SAFETY_STALE")
            if not bool(safe) or str(status).strip().upper() not in (
                    "SAFE", "GPS_DR_SAFE"):
                return self._stop("GPS_DR_PATH_UNSAFE")
            return self._pair("gps", now, "OBSTACLE_GPS_DR_FALLBACK")
        return self._camera_then_gps(now, "GENERAL_CAMERA")

    def _slope(self, now: float) -> ModeDecision:
        # A fresh zero is a valid camera command, not a missing command.
        camera = self._pair("camera", now, "SLOPE_CAMERA")
        if camera.ready:
            return camera
        gps = self._pair("gps", now, "SLOPE_GPS_FALLBACK")
        return gps if gps.ready else self._stop("SLOPE_NO_FRESH_PATH")

    def _intersection(self, now: float) -> ModeDecision:
        auth_fresh, authority = self._signal("camera_authority", now)
        source_fresh, source = self._signal("camera_source", now)
        if not auth_fresh or not source_fresh:
            return self._stop("INTERSECTION_SIGNAL_STALE")
        source = str(source).strip().upper()
        if bool(authority):
            drive_ok, _, drive = self._drive("camera", now)
            if not drive_ok or float(drive) != 0.0 or source != "MISSION":
                return self._stop("INTERSECTION_STOP_CONTRACT_INVALID")
            wheel_ok, wheel_reason, wheel = self._wheel("gps", now)
            if not wheel_ok:
                return self._stop("INTERSECTION_GPS_WHEEL_%s" % wheel_reason)
            return self._result(0.0, wheel, "camera", "gps",
                                "INTERSECTION_STOP")
        if source != "GPS_DR":
            return self._stop("INTERSECTION_SIGNAL_UNKNOWN")
        return self._pair("gps", now, "INTERSECTION_GO")

    def _avoidance(self, now: float) -> ModeDecision:
        active_fresh, active = self._signal("avoidance_active", now)
        if active_fresh and bool(active):
            return self._pair("lidar", now, "AVOIDANCE_LIDAR")
        # False or stale removes LiDAR ownership immediately.
        return self._camera_then_gps(now, "AVOIDANCE_INACTIVE")

    def _parking(self, now: float) -> ModeDecision:
        active_fresh, active = self._signal("parking_active", now)
        if not active_fresh or not bool(active):
            return self._stop("PARKING_INACTIVE_OR_STALE")
        return self._pair("parking", now, "PARKING")

    def _acceleration(self, now: float) -> ModeDecision:
        stop_fresh, lidar_stop = self._signal("lidar_stop", now)
        status_fresh, status_valid = self._signal("lidar_status_valid", now)
        if not stop_fresh or not status_fresh or not bool(status_valid):
            return self._stop("MODE9_LIDAR_STATUS_STALE", hard_stop=True)
        if bool(lidar_stop):
            return self._stop("MODE9_LIDAR_STOP", hard_stop=True)
        lidar_ok, lidar_reason, lidar_drive = self._drive("lidar", now)
        if not lidar_ok:
            return self._stop("MODE9_LIDAR_DRIVE_%s" % lidar_reason,
                              hard_stop=True)

        camera_ok, _, camera_drive = self._drive("camera", now)
        final_drive = float(lidar_drive)
        drive_source = "lidar"
        if camera_ok:
            # Both are upper bounds.  Preserve the camera mode-9 latch by
            # choosing the safer (lower forward) command every cycle.
            final_drive = min(float(lidar_drive), float(camera_drive))
            drive_source = "lidar+camera_limit"

        wheel_ok, _, wheel = self._wheel("camera", now)
        wheel_source = "camera"
        if not wheel_ok:
            wheel_ok, wheel_reason, wheel = self._wheel("gps", now)
            wheel_source = "gps"
            if not wheel_ok:
                return self._stop("MODE9_NO_WHEEL(%s)" % wheel_reason)
        return self._result(final_drive, wheel, drive_source, wheel_source,
                            "MODE9")

    def evaluate(self, now: float) -> ModeDecision:
        if (self.mode not in VALID_MODES or self.mode_received_at is None or
                float(now) - self.mode_received_at > self.mode_timeout_s):
            return self._stop("MODE_INVALID_OR_STALE")
        if self.mode in GENERAL_CAMERA_MODES:
            return self._general(now)
        if self.mode == "2":
            return self._slope(now)
        if self.mode in INTERSECTION_MODES:
            return self._intersection(now)
        if self.mode == "5":
            return self._avoidance(now)
        if self.mode in PARKING_MODES:
            return self._parking(now)
        if self.mode == "9":
            return self._acceleration(now)
        return self._stop("MODE_POLICY_MISSING")

    def _result(self, drive: float, wheel: int, drive_source: str,
                wheel_source: str, state: str) -> ModeDecision:
        wheel = max(-self.max_wheel_deg, min(self.max_wheel_deg, int(wheel)))
        return ModeDecision(float(drive), wheel, drive_source, wheel_source,
                            state, True, False)

    @staticmethod
    def _stop(state: str, hard_stop: bool = False) -> ModeDecision:
        return ModeDecision(safety_state=state, hard_stop=hard_stop)
