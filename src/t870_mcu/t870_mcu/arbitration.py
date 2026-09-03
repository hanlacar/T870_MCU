"""명령 중재 순수 로직 (ROS 의존 없음).

v3 구조
-------
구동(drive) = 고정 우선순위      라이다 > 카메라 > GPS
조향(wheel) = 모드 게이트        주차 모드에서만 라이다, 그 외 카메라
급정거(stop) = 별도 채널          모드·우선순위 무시, 즉시 정지

구동과 조향은 완전히 독립적으로 결정된다.
조향 소스가 값을 안 줘도 구동은 절대 막히지 않는다.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


CENTER_SOURCE = "center"     # 조향 0도 고정
NO_SOURCE = "none"           # 소스 없음
FAILSAFE = "failsafe"        # 권한 소스가 값을 안 줌


# ============================================================
# 상태 컨테이너
# ============================================================

@dataclass
class ChannelState:
    """한 채널(구동/조향/정지)의 최신값과 유효성."""

    value: float = 0.0
    received_at: Optional[float] = None
    valid: bool = False
    reason: str = "never_received"

    def update(self, value: float, now: float, valid: bool, reason: str = "ok") -> None:
        self.value = value
        self.received_at = now
        self.valid = valid
        self.reason = reason

    def is_fresh(self, now: float, timeout_s: float) -> bool:
        if self.received_at is None:
            return False
        return (now - self.received_at) <= timeout_s


@dataclass
class SourceState:
    """소스 하나의 구동/조향/정지 상태. 서로 독립."""

    drive: ChannelState = field(default_factory=ChannelState)
    wheel: ChannelState = field(default_factory=ChannelState)
    stop: ChannelState = field(default_factory=ChannelState)


@dataclass
class ArbitrationStatus:
    mode: str = "IDLE"
    drive_source: str = "stop"
    wheel_source: str = "stop"
    safety_state: str = "IDLE"
    ready: bool = False


# ============================================================
# 입력 관리
# ============================================================

class InputManager:
    """소스별 최신 구동/조향/정지 값을 보관한다."""

    def __init__(self, source_names: Iterable[str]):
        names = [str(n).strip() for n in source_names if str(n).strip()]
        if not names:
            raise ValueError("at least one command source is required")
        if len(set(names)) != len(names):
            raise ValueError("duplicate command source name")
        self.states: Dict[str, SourceState] = {n: SourceState() for n in names}

        #  🔴 0831 — 게이트로 막힌 소스
        #    구동·조향 둘 다 "값이 없는 것" 으로 취급한다.
        #    급정거(stop)는 절대 막지 않는다. 안전은 게이트보다 위다.
        self.blocked: Dict[str, str] = {}

    # ---- 갱신 ----

    def update_drive(self, source, value, now, valid, reason) -> None:
        self.states[source].drive.update(float(value), now, valid, reason)

    def update_wheel(self, source, value, now, valid, reason) -> None:
        self.states[source].wheel.update(float(value), now, valid, reason)

    def update_stop(self, source, value, now) -> None:
        self.states[source].stop.update(1.0 if value else 0.0, now, True, "ok")

    # ---- 조회 ----

    def invalidate_all(self, reason: str = "mode_changed") -> None:
        """모든 소스의 구동·조향 명령을 즉시 무효화한다.

        ★ 왜 필요한가 (0901)
          모드가 7(T주차)에서 8(정지선까지)로 넘어가는 순간, 라이다가 마지막에
          보낸 조향값은 타임아웃(0.5초) 전까지 여전히 "신선한" 값이다.
          8번의 권한자는 카메라인데, 카메라가 아직 첫 명령을 안 보냈다면
          그 0.5초 동안 **직전 구간의 명령이 그대로 살아있다.**
          주차 막판의 큰 조향각이 다음 구간 출발에 그대로 먹히면 위험하다.

          급정거(stop)는 건드리지 않는다. 안전 신호는 모드와 무관하게 유효하다.
        """
        for state in self.states.values():
            state.drive.valid = False
            state.drive.reason = reason
            state.wheel.valid = False
            state.wheel.reason = reason

    def set_blocked(self, blocked: Dict[str, str]) -> None:
        """{소스: 사유} 로 막는다. 매 주기 통째로 갈아끼운다."""
        self.blocked = dict(blocked or {})

    def drive_status(self, source, now, timeout_s) -> Tuple[bool, str, float]:
        if source in self.blocked:
            return False, self.blocked[source], self.states[source].drive.value
        st = self.states[source].drive
        if not st.valid:
            return False, st.reason, st.value
        if not st.is_fresh(now, timeout_s):
            return False, "timeout", st.value
        return True, "ok", st.value

    def wheel_status(self, source, now, timeout_s) -> Tuple[bool, str, int]:
        if source in self.blocked:
            return (False, self.blocked[source],
                    int(round(self.states[source].wheel.value)))
        st = self.states[source].wheel
        if not st.valid:
            return False, st.reason, int(round(st.value))
        if not st.is_fresh(now, timeout_s):
            return False, "timeout", int(round(st.value))
        return True, "ok", int(round(st.value))

    def stop_asserted(self, source, now, timeout_s) -> bool:
        """정지 요청이 살아있는가.

        '최근에 true 로 온 것'만 유효하다. 오래된 true 를 계속 믿으면
        장애물이 치워져도 차가 영영 못 움직인다.
        """
        #  ★ 게이트로 막힌 소스도 급정거는 받는다. 안전이 우선이다.
        st = self.states[source].stop
        if st.value < 0.5:
            return False
        return st.is_fresh(now, timeout_s)

    def any_stop(self, sources, now, timeout_s) -> List[str]:
        """정지를 요청 중인 소스 목록."""
        return [s for s in sources if self.stop_asserted(s, now, timeout_s)]


# ============================================================
# 구동 — 고정 우선순위
# ============================================================

class PrioritySelector:
    """우선순위 목록에서 '살아있는 것 중 가장 높은' 소스를 고른다.

    priority = ["lidar", "camera", "gps"]
      라이다가 값을 주면 라이다가 이김.
      라이다가 조용하면 카메라, 카메라도 없으면 GPS.
      전부 없으면 (None, 0.0, 사유목록).
    """

    def __init__(self, priority: Iterable[str], known_sources: Iterable[str]):
        self.priority = [str(p).strip().lower() for p in priority if str(p).strip()]
        known = set(known_sources)
        unknown = [p for p in self.priority if p not in known]
        if unknown:
            raise ValueError("drive_priority 에 알 수 없는 소스: %s" % unknown)
        if not self.priority:
            raise ValueError("drive_priority 가 비어 있음")

    def select(self, inputs: "InputManager", now: float,
               timeout_s: float) -> Tuple[Optional[str], float, List[str]]:
        tried = []
        for src in self.priority:
            ok, reason, value = inputs.drive_status(src, now, timeout_s)
            if ok:
                return src, value, tried
            tried.append("%s:%s" % (src, reason))
        return None, 0.0, tried


# ============================================================
# 조향 — 모드 게이트
# ============================================================

class WheelGate:
    """모드에 따라 조향 권한을 가진 소스를 정한다.

    default_owner = "camera"
    overrides = ["T_PARK:lidar", "PARALLEL_PARK:lidar"]

    → 주차 모드에서만 라이다가 조향, 그 외에는 카메라.
      권한 없는 소스가 조향을 발행해도 무시된다.
    """

    def __init__(self, default_owner: str, override_entries: Iterable[str],
                 known_sources: Iterable[str], known_modes: Iterable[str] = (),
                 fallback_chain: Iterable[str] = ()):
        known = set(known_sources) | {CENTER_SOURCE, NO_SOURCE}
        self.known_sources = known          # 런타임 권한 변경 검증에 쓴다

        #  ★ 0902 — 조향 폴백 사슬.
        #
        #    지금까지 조향은 "모드가 정한 주인 한 명" 뿐이었다. 그 주인이
        #    값을 안 주면 곧바로 중앙 고정(직진)이었다. 구동은
        #    lidar→camera→gps 로 내려가는데 조향은 안 내려갔다.
        #
        #    그래서 카메라 노드를 안 띄우고 GPS 만 꽂아 시험하면
        #      · 구동은 GPS 로 폴백해서 나간다
        #      · 조향은 카메라가 주인인데 침묵 → 중앙 고정
        #    즉 "차가 직진만 한다 = 사실상 못 움직인다" 가 됐다.
        #    실제로 현장 시험이 이것 때문에 막혔다.
        #
        #    이제 주인이 침묵하면 이 순서대로 다음 소스를 찾는다.
        #    끝까지 아무도 없을 때만 중앙으로 간다.
        self.fallback_chain: List[str] = []
        for entry in fallback_chain:
            name = str(entry).strip().lower()
            if not name:
                continue
            if name not in known:
                raise ValueError("wheel_fallback_chain 에 알 수 없는 소스: %s" % name)
            if name not in self.fallback_chain:
                self.fallback_chain.append(name)
        self.default_owner = str(default_owner).strip().lower()
        if self.default_owner not in known:
            raise ValueError("wheel_owner_default 가 알 수 없는 소스: %s"
                             % self.default_owner)

        self.overrides: Dict[str, str] = {}
        for entry in override_entries:
            text = str(entry).strip()
            if not text:
                continue
            if ":" not in text:
                raise ValueError("wheel_owner_overrides 형식은 MODE:SOURCE : %s" % text)
            mode, owner = text.split(":", 1)
            owner = owner.strip().lower()
            if owner not in known:
                raise ValueError("wheel_owner_overrides 에 알 수 없는 소스: %s" % owner)
            self.overrides[mode.strip().upper()] = owner

        # ---- 모드 문자열 화이트리스트 ----
        #
        # ★ 이게 없으면 오타가 조용히 넘어간다.
        #   "T_PARK" 를 "TPARK" 로 발행하면 override 에 안 걸려 기본 소유자
        #   (카메라)가 조향을 가져간다. 에러도 경고도 없다. 주차 구간에서
        #   이게 나면 주차 자체가 실패하는데 원인을 찾기가 매우 어렵다.
        #
        # known_modes 가 비어 있으면 검증하지 않는다 (기존 동작).
        self.known_modes = set(
            str(m).strip().upper() for m in known_modes if str(m).strip())

        unknown_override = [m for m in self.overrides
                            if self.known_modes and m not in self.known_modes]
        if unknown_override:
            raise ValueError(
                "wheel_owner_overrides 의 모드가 known_modes 에 없다: %s"
                % sorted(unknown_override))

    def set_owner(self, mode: str, source: str) -> str:
        """모드 하나의 조향 권한자를 **런타임에** 바꾼다.

        ★ 왜 필요한가 (0902)
          9/6 현장 계획에 "카메라 or GPS + LIDAR" 처럼 아직 안 정해진 구간이
          여럿 있다. 둘 다 돌려보고 정하는 건데, 그때마다 yaml 고치고
          colcon build 하고 노드를 다시 띄우면 한 번에 몇 분씩 날아간다.
          현장에서는 그 시간이 없다.

        source 를 빈 문자열이나 "default" 로 주면 기본 권한자로 되돌린다.
        반환값은 사람이 읽을 결과 문자열.
        """
        mode = str(mode).strip().upper()
        source = str(source).strip().lower()

        if source in ("", "default", "기본"):
            if mode in self.overrides:
                del self.overrides[mode]
            return "모드 %s → 기본 권한자(%s)" % (mode, self.default_owner)

        if source not in self.known_sources:
            return ("거부: '%s' 는 없는 소스다. 가능: %s"
                    % (source, ", ".join(sorted(self.known_sources))))

        if self.known_modes and mode not in self.known_modes:
            return "거부: '%s' 는 없는 모드다 (0~11)" % mode

        self.overrides[mode] = source
        return "모드 %s → %s" % (mode, source)

    def owner_table(self) -> str:
        """지금 권한표를 한 줄로. 상태 토픽으로 내보내 눈으로 확인한다."""
        parts = []
        for mode in sorted(self.known_modes, key=lambda m: (len(m), m)):
            parts.append("%s:%s" % (mode, self.overrides.get(mode, self.default_owner)))
        return " ".join(parts)

    def is_known(self, mode: str) -> bool:
        """known_modes 에 있는 모드인가. 목록이 비어 있으면 항상 True."""
        if not self.known_modes:
            return True
        return str(mode).strip().upper() in self.known_modes

    def owner(self, mode: str, gate_owner: Optional[str] = None) -> str:
        """이 모드의 조향 권한자.

        gate_owner 가 주어지면 그것이 최우선이다 (회피 게이트가 열린 순간).
        게이트는 런타임 신호라 모드별 정적 표로는 표현할 수 없다.
        """
        if gate_owner:
            return str(gate_owner).strip().lower()
        return self.overrides.get(str(mode).strip().upper(), self.default_owner)

    def resolve(self, mode: str, inputs: "InputManager", now: float,
                timeout_s: float, failsafe_value: int,
                gate_owner: Optional[str] = None) -> Tuple[int, str, bool, str]:
        """(조향각, 사용된소스, 권한소스정상여부, 사유) 반환.

        권한 소스가 값을 안 주면 failsafe 값으로 폴백하되 '폴백 중'임을
        별도로 알린다. 구동은 이와 무관하게 계속된다.
        """
        own = self.owner(mode, gate_owner)
        if own in (CENTER_SOURCE, NO_SOURCE):
            return 0, CENTER_SOURCE, True, "no_wheel_source_by_design"

        ok, reason, value = inputs.wheel_status(own, now, timeout_s)
        if ok:
            return int(value), own, True, "ok"

        #  주인이 침묵 → 폴백 사슬을 순서대로 훑는다.
        #  ok=False 를 유지해서 "폴백 중" 이라는 사실은 계속 알린다.
        tried = ["%s:%s" % (own, reason)]
        for alt in self.fallback_chain:
            if alt == own or alt in (CENTER_SOURCE, NO_SOURCE):
                continue
            alt_ok, alt_reason, alt_value = inputs.wheel_status(alt, now, timeout_s)
            if alt_ok:
                return (int(alt_value), alt, False,
                        "fallback:%s <- %s" % (alt, ",".join(tried)))
            tried.append("%s:%s" % (alt, alt_reason))

        return failsafe_value, FAILSAFE, False, ",".join(tried)


# ============================================================
# 값 검증
# ============================================================

class SafetyManager:
    """명령값 검증. 범위를 벗어나면 클램프하지 않고 거부한다."""

    def __init__(self, drive_validation_mode, drive_allowed_values,
                 drive_min, drive_max, wheel_min, wheel_max,
                 drive_value_tolerance=1e-6):
        mode = str(drive_validation_mode).strip().lower()
        if mode not in ("allowed_values", "range"):
            raise ValueError("drive_validation_mode must be allowed_values or range")
        if drive_min > drive_max:
            raise ValueError("drive_min must be <= drive_max")
        if wheel_min > wheel_max:
            raise ValueError("wheel_min must be <= wheel_max")

        self.drive_validation_mode = mode
        self.drive_allowed_values = [float(v) for v in drive_allowed_values]
        self.drive_min = float(drive_min)
        self.drive_max = float(drive_max)
        self.wheel_min = int(wheel_min)
        self.wheel_max = int(wheel_max)
        self.drive_value_tolerance = float(drive_value_tolerance)

        if mode == "allowed_values" and not self.drive_allowed_values:
            raise ValueError("drive_allowed_values cannot be empty")

    def validate_drive(self, value) -> Tuple[bool, str]:
        value = float(value)
        if not math.isfinite(value):
            return False, "non_finite"
        if self.drive_validation_mode == "range":
            if value < self.drive_min or value > self.drive_max:
                return False, "out_of_range"
            return True, "ok"
        for allowed in self.drive_allowed_values:
            if abs(value - allowed) <= self.drive_value_tolerance:
                return True, "ok"
        return False, "not_allowed"

    def validate_wheel(self, value) -> Tuple[bool, str]:
        if isinstance(value, bool):
            return False, "invalid_type"
        try:
            numeric = int(value)
        except (TypeError, ValueError, OverflowError):
            return False, "invalid_type"
        if numeric < self.wheel_min or numeric > self.wheel_max:
            return False, "out_of_range"
        return True, "ok"
