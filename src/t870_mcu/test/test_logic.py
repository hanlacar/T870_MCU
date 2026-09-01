"""t870_mcu 순수 로직 테스트 (ROS 불필요).

실행:
    cd <워크스페이스>/src/t870_mcu && python3 -m pytest test/ -v
"""

import math

import pytest

from t870_mcu.protocol import (
    drive_serial_command,
    parse_drive_stage,
    parse_status,
    valid_wheel_deg,
    wheel_serial_command,
)
from t870_mcu.arbitration import (
    InputManager,
    PrioritySelector,
    SafetyManager,
    WheelGate,
    CENTER_SOURCE,
    FAILSAFE,
)

SOURCES = ["lidar", "camera", "gps", "manual"]


# ============================================================
# 프로토콜
# ============================================================

def test_drive_protocol_mapping():
    assert drive_serial_command(-1) == "6.00"
    assert drive_serial_command(0) == "1.00"
    assert drive_serial_command(1) == "2.00"
    assert drive_serial_command(2) == "3.00"
    assert drive_serial_command(3) == "4.00"


def test_drive_rejects_non_stage_values():
    assert parse_drive_stage(1.0) == 1
    assert parse_drive_stage(1.2) is None
    assert parse_drive_stage(4.0) is None
    assert parse_drive_stage(math.nan) is None
    assert parse_drive_stage(math.inf) is None


def test_wheel_validation_no_clamp():
    assert valid_wheel_deg(-27) and valid_wheel_deg(27)
    assert not valid_wheel_deg(-28) and not valid_wheel_deg(28)


def test_wheel_w_protocol():
    assert wheel_serial_command(0, 27, 440, "W") == "W0"
    assert wheel_serial_command(27, 27, 440, "W") == "W440"
    assert wheel_serial_command(-27, 27, 440, "W") == "W-440"


# ---- STATUS 회귀 테스트 -------------------------------------
# v28 은 fault 필드에 숫자가 아니라 "NONE" 문자열을 넣는다.
# 이전 파서는 여기서 예외가 나 라인 전체를 버렸고, 그 결과
# /drive, /rpm, /arduino/raw_status, /odom 이 전혀 발행되지 않았다.

REAL_LINES = [
    "STATUS,READY,NONE,251,363,0,0.00,0,0,0,440",
    "STATUS,ACTIVE,NONE,296,363,51,13.52,1624,163,163,440",
]


def test_real_vehicle_status_with_text_fault_is_parsed():
    for line in REAL_LINES:
        s = parse_status(line)
        assert s is not None, "실차 STATUS 파싱 실패: %s" % line
        assert s["fault"] == 0 and s["fault_text"] == "NONE"


def test_real_vehicle_status_fields():
    s = parse_status("STATUS,ACTIVE,NONE,296,363,51,13.52,1624,163,163,440")
    assert s["state"] == "ACTIVE"
    assert s["adc"] == 296 and s["rpm"] == 13.52
    assert s["encoder_count"] == 1624 and s["steer_ms"] == 163


def test_text_and_numeric_fault_both_detected():
    assert parse_status("STATUS,FAULT,POT_RANGE,251,363,0,0,0,0,0,440")["fault"] == 1
    assert parse_status("STATUS,READY,0,1,2,3,4.0,5,6,7,8")["fault"] == 0
    assert parse_status("STATUS,FAULT,3,1,2,3,4.0,5,6,7,8")["fault"] == 1


def test_broken_field_does_not_discard_whole_line():
    s = parse_status("STATUS,READY,NONE,xx,363,0,BAD,555,0,0,440")
    assert s is not None and s["encoder_count"] == 555
    assert s["adc"] == 0 and s["rpm"] == 0.0


def test_malformed_lines_return_none():
    for bad in ("STATUS,READY", "garbage line", "", None):
        assert parse_status(bad) is None


# ============================================================
# 구동 — 고정 우선순위
# ============================================================

@pytest.fixture
def inputs():
    return InputManager(SOURCES)


@pytest.fixture
def prio():
    return PrioritySelector(["lidar", "camera", "gps"], SOURCES)


def test_lidar_beats_camera(inputs, prio):
    now = 100.0
    inputs.update_drive("camera", 2.0, now, True, "ok")
    inputs.update_drive("lidar", 1.0, now, True, "ok")
    src, val, _ = prio.select(inputs, now, 0.5)
    assert (src, val) == ("lidar", 1.0)


def test_camera_wins_when_lidar_silent(inputs, prio):
    """라이다는 평소 조용하다. 그때는 카메라가 운전한다."""
    now = 100.0
    inputs.update_drive("camera", 2.0, now, True, "ok")
    src, val, _ = prio.select(inputs, now, 0.5)
    assert (src, val) == ("camera", 2.0)


def test_falls_through_to_gps(inputs, prio):
    now = 100.0
    inputs.update_drive("gps", 1.0, now, True, "ok")
    src, val, _ = prio.select(inputs, now, 0.5)
    assert (src, val) == ("gps", 1.0)


def test_no_source_means_stop(inputs, prio):
    src, val, tried = prio.select(inputs, 100.0, 0.5)
    assert src is None and val == 0.0
    assert len(tried) == 3


def test_stale_high_priority_yields_to_fresh_lower(inputs, prio):
    """라이다가 끊기면 카메라가 이어받는다."""
    inputs.update_drive("lidar", 1.0, 100.0, True, "ok")
    inputs.update_drive("camera", 2.0, 100.9, True, "ok")
    src, val, _ = prio.select(inputs, 100.9, 0.5)
    assert (src, val) == ("camera", 2.0)


def test_invalid_value_disqualifies_source(inputs, prio):
    """허용 안 된 값을 준 소스는 이기지 못한다."""
    now = 100.0
    inputs.update_drive("lidar", 1.5, now, False, "not_allowed")
    inputs.update_drive("camera", 2.0, now, True, "ok")
    src, _, _ = prio.select(inputs, now, 0.5)
    assert src == "camera"


def test_priority_rejects_unknown_source():
    with pytest.raises(ValueError):
        PrioritySelector(["radar"], SOURCES)


# ============================================================
# 조향 — 모드 게이트
# ============================================================

@pytest.fixture
def gate():
    return WheelGate("camera", ["T_PARK:lidar", "PARALLEL_PARK:lidar"], SOURCES)


def test_wheel_owner_by_mode(gate):
    assert gate.owner("NORMAL") == "camera"
    assert gate.owner("t_park") == "lidar"
    assert gate.owner("PARALLEL_PARK") == "lidar"
    assert gate.owner("ANYTHING_ELSE") == "camera"


def test_lidar_wheel_ignored_outside_parking(inputs, gate):
    """★ 일반 주행에서는 라이다 조향이 무시되고 카메라가 쓰인다."""
    now = 100.0
    inputs.update_wheel("lidar", 20, now, True, "ok")
    inputs.update_wheel("camera", -5, now, True, "ok")
    val, used, ok, _ = gate.resolve("NORMAL", inputs, now, 0.5, 0)
    assert (val, used, ok) == (-5, "camera", True)


def test_lidar_wheel_used_in_parking(inputs, gate):
    now = 100.0
    inputs.update_wheel("lidar", 20, now, True, "ok")
    inputs.update_wheel("camera", -5, now, True, "ok")
    val, used, ok, _ = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (val, used, ok) == (20, "lidar", True)


def test_wheel_absent_falls_back_to_center(inputs, gate):
    """★ 조향이 없어도 구동은 별개다. 조향만 0도로 폴백."""
    now = 100.0
    val, used, ok, reason = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (val, used, ok) == (0, FAILSAFE, False)
    assert "never_received" in reason


def test_wheel_timeout_falls_back(inputs, gate):
    inputs.update_wheel("camera", 10, 100.0, True, "ok")
    val, used, ok, reason = gate.resolve("NORMAL", inputs, 100.9, 0.5, 0)
    assert (val, used, ok) == (0, FAILSAFE, False)
    assert "timeout" in reason


def test_wheel_hold_last_failsafe(inputs, gate):
    val, used, ok, _ = gate.resolve("NORMAL", inputs, 100.0, 0.5, 15)
    assert val == 15 and used == FAILSAFE and not ok


def test_center_keyword_is_by_design(inputs):
    g = WheelGate("center", [], SOURCES)
    val, used, ok, reason = g.resolve("NORMAL", inputs, 100.0, 0.5, 0)
    assert (val, used, ok) == (0, CENTER_SOURCE, True)
    assert reason == "no_wheel_source_by_design"


# ============================================================
# 급정거
# ============================================================

def test_stop_is_detected(inputs):
    inputs.update_stop("lidar", True, 100.0)
    assert inputs.stop_asserted("lidar", 100.0, 0.5)
    assert inputs.any_stop(["lidar", "camera"], 100.0, 0.5) == ["lidar"]


def test_stop_releases_when_false(inputs):
    inputs.update_stop("lidar", True, 100.0)
    inputs.update_stop("lidar", False, 100.1)
    assert not inputs.stop_asserted("lidar", 100.1, 0.5)


def test_stale_stop_is_released(inputs):
    """★ 오래된 true 를 계속 믿으면 장애물이 치워져도 영영 못 움직인다."""
    inputs.update_stop("lidar", True, 100.0)
    assert not inputs.stop_asserted("lidar", 100.9, 0.5)


def test_any_stop_lists_all_asserting_sources(inputs):
    inputs.update_stop("lidar", True, 100.0)
    inputs.update_stop("camera", True, 100.0)
    assert set(inputs.any_stop(["lidar", "camera"], 100.0, 0.5)) == {"lidar", "camera"}


# ============================================================
# 값 검증
# ============================================================

@pytest.fixture
def safety():
    return SafetyManager("allowed_values", [-1.0, 0.0, 1.0, 2.0, 3.0],
                         -1.0, 3.0, -27, 27)


def test_drive_allowed_values(safety):
    assert safety.validate_drive(2.0) == (True, "ok")
    assert not safety.validate_drive(1.5)[0]
    assert not safety.validate_drive(4.0)[0]


def test_non_finite_drive_is_rejected(safety):
    assert safety.validate_drive(math.nan) == (False, "non_finite")
    assert safety.validate_drive(math.inf) == (False, "non_finite")


def test_wheel_range_is_rejected_not_clamped(safety):
    assert safety.validate_wheel(27) == (True, "ok")
    assert safety.validate_wheel(28) == (False, "out_of_range")
    assert safety.validate_wheel(-28) == (False, "out_of_range")


# ============================================================
# 통합 시나리오 — 실제 미션 상황
# ============================================================

def test_scenario_s_course(inputs, prio, gate):
    """S코스: 카메라가 회피 주행, 라이다는 감시만."""
    now = 100.0
    inputs.update_drive("camera", 1.0, now, True, "ok")
    inputs.update_wheel("camera", 15, now, True, "ok")
    inputs.update_wheel("lidar", -20, now, True, "ok")   # 무시돼야 함

    src, dval, _ = prio.select(inputs, now, 0.5)
    wval, wused, _, _ = gate.resolve("NORMAL", inputs, now, 0.5, 0)
    assert (src, dval) == ("camera", 1.0)
    assert (wval, wused) == (15, "camera")
    assert not inputs.any_stop(["lidar", "camera"], now, 0.5)


def test_scenario_sudden_obstacle(inputs):
    """돌발: 라이다 급정거가 모든 것을 이긴다."""
    now = 100.0
    inputs.update_drive("camera", 3.0, now, True, "ok")
    inputs.update_stop("lidar", True, now)
    assert inputs.any_stop(["lidar", "camera"], now, 0.5) == ["lidar"]


def test_scenario_t_parking(inputs, prio, gate):
    """T자 주차: 라이다가 구동과 조향을 모두 가져간다."""
    now = 100.0
    inputs.update_drive("camera", 2.0, now, True, "ok")
    inputs.update_drive("lidar", -1.0, now, True, "ok")
    inputs.update_wheel("lidar", 27, now, True, "ok")

    src, dval, _ = prio.select(inputs, now, 0.5)
    wval, wused, _, _ = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (src, dval) == ("lidar", -1.0)
    assert (wval, wused) == (27, "lidar")


def test_scenario_lidar_drive_without_wheel(inputs, prio, gate):
    """★ 라이다가 구동만 주고 조향을 안 줘도 주행은 계속된다."""
    now = 100.0
    inputs.update_drive("lidar", 2.0, now, True, "ok")
    src, dval, _ = prio.select(inputs, now, 0.5)
    wval, wused, wok, _ = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (src, dval) == ("lidar", 2.0)      # 구동 살아있음
    assert (wval, wok) == (0, False)          # 조향만 폴백


# ==========================================================
# 구독 진단 (0829) — 타입·QoS 불일치를 잡아내는가
#
#  이 두 가지는 ROS 가 에러를 안 낸다. 메시지만 조용히 안 온다.
#  "저쪽은 쏘는데 나는 못 받는" 상황의 대부분이 이것이라 회귀 시험을 둔다.
# ==========================================================
from t870_mcu.diagnostics import check_subscriptions


class _Rel:
    def __init__(self, name):
        self.name = name


class _Qos:
    def __init__(self, name):
        self.reliability = _Rel(name)


class _Info:
    def __init__(self, node, typ, rel="RELIABLE"):
        self.node_namespace = "/"
        self.node_name = node
        self.topic_type = typ
        self.qos_profile = _Qos(rel)


class _Log:
    def __init__(self):
        self.msgs = []

    def error(self, m):
        self.msgs.append(m)


class _Node:
    def __init__(self, table):
        self.table = table
        self._log = _Log()

    def get_logger(self):
        return self._log

    def get_publishers_info_by_topic(self, topic):
        return self.table.get(topic, [])


def test_진단_정상이면_조용하다():
    n = _Node({"/a": [_Info("x", "std_msgs/msg/Float32")]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Float32", "구동")], set())
    assert n._log.msgs == []


def test_진단_발행자없으면_조용하다():
    # 팀 노드가 늦게 뜨는 것은 정상이다. 이걸 에러로 찍으면 소음이 된다.
    n = _Node({})
    check_subscriptions(n, [("/a", "std_msgs/msg/Float32", "구동")], set())
    assert n._log.msgs == []


def test_진단_타입불일치를_잡는다():
    n = _Node({"/a": [_Info("lidar", "std_msgs/msg/Float32")]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Int32", "조향")], set())
    assert len(n._log.msgs) == 1
    assert "타입이 다르다" in n._log.msgs[0]


def test_진단_QoS불일치를_잡는다():
    n = _Node({"/a": [_Info("cam", "std_msgs/msg/Bool", "BEST_EFFORT")]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Bool", "급정거")], set())
    assert len(n._log.msgs) == 1
    assert "QoS" in n._log.msgs[0]


def test_진단_같은문제를_두번찍지않는다():
    n = _Node({"/a": [_Info("cam", "std_msgs/msg/Bool", "BEST_EFFORT")]})
    seen = set()
    spec = [("/a", "std_msgs/msg/Bool", "급정거")]
    check_subscriptions(n, spec, seen)
    check_subscriptions(n, spec, seen)
    check_subscriptions(n, spec, seen)
    assert len(n._log.msgs) == 1


def test_진단_reliability가_정수여도_동작한다():
    # rclpy 구현에 따라 enum 이 아니라 정수로 오는 경우가 있다
    class _IntQos:
        reliability = 2          # 2 = BEST_EFFORT

    info = _Info("cam", "std_msgs/msg/Bool")
    info.qos_profile = _IntQos()
    n = _Node({"/a": [info]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Bool", "급정거")], set())
    assert len(n._log.msgs) == 1


# ==========================================================
# 엔코더 누적값 검증 (0829)
#
#  증상: 키보드로 전진하다 멈추면 /mcu/encoder 가 0 으로 초기화됐다.
#  원인: STATUS 필드가 시리얼 노이즈로 깨지면 파서가 기본값 0 을 줬다.
#        제동은 역토크 펄스라 노이즈가 가장 큰 순간이고, 그래서
#        하필 "멈출 때마다" 초기화되는 것처럼 보였다.
# ==========================================================
from t870_mcu.protocol import encoder_sanity, parse_status

GOOD = "STATUS,ACTIVE,NONE,251,363,100,12.34,457,120,120,440,7,14,1,0"


def test_정상_STATUS_는_엔코더를_읽는다():
    assert parse_status(GOOD)["encoder_count"] == 457


def test_엔코더필드가_깨지면_0이_아니라_None():
    #  예전에는 여기서 0 이 나왔다. 그게 "초기화" 증상의 원인이었다.
    broken = "STATUS,ACTIVE,NONE,251,363,100,12.34,4x5,120,120,440"
    assert parse_status(broken)["encoder_count"] is None


def test_엔코더필드가_비어도_None():
    empty = "STATUS,ACTIVE,NONE,251,363,100,12.34,,120,120,440"
    assert parse_status(empty)["encoder_count"] is None


def test_sanity_None은_거부():
    ok, why = encoder_sanity(None, 100, 0.2, 2000.0)
    assert not ok and "읽을 수 없다" in why


def test_sanity_첫값은_무조건_받는다():
    ok, _ = encoder_sanity(12345, None, 0.2, 2000.0)
    assert ok


def test_sanity_정상_주행은_통과():
    #  0.5m/s, 199.8 counts/m → 0.2초에 약 20카운트
    ok, _ = encoder_sanity(120, 100, 0.2, 2000.0)
    assert ok


def test_sanity_최고속의_두배도_통과():
    #  실수로 정상 주행을 걸러내면 안 된다. 0.2초에 320카운트(=8m/s) 도 통과
    ok, _ = encoder_sanity(420, 100, 0.2, 2000.0)
    assert ok


def test_sanity_말도안되는_점프는_거부():
    #  필드가 밀려 steer_limit_ms(440) 같은 엉뚱한 값이 들어온 경우
    ok, why = encoder_sanity(50000, 100, 0.2, 2000.0)
    assert not ok and "튀었다" in why


def test_sanity_0으로_떨어지는_것도_거부():
    #  바로 이 증상. 누적 5000 에서 갑자기 0 은 물리적으로 불가능하다.
    ok, why = encoder_sanity(0, 5000, 0.1, 2000.0)
    assert not ok


def test_sanity_후진_부호반전은_통과():
    #  전진 누적 300 → 후진으로 바뀌며 조금 줄어드는 것은 정상
    ok, _ = encoder_sanity(280, 300, 0.2, 2000.0)
    assert ok


def test_sanity_한계를_0으로_두면_검사를_끈다():
    ok, _ = encoder_sanity(999999, 100, 0.1, 0.0)
    assert ok


def test_sanity_dt가_0이어도_안터진다():
    ok, _ = encoder_sanity(150, 100, 0.0, 2000.0)
    assert ok


# ============================================================
# 0831 — 번호 모드 + 조향 권한
#
#   0830 에 라이다로 S자 코스를 연습했는데 차가 직진만 했다.
#   원인은 그 구간의 조향 권한자가 카메라였던 것. 아래는 그 규칙이
#   번호 방식에서도 의도대로 도는지 고정해 두는 시험이다.
# ============================================================

from t870_mcu.arbitration import WheelGate

NUM_MODES = [str(i) for i in range(0, 12)]      # 0 대기 + 1~11 구간
NUM_OVERRIDES = ["5:lidar", "7:lidar", "10:lidar"]
SOURCES = ["lidar", "camera", "gps", "manual"]


def _gate():
    return WheelGate("camera", NUM_OVERRIDES, SOURCES, NUM_MODES)


def test_번호_S코스는_라이다가_조향한다():
    #  5 = S_COURSE. 이게 camera 로 나오면 라이다 우회가 통째로 무시된다.
    assert _gate().owner("5") == "lidar"


def test_번호_주차_두_구간도_라이다():
    g = _gate()
    assert g.owner("7") == "lidar"       # T 주차
    assert g.owner("10") == "lidar"      # 평행 주차


def test_번호_나머지_구간은_카메라():
    g = _gate()
    for m in ("0", "1", "2", "3", "4", "6", "8", "9", "11"):
        assert g.owner(m) == "camera", m


def test_교차로_두_구간이_서로_다른_모드다():
    #  번호 방식의 핵심. 이름 방식에서는 4번과 6번이 구분되지 않아
    #  카메라가 어느 교차로인지 알 수 없었다.
    g = _gate()
    assert g.is_known("4") and g.is_known("6")
    assert "4" != "6"


def test_모르는_번호는_거부된다():
    #  경로가 12구간이므로 13 은 없다. 오타가 조용히 넘어가면 안 된다.
    g = _gate()
    assert not g.is_known("12")
    assert not g.is_known("S_COURSE")     # 별칭 치환 전에는 모르는 값


def test_override_가_known_modes에_없으면_터진다():
    #  yaml 오타를 띄우는 순간 잡는다. 조용히 기본 소유자로 넘어가면
    #  주차 구간에서 조향이 엉뚱한 팀에게 간다.
    import pytest
    with pytest.raises(ValueError):
        WheelGate("camera", ["99:lidar"], SOURCES, NUM_MODES)


def test_별칭표_형식_파싱():
    #  manager 가 하는 것과 같은 변환. 옛 이름이 번호로 바뀌는지.
    raw = ["S_COURSE:5", "T_PARK:7", "PARALLEL_PARK:10", "NORMAL:1"]
    table = {}
    for e in raw:
        old, new = e.split(":", 1)
        table[old.strip().upper()] = new.strip().upper()
    assert table["S_COURSE"] == "5"
    assert table["NORMAL"] == "1"
    #  별칭을 거친 값은 known_modes 안에 있어야 한다
    for v in table.values():
        assert v in NUM_MODES


# ============================================================
# S 코스 실행기 판정 로직 (0831)
#   ROS 없이 시험할 수 있도록 순수 함수로 뺐다.
# ============================================================

import importlib.util as _ilu
import os as _os

_sc_path = _os.path.join(_os.path.dirname(__file__), "..", "..", "..",
                         "tools", "s_course_v1_0831.py")
_sc = None
if _os.path.isfile(_sc_path):
    _spec = _ilu.spec_from_file_location("s_course_tool", _sc_path)
    _sc = _ilu.module_from_spec(_spec)
    _spec.loader.exec_module(_sc)


def _need_tool():
    import pytest
    if _sc is None:
        pytest.skip("s_course_v1_0831.py 를 찾을 수 없다")


def _syms(st):
    _need_tool()
    return [sym for sym, _, _ in _sc.diagnose(st)[1]]


OKAY = {"mgr_alive": True, "mgr_mode": "5", "wheel_src": "lidar",
        "drive_src": "lidar", "safety": "OK", "avoid_active": True,
        "lidar_wheel_hz": 10.0, "lidar_drive_hz": 10.0}


def test_s코스_전부_정상이면_통과():
    _need_tool()
    ok, _ = _sc.diagnose(dict(OKAY))
    assert ok


def test_s코스_매니저_없으면_거기서_멈춘다():
    _need_tool()
    ok, rows = _sc.diagnose({"mgr_alive": False})
    assert not ok and len(rows) == 1


def test_s코스_조향권한이_카메라면_잡아낸다():
    #  0830 에 실제로 났던 상황. 이걸 못 잡으면 도구를 만든 의미가 없다.
    st = dict(OKAY); st["wheel_src"] = "camera"
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert any("조향 권한이 'camera'" in t for _, t, _ in rows)


def test_s코스_failsafe면_직진만_한다고_알려준다():
    st = dict(OKAY); st["wheel_src"] = "failsafe"
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert any("직진만" in h for _, _, h in rows)


def test_s코스_모드가_안_들어가면_잡아낸다():
    st = dict(OKAY); st["mgr_mode"] = "0"
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert any("모드가 5가 아니다" in t for _, t, _ in rows)


def test_s코스_라이다가_조용하면_잡아낸다():
    st = dict(OKAY); st["lidar_wheel_hz"] = 0.0
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert any("라이다 조향이 안 온다" in t for _, t, _ in rows)


def test_s코스_급정거는_치명으로_본다():
    st = dict(OKAY); st["safety"] = "EMERGENCY_STOP(lidar)"
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert "🔴" in _syms(st)


# ============================================================
# 회피 활성 신호 (0831)
#   라이다는 /avoidance/active 가 참일 때만 조향을 발행한다.
#   권한(wheel_owner_overrides)과는 별개의 조건이라 둘 다 맞아야 한다.
# ============================================================

def _avoid_on(mode, modes):
    """manager 가 하는 판정과 같은 규칙."""
    return str(mode).strip().upper() in set(
        str(m).strip().upper() for m in modes if str(m).strip())


def test_게이트구간_5번에서만_적용된다():
    modes = ["5"]
    assert _avoid_on("5", modes)
    for m in ("0", "1", "2", "3", "4", "6", "7", "8", "9", "10", "11"):
        assert not _avoid_on(m, modes), m


def test_게이트구간_목록이_비면_아무것도_안_막는다():
    for m in ("5", "7", "10"):
        assert not _avoid_on(m, [])


def test_게이트구간_여러_개도_가능():
    modes = ["5", "3"]
    assert _avoid_on("5", modes) and _avoid_on("3", modes)
    assert not _avoid_on("4", modes)


def test_s코스_게이트_닫히면_잡아낸다():
    _need_tool()
    st = dict(OKAY); st["avoid_active"] = False
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert any("게이트 = false" in t for _, t, _ in rows)


def test_s코스_게이트_신호가_없으면_치명으로_본다():
    #  신호를 못 받으면 중재기가 라이다를 막으므로 차가 안 움직인다.
    #  그걸 "라이다 노드가 죽었나" 로 오해하지 않게 따로 알린다.
    _need_tool()
    st = dict(OKAY); st["avoid_active"] = None
    ok, rows = _sc.diagnose(st)
    assert not ok
    assert any("게이트 신호가 안 온다" in t for _, t, _ in rows)


def test_s코스_회피신호_켜지면_통과():
    _need_tool()
    st = dict(OKAY); st["avoid_active"] = True
    ok, _ = _sc.diagnose(st)
    assert ok


# ============================================================
# 회피 게이트 — 중재기가 강제한다 (0831)
#
#   "5번 구간에서 /avoidance/active 가 true 일 때만
#    라이다가 조향과 속도를 조절할 수 있다."
#
#   라이다의 자제에 기대지 않고 중재기가 막는다. 라이다 노드에 버그가
#   있거나 옛 버전이 돌아도 차가 안 움직이게 하려면 여기서 막아야 한다.
# ============================================================

from t870_mcu.arbitration import InputManager, PrioritySelector

GSRC = ["lidar", "camera", "gps", "manual"]


def _rig():
    inp = InputManager(GSRC)
    sel = PrioritySelector(["lidar", "camera", "gps"], GSRC)
    gate = WheelGate("camera", ["5:lidar", "7:lidar", "10:lidar"],
                     GSRC, [str(i) for i in range(12)])
    now = 100.0
    inp.update_drive("lidar", 2.0, now, True, "ok")
    inp.update_wheel("lidar", 15, now, True, "ok")
    inp.update_stop("lidar", False, now)
    return inp, sel, gate, now


def test_게이트_열리면_라이다가_구동과_조향을_한다():
    inp, sel, gate, now = _rig()
    src, val, _ = sel.select(inp, now, 0.5)
    assert src == "lidar" and val == 2.0
    w, wsrc, wok, _ = gate.resolve("5", inp, now, 0.5, 0)
    assert wok and wsrc == "lidar" and w == 15


def test_게이트_닫히면_구동이_막힌다():
    #  ★ 속도까지 막는 것이 이번 요구사항의 핵심이다.
    #    구동은 원래 고정 우선순위라 모드와 무관하게 라이다가 1순위였다.
    inp, sel, gate, now = _rig()
    inp.set_blocked({"lidar": "gate_false"})
    src, val, _ = sel.select(inp, now, 0.5)
    assert src is None and val == 0.0


def test_게이트_닫히면_조향도_막힌다():
    inp, sel, gate, now = _rig()
    inp.set_blocked({"lidar": "gate_false"})
    w, wsrc, wok, reason = gate.resolve("5", inp, now, 0.5, 0)
    assert not wok
    assert w == 0                      # failsafe = 중앙
    assert "gate_false" in reason


def test_게이트_닫혀도_급정거는_통한다():
    #  🔴 안전은 게이트보다 위다. 이게 깨지면 장애물을 못 피한다.
    inp, sel, gate, now = _rig()
    inp.set_blocked({"lidar": "gate_false"})
    inp.update_stop("lidar", True, now)
    assert inp.any_stop(["lidar", "camera", "gps"], now, 0.5) == ["lidar"]


def test_게이트는_지정_소스만_막는다():
    inp, sel, gate, now = _rig()
    inp.update_drive("camera", 1.0, now, True, "ok")
    inp.set_blocked({"lidar": "gate_false"})
    src, val, _ = sel.select(inp, now, 0.5)
    assert src == "camera" and val == 1.0


def test_게이트_다시_열면_복구된다():
    inp, sel, gate, now = _rig()
    inp.set_blocked({"lidar": "gate_false"})
    inp.set_blocked({})
    src, _, _ = sel.select(inp, now, 0.5)
    assert src == "lidar"
    _, wsrc, wok, _ = gate.resolve("5", inp, now, 0.5, 0)
    assert wok and wsrc == "lidar"
