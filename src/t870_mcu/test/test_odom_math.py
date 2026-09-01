#!/usr/bin/env python3
"""odom_math 단위시험 — 차 없이 돌아간다.

    cd ~/mcu_ws/src/t870_mcu && PYTHONPATH=. python3 -m pytest test/ -q
"""

import math
import pytest

from t870_mcu.odom_math import (
    center_twist,
    front_wheel_to_rear_axle,
    integrate_rear_axle,
    rear_to_center,
    yaw_rate,
)

L = 0.73
T = 0.60


# ============================================================
# yaw_rate
# ============================================================

def test_yaw_rate_straight_is_zero():
    assert yaw_rate(1.0, 0.0, L) == 0.0


def test_yaw_rate_left_is_positive():
    """REP-103: 좌회전(+delta) 이면 yaw 가 증가한다."""
    assert yaw_rate(1.0, math.radians(20), L) > 0.0


def test_yaw_rate_bad_wheelbase_is_zero():
    assert yaw_rate(1.0, 0.5, 0.0) == 0.0


# ============================================================
# front_wheel_to_rear_axle
# ============================================================

def test_wheel_straight_unchanged():
    """직진에서는 어떤 설정이든 거리를 건드리지 않는다."""
    assert front_wheel_to_rear_axle(1.0, 0.0, L, T, "left") == 1.0
    assert front_wheel_to_rear_axle(1.0, 0.0, L, 0.0, "left") == 1.0


def test_wheel_track_zero_falls_back_to_cos():
    """track=0 이면 예전 v37 동작(cos 보정)을 그대로 유지한다 — 회귀 보장."""
    for deg in (5, 10, 20, 27):
        d = math.radians(deg)
        got = front_wheel_to_rear_axle(1.0, d, L, 0.0, "left")
        assert got == pytest.approx(math.cos(d), abs=1e-12)


def test_wheel_inner_outer_split_left_turn():
    """좌회전이면 왼쪽 바퀴가 안쪽(짧게), 오른쪽이 바깥쪽(길게) 돈다.

    같은 1 m 를 굴렀다면 안쪽 바퀴 쪽이 실제 차는 더 많이 간 것이다
    → 환산 계수가 1 보다 크다. 바깥쪽은 그 반대.
    """
    d = math.radians(20)
    left = front_wheel_to_rear_axle(1.0, d, L, T, "left")
    right = front_wheel_to_rear_axle(1.0, d, L, T, "right")
    assert left > 1.0 > right
    # 0831 계산표: 안쪽 0.925, 바깥 1.206 (역수를 취한 값이 환산계수)
    assert left == pytest.approx(1.0 / 0.9250, rel=1e-3)
    assert right == pytest.approx(1.0 / 1.2058, rel=1e-3)


def test_wheel_right_turn_mirrors_left_turn():
    """우회전에서 왼쪽 바퀴 = 좌회전에서 오른쪽 바퀴 (대칭)."""
    d = math.radians(20)
    a = front_wheel_to_rear_axle(1.0, -d, L, T, "left")
    b = front_wheel_to_rear_axle(1.0, +d, L, T, "right")
    assert a == pytest.approx(b, rel=1e-12)


def test_wheel_correction_grows_with_steer():
    """조향각이 클수록 좌우 편차가 커진다."""
    prev = 0.0
    for deg in (5, 10, 20, 27):
        d = math.radians(deg)
        spread = abs(front_wheel_to_rear_axle(1.0, d, L, T, "left")
                     - front_wheel_to_rear_axle(1.0, d, L, T, "right"))
        assert spread > prev
        prev = spread


def test_wheel_sign_preserved_on_reverse():
    """후진(음수 거리)이면 결과도 음수여야 한다."""
    got = front_wheel_to_rear_axle(-1.0, math.radians(20), L, T, "left")
    assert got < 0.0


# ============================================================
# integrate_rear_axle
# ============================================================

def test_integrate_straight_moves_along_heading():
    x, y, th = integrate_rear_axle(0.0, 0.0, 0.0, 2.0, 0.0, L)
    assert (x, y, th) == pytest.approx((2.0, 0.0, 0.0))


def test_integrate_straight_at_90deg():
    x, y, th = integrate_rear_axle(0.0, 0.0, math.pi / 2, 2.0, 0.0, L)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(2.0)


def test_integrate_left_turn_curves_left():
    x, y, th = integrate_rear_axle(0.0, 0.0, 0.0, 1.0, math.radians(20), L)
    assert th > 0.0 and y > 0.0


def test_integrate_quarter_circle_lands_on_radius():
    """최대조향으로 90도 돌면 (R, R) 부근에 도착한다. 원호 적분 검증."""
    delta = math.radians(27)
    radius = L / math.tan(delta)
    arc = radius * (math.pi / 2)
    x = y = th = 0.0
    for _ in range(2000):                       # 잘게 쪼개 적분
        x, y, th = integrate_rear_axle(x, y, th, arc / 2000.0, delta, L)
    assert th == pytest.approx(math.pi / 2, abs=1e-6)
    assert x == pytest.approx(radius, abs=1e-3)
    assert y == pytest.approx(radius, abs=1e-3)


def test_integrate_theta_stays_normalized():
    """한 바퀴 넘게 돌아도 theta 가 [-pi, pi) 를 벗어나지 않는다."""
    x = y = th = 0.0
    for _ in range(5000):
        x, y, th = integrate_rear_axle(x, y, th, 0.01, math.radians(27), L)
        assert -math.pi <= th < math.pi + 1e-12


def test_integrate_bad_wheelbase_is_noop():
    assert integrate_rear_axle(1.0, 2.0, 0.3, 5.0, 0.2, 0.0) == (1.0, 2.0, 0.3)


# ============================================================
# rear_to_center
# ============================================================

def test_center_is_half_wheelbase_ahead():
    x, y = rear_to_center(0.0, 0.0, 0.0, L)
    assert (x, y) == pytest.approx((L / 2, 0.0))


def test_center_follows_heading():
    x, y = rear_to_center(0.0, 0.0, math.pi / 2, L)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(L / 2)


def test_center_offset_distance_is_constant():
    """방위가 어떻든 뒤축과 중심의 거리는 항상 L/2 다."""
    for deg in range(0, 360, 30):
        th = math.radians(deg)
        x, y = rear_to_center(1.0, 2.0, th, L)
        assert math.hypot(x - 1.0, y - 2.0) == pytest.approx(L / 2)


# ============================================================
# center_twist
# ============================================================

def test_twist_straight_has_no_lateral():
    vx, vy, wz = center_twist(1.0, 0.0, L)
    assert (vx, vy, wz) == pytest.approx((1.0, 0.0, 0.0))


def test_twist_center_has_lateral_velocity():
    """★ 중심점은 회전 중 vy 가 0 이 아니다. 0831 수치검증값과 대조."""
    vx, vy, wz = center_twist(0.5, math.radians(20), L)
    assert vx == pytest.approx(0.5)
    assert wz == pytest.approx(0.24929, abs=1e-5)
    assert vy == pytest.approx(0.09099, abs=1e-5)
    assert vy == pytest.approx(0.5 * L * wz)


def test_twist_right_turn_lateral_flips():
    _, vy_l, _ = center_twist(0.5, math.radians(20), L)
    _, vy_r, _ = center_twist(0.5, math.radians(-20), L)
    assert vy_l == pytest.approx(-vy_r)


# ============================================================
# 통합 — 엔코더 카운트에서 자세까지
# ============================================================

def test_full_chain_straight_10m():
    """직진 10 m: 어떤 보정을 켜도 결과가 같아야 한다."""
    x = y = th = 0.0
    for _ in range(1000):
        d = front_wheel_to_rear_axle(0.01, 0.0, L, T, "left")
        x, y, th = integrate_rear_axle(x, y, th, d, 0.0, L)
    cx, cy = rear_to_center(x, y, th, L)
    assert x == pytest.approx(10.0)
    assert cx == pytest.approx(10.0 + L / 2)
    assert cy == pytest.approx(0.0, abs=1e-9)


def test_full_chain_wheel_correction_changes_result():
    """윤거 보정을 켜고 끄면 선회 궤적이 실제로 달라진다.

    이게 같게 나오면 보정이 코드에서 죽어 있다는 뜻이라 잡아낸다.
    """
    delta = math.radians(25)

    def run(track):
        x = y = th = 0.0
        for _ in range(500):
            d = front_wheel_to_rear_axle(0.01, delta, L, track, "left")
            x, y, th = integrate_rear_axle(x, y, th, d, delta, L)
        return x, y, th

    a = run(0.0)
    b = run(T)
    assert abs(a[2] - b[2]) > 0.05          # 방위가 눈에 띄게 다르다


# ============================================================
# 0901 — 정중앙 기준 발행 확정. 원점 이동과 static TF 의 짝 검증
# ============================================================

def test_center_offset_matches_frames_yaml_shift():
    """static TF 의 x 를 0.365 빼는 것과 오돔 원점 이동은 같은 값이어야 한다.

    한쪽만 바꾸면 센서가 본 것이 통째로 어긋난다. 숫자를 여기 고정해 둔다.
    """
    shift = L / 2
    assert shift == pytest.approx(0.365)
    # t870_frames.yaml 에 실제로 들어간 값
    assert 1.10 - shift == pytest.approx(0.735)
    assert 1.05 - shift == pytest.approx(0.685)
    assert 0.20 - shift == pytest.approx(-0.165)


def test_center_pose_equals_rear_pose_plus_shift():
    """같은 주행에서 center 자세 = rear 자세 + L/2 (진행방향)."""
    x = y = th = 0.0
    for _ in range(300):
        x, y, th = integrate_rear_axle(x, y, th, 0.01, math.radians(15), L)
    cx, cy = rear_to_center(x, y, th, L)
    assert math.hypot(cx - x, cy - y) == pytest.approx(L / 2)
    # 진행 방향으로 앞서 있다
    assert (cx - x) * math.cos(th) + (cy - y) * math.sin(th) > 0


def test_center_start_is_ahead_of_rear_start():
    """정지 상태에서도 중심점은 뒤축보다 0.365 m 앞이다."""
    cx, cy = rear_to_center(0.0, 0.0, 0.0, L)
    assert cx == pytest.approx(0.365)
    assert cy == pytest.approx(0.0)


def test_center_origin_starts_at_zero():
    """center 기준이면 뒤축을 -L/2 에서 시작시켜야 발행 위치가 (0,0) 이다.

    안 그러면 정지 상태에서 base_link 가 odom 원점보다 36 cm 앞에 찍혀
    "출발도 안 했는데 앞에 있다" 로 보인다. 브릿지 _odom_origin_x() 와 짝.
    """
    x0 = -0.5 * L
    px, py = rear_to_center(x0, 0.0, 0.0, L)
    assert px == pytest.approx(0.0, abs=1e-12)
    assert py == pytest.approx(0.0, abs=1e-12)


def test_center_origin_holds_at_any_heading():
    """어느 방향을 보고 있어도 시작점 보정은 정확히 원점을 준다."""
    for deg in range(0, 360, 45):
        th = math.radians(deg)
        x0 = -0.5 * L * math.cos(th)
        y0 = -0.5 * L * math.sin(th)
        px, py = rear_to_center(x0, y0, th, L)
        assert math.hypot(px, py) == pytest.approx(0.0, abs=1e-12)
