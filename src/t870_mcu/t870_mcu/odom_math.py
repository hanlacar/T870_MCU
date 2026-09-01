#!/usr/bin/env python3
"""오도메트리 순수 수학 (ROS 의존 없음).

여기 있는 함수는 전부 부작용이 없다. 그래서 차 없이 pytest 로 검증한다.
bridge_node.update_odom() 은 이 함수들을 호출만 한다.

--------------------------------------------------------------------
T870 의 물리 구조 (0831 확정)
--------------------------------------------------------------------
* 4WD. 앞축·뒤축 **둘 다** 구동한다. 그래서 "구동하지 않는 바퀴" 가 없다.
* 앞축·뒤축 모두 **오픈 디퍼렌셜**. 근거(0831 실측):
    - 공중에서 앞은 왼쪽, 뒤는 오른쪽 바퀴가 더 강하게 돈다
    - 한쪽을 손으로 잡으면 반대쪽이 더 빨리 돈다
    - 전원을 끊고 한쪽을 돌리면 반대쪽이 반대로 돈다
  전부 정상 동작이다. 고장이 아니다.
* 엔코더는 **앞바퀴 한쪽에만** 달려 있다 (A상 단일, 방향 판별 불가).

--------------------------------------------------------------------
그래서 생기는 오차 두 가지
--------------------------------------------------------------------
(1) 차동 슬립 — 토크가 접지가 나쁜 쪽으로 몰려 그 바퀴만 헛돈다.
    출발 순간에 특히 크다. **이 모듈로는 못 고친다.**
    라이다 스캔매칭 오돔으로 대체하거나 다섯 번째 비구동 바퀴가 필요하다.

(2) 선회 기하 — 엔코더 바퀴가 차 중심선에서 윤거의 절반만큼 옆에 있다.
    선회 중 안쪽 바퀴와 바깥쪽 바퀴는 아예 다른 거리를 돈다.
    **이건 여기서 정확히 고칠 수 있다.**

    윤거 0.6 m, 축거 0.73 m 기준 (뒤축 중심 이동거리 대비 앞바퀴 이동거리):

        조향각   안쪽 바퀴   바깥쪽 바퀴   좌우 차이
          10°     0.944       1.087        14 %
          20°     0.925       1.206        28 %
          27°     0.941       1.312        37 %

    90° 코너 한 번에 안/바깥 바퀴는 t·(pi/2) = 0.94 m 나 차이 난다.

--------------------------------------------------------------------
기존 cos(delta) 보정에 대하여
--------------------------------------------------------------------
v37 브릿지는 d_rear = d_front * cos(delta) 를 써 왔다.
이 식 자체는 맞다 — 단 **앞축 중심** 에 대해서만 맞다.
    앞축중심 선회반경 = L/sin(delta),  뒤축중심 = L/tan(delta)
    → 비 = cos(delta)                    (수치 검증 완료)
바퀴가 중심에서 t/2 만큼 옆에 있다는 사실이 빠져 있다.
track_m 을 실측해 넣으면 full 보정(front_wheel_to_rear_axle)이 켜진다.
track_m = 0 이면 예전 cos(delta) 동작을 그대로 유지한다 (회귀 보장).
"""

import math

__all__ = [
    "front_wheel_to_rear_axle",
    "integrate_rear_axle",
    "rear_to_center",
    "center_twist",
    "yaw_rate",
]


def yaw_rate(v, steer_rad, wheelbase):
    """자전거 모델 회전율 [rad/s]. omega = v * tan(delta) / L."""
    if wheelbase <= 0.0:
        return 0.0
    return v * math.tan(steer_rad) / wheelbase


def front_wheel_to_rear_axle(d_wheel, steer_rad, wheelbase, track=0.0,
                             encoder_side="left"):
    """앞바퀴 한쪽이 굴러간 거리를 뒤축 중심 이동거리로 환산한다.

    d_wheel      : 엔코더가 센 그 바퀴의 이동거리 [m] (부호 있음)
    steer_rad    : 조향각 [rad]. + = 좌회전 (REP-103)
    wheelbase    : 축거 L [m]
    track        : 앞 윤거 t [m]. **0 이면 t/2 보정을 끄고 cos(delta) 만 쓴다**
    encoder_side : 엔코더가 달린 바퀴. "left" 또는 "right"

    반환: 뒤축 중심 이동거리 [m]

    기하
    ----
    좌회전(delta>0)이면 순간회전중심(ICR)은 차 왼쪽에 있다.
    뒤축 중심 선회반경          R = L / tan(delta)
    앞 왼쪽 바퀴의 선회반경   r_L = hypot(L, R - t/2)      (안쪽)
    앞 오른쪽 바퀴의 선회반경 r_R = hypot(L, R + t/2)      (바깥쪽)
    같은 각도를 도니까 이동거리 비 = 반경 비.
        d_rear = d_wheel * R / r_wheel

    우회전(delta<0)이면 좌우가 뒤바뀐다. 아래에서 부호로 처리한다.
    직진(delta=0)이면 R 이 무한대라 그냥 d_wheel 을 그대로 쓴다.
    """
    if wheelbase <= 0.0:
        return d_wheel

    t = math.tan(steer_rad)
    if abs(t) < 1e-9:
        return d_wheel                      # 직진 — 보정 없음

    if track <= 0.0:
        # 예전 동작: 앞축 "중심" 기준 보정. 바퀴 위치를 모르면 이게 최선이다.
        return d_wheel * math.cos(steer_rad)

    radius = wheelbase / t                  # 부호 있음. + = 좌회전
    half = 0.5 * float(track)

    # ICR 은 좌회전이면 왼쪽(+y). 엔코더 바퀴가 안쪽이면 반경이 작다.
    #   좌회전 + 왼쪽바퀴  → 안쪽 → R - t/2
    #   좌회전 + 오른쪽바퀴 → 바깥 → R + t/2
    #   우회전이면 radius 가 음수라 아래 식이 자동으로 뒤집힌다.
    side = -1.0 if str(encoder_side).strip().lower().startswith("l") else +1.0
    lateral = radius + side * half

    r_wheel = math.hypot(wheelbase, lateral)
    if r_wheel < 1e-9:
        return d_wheel
    return d_wheel * abs(radius) / r_wheel


def integrate_rear_axle(x, y, theta, d, steer_rad, wheelbase):
    """뒤축 중심 자세를 한 스텝 적분한다.

    d 는 **뒤축 중심** 이동거리다 (front_wheel_to_rear_axle 을 먼저 통과시킬 것).
    회전 중에는 직선이 아니라 원호를 따라간다. 원호 적분을 그대로 쓴다.

    반환: (x, y, theta)  — theta 는 [-pi, pi) 로 정규화한다.
    """
    if wheelbase <= 0.0:
        return x, y, theta

    dtheta = d * math.tan(steer_rad) / wheelbase

    if abs(dtheta) < 1e-9:
        x += d * math.cos(theta)
        y += d * math.sin(theta)
    else:
        radius = d / dtheta                 # 이 스텝의 선회반경
        x += radius * (math.sin(theta + dtheta) - math.sin(theta))
        y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
        theta += dtheta
        theta = math.atan2(math.sin(theta), math.cos(theta))
    return x, y, theta


def rear_to_center(x_r, y_r, theta, wheelbase):
    """뒤축 중심 위치를 네 바퀴 중심(차량 정중앙) 위치로 옮긴다.

    0831 확정: base_link 원점 = 네 바퀴 허브 4점의 기하 중심.
    뒤축 중심에서 차체 전방으로 정확히 L/2 앞이다 (앞뒤 윤거가 같을 때).
    """
    half = 0.5 * wheelbase
    return (x_r + half * math.cos(theta),
            y_r + half * math.sin(theta))


def center_twist(v, steer_rad, wheelbase):
    """4바퀴 중심점의 차체 기준 속도 (vx, vy, wz).

    ★ 뒤축 중심과 달리 중심점은 **회전 중 횡속도가 0 이 아니다.**
      중심점은 뒤축에서 L/2 앞에 있고, 차체가 wz 로 돌면 그 점은
      옆으로 (L/2)*wz 만큼 쓸린다. 이걸 0 으로 두면 EKF 가 조용히 틀린다.

    수치미분으로 검증 완료 (v=0.5, delta=20deg, L=0.73):
        wz = 0.24929 rad/s,  vy = 0.09099 m/s = (L/2)*wz
    """
    wz = yaw_rate(v, steer_rad, wheelbase)
    return v, 0.5 * wheelbase * wz, wz
