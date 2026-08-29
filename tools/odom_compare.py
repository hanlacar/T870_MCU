#!/usr/bin/env python3
"""odom 이 실측과 다를 때 — 어느 단계에서 틀어지는지 갈라낸다.

odom 은 여러 손을 거친다. "다르다" 만으로는 누구 문제인지 알 수 없다.

    바퀴 회전 → 엔코더 카운트 → (÷ counts_per_meter) → 거리
                     │                                   │
                     │                                   └→ 우리 /mcu/distance_m
                     └→ /mcu/encoder → wheel_odom
                                        (× encoder_m_per_tick, IMU yaw) → /odom

이 도구는 **네 값을 동시에** 재서 어디서 어긋나는지 보여준다.
줄자로 잰 실제 거리를 입력하면 실측 counts_per_meter 까지 역산한다.

사용법
    1. 차를 출발선에 세우고 바퀴에 표시를 한다
    2. python3 tools/odom_compare.py          ← 시작 (값을 찍는다)
    3. 차를 직선으로 5m 쯤 몬다 (곡선 금지 — 직선이어야 비교가 성립한다)
    4. Enter → 줄자로 잰 거리 입력
    5. 표를 보고 어느 줄이 틀렸는지 확인

옵션
    --cpm 199.8        우리 설정값 (기본은 yaml 에서 읽는다)
    --mcu-ns /mcu      MCU 네임스페이스
    --odom-topic /odom wheel_odom 이 내는 토픽
"""

import argparse
import math
import os
import re
import sys


# ==========================================================
# 순수 계산 — ROS 없이 시험할 수 있도록 분리
# ==========================================================
def analyze(counts, our_dist, their_dist, real_dist, cpm):
    """네 값을 비교해 (표_행들, 결론들) 을 만든다.

    counts      /mcu/encoder 증가량 (원시 카운트)
    our_dist    /mcu/distance_m 증가량 [m]   (없으면 None)
    their_dist  /odom 이동거리 [m]           (없으면 None)
    real_dist   줄자 실측 [m]
    cpm         설정된 counts_per_meter
    """
    rows = []
    notes = []

    rows.append(("줄자 실측", "%.3f m" % real_dist, "기준"))
    rows.append(("엔코더 카운트", "%d" % counts, "하드웨어가 센 값"))

    if real_dist > 0 and counts != 0:
        measured_cpm = abs(counts) / real_dist
        rows.append(("실측 counts_per_meter", "%.1f" % measured_cpm,
                     "설정값 %.1f" % cpm))
        ratio = measured_cpm / cpm if cpm > 0 else float("inf")
        if abs(ratio - 1.0) > 0.10:
            notes.append(
                "🔴 counts_per_meter 가 %.1f 배 어긋난다 (설정 %.1f / 실측 %.1f). "
                "→ 우리 문제. yaml 의 counts_per_meter 를 %.1f 로 고칠 것."
                % (ratio, cpm, measured_cpm, measured_cpm))
        else:
            notes.append(
                "✅ counts_per_meter 는 맞다 (설정 %.1f / 실측 %.1f, 오차 %.1f%%). "
                "→ 엔코더와 우리 환산은 정상."
                % (cpm, measured_cpm, abs(ratio - 1.0) * 100.0))

    if our_dist is not None:
        rows.append(("우리 /mcu/distance_m", "%.3f m" % our_dist,
                     "%+.1f%%" % _err(our_dist, real_dist)))
        if abs(_err(our_dist, real_dist)) > 10.0:
            notes.append(
                "🔴 우리 /mcu/distance_m 가 실측과 %+.1f%% 다르다. "
                "카운트는 맞는데 이게 틀리면 odom_steer_compensation 을 의심할 것 "
                "(조향각 추정으로 cos 를 곱한다. 그 추정을 아직 못 믿는다)."
                % _err(our_dist, real_dist))

    if their_dist is not None:
        rows.append(("wheel_odom /odom", "%.3f m" % their_dist,
                     "%+.1f%%" % _err(their_dist, real_dist)))
        if abs(_err(their_dist, real_dist)) > 10.0:
            #  encoder_m_per_tick 을 역산해 준다
            if counts != 0:
                needed = real_dist / abs(counts)
                notes.append(
                    "🔴 wheel_odom 의 /odom 이 실측과 %+.1f%% 다르다. "
                    "→ 미션팀 문제. encoder_m_per_tick 을 %.6f 로 고칠 것 "
                    "(지금 값으로 역산하면 %.6f 로 돌고 있다)."
                    % (_err(their_dist, real_dist), needed,
                       abs(their_dist) / abs(counts)))
        else:
            notes.append("✅ wheel_odom /odom 도 실측과 맞는다.")

    if our_dist is not None and their_dist is not None:
        if abs(_err(our_dist, real_dist)) <= 10.0 < abs(_err(their_dist, real_dist)):
            notes.append(
                "→ 우리 값은 맞고 그쪽 값만 틀리다. **우리 문제가 아니다.**")
        elif abs(_err(their_dist, real_dist)) <= 10.0 < abs(_err(our_dist, real_dist)):
            notes.append(
                "→ 그쪽 값은 맞고 우리 값만 틀리다. **우리 문제다.**")

    return rows, notes


def _err(value, real):
    if real == 0:
        return 0.0
    return (value - real) / real * 100.0


def read_cpm_from_yaml():
    """yaml 에서 counts_per_meter 를 읽는다. 못 찾으면 None."""
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "src", "t870_mcu", "config",
                              "t870_mcu.yaml"),
                 os.path.expanduser("~/mcu_ws/src/t870_mcu/config/t870_mcu.yaml")):
        if not os.path.isfile(cand):
            continue
        try:
            with open(cand, encoding="utf-8") as fh:
                m = re.search(r"^\s*counts_per_meter:\s*([0-9.]+)", fh.read(), re.M)
            if m:
                return float(m.group(1))
        except Exception:
            pass
    return None


# ==========================================================
# ROS 부분
# ==========================================================
def main():
    ap = argparse.ArgumentParser(description="odom 실측 비교")
    ap.add_argument("--cpm", type=float, default=None,
                    help="counts_per_meter (기본: yaml 에서 읽음)")
    ap.add_argument("--mcu-ns", default="/mcu")
    ap.add_argument("--odom-topic", default="/odom")
    args = ap.parse_args()

    cpm = args.cpm if args.cpm else read_cpm_from_yaml()
    if not cpm:
        print("counts_per_meter 를 못 찾았다. --cpm 으로 직접 줄 것.")
        return 1

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32, Float32
    from nav_msgs.msg import Odometry

    class Watch(Node):
        def __init__(self):
            super().__init__("odom_compare")
            self.enc = None
            self.dist = None
            self.pos = None
            ns = args.mcu_ns.rstrip("/")
            self.create_subscription(
                Int32, ns + "/encoder",
                lambda m: setattr(self, "enc", int(m.data)), 10)
            self.create_subscription(
                Float32, ns + "/distance_m",
                lambda m: setattr(self, "dist", float(m.data)), 10)
            self.create_subscription(
                Odometry, args.odom_topic, self._cb_odom, 10)

        def _cb_odom(self, m):
            self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)

        def pump(self, seconds):
            end = self.get_clock().now().nanoseconds * 1e-9 + seconds
            while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
                rclpy.spin_once(self, timeout_sec=0.05)

    rclpy.init()
    node = Watch()
    print("토픽 수신을 기다린다 (최대 5초)...")
    node.pump(5.0)

    missing = []
    if node.enc is None:
        missing.append("%s/encoder" % args.mcu_ns.rstrip("/"))
    if node.dist is None:
        missing.append("%s/distance_m" % args.mcu_ns.rstrip("/"))
    if node.pos is None:
        missing.append(args.odom_topic)
    if missing:
        print("\n안 들어오는 토픽: %s" % ", ".join(missing))
        print("그 항목은 비교에서 빠진다. 계속하려면 Enter, 중단하려면 Ctrl-C")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            node.destroy_node(); rclpy.shutdown(); return 1

    start_enc, start_dist, start_pos = node.enc, node.dist, node.pos
    print("\n─────────── 시작값 ───────────")
    print("  엔코더        %s" % start_enc)
    print("  distance_m    %s" % start_dist)
    print("  odom 위치     %s" % (start_pos,))
    print("──────────────────────────────")
    print("\n이제 차를 **직선으로** 5m 쯤 몰고 세운 뒤 Enter.")
    print("(곡선으로 가면 줄자 거리와 주행 거리가 달라 비교가 안 된다)")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        node.destroy_node(); rclpy.shutdown(); return 1

    node.pump(1.0)
    end_enc, end_dist, end_pos = node.enc, node.dist, node.pos

    try:
        real = float(input("줄자로 잰 실제 이동거리 [m]: ").strip())
    except (ValueError, EOFError, KeyboardInterrupt):
        print("숫자를 못 읽었다. 중단.")
        node.destroy_node(); rclpy.shutdown(); return 1

    counts = (end_enc - start_enc) if None not in (end_enc, start_enc) else 0
    our = ((end_dist - start_dist)
           if None not in (end_dist, start_dist) else None)
    theirs = None
    if None not in (end_pos, start_pos):
        theirs = math.hypot(end_pos[0] - start_pos[0],
                            end_pos[1] - start_pos[1])

    rows, notes = analyze(counts, our, theirs, real, cpm)

    print("\n" + "=" * 62)
    print("%-24s %-14s %s" % ("항목", "값", "비고"))
    print("-" * 62)
    for name, val, note in rows:
        print("%-24s %-14s %s" % (name, val, note))
    print("=" * 62)
    for n in notes:
        print("\n" + n)
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
