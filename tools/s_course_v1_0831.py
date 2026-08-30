#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s_course_v1_0831.py — S 코스(5번 구간) 전용 실행기.

    python3 tools/s_course_v1_0831.py

하는 일은 두 가지뿐이다.
  1. /drive_mode 에 "5" 를 계속 발행한다 (S 코스 구간)
  2. 그래서 실제로 라이다가 조향을 잡았는지 화면으로 보여준다

★ 이 도구는 차를 몰지 않는다.
  구동과 조향은 라이다 회피 노드가 낸다. 이 도구는 구간만 잡아준다.
  (모드를 안 잡으면 5번 구간이 아니라 조향 권한이 카메라에 있고,
   라이다가 아무리 조향값을 보내도 매니저가 통째로 무시한다.
   0830 에 S자 연습이 안 되던 이유가 정확히 이것이다.)

★ 5번 구간에서 라이다가 조향과 속도를 조절하려면 조건이 셋이다.
    ① 모드가 5           이 도구가 잡아준다
    ② 조향 권한이 lidar   yaml 의 wheel_owner_overrides "5:lidar"
    ③ 회피 게이트가 true  라이다가 /avoidance/active 를 발행
  셋 중 하나만 빠져도 차는 직진만 한다. 화면이 어느 것인지 알려준다.

먼저 매니저·브릿지가 떠 있어야 한다:
    cd ~/T870_MCU && ./실행.sh

토픽 이름이 다르면 인자로 준다:
    --mode-topic /drive_mode  --mcu-ns /mcu
    --lidar-wheel /lidar_wheel  --lidar-drive /lidar_drive
"""

import argparse
import sys
import time


# ============================================================
# 판정 — ROS 없이 시험할 수 있도록 순수 함수로 분리
# ============================================================

TARGET_MODE = "5"


def _wrap(text, width):
    """긴 안내문을 화면 폭에 맞춰 자른다."""
    out, line = [], ""
    for word in text.split():
        if len(line) + len(word) + 1 > width:
            out.append(line)
            line = word
        else:
            line = (line + " " + word).strip()
    if line:
        out.append(line)
    return out


def diagnose(st):
    """상태 dict 로 점검 결과를 만든다.

    반환: (전체통과여부, [(표시기호, 제목, 설명), ...])

    st 의 키
        mgr_alive      매니저 토픽을 하나라도 받았나
        mgr_mode       매니저가 받아들인 모드 (없으면 None)
        wheel_src      /mcu/active_wheel_source (없으면 None)
        drive_src      /mcu/active_drive_source (없으면 None)
        safety         /mcu/safety_state (없으면 None)
        lidar_wheel_hz 라이다 조향 발행 주기 [Hz]
        lidar_drive_hz 라이다 구동 발행 주기 [Hz]
        avoid_active   /avoidance/active 값 (없으면 None = 한 번도 못 받음)
    """
    rows = []

    # ---- 1. 매니저가 살아 있나 ----
    if not st.get("mgr_alive"):
        rows.append(("🔴", "매니저 응답 없음",
                     "./실행.sh 로 매니저를 먼저 띄울 것. "
                     "DOMAIN_ID/RMW 가 다르면 서로 안 보인다 "
                     "(source t870_env.sh)."))
        return False, rows
    rows.append(("✅", "매니저 살아 있음", ""))

    # ---- 2. 모드가 5로 들어갔나 ----
    mode = st.get("mgr_mode")
    if mode is None:
        rows.append(("🟡", "모드 확인 중", "/mcu/current_mode 를 기다리는 중"))
    elif mode != TARGET_MODE:
        rows.append(("🔴", "모드가 5가 아니다 (지금 %s)" % mode,
                     "발행 토픽이 매니저의 mode_topic 과 다르거나, "
                     "'5' 가 known_modes 에 없어 거부된 것. "
                     "매니저 로그를 볼 것."))
    else:
        rows.append(("✅", "모드 5 (S 코스)", ""))

    # ---- 3. 조향 권한이 라이다인가 ----
    wsrc = st.get("wheel_src")
    if wsrc is None:
        rows.append(("🟡", "조향 권한 확인 중", ""))
    elif wsrc == "lidar":
        rows.append(("✅", "조향 권한 = lidar", ""))
    elif wsrc == "failsafe":
        rows.append(("🔴", "조향 중앙 고정 (failsafe)",
                     "권한자가 조향값을 안 보낸다. 차는 직진만 한다. "
                     "라이다 노드가 떠 있는지, 토픽 이름·타입이 맞는지 확인."))
    else:
        rows.append(("🔴", "조향 권한이 '%s' 다" % wsrc,
                     "yaml 의 wheel_owner_overrides 에 \"5:lidar\" 가 "
                     "있는지 확인하고, 고쳤으면 colcon build 를 다시 할 것 "
                     "(yaml 은 symlink-install 이어도 다시 빌드해야 한다)."))

    # ---- 3-b. 회피 게이트가 열렸나 ----
    #   이 구간에서 게이트가 참일 때만 중재기가 라이다의
    #   **구동과 조향을 둘 다** 받아들인다. 신호는 라이다가 낸다.
    av = st.get("avoid_active")
    if av is None:
        rows.append(("🔴", "회피 게이트 신호가 안 온다",
                     "라이다 회피 노드가 /avoidance/active 를 발행해야 한다. "
                     "한 번도 못 받으면 중재기가 라이다의 구동·조향을 "
                     "막는다 (안전 우선). 급정거는 그대로 받는다."))
    elif av:
        rows.append(("✅", "회피 게이트 = true (열림)", ""))
    else:
        rows.append(("🔴", "회피 게이트 = false (닫힘)",
                     "라이다가 false 를 보내는 중이다. 중재기가 라이다의 "
                     "구동과 조향을 받지 않는다. 회피 노드가 언제 true 를 "
                     "내는지 확인할 것."))

    # ---- 4. 라이다가 실제로 값을 쏘고 있나 ----
    wh = st.get("lidar_wheel_hz", 0.0)
    if wh >= 1.0:
        rows.append(("✅", "라이다 조향 %.1f Hz" % wh, ""))
    else:
        rows.append(("🔴", "라이다 조향이 안 온다 (%.1f Hz)" % wh,
                     "회피 노드가 떠 있나. 토픽 이름이 맞나. "
                     "ros2 topic hz 로 직접 확인할 것."))

    dh = st.get("lidar_drive_hz", 0.0)
    if dh >= 1.0:
        rows.append(("✅", "라이다 구동 %.1f Hz" % dh, ""))
    else:
        rows.append(("🟡", "라이다 구동이 안 온다 (%.1f Hz)" % dh,
                     "조향만 하고 구동은 다른 팀이 낼 수도 있다. "
                     "차가 안 움직이면 이것부터 볼 것."))

    # ---- 5. 안전 상태 ----
    safety = st.get("safety")
    if safety is None:
        rows.append(("🟡", "안전 상태 확인 중", ""))
    elif safety == "OK":
        rows.append(("✅", "안전 상태 OK", ""))
    elif safety.startswith("EMERGENCY_STOP"):
        rows.append(("🔴", "급정거 걸림 — %s" % safety,
                     "라이다 급정거나 E-Stop 이 잡혀 있다. 조종 화면에서 R."))
    elif "NO_DRIVE_SOURCE" in safety:
        rows.append(("🔴", "구동 명령이 없다 — %s" % safety,
                     "아무도 구동을 안 보내고 있다. 차는 안 움직인다."))
    else:
        rows.append(("🟡", "안전 상태 %s" % safety, ""))

    ok = all(sym == "✅" for sym, _, _ in rows)
    return ok, rows


# ============================================================
# ROS
# ============================================================

def main():
    ap = argparse.ArgumentParser(description="S 코스(5번) 전용 실행기")
    ap.add_argument("--mode-topic", default="/drive_mode")
    ap.add_argument("--mcu-ns", default="/mcu")
    ap.add_argument("--lidar-wheel", default="/lidar_wheel")
    ap.add_argument("--lidar-drive", default="/lidar_drive")
    ap.add_argument("--avoid-topic", default="/avoidance/active")
    ap.add_argument("--hz", type=float, default=5.0,
                    help="모드 발행 주기")
    ap.add_argument("--mode", default=TARGET_MODE,
                    help="발행할 구간 번호 (기본 5 = S 코스)")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, Int32, String

    ns = args.mcu_ns.rstrip("/")

    class SCourse(Node):
        def __init__(self):
            super().__init__("s_course_runner")
            self.st = {"mgr_alive": False, "mgr_mode": None,
                       "wheel_src": None, "drive_src": None, "safety": None,
                       "avoid_active": None,
                       "lidar_wheel_hz": 0.0, "lidar_drive_hz": 0.0}
            self._wt = []          # 라이다 조향 수신 시각
            self._dt = []          # 라이다 구동 수신 시각

            self.pub_mode = self.create_publisher(String, args.mode_topic, 10)

            def sub(topic, key):
                self.create_subscription(
                    String, topic, lambda m, k=key: self._set(k, str(m.data)), 10)

            sub(ns + "/current_mode", "mgr_mode")
            sub(ns + "/active_wheel_source", "wheel_src")
            sub(ns + "/active_drive_source", "drive_src")
            sub(ns + "/safety_state", "safety")

            self.create_subscription(
                Bool, args.avoid_topic,
                lambda m: self._set("avoid_active", bool(m.data)), 10)
            self.create_subscription(
                Int32, args.lidar_wheel, lambda m: self._tick(self._wt), 10)
            self.create_subscription(
                Float32, args.lidar_drive, lambda m: self._tick(self._dt), 10)

            self.create_timer(1.0 / max(args.hz, 0.1), self._pub)
            self.create_timer(0.5, self._draw)

        def _set(self, key, value):
            self.st[key] = value
            self.st["mgr_alive"] = True

        def _tick(self, buf):
            buf.append(time.monotonic())
            if len(buf) > 40:
                del buf[:-40]

        @staticmethod
        def _hz(buf):
            now = time.monotonic()
            recent = [t for t in buf if now - t < 2.0]
            return len(recent) / 2.0 if len(recent) >= 2 else 0.0

        def _pub(self):
            m = String(); m.data = str(args.mode); self.pub_mode.publish(m)

        def _draw(self):
            self.st["lidar_wheel_hz"] = self._hz(self._wt)
            self.st["lidar_drive_hz"] = self._hz(self._dt)
            ok, rows = diagnose(self.st)

            sys.stdout.write("\033[2J\033[H")
            print("=" * 62)
            print("  S 코스 실행기 — '%s' 번 구간을 %s 에 계속 발행"
                  % (args.mode, args.mode_topic))
            print("=" * 62)
            for sym, title, hint in rows:
                print("  %s %s" % (sym, title))
                if hint:
                    for line in _wrap(hint, 54):
                        print("       %s" % line)
            print("-" * 62)
            if ok:
                print("  \033[32m전부 정상. 장애물을 놓고 회피가 도는지 보면 된다.\033[0m")
            else:
                print("  🔴 위의 빨간 항목을 먼저 해결할 것.")
            print("=" * 62)
            print("  Ctrl-C 로 종료 (종료하면 모드 발행이 멈춘다)")

    rclpy.init()
    node = SCourse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\n종료. 모드 발행을 멈췄다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
