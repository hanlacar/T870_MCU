#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mode_sim_v2_0831.py  —  모드 시뮬레이터 + 중재 테스트 하네스

키보드로 구간 모드를 바꿔가며 중재기(manager_node)가 제대로 동작하는지 본다.

🔴 0831 에 고친 것 — 이전 버전은 아예 동작하지 않았다
  ① 발행 토픽 기본값이 "/vehicle_mode" 였다.
     0829 에 매니저를 "/drive_mode" 로 옮겼는데 이 파일이 뒤에 남았다.
     매니저가 한 글자도 못 받아서, 눌러도 모드가 안 바뀌었다.
  ② 모드 목록이 낡아 있었다. ACCELERATION(→ACCEL), MANUAL(없는 모드)이
     남아 있고 S_COURSE / D_COURSE / OUT 은 아예 키가 없었다.
     그래서 S자 코스를 키보드로 넣을 방법이 없었다.
  ③ 조향 권한 표시를 코드에 박아뒀다("주차면 lidar"). yaml 을 고쳐도
     화면은 옛 규칙을 보여줬다. 이제 매니저가 실제로 쓰는 값
     (/mcu/active_wheel_source)을 그대로 보여준다.
  ④ 보낸 모드와 매니저가 받아들인 모드를 나란히 띄운다.
     둘이 다르면 화면에 크게 뜬다 — 토픽·이름 불일치를 즉시 알 수 있다.

각 팀 노드가 없어도 카메라/라이다/GPS 명령을 흉내낼 수 있어서,
'주차 모드에서만 라이다가 조향을 가져가는가' 같은 규칙을 혼자 검증할 수 있다.

실행 (터미널):
    source /opt/ros/jazzy/setup.bash
    source <워크스페이스>/install/setup.bash
    python3 mode_sim.py

※ manager 가 떠 있어야 한다:
    ros2 launch t870_mcu t870_mcu.launch.py

※ 차를 안 움직이고 로직만 보려면 브릿지 없이 매니저만:
    ros2 launch t870_mcu t870_mcu.launch.py bridge:=false
"""

import select
import argparse
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


#  키 → (발행할 모드 문자열, 설명)
#
#  0831 부터 모드는 **경로 순번**이다. 이름이 아니다.
#  카메라가 구간마다 다른 로직을 쓰는데, 이름 방식에서는 NORMAL 이
#  경로에 5번 나와서 몇 번째인지 구분할 수 없었다. 번호는 그 자체가
#  구간 식별자라 그 문제가 없다.
#
#  ⚠ 이 표는 yaml 의 known_modes 와 일치해야 한다.
#     tools/audit.py 가 자동으로 대조한다 (0831 추가).
MODES = {
    "0": ("0",  "대기 (출발 전)"),
    "1": ("1",  "출발 직선"),
    "2": ("2",  "경사로"),
    "3": ("3",  "D 코스  직각·굴절"),
    "4": ("4",  "교차로"),
    "5": ("5",  "S 코스  곡선 + 장애물     ★ 라이다 조향"),
    "6": ("6",  "교차로"),
    "7": ("7",  "T 주차                   ★ 라이다 조향"),
    "8": ("8",  "좌회전"),
    "9": ("9",  "가속 구간"),
    "a": ("10", "평행 주차                ★ 라이다 조향"),
    "b": ("11", "마지막 주행 (종점까지)"),
}

SIM_SOURCES = ["camera", "lidar", "gps"]


class ModeSim(Node):

    def __init__(self, mcu_ns="/mcu", mode_topic="/drive_mode",
                 estop_topic="/estop_lock"):
        # 토픽 이름은 전부 인자로 받는다. 코드에 박아두면 팀마다 이름이
        # 다를 때 파일을 고쳐야 한다.
        ns = mcu_ns.rstrip("/")
        super().__init__("mode_sim")

        self.mode = "IDLE"

        # 시뮬레이션 상태: 각 소스가 발행 중인가
        self.sim_on = {s: False for s in SIM_SOURCES}
        self.sim_drive = {s: 1.0 for s in SIM_SOURCES}
        self.sim_wheel = {s: 0 for s in SIM_SOURCES}
        self.sim_stop = False
        self.estop = False

        # 매니저 출력 감시
        self.out_drive = 0.0
        self.out_wheel = 0
        self.out_stop = False
        self.act_drive_src = "?"
        self.act_wheel_src = "?"
        self.safety = "?"
        self.mgr_mode = "?"          # 매니저가 실제로 받아들인 모드
        self.last_rx = 0.0

        # ---- 발행 ----
        self.pub_mode = self.create_publisher(String, mode_topic, 10)
        self.pub_estop = self.create_publisher(Bool, estop_topic, 10)
        self.pub_d = {s: self.create_publisher(Float32, "/%s_drive" % s, 10)
                      for s in SIM_SOURCES}
        self.pub_w = {s: self.create_publisher(Int32, "/%s_wheel" % s, 10)
                      for s in SIM_SOURCES}
        # 급정거는 라이다 하나만 시뮬레이션한다 (매니저 stop_sources 기준)
        self.pub_s = self.create_publisher(Bool, "/lidar_stop", 10)

        # ---- 구독 (매니저 출력) ----
        self.create_subscription(Float32, ns + "/cmd_drive", self._cb_d, 10)
        self.create_subscription(Int32, ns + "/cmd_wheel", self._cb_w, 10)
        self.create_subscription(Bool, ns + "/cmd_stop", self._cb_s, 10)
        self.create_subscription(
            String, ns + "/active_drive_source", self._cb_ads, 10)
        self.create_subscription(
            String, ns + "/active_wheel_source", self._cb_aws, 10)
        self.create_subscription(String, ns + "/safety_state", self._cb_safety, 10)
        #  ★ 보낸 모드가 실제로 먹혔는지 보려면 이게 있어야 한다
        self.create_subscription(String, ns + "/current_mode", self._cb_cmode, 10)

        self.create_timer(0.1, self._publish_loop)   # 10Hz — 타임아웃 0.5s 충족

    # ---------- 구독 콜백 ----------

    def _cb_d(self, m):
        self.out_drive = float(m.data); self.last_rx = time.monotonic()

    def _cb_w(self, m):
        self.out_wheel = int(m.data)

    def _cb_s(self, m):
        self.out_stop = bool(m.data)

    def _cb_ads(self, m):
        self.act_drive_src = str(m.data)

    def _cb_aws(self, m):
        self.act_wheel_src = str(m.data)

    def _cb_safety(self, m):
        self.safety = str(m.data)

    def _cb_cmode(self, m):
        self.mgr_mode = str(m.data)

    # ---------- 주기 발행 ----------

    def _publish_loop(self):
        m = String(); m.data = self.mode; self.pub_mode.publish(m)

        for s in SIM_SOURCES:
            if self.sim_on[s]:
                d = Float32(); d.data = float(self.sim_drive[s])
                self.pub_d[s].publish(d)
                w = Int32(); w.data = int(self.sim_wheel[s])
                self.pub_w[s].publish(w)

        b = Bool(); b.data = self.sim_stop; self.pub_s.publish(b)
        if self.estop:
            e = Bool(); e.data = True; self.pub_estop.publish(e)


# ============================================================
# 화면
# ============================================================

def render(node):
    out = []
    w = out.append
    w("\033[2J\033[H")
    w("┌─ 모드 시뮬레이터 ─ 중재 테스트 ──────────────────────────")
    w("│")

    # 모드 — 보낸 값과 매니저가 받아들인 값을 나란히
    desc = next((d for k, (mm, d) in MODES.items() if mm == node.mode), "")
    w("│  보낸 모드   \033[1;36m%-6s\033[0m %s" % (node.mode, desc))

    if node.mgr_mode == "?":
        w("│  매니저 모드 \033[90m아직 수신 없음\033[0m  "
          "(매니저가 떠 있나? 토픽 이름이 맞나?)")
    elif node.mgr_mode != node.mode:
        w("│  매니저 모드 \033[1;31m%-6s ← 다르다!\033[0m" % node.mgr_mode)
        w("│    보낸 값이 안 먹었다. 원인: 토픽 이름 불일치, 또는")
        w("│    known_modes 에 없는 값이라 거부됨(policy=keep 이면 직전 유지).")
        w("│    매니저 로그를 볼 것.")
    else:
        w("│  매니저 모드 \033[32m%-6s ✓\033[0m" % node.mgr_mode)

    # 조향 권한 — 매니저가 실제로 쓰는 값. 추측하지 않는다.
    owner = node.act_wheel_src
    note = ""
    if owner == "failsafe":
        note = "  \033[31m← 권한자가 값을 안 준다. 조향 중앙 고정\033[0m"
    w("│  조향 권한   \033[33m%s\033[0m%s" % (owner, note))
    w("│")

    # 시뮬 소스
    w("│  ── 소스 시뮬레이션 ──────────────────────────────")
    for s in SIM_SOURCES:
        mark = "\033[32m● ON \033[0m" if node.sim_on[s] else "\033[90m○ off\033[0m"
        gate = ""
        if not node.sim_on[s]:
            pass
        elif s == owner:
            gate = "  \033[33m← 조향 권한\033[0m"
        elif s in ("lidar", "camera"):
            gate = "  \033[90m(조향 무시됨)\033[0m"
        w("│   %-7s %s  drive=%+.0f  wheel=%+3d%s"
          % (s, mark, node.sim_drive[s], node.sim_wheel[s], gate))
    stop_mark = "\033[31m● STOP\033[0m" if node.sim_stop else "\033[90m○ off\033[0m"
    w("│   lidar_stop %s" % stop_mark)
    es_mark = "\033[31m● LATCH\033[0m" if node.estop else "\033[90m○ off\033[0m"
    w("│   estop_lock %s" % es_mark)
    w("│")

    # 매니저 출력
    age = time.monotonic() - node.last_rx if node.last_rx else 999
    if age > 1.0:
        w("│  ── 매니저 출력 ─── \033[31m수신 없음 (매니저 실행 중인가?)\033[0m")
    else:
        w("│  ── 매니저 출력 ─────────────────────────────────")
    w("│   /mcu/cmd_drive  \033[1m%+.1f\033[0m      /mcu/cmd_wheel  \033[1m%+d\033[0m      /mcu/cmd_stop  %s"
      % (node.out_drive, node.out_wheel,
         "\033[31mTRUE\033[0m" if node.out_stop else "false"))
    w("│   구동 소스   \033[36m%-10s\033[0m 조향 소스  \033[36m%s\033[0m"
      % (node.act_drive_src, node.act_wheel_src))

    safety = node.safety
    if safety.startswith("OK"):
        safety = "\033[32m%s\033[0m" % safety
    elif safety != "?":
        safety = "\033[31m%s\033[0m" % safety
    w("│   안전 상태   %s" % safety)
    w("│")

    # 키 안내
    w("│  ── 조작 ────────────────────────────────────────")
    w("│   0~9,a,b  모드 전환 (a=10  b=11)")
    w("│   c/l/g 카메라/라이다/GPS 발행 ON·OFF")
    w("│   C/L/G 그 소스의 drive 단계 순환 (0→1→2→3→-1)")
    w("│   ←  →  선택된 소스의 조향각 ±5도    t  조향 대상 전환")
    w("│   s     lidar_stop 토글 (급정거)")
    w("│   e     estop_lock 토글 (래치)")
    w("│   q     종료")
    w("│")
    w("│   조향 편집 대상: \033[33m%s\033[0m" % node.wheel_target)
    w("└──────────────────────────────────────────────────────────")

    sys.stdout.write("\r\n".join(out) + "\r\n")
    sys.stdout.flush()


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    r, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not r:
        return "ESC"
    seq = sys.stdin.read(2)
    return {"[C": "RIGHT", "[D": "LEFT"}.get(seq)


DRIVE_CYCLE = [0.0, 1.0, 2.0, 3.0, -1.0]


def handle_key(node, k):
    if k in MODES:
        node.mode = MODES[k][0]

    elif k == "c":
        node.sim_on["camera"] = not node.sim_on["camera"]
    elif k == "l":
        node.sim_on["lidar"] = not node.sim_on["lidar"]
    elif k == "g":
        node.sim_on["gps"] = not node.sim_on["gps"]

    elif k in ("C", "L", "G"):
        src = {"C": "camera", "L": "lidar", "G": "gps"}[k]
        cur = node.sim_drive[src]
        idx = DRIVE_CYCLE.index(cur) if cur in DRIVE_CYCLE else 0
        node.sim_drive[src] = DRIVE_CYCLE[(idx + 1) % len(DRIVE_CYCLE)]

    elif k == "t":
        i = SIM_SOURCES.index(node.wheel_target)
        node.wheel_target = SIM_SOURCES[(i + 1) % len(SIM_SOURCES)]
    elif k == "LEFT":
        t = node.wheel_target
        node.sim_wheel[t] = max(-27, node.sim_wheel[t] - 5)
    elif k == "RIGHT":
        t = node.wheel_target
        node.sim_wheel[t] = min(27, node.sim_wheel[t] + 5)

    elif k == "s":
        node.sim_stop = not node.sim_stop
    elif k == "e":
        node.estop = not node.estop


def main():
    ap = argparse.ArgumentParser(
        description="모드 전환 시뮬레이션 — 차 없이 중재 로직만 확인")
    ap.add_argument("--mcu-ns", default="/mcu",
                    help="MCU 발행 토픽 접두어")
    #  🔴 매니저의 mode_topic 과 반드시 같아야 한다.
    #     0829 에 /vehicle_mode → /drive_mode 로 옮겼다.
    #     (GPS팀이 /vehicle_mode 를 Int32 로 쓰고 있어 이름이 겹쳤다)
    ap.add_argument("--mode-topic", default="/drive_mode")
    ap.add_argument("--estop-topic", default="/estop_lock")
    args = ap.parse_args()

    rclpy.init()
    node = ModeSim(args.mcu_ns, args.mode_topic, args.estop_topic)
    node.wheel_target = "camera"

    threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True).start()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    last_draw = 0.0
    try:
        tty.setraw(fd)
        while True:
            k = read_key(0.05)
            if k in ("q", "Q", "\x03"):
                break
            if k:
                handle_key(node, k)
            now = time.monotonic()
            if now - last_draw > 0.15:
                render(node)
                last_draw = now
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        # 종료 시 정지 상태로
        node.sim_on = {s: False for s in SIM_SOURCES}
        node.sim_stop = False
        node.mode = "IDLE"
        for _ in range(5):
            node._publish_loop()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
        print("\n모드 IDLE, 소스 전부 OFF 로 두고 종료했습니다.")


if __name__ == "__main__":
    main()
