#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""owner_v1_0902.py — 현장에서 조향 권한을 즉시 바꾼다.

    python3 tools/owner_v1_0902.py

왜 필요한가
-----------
9/6 현장 계획에 "카메라 or GPS + LIDAR" 처럼 아직 안 정해진 구간이 있다
(5번 S자, 7번 T주차, 9번 가속 후 좌회전). 둘 다 돌려보고 정하는 건데,
그때마다 yaml 고치고 colcon build 하고 노드를 다시 띄우면 한 번에 몇 분씩
날아간다. 현장에서는 그 시간이 없다.

이 도구는 /mcu/set_wheel_owner 로 한 줄 쏴서 즉시 바꾼다. 재시작 불필요.

⚠ 여기서 바꾼 것은 노드를 다시 띄우면 사라진다.
  확정되면 config/t870_mcu.yaml 의 wheel_owner_overrides 에 옮겨 적을 것.

키
--
    5 7 9      그 모드의 권한자를 순환 (camera → lidar → gps → 기본)
    0~11 은 두 자리도 됨 (1 0 을 연달아 치면 10번)
    t          지금 표 보기
    r          전부 기본값으로 되돌리기
    q          종료

터미널에서 직접 쏘고 싶으면:
    ros2 topic pub --once /mcu/set_wheel_owner std_msgs/msg/String "{data: '5:lidar'}"
    ros2 topic pub --once /mcu/set_wheel_owner std_msgs/msg/String "{data: '5:'}"
    ros2 topic echo /mcu/wheel_owner_table
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SET_TOPIC = "/mcu/set_wheel_owner"
TABLE_TOPIC = "/mcu/wheel_owner_table"
MODE_TOPIC = "/mcu/current_mode"
WHEEL_SRC_TOPIC = "/mcu/active_wheel_source"

#  순환 순서. 마지막 ""(빈 값) 은 yaml 기본값으로 되돌리기.
CYCLE = ["camera", "lidar", "gps", ""]

#  9/6 현장 계획 (팀 화이트보드)
PLAN = {
    "0": "대기",
    "1": "출발~경사로 전       카메라",
    "2": "경사로               카메라",
    "3": "직각코너~정지선      LIDAR + 카메라",
    "4": "정지선~교차로        카메라 + GPS (NAV2)",
    "5": "S자              ★  카메라 or GPS + LIDAR",
    "6": "정지선~T주차 우회전  4와 동일",
    "7": "T주차            ★  Nav2 or GPS + LIDAR",
    "8": "T주차~정지선         카메라 + GPS",
    "9": "가속 후 좌회전       카메라 or GPS + LIDAR",
    "10": "평행주차            7과 동일",
    "11": "출차                카메라 + GPS",
}


class OwnerTool(Node):
    def __init__(self):
        super().__init__("owner_tool")
        self.pub = self.create_publisher(String, SET_TOPIC, 10)
        self.table = "(아직 못 받음 — 아무 키나 눌러 한 번 바꿔보면 온다)"
        self.mode = "?"
        self.wsrc = "?"
        self.idx = {}
        self.create_subscription(String, TABLE_TOPIC, self._cb_table, 10)
        self.create_subscription(String, MODE_TOPIC, self._cb_mode, 10)
        self.create_subscription(String, WHEEL_SRC_TOPIC, self._cb_wsrc, 10)

    def _cb_table(self, msg):
        self.table = str(msg.data)

    def _cb_mode(self, msg):
        self.mode = str(msg.data)

    def _cb_wsrc(self, msg):
        self.wsrc = str(msg.data)

    def cycle(self, mode):
        i = (self.idx.get(mode, -1) + 1) % len(CYCLE)
        self.idx[mode] = i
        source = CYCLE[i]
        self.pub.publish(String(data="%s:%s" % (mode, source)))
        return source or "(기본값)"

    def reset_all(self):
        for mode in PLAN:
            self.pub.publish(String(data="%s:" % mode))
        self.idx.clear()


def draw(node, msg=""):
    sys.stdout.write("\033[H\033[J")
    print("┌─ 조향 권한 전환 (9/6 현장용) ────────────────────────────")
    print("│")
    print("│  지금 모드 %-6s   조향 소스 %s" % (node.mode, node.wsrc))
    print("│")
    print("│  권한표")
    for line in _wrap(node.table, 54):
        print("│    %s" % line)
    print("│")
    print("│  9/6 계획")
    for mode in sorted(PLAN, key=lambda m: int(m)):
        print("│    %-3s %s" % (mode, PLAN[mode]))
    print("│")
    print("│  키   5 7 9 … 모드 번호를 누르면 권한자가 순환한다")
    print("│       (camera → lidar → gps → 기본값)")
    print("│       두 자리는 연달아: 1 다음 0 = 10번")
    print("│       t 표 새로고침   r 전부 기본값   q 종료")
    if msg:
        print("│")
        print("│  \033[33m%s\033[0m" % msg)
    print("└──────────────────────────────────────────────────────────")


def _wrap(text, width):
    words, line, out = text.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > width:
            out.append(line)
            line = w
        else:
            line = (line + " " + w).strip()
    if line:
        out.append(line)
    return out or [""]


def main():
    rclpy.init()
    node = OwnerTool()
    old = termios.tcgetattr(sys.stdin)
    pending = ""
    msg = ""
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            draw(node, msg)

            import select
            if not select.select([sys.stdin], [], [], 0.2)[0]:
                continue
            key = sys.stdin.read(1)

            if key == "q":
                break
            if key == "t":
                msg = "표 갱신을 기다린다 (권한을 한 번 바꾸면 즉시 온다)"
                continue
            if key == "r":
                node.reset_all()
                msg = "전부 yaml 기본값으로 되돌렸다"
                pending = ""
                continue

            if not key.isdigit():
                continue

            #  두 자리 처리: 1 을 누르고 0.6초 안에 0 또는 1 이 오면 10/11
            cand = pending + key
            if pending and cand in PLAN:
                mode, pending = cand, ""
            elif key == "1":
                pending = "1"
                msg = "1… 10/11 이면 다음 숫자를 이어서, 1번이면 잠시 기다릴 것"
                continue
            else:
                mode, pending = key, ""

            if mode not in PLAN:
                msg = "없는 모드: %s" % mode
                continue
            msg = "모드 %s → %s" % (mode, node.cycle(mode))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n종료했습니다. 바꾼 권한은 매니저를 다시 띄우면 사라집니다.")
        print("확정된 설정은 config/t870_mcu.yaml 의 wheel_owner_overrides 에 적으세요.")


if __name__ == "__main__":
    main()
