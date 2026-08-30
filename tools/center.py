#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""center.py — 앞바퀴 직진 중앙을 찾아서 등록한다. ROS 불필요.

각도(도)가 아니라 **조향모터를 켜는 시간(ms)** 단위로 민다.
A0 값은 실제 각도 센서가 아니므로 판정에 쓰지 않는다.
**차가 실제로 곧게 가는지**만 보고 정한다.

    python3 tools/center.py

── 조향 (미세 조정) ─────────────────────────────
    a / d      1ms   좌 / 우
    A / D      10ms  좌 / 우
    z / x      50ms  좌 / 우

── 주행 (내가 직접 몬다) ────────────────────────
    w          전진 1단 (느림)      계속 간다
    e          전진 2단
    s          후진 1단
    스페이스    정지
    ※ 안전을 위해 10초가 지나면 스스로 멈춘다

── 자동 시험 (손 떼고) ──────────────────────────
    t          5초 직진 시험 (2단). 곧게 가는지 눈으로만 본다

── 중앙 ────────────────────────────────────────
    m      ★  지금 이 위치를 중앙으로 등록 (STEER_ZERO)
    c          등록된 중앙으로 복귀
    r          새로고침       q  종료

찾는 법
    1. t 로 달려본다  →  왼쪽으로 휘면 d, 오른쪽으로 휘면 a
    2. 곧게 갈 때까지 반복 (처음엔 10ms, 가까워지면 1ms)
    3. 곧게 가면  m  →  그 자리가 중앙(누적 0)이 된다
"""
import glob
import sys
import termios
import threading
import time
import tty

import serial

STOP = "1.00"
FWD1, FWD2, REV1 = "2.00", "3.00", "6.00"
TEST_CMD = FWD2
TEST_SEC = 5.0
MANUAL_MAX_SEC = 10.0        # 수동 주행 자동 차단
SEND_HZ = 10.0

port = (glob.glob("/dev/t870_mcu") or sorted(glob.glob("/dev/serial/by-id/*"))
        or sorted(glob.glob("/dev/ttyACM*")) or sorted(glob.glob("/dev/ttyUSB*")))
if not port:
    sys.exit("포트를 못 찾았다. USB 확인.")
ser = serial.Serial(port[0], 115200, timeout=1)
print("포트 %s" % port[0])
time.sleep(2.0)

lock = threading.Lock()
box = {"net": None, "adc": None, "cnt": None, "st": "?",
       "rx": 0, "tele": None, "msg": "", "alive": True}
drive = {"cmd": STOP, "until": 0.0, "base": None, "label": "정지"}


def send(t):
    with lock:
        try:
            ser.write((t + "\n").encode()); ser.flush()
        except Exception:
            pass


def rx():
    while box["alive"]:
        try:
            line = ser.readline().decode("utf8", "replace").strip()
        except Exception:
            return
        if not line:
            continue
        if line.startswith("TELEMETRY,"):
            box["tele"] = line.split(",")[1].strip().upper() == "ON"
        elif line.startswith("STEER_OK,"):
            f = line.split(",")
            if len(f) >= 3:
                try:
                    box["net"] = int(f[1]); box["adc"] = int(f[2])
                except ValueError:
                    pass
            box["msg"] = line
        elif line.startswith("STEER_ZERO,"):
            box["net"] = 0
            try:
                box["adc"] = int(line.split(",")[1])
            except ValueError:
                pass
            box["msg"] = "★ 중앙 등록됨 — " + line
        elif line.startswith("STEER_LIMIT_REACHED"):
            box["msg"] = "끝까지 갔다 — " + line
        elif line.startswith("STATUS"):
            f = line.split(",")
            if len(f) > 8:
                try:
                    box["st"] = f[1]
                    box["adc"] = int(float(f[3]))
                    box["cnt"] = int(float(f[7]))
                    box["net"] = int(float(f[8]))
                    box["rx"] += 1
                except ValueError:
                    pass


def tx():
    """구동 명령을 계속 보낸다. 워치독이 2초라 끊기면 차가 선다."""
    while box["alive"]:
        now = time.time()
        if drive["cmd"] != STOP and now > drive["until"]:
            drive["cmd"] = STOP
            drive["label"] = "정지 (시간 초과)"
        send(drive["cmd"])
        time.sleep(1.0 / SEND_HZ)


threading.Thread(target=rx, daemon=True).start()
threading.Thread(target=tx, daemon=True).start()


def poll_once(timeout=0.6):
    before = box["rx"]; send("S")
    end = time.time() + timeout
    while box["rx"] == before and time.time() < end:
        time.sleep(0.01)


def set_telemetry(want):
    box["tele"] = None; send("Q")
    end = time.time() + 1.0
    while box["tele"] is None and time.time() < end:
        time.sleep(0.02)
    if box["tele"] is None:
        return False
    if box["tele"] != want:
        box["tele"] = None; send("Q")
        end = time.time() + 1.0
        while box["tele"] is None and time.time() < end:
            time.sleep(0.02)
    return box["tele"] == want


poll_once()
if box["net"] is None:
    box["alive"] = False
    sys.exit("STATUS 응답 없음. 아두이노 전원/USB 확인.")
auto = set_telemetry(True)


def moved():
    if drive["base"] is None or box["cnt"] is None:
        return 0
    return box["cnt"] - drive["base"]


def draw():
    sys.stdout.write("\033[2J\033[H")
    print("=" * 62)
    print("  T870 조향 중앙 찾기")
    print("=" * 62)
    print("  누적 조향   %s ms      (우 +, 좌 -)"
          % ("%+d" % box["net"] if box["net"] is not None else "?"))
    print("  A0 참고값   %s          (각도 센서 아님. 판정에 쓰지 말 것)"
          % box["adc"])
    print("  구동        %-16s  카운트 %+d (약 %.2f m)"
          % (drive["label"], moved(), moved() / 199.8))
    print("  상태        %s" % box["st"])
    print("-" * 62)
    print("  조향   a/d 1ms    A/D 10ms    z/x 50ms   (좌/우)")
    print("  주행   w 전진1단   e 전진2단   s 후진1단   [스페이스] 정지")
    print("  시험   t 5초 직진 시험 (손 떼고 자동)")
    print("  중앙   m ★등록     c 복귀      r 새로고침   q 종료")
    print("-" * 62)
    print("  %s" % box["msg"])
    print()
    print("  왼쪽으로 휘면 d, 오른쪽으로 휘면 a. 곧게 가면 m.")


def go(cmd, label):
    drive["base"] = box["cnt"]
    drive["cmd"] = cmd
    drive["label"] = label
    drive["until"] = time.time() + MANUAL_MAX_SEC
    box["msg"] = "%s — 스페이스로 정지 (최대 %.0f초)" % (label, MANUAL_MAX_SEC)


def halt():
    drive["cmd"] = STOP
    drive["label"] = "정지"
    box["msg"] = "정지"


def test_drive():
    halt()
    sys.stdout.write("\033[2J\033[H")
    print("5초 직진 시험. 앞이 비었는지 확인. 멈추려면 Ctrl-C")
    for i in (3, 2, 1):
        print("  %d..." % i); time.sleep(1.0)
    base = box["cnt"] or 0
    drive["base"] = base
    drive["cmd"] = TEST_CMD
    drive["label"] = "시험 주행"
    t0 = time.time()
    try:
        while time.time() - t0 < TEST_SEC:
            drive["until"] = time.time() + 1.0
            sys.stdout.write("\r  %.1fs  카운트 %6d "
                             % (time.time() - t0, (box["cnt"] or 0) - base))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    halt()
    time.sleep(2.0)
    poll_once()
    d = (box["cnt"] or 0) - base
    box["msg"] = ("시험 %d 카운트 (약 %.2f m). 어느 쪽으로 휘었나?"
                  % (d, d / 199.8))
    print("\n  끝. 아무 키나 누르면 돌아간다.")
    sys.stdin.read(1)


KEYS = {"a": ("A", 1), "d": ("D", 1),
        "A": ("A", 10), "D": ("D", 10),
        "z": ("A", 50), "x": ("D", 50)}

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
try:
    tty.setcbreak(fd)
    draw()
    while True:
        k = sys.stdin.read(1)
        if k == "q":
            break
        elif k == " ":
            halt()
        elif k == "w":
            go(FWD1, "전진 1단")
        elif k == "e":
            go(FWD2, "전진 2단")
        elif k == "s":
            go(REV1, "후진 1단")
        elif k in KEYS:
            side, ms = KEYS[k]
            send("%s%d" % (side, ms))
            box["msg"] = "%s%d 전송" % (side, ms)
            time.sleep(0.25); poll_once()
        elif k == "m":
            halt(); send("M"); time.sleep(0.25); poll_once()
        elif k == "c":
            halt(); send("C"); box["msg"] = "중앙 복귀"
            time.sleep(0.8); poll_once()
        elif k == "t":
            test_drive()
        elif k == "r":
            poll_once()
        draw()
except KeyboardInterrupt:
    pass
finally:
    halt()
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    time.sleep(0.4)
    box["alive"] = False
    for _ in range(5):
        send(STOP); time.sleep(0.05)
    if auto:
        set_telemetry(False)
    try:
        ser.close()
    except Exception:
        pass
    print("\n종료. 등록한 중앙은 아두이노 전원을 끄면 사라진다.")
