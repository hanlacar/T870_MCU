#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run5_v3_0830.py — 2단으로 딱 5초 전진하고 멈춘다. 조향은 절대 건드리지 않는다.

    python3 tools/run5_v3_0830.py

★ 조향 보호 (이 도구의 핵심)
  - 조향 명령(A/D/C/M/L/R/V/W)을 **한 개도 보내지 않는다**
  - 시작 전과 끝난 뒤 조향 누적을 찍어 **안 변했는지 눈으로 확인**시킨다

  🔴 v3: v2 는 리셋을 막으려고 DTR/RTS 를 내린 채로 열었는데,
     보드에 따라 그 상태면 아두이노가 리셋에 물려 한 줄도 못 보낸다.
     실제로 10초 내내 무응답이었다. 그냥 평범하게 연다.
     리셋이 걸려도 무해하다 —
       · 바퀴는 물리적으로 움직이지 않는다 (setup 은 조향모터를 끄기만 한다)
       · steerNetMs 가 0 으로 잡히는데, 손으로 등록한 중앙도 0 이라 같다

회선 부하
  펌웨어 자동 발행(Q, 500ms)을 켜서 주행 중에는 아무것도 묻지 않는다.
  구간 경계에서만 S 를 한 번 쏜다. → 주행 중 회선 = 구동 명령 10Hz 뿐.
"""
import glob
import sys
import threading
import time

import serial

STOP = "1.00"
DRIVE = "3.00"          # 전진 2단
RUN_SEC = 5.0
COAST_SEC = 3.0
SEND_HZ = 10.0
CPM = 199.8             # 표시용. 실제 값은 줄자로 정한다

port = (glob.glob("/dev/t870_mcu") or sorted(glob.glob("/dev/serial/by-id/*"))
        or sorted(glob.glob("/dev/ttyACM*")) or sorted(glob.glob("/dev/ttyUSB*")))
if not port:
    sys.exit("포트를 못 찾았다. USB 확인.")

#  다른 도구(center.py, run10.py)와 똑같이 평범하게 연다.
try:
    ser = serial.Serial(port[0], 115200, timeout=1)
except Exception as exc:
    sys.exit("포트를 못 열었다: %s\n  다른 프로그램이 잡고 있는지 확인:\n"
             "    pgrep -af 'mcu_bridge|center|run10|serial_console'" % exc)
print("포트 %s" % port[0])
print("  아두이노 부팅 대기...")
time.sleep(2.0)

lock = threading.Lock()
box = {"cnt": None, "net": None, "adc": None, "st": "?", "rx": 0,
       "tele": None, "booted": None}


def send(t):
    with lock:
        try:
            ser.write((t + "\n").encode()); ser.flush()
        except Exception:
            pass


def rx():
    while True:
        try:
            line = ser.readline().decode("utf8", "replace").strip()
        except Exception:
            return
        if not line:
            continue
        if line.startswith("MCU_BOOT"):
            box["booted"] = line          # 리셋이 걸렸다는 증거
        if line.startswith("TELEMETRY,"):
            box["tele"] = line.split(",")[1].strip().upper() == "ON"
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


threading.Thread(target=rx, daemon=True).start()


def fresh(fallback=None):
    before = box["rx"]
    send("S")
    end = time.time() + 0.7
    while box["rx"] == before and time.time() < end:
        time.sleep(0.01)
    return box["cnt"] if box["cnt"] is not None else fallback


def set_telemetry(want):
    box["tele"] = None
    send("Q")
    end = time.time() + 1.0
    while box["tele"] is None and time.time() < end:
        time.sleep(0.02)
    if box["tele"] is None:
        return False
    if box["tele"] != want:
        box["tele"] = None
        send("Q")
        end = time.time() + 1.0
        while box["tele"] is None and time.time() < end:
            time.sleep(0.02)
    return box["tele"] == want


def wait_status(seconds=10.0):
    """S 를 계속 물어보며 STATUS 가 올 때까지 기다린다.

    🔴 v2: v1 은 S 를 딱 한 번 쏘고 0.7초 만에 포기했다.
      리셋이 걸리면 부팅에 2초 넘게 걸리므로 그 사이에 실패했다.
      리셋이 걸리든 안 걸리든 되게 만든다.
    """
    end = time.time() + seconds
    dots = 0
    while time.time() < end:
        send("S")
        t = time.time() + 0.3
        while time.time() < t:
            if box["cnt"] is not None:
                return True
            time.sleep(0.02)
        dots += 1
        sys.stdout.write("\r  응답 기다리는 중%s   " % ("." * (dots % 4)))
        sys.stdout.flush()
    return False


if not wait_status(10.0):
    print()
    sys.exit("STATUS 응답 없음 (10초).\n"
             "  확인:\n"
             "    1) 아두이노 USB 케이블\n"
             "    2) 다른 프로그램이 포트를 잡고 있나 —\n"
             "       pgrep -af 'mcu_bridge|center|run10|serial_console'\n"
             "    3) 아두이노 IDE 시리얼 모니터가 열려 있나")
print("\r" + " " * 40 + "\r", end="")
if box["booted"]:
    print("  ※ 포트를 열며 리셋됨 (%s). 바퀴는 안 움직였고,"
          % box["booted"])
    print("    조향 누적도 0 = 네가 등록한 중앙과 같은 값이다.")

net_before = box["net"]
print("  카운트 %d / 조향누적 %+d ms / A0 %s / 상태 %s"
      % (box["cnt"], net_before, box["adc"], box["st"]))
auto = set_telemetry(True)

print("\n2단으로 %.0f초 전진한다. 조향은 건드리지 않는다." % RUN_SEC)
print("앞이 비었는지 확인. 멈추려면 Ctrl-C")
for i in (3, 2, 1):
    print("  %d..." % i)
    time.sleep(1.0)

base = box["cnt"]
t0 = time.time()
try:
    while time.time() - t0 < RUN_SEC:
        send(DRIVE)
        sys.stdout.write("\r  %4.1fs  카운트 %7d  %-8s"
                         % (time.time() - t0, (box["cnt"] or base) - base,
                            box["st"]))
        sys.stdout.flush()
        time.sleep(1.0 / SEND_HZ)
    run_time = time.time() - t0
except KeyboardInterrupt:
    run_time = time.time() - t0
    print("\n  [중단]")

send(STOP)
stop_at = fresh(base)
run = stop_at - base

print("\n  정지. 코스팅 %.0f초..." % COAST_SEC)
end = time.time() + COAST_SEC
while time.time() < end:
    send(STOP)
    time.sleep(1.0 / SEND_HZ)
coast = fresh(stop_at) - stop_at

if auto:
    set_telemetry(False)
for _ in range(5):
    send(STOP)
    time.sleep(0.05)

net_after = box["net"]

print("\n" + "=" * 56)
print("  주행 카운트   %d" % run)
print("  주행 시간     %.2f 초" % run_time)
print("  코스팅        %d 카운트" % coast)
print("  --------------------------------------")
print("  %.1f 기준     %.2f m   %.3f m/s"
      % (CPM, run / CPM, run / CPM / run_time if run_time else 0))
print("  ★ 줄자로 잰 실제 거리 D 라면  counts_per_meter = %d / D" % run)
print("  --------------------------------------")
print("  조향 누적     %+d ms  →  %+d ms   %s"
      % (net_before, net_after,
         "✅ 안 변했다" if net_before == net_after else "🔴 변했다!"))
print("=" * 56)

ser.close()
