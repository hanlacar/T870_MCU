#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""push_v1_0830.py — 모터 끄고 손으로 밀어서 counts_per_meter 를 잰다.

    python3 tools/push_v1_0830.py

왜 이걸 하나
  주행으로 잰 값이 회차마다 674 / 782 / 852 로 흔들렸다. 원인 후보가 셋인데
  손밀기 한 번이면 셋 다 갈린다.

    슬립      모터가 안 도니 바퀴가 헛돌 수 없다
    노이즈    모터 전류가 없으니 엔코더 선에 실릴 것이 없다
    툴 차이   구동 명령을 아예 안 보낸다. STATUS 만 읽는다

  ★ 구동 명령(2.00/3.00/...)도 조향 명령(A/D/C/M)도 **한 개도 안 보낸다.**
    S 만 물어본다.

★ 앞으로만 밀 것
  구동 엔코더는 A상 하나뿐이라 방향을 모른다. 펌웨어가 '마지막 명령 방향'
  으로 부호를 붙이므로(v37 1792줄), 뒤로 밀면 부호가 거짓말을 한다.

쓰는 법
  1. 출발선에 세우고 Enter
  2. 앞으로 민다 (직선)
  3. 도착선에서 Enter
  4. 줄자로 잰 거리 입력
  5. 3번 반복하고 Ctrl-C
"""
import glob
import sys
import threading
import time

import serial

port = (glob.glob("/dev/t870_mcu") or sorted(glob.glob("/dev/serial/by-id/*"))
        or sorted(glob.glob("/dev/ttyACM*")) or sorted(glob.glob("/dev/ttyUSB*")))
if not port:
    sys.exit("포트를 못 찾았다. USB 확인.")
try:
    ser = serial.Serial(port[0], 115200, timeout=1)
except Exception as exc:
    sys.exit("포트를 못 열었다: %s\n"
             "  pgrep -af 'mcu_bridge|center|run5|run10|serial_console'" % exc)
print("포트 %s" % port[0])
print("  아두이노 부팅 대기...")
time.sleep(2.0)

lock = threading.Lock()
box = {"cnt": None, "rx": 0, "live": False, "base": 0}


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
        if line.startswith("STATUS"):
            f = line.split(",")
            if len(f) > 7:
                try:
                    box["cnt"] = int(float(f[7])); box["rx"] += 1
                except ValueError:
                    pass


def poll():
    while True:
        send("S")
        time.sleep(0.1)


def show():
    while True:
        if box["live"]:
            sys.stdout.write("\r  카운트 %8d " % ((box["cnt"] or 0) - box["base"]))
            sys.stdout.flush()
        time.sleep(0.1)


for f in (rx, poll, show):
    threading.Thread(target=f, daemon=True).start()


def fresh():
    before = box["rx"]
    send("S")
    end = time.time() + 0.7
    while box["rx"] == before and time.time() < end:
        time.sleep(0.01)
    return box["cnt"]


end = time.time() + 10.0
while box["cnt"] is None and time.time() < end:
    time.sleep(0.05)
if box["cnt"] is None:
    sys.exit("STATUS 응답 없음. 아두이노 전원/USB 확인.")
print("  ✓ STATUS 수신 (카운트 %d)" % box["cnt"])
print("  ※ 이 도구는 구동·조향 명령을 한 개도 보내지 않는다.\n")

runs = []
try:
    while True:
        print("출발선에 세우고 Enter   (그만하려면 Ctrl-C)")
        input()
        box["base"] = fresh()
        box["live"] = True
        print("  ▶ 앞으로 민다. 도착선에서 Enter")
        input()
        box["live"] = False
        delta = fresh() - box["base"]
        print()
        d = float(input("  줄자로 잰 거리 [m] (예 4.67): ").strip())
        if d <= 0:
            print("  거리가 0 이하다. 이 회차는 버린다.\n")
            continue
        cpm = delta / d
        runs.append((delta, d, cpm))
        print("  → %d 카운트 / %.2f m = %.1f counts/m\n" % (delta, d, cpm))
except (KeyboardInterrupt, EOFError, ValueError):
    pass

print("\n" + "=" * 56)
if runs:
    vals = [c for _, _, c in runs]
    avg = sum(vals) / len(vals)
    for i, (n, d, c) in enumerate(runs, 1):
        print("  %d회차  %6d 카운트 / %.2f m = %7.1f counts/m" % (i, n, d, c))
    print("  " + "-" * 50)
    print("  평균 %.1f counts/m" % avg)
    if len(vals) > 1:
        print("  편차 %.1f%%  (최소 %.1f / 최대 %.1f)"
              % ((max(vals) - min(vals)) / avg * 100, min(vals), max(vals)))
    print("  " + "-" * 50)
    print("  판정")
    if len(vals) > 1 and (max(vals) - min(vals)) / avg > 0.10:
        print("    🔴 손으로 밀어도 편차가 10%% 넘는다.")
        print("       모터를 안 돌렸으니 슬립도 모터노이즈도 아니다.")
        print("       → 엔코더 자체(디스크·센서·배선)를 봐야 한다.")
    else:
        print("    ✅ 손밀기는 일정하다 (편차 %.1f%%). 이 값이 기하학적 기준이다."
              % ((max(vals) - min(vals)) / avg * 100 if len(vals) > 1 else 0.0))
        print("       주행 값 674~852 와 비교:")
        print("         주행이 훨씬 크면  → 구동 중에만 생기는 문제")
        print("                             (슬립 또는 모터 노이즈)")
        print("         비슷하면          → 이 값이 진짜 counts_per_meter")
else:
    print("  기록된 회차가 없다.")
print("=" * 56)
ser.close()
