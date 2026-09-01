#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_v1_0831.py — 혼자 쓰는 수동 조종기. ROS 불필요, 시리얼 직결.

    python3 tools/drive_v1_0831.py

    조향 한 번 누를 때 움직이는 시간을 바꾸려면
        python3 tools/drive_v1_0831.py --steer-ms 20

    직진 중앙의 A0 기준값을 바꾸려면 (0830 실측 223)
        python3 tools/drive_v1_0831.py --center-adc 223

★ 시작할 때 바퀴를 건드리지 않는다.
  켜자마자 조향이 제멋대로 움직이던 것은 뺀 기능이다.
  중앙이 필요하면 z(A0 기준으로 맞추기) 또는 c(누적 0 복귀)를 직접 누른다.

키
  ── 주행 ─────────────────────────────
    w  전진 1단     e  전진 2단     r  전진 3단
    s  후진 1단     x  후진 2단
    [스페이스]  정지
    ※ 안전을 위해 계속 가다가 10초가 지나면 스스로 멈춘다

  ── 조향 ─────────────────────────────
    a / d   좌 / 우 (기본 30ms)
    A / D   좌 / 우 크게 (기본의 5배)
    [ / ]   조향 한 번의 시간 줄이기 / 늘리기

  ── 중앙 ─────────────────────────────
    z   A0 를 기준값(기본 223)으로 맞춘다  ← 펌웨어 시크. 최대 4초, 안전상한 있음
    c   조향 누적 0 으로 복귀
    m   지금 위치를 중앙(누적 0)으로 등록

  ── 기타 ─────────────────────────────
    f   급제동 (역토크)
    0   즉시정지
    o   엔코더 0 으로
    q   종료 (정지 명령을 보내고 나간다)
"""

import argparse
import glob
import select
import sys
import termios
import threading
import time
import tty

import serial

STOP = "1.00"
FWD = {1: "2.00", 2: "3.00", 3: "4.00"}
REV = {1: "6.00", 2: "7.00", 3: "8.00"}
SEND_HZ = 10.0
MAX_DRIVE_SEC = 10.0        # 계속 주행 자동 차단
POLL_HZ = 2.0               # 자동 발행이 안 켜졌을 때만 쓰는 폴링


def main():
    ap = argparse.ArgumentParser(description="T870 수동 조종 (시리얼 직결)")
    ap.add_argument("--port", default=None, help="기본은 자동 탐색")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--steer-ms", type=int, default=30,
                    help="조향 한 번에 모터를 켜는 시간 [ms]")
    ap.add_argument("--center-adc", type=int, default=223,
                    help="직진 중앙의 A0 값 (0830 실측 223)")
    ap.add_argument("--cpm", type=float, default=199.8,
                    help="거리 표시용 counts_per_meter (슬립 때문에 참고값)")
    args = ap.parse_args()

    port = args.port or next(
        (p[0] for p in (sorted(glob.glob(g)) for g in
                        ("/dev/t870_mcu", "/dev/serial/by-id/*",
                         "/dev/ttyACM*", "/dev/ttyUSB*")) if p), None)
    if not port:
        sys.exit("포트를 못 찾았다. USB 확인.\n"
                 "  python3 tools/check_ports.py 로 목록을 볼 수 있다.")
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except Exception as exc:
        sys.exit("포트를 못 열었다: %s\n"
                 "  다른 프로그램이 잡고 있는지 확인:\n"
                 "    pgrep -af 'mcu_bridge|center|run_v4|s_course|serial_console'"
                 % exc)
    print("포트 %s" % port)
    print("  아두이노 부팅 대기...")
    time.sleep(2.0)

    lock = threading.Lock()
    box = {"alive": True, "rx": 0, "tele": None, "msg": "",
           "state": "?", "fault": "?", "adc": None, "net": None,
           "cnt": None, "rpm": 0.0, "pwm": 0, "last": ""}
    drive = {"cmd": STOP, "until": 0.0, "label": "정지", "base": None}
    steer_ms = max(1, int(args.steer_ms))

    def send(text):
        with lock:
            try:
                ser.write((text + "\n").encode()); ser.flush()
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
            elif line.startswith("STATUS"):
                f = line.split(",")
                if len(f) > 8:
                    try:
                        box["state"] = f[1]; box["fault"] = f[2]
                        box["adc"] = int(float(f[3]))
                        box["pwm"] = int(float(f[5]))
                        box["rpm"] = float(f[6])
                        box["cnt"] = int(float(f[7]))
                        box["net"] = int(float(f[8]))
                        box["rx"] += 1
                    except ValueError:
                        pass
            else:
                box["last"] = line          # STEER_OK, SEEK_*, BRAKE_* 등

    def tx():
        last_poll = 0.0
        while box["alive"]:
            now = time.time()
            if drive["cmd"] != STOP and now > drive["until"]:
                drive["cmd"] = STOP
                drive["label"] = "정지 (10초 초과)"
            send(drive["cmd"])
            if not box["tele"] and now - last_poll >= 1.0 / POLL_HZ:
                send("S"); last_poll = now
            time.sleep(1.0 / SEND_HZ)

    threading.Thread(target=rx, daemon=True).start()

    #  살아있는지 먼저 확인 (10초까지 기다린다)
    end = time.time() + 10.0
    while box["cnt"] is None and time.time() < end:
        send("S"); time.sleep(0.3)
    if box["cnt"] is None:
        box["alive"] = False
        sys.exit("STATUS 응답 없음.\n"
                 "  1) 아두이노 USB\n"
                 "  2) 다른 프로그램이 포트를 잡고 있나\n"
                 "  3) 아두이노 IDE 시리얼 모니터가 열려 있나")

    #  자동 발행을 켠다 (회선 부하를 낮춘다)
    box["tele"] = None; send("Q")
    t = time.time() + 1.0
    while box["tele"] is None and time.time() < t:
        time.sleep(0.02)
    if box["tele"] is False:
        box["tele"] = None; send("Q")
        t = time.time() + 1.0
        while box["tele"] is None and time.time() < t:
            time.sleep(0.02)

    threading.Thread(target=tx, daemon=True).start()

    def go(cmd, label):
        drive["cmd"] = cmd
        drive["label"] = label
        drive["until"] = time.time() + MAX_DRIVE_SEC
        box["msg"] = "%s — 스페이스로 정지 (최대 %.0f초)" % (label, MAX_DRIVE_SEC)

    def halt():
        drive["cmd"] = STOP
        drive["label"] = "정지"
        box["msg"] = "정지"

    def draw():
        w = sys.stdout.write
        w("\033[2J\033[H")
        w("┌─ T870 수동 조종 ─────────────────────────────────────\n")
        w("│\n")
        w("│  구동   \033[1;36m%-16s\033[0m PWM %-4d RPM %6.2f\n"
          % (drive["label"], box["pwm"], box["rpm"]))
        net = box["net"]
        adc = box["adc"]
        off = ("%+d" % (adc - args.center_adc)) if adc is not None else "?"
        w("│  조향   누적 \033[1m%s\033[0m ms   한 번 %d ms\n"
          % (("%+d" % net) if net is not None else "?", steer_ms))
        w("│         A0 %s   (중앙 기준 %d 대비 %s)\n"
          % (adc, args.center_adc, off))
        cnt = box["cnt"]
        dist = (cnt / args.cpm) if (cnt is not None and args.cpm) else 0.0
        w("│  엔코더 %s   (바퀴 기준 %.2f m — 슬립 있어 참고만)\n"
          % (cnt, dist))
        w("│  상태   %s / %s\n" % (box["state"], box["fault"]))
        w("│\n")
        w("│  ── 키 ──────────────────────────────────────────\n")
        w("│   w 전진1  e 전진2  r 전진3   s 후진1  x 후진2\n")
        w("│   [스페이스] 정지        f 급제동     0 즉시정지\n")
        w("│   a/d 조향 좌/우   A/D 크게   [ ] 조향시간 조절\n")
        w("│   z A0 %d 로 맞추기   c 누적0 복귀   m 지금을 중앙\n"
          % args.center_adc)
        w("│   o 엔코더 0        q 종료\n")
        w("│\n")
        w("│  %s\n" % box["msg"])
        w("│  MCU: %s\n" % box["last"][:52])
        w("└──────────────────────────────────────────────────────\n")
        sys.stdout.flush()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        draw()
        last_draw = 0.0
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                k = sys.stdin.read(1)
                if k == "q":
                    break
                elif k == " ":
                    halt()
                elif k == "w":
                    go(FWD[1], "전진 1단")
                elif k == "e":
                    go(FWD[2], "전진 2단")
                elif k == "r":
                    go(FWD[3], "전진 3단")
                elif k == "s":
                    go(REV[1], "후진 1단")
                elif k == "x":
                    go(REV[2], "후진 2단")
                elif k == "a":
                    send("A%d" % steer_ms); box["msg"] = "좌 %dms" % steer_ms
                elif k == "d":
                    send("D%d" % steer_ms); box["msg"] = "우 %dms" % steer_ms
                elif k == "A":
                    send("A%d" % (steer_ms * 5)); box["msg"] = "좌 %dms" % (steer_ms * 5)
                elif k == "D":
                    send("D%d" % (steer_ms * 5)); box["msg"] = "우 %dms" % (steer_ms * 5)
                elif k == "[":
                    steer_ms = max(1, steer_ms - 5); box["msg"] = "조향 %dms" % steer_ms
                elif k == "]":
                    steer_ms = min(400, steer_ms + 5); box["msg"] = "조향 %dms" % steer_ms
                elif k == "z":
                    halt(); send("AS%d" % args.center_adc)
                    box["msg"] = "A0 %d 로 맞추는 중 (최대 4초)" % args.center_adc
                elif k == "c":
                    halt(); send("C"); box["msg"] = "누적 0 으로 복귀"
                elif k == "m":
                    halt(); send("M"); box["msg"] = "지금 위치를 중앙으로 등록"
                elif k == "f":
                    halt(); send("B"); box["msg"] = "급제동"
                elif k == "0":
                    halt(); send("X"); box["msg"] = "즉시정지"
                elif k == "o":
                    send("O"); box["msg"] = "엔코더 0 으로"
                draw(); last_draw = time.time()
            elif time.time() - last_draw > 0.3:
                draw(); last_draw = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        halt()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        time.sleep(0.3)
        box["alive"] = False
        for _ in range(6):
            send(STOP); time.sleep(0.05)
        if box["tele"]:
            send("Q")
        try:
            ser.close()
        except Exception:
            pass
        print("\n정지 명령을 보내고 종료했다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
