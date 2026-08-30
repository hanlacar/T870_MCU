#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run10.py — 정해진 단계로 정해진 초만큼 굴리고 스스로 정지. ROS 불필요."""
import glob, sys, threading, time
import serial

STAGE = {1: "2.00", 2: "3.00", 3: "4.00", -1: "6.00", -2: "7.00", -3: "8.00"}
STOP = "1.00"

stage = int(sys.argv[1]) if len(sys.argv) > 1 else 2
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
cmd = STAGE.get(stage)
if cmd is None or secs <= 0 or secs > 30:
    sys.exit("단계는 1~3 / -1~-3, 초는 0~30")

port = (glob.glob("/dev/t870_mcu") or sorted(glob.glob("/dev/serial/by-id/*"))
        or sorted(glob.glob("/dev/ttyACM*")) or sorted(glob.glob("/dev/ttyUSB*")))
if not port:
    sys.exit("포트를 못 찾았다. USB 확인.")
ser = serial.Serial(port[0], 115200, timeout=1)
print("포트 %s" % port[0])
time.sleep(2.0)

lock = threading.Lock()
box = {"n": None, "rpm": 0.0, "st": "?", "rx": 0}


def send(t):
    with lock:
        ser.write((t + "\n").encode()); ser.flush()


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
                    box["st"] = f[1]; box["rpm"] = float(f[6])
                    box["n"] = int(float(f[7])); box["rx"] += 1
                except ValueError:
                    pass


def poll():
    while True:
        send("S"); time.sleep(0.5)


threading.Thread(target=rx, daemon=True).start()
threading.Thread(target=poll, daemon=True).start()

end = time.time() + 4
while box["n"] is None and time.time() < end:
    time.sleep(0.05)
if box["n"] is None:
    sys.exit("STATUS 응답 없음. 아두이노 전원/USB 확인.")
print("✓ STATUS 수신 — 카운트 %d, 상태 %s" % (box["n"], box["st"]))


def fresh(fallback):
    before = box["rx"]; send("S")
    t = time.time() + 0.5
    while box["rx"] == before and time.time() < t:
        time.sleep(0.01)
    return box["n"] if box["n"] is not None else fallback


print("\n%s %d단 (%s) 을 %.1f초. 앞이 비었는지 확인. 멈추려면 Ctrl-C"
      % ("전진" if stage > 0 else "후진", abs(stage), cmd, secs))
for i in (3, 2, 1):
    print("  %d..." % i); time.sleep(1.0)

base = box["n"]; t0 = time.time()
try:
    while time.time() - t0 < secs:
        send(cmd)
        sys.stdout.write("\r  %4.1fs  카운트 %7d  RPM %6.2f  %-8s"
                         % (time.time() - t0, (box["n"] or 0) - base,
                            box["rpm"], box["st"]))
        sys.stdout.flush()
        time.sleep(0.1)
    rt = time.time() - t0
except KeyboardInterrupt:
    rt = time.time() - t0
    print("\n  [중단]")

send(STOP)
stop_at = fresh(base)
run = stop_at - base
print("\n  정지. 코스팅 3초...")
ct = time.time() + 3.0
while time.time() < ct:
    send(STOP); time.sleep(0.1)
coast = fresh(stop_at) - stop_at

print("\n" + "=" * 54)
print("  주행 카운트   %d" % run)
print("  주행 시간     %.2f 초" % rt)
print("  코스팅        %d 카운트" % coast)
print("  ------------------------------------")
print("  199.8 기준    %.2f m   %.3f m/s" % (run / 199.8, run / 199.8 / rt if rt else 0))
print("  ★ 줄자로 잰 실제 거리 D 라면  counts_per_meter = %d / D" % run)
print("=" * 54)
for _ in range(5):
    send(STOP); time.sleep(0.05)
ser.close()
