#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""serial_console.py — MCU 에 명령을 직접 쳐 넣는 콘솔 (ROS 불필요)

현장에서 이런 걸 하려면 필요하다.

    조향 끝값 재기        A600  →  STATUS 의 steer_adc 를 읽는다
    안티롤백 시험         AR1 / AR2 / AR0 / ARP70
    급제동 로그 보기      B     →  BRAKE_DONE,... 이 뜨는지
    명령 목록             ?
    오도 리셋             O

쓰는 법
    python3 tools/serial_console.py

    포트를 직접 주려면
    python3 tools/serial_console.py --port /dev/ttyACM0

    STATUS 가 너무 빨리 흐르면 시작할 때부터 접어둔다
    python3 tools/serial_console.py --quiet

콘솔 안에서
    명령을 치고 Enter      그대로 MCU 로 나간다 (대문자 변환 안 함)
    s                      STATUS 표시 켜기/끄기
    .                      마지막 STATUS 한 줄만 보기
    q                      정지 명령(0.00)을 보내고 종료

⚠ 브릿지나 아두이노 시리얼 모니터가 켜져 있으면 포트를 못 잡는다.
   먼저 다 끌 것.  확인:  pgrep -af mcu_bridge
"""

import argparse
import glob
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial 없음:  sudo apt install python3-serial")


def find_port():
    """USB 장치 정보로 먼저 찾고, 없으면 ttyACM/ttyUSB 로 떨어진다."""
    for pat in ("/dev/t870_mcu",
                "/dev/serial/by-id/*",
                "/dev/ttyACM*",
                "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


class Reader(threading.Thread):
    """수신 전용 스레드. STATUS 는 접을 수 있게 따로 센다."""

    daemon = True

    def __init__(self, ser, show_status):
        threading.Thread.__init__(self)
        self.ser = ser
        self.show_status = show_status
        self.alive = True
        self.last_status = ""
        self.status_count = 0

    def run(self):
        while self.alive:
            try:
                raw = self.ser.readline()
            except Exception as exc:            # 케이블이 빠지면 여기로 온다
                print("\n[수신 끊김] %s" % exc)
                self.alive = False
                return
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            if line.startswith("STATUS"):
                self.last_status = line
                self.status_count += 1
                if not self.show_status:
                    continue
            print(line)


def main():
    ap = argparse.ArgumentParser(description="MCU 시리얼 콘솔")
    ap.add_argument("--port", default=None, help="기본은 자동 탐색")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--quiet", action="store_true",
                    help="STATUS 를 처음부터 접어둔다")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("포트를 못 찾았다. 케이블을 확인하거나 --port 로 직접 줄 것.\n"
                 "  python3 tools/check_ports.py  로 목록을 볼 수 있다.")

    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except Exception as exc:
        sys.exit("포트를 못 열었다: %s\n"
                 "  다른 프로그램이 잡고 있을 수 있다.\n"
                 "    pgrep -af mcu_bridge\n"
                 "    sudo fuser -k /dev/ttyACM*" % exc)

    print("=" * 62)
    print("포트 %s  @ %d" % (port, args.baud))
    print("  명령을 치고 Enter.   s = STATUS 접기/펴기,  . = 마지막 STATUS,")
    print("  q = 정지 후 종료")
    print("  자주 쓰는 것:  ?  도움말   |  A600 D600 조향 끝   |  0.00 정지")
    print("                 AR1 AR2 AR0 안티롤백  |  B 급제동  |  O 오도리셋")
    print("=" * 62)

    reader = Reader(ser, show_status=not args.quiet)
    reader.start()
    time.sleep(0.3)

    try:
        while reader.alive:
            try:
                text = input()
            except EOFError:
                break
            cmd = text.strip()

            if cmd == "q":
                break
            if cmd == "s":
                reader.show_status = not reader.show_status
                print("[STATUS %s]" % ("펴기" if reader.show_status else "접기"))
                continue
            if cmd == ".":
                print("[마지막 STATUS] %s" % (reader.last_status or "아직 없음"))
                continue
            if not cmd:
                continue

            ser.write((cmd + "\n").encode("utf-8"))
            ser.flush()
            print("[보냄] %s" % cmd)
    except KeyboardInterrupt:
        pass
    finally:
        #  ★ 나갈 때는 반드시 세운다. 콘솔을 닫았는데 차가 굴러가면 안 된다.
        try:
            ser.write(b"0.00\n")
            ser.flush()
            time.sleep(0.2)
        except Exception:
            pass
        reader.alive = False
        try:
            ser.close()
        except Exception:
            pass
        print("\n정지 명령을 보내고 종료했다. (STATUS %d줄 수신)"
              % reader.status_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
