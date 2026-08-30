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
    !                      ★ 자동 주행 시험 — 2단으로 10초 굴리고 스스로 선다
    !2,10                  단계,초 를 직접 준다 (음수 = 후진.  예: !-1,5)
    q                      정지 명령(0.00)을 보내고 종료

자동 주행(!) 이 하는 일
    출발 전 카운트를 적어두고 → 정해진 초 동안 명령을 10Hz 로 계속 보내고
    → 스스로 정지 명령을 보내고 → 멈출 때까지 코스팅 카운트를 더 센다.
    주행 카운트 / 시간 / 평균속도 / 코스팅 거리를 표로 찍는다.
    ★ 달리는 중 Ctrl-C 를 누르면 즉시 정지 명령이 나간다.

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


#  펌웨어의 구동 단계 명령 (stageToPwm 참고)
#    1.00 정지    2.00 전진1단  3.00 전진2단  4.00 전진3단
#    5.00 정지    6.00 후진1단  7.00 후진2단  8.00 후진3단
STAGE_CMD = {0: "1.00", 1: "2.00", 2: "3.00", 3: "4.00",
             -1: "6.00", -2: "7.00", -3: "8.00"}
STOP_CMD = "1.00"

#  워치독이 2초다. 그보다 훨씬 자주 보내야 차가 멈추지 않는다.
SEND_HZ = 10.0
MAX_SECONDS = 30.0        # 사람이 뛰어서 못 따라가는 길이는 막는다

#  🔴 펌웨어는 telemetryEnabled = false 로 뜬다.
#     즉 가만히 있으면 STATUS 를 한 줄도 안 준다. "S" 를 보내야 한 줄 준다.
#     (Q 로 자동 발행을 켤 수도 있지만 토글이라 지금 상태를 모르면 위험하다.
#      직접 물어보는 쪽이 확실하다 — measure.py 도 같은 방식이다.)
POLL_HZ = 10.0

_write_lock = threading.Lock()


def send(ser, text):
    """어느 스레드에서 보내든 줄이 섞이지 않게 한다."""
    with _write_lock:
        ser.write((text + "\n").encode("utf-8"))
        ser.flush()


class Poller(threading.Thread):
    """STATUS 를 주기적으로 요청한다. 이게 없으면 카운트를 영영 못 본다."""

    daemon = True

    def __init__(self, ser):
        threading.Thread.__init__(self)
        self.ser = ser
        self.alive = True

    def run(self):
        while self.alive:
            try:
                send(self.ser, "S")
            except Exception:
                return
            time.sleep(1.0 / POLL_HZ)


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
        self.count = None          # STATUS 의 누적 엔코더 카운트
        self.rpm = 0.0
        self.state = "?"

    def _parse(self, line):
        """STATUS,state,fault,adc,target,pwm,rpm,encoder,... 에서 필요한 것만."""
        f = line.split(",")
        if len(f) < 8:
            return
        try:
            self.state = f[1]
            self.rpm = float(f[6])
            self.count = int(float(f[7]))
        except (ValueError, IndexError):
            pass                       # 깨진 줄은 조용히 버린다

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
                self._parse(line)
                if not self.show_status:
                    continue
            print(line)


def _fresh_count(ser, reader, fallback):
    """방금 찍은 카운트를 받아온다.

    폴링 주기(0.1초) 만큼 값이 늦을 수 있는데, 구간의 끝에서는 그게
    그대로 거리 오차가 된다. 그래서 경계에서는 한 번 더 물어보고 기다린다.
    """
    before = reader.status_count
    send(ser, "S")
    end = time.monotonic() + 0.5
    while reader.status_count == before and time.monotonic() < end:
        time.sleep(0.01)
    return reader.count if reader.count is not None else fallback


def timed_run(ser, reader, stage, seconds, cpm):
    """단계 <stage> 로 <seconds> 초 굴리고 스스로 세운다.

    ROS 를 거치지 않는다. 브릿지가 죽어 있어도 된다.
    """
    cmd = STAGE_CMD.get(stage)
    if cmd is None:
        print("[!] 단계는 1~3 (전진) 또는 -1~-3 (후진) 이다.")
        return
    if seconds <= 0 or seconds > MAX_SECONDS:
        print("[!] 초는 0 초과 %.0f 이하로." % MAX_SECONDS)
        return
    if reader.count is None:
        print("[!] 아직 STATUS 를 못 받았다. 잠시 뒤 다시.")
        return

    where = "전진" if stage > 0 else "후진"
    print("=" * 62)
    print("  자동 주행: %s %d단 (%s) 을 %.1f초" % (where, abs(stage), cmd, seconds))
    print("  ★ 앞이 비었는지 확인. 멈추려면 Ctrl-C")
    print("=" * 62)
    for n in (3, 2, 1):
        print("  %d..." % n)
        time.sleep(1.0)

    base = reader.count
    t0 = time.monotonic()
    stopped_at = None

    try:
        #  ---- 주행 ----
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= seconds:
                break
            send(ser, cmd)
            sys.stdout.write("\r  %4.1fs  카운트 %8d  RPM %6.2f  %-8s"
                             % (elapsed, (reader.count or 0) - base,
                                reader.rpm, reader.state))
            sys.stdout.flush()
            time.sleep(1.0 / SEND_HZ)
        run_time = time.monotonic() - t0
        send(ser, STOP_CMD)             # 먼저 세운다
        stopped_at = _fresh_count(ser, reader, base)
        run_counts = stopped_at - base
    except KeyboardInterrupt:
        run_time = time.monotonic() - t0
        send(ser, STOP_CMD)
        stopped_at = _fresh_count(ser, reader, base)
        run_counts = stopped_at - base
        print("\n  [중단] 정지 명령을 보낸다")

    #  ---- 정지 + 코스팅 ----
    send(ser, STOP_CMD)
    print("\n  정지 명령 전송. 코스팅을 센다 (3초)...")
    coast_end = time.monotonic() + 3.0
    while time.monotonic() < coast_end:
        send(ser, STOP_CMD)
        time.sleep(1.0 / SEND_HZ)
    coast_counts = _fresh_count(ser, reader, stopped_at) - stopped_at

    #  ---- 결과 ----
    speed_cps = run_counts / run_time if run_time else 0.0
    print("\n" + "=" * 62)
    print("  주행 카운트     %d" % run_counts)
    print("  주행 시간       %.2f 초" % run_time)
    print("  코스팅 카운트   %d" % coast_counts)
    if cpm > 0:
        dist = run_counts / cpm
        print("  ─────────────────────────────────────────")
        print("  주행 거리(환산) %.2f m   ← counts_per_meter %.1f 기준" % (dist, cpm))
        print("  평균 속도       %.3f m/s  (%.2f km/h)"
              % (dist / run_time if run_time else 0.0,
                 (dist / run_time if run_time else 0.0) * 3.6))
        print("  코스팅 거리     %.3f m" % (coast_counts / cpm))
    print("  ─────────────────────────────────────────")
    print("  ★ 줄자로 실제 거리를 재서 비교할 것.")
    if run_counts:
        print("    실측이 D m 였다면  counts_per_meter = %d / D" % run_counts)
    print("=" * 62)


def main():
    ap = argparse.ArgumentParser(description="MCU 시리얼 콘솔")
    ap.add_argument("--port", default=None, help="기본은 자동 탐색")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--quiet", action="store_true",
                    help="STATUS 를 처음부터 접어둔다")
    ap.add_argument("--cpm", type=float, default=199.8,
                    help="counts_per_meter — 자동 주행 결과를 m 로 환산할 때 쓴다")
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
    print("  ★ 자동 주행:  !  = 2단 10초 후 자동 정지   |  !2,10 / !-1,5")
    print("  자주 쓰는 것:  ?  도움말   |  A600 D600 조향 끝   |  1.00 정지")
    print("                 AR1 AR2 AR0 안티롤백  |  B 급제동  |  O 오도리셋")
    print("=" * 62)

    reader = Reader(ser, show_status=not args.quiet)
    reader.start()

    poller = Poller(ser)
    poller.start()

    #  STATUS 가 실제로 오는지 여기서 확인한다.
    #  안 오면 자동 주행이 "카운트를 못 받았다" 로 막히는데,
    #  그걸 명령을 친 다음에 알게 되면 늦다.
    deadline = time.time() + 4.0
    while reader.count is None and time.time() < deadline:
        time.sleep(0.1)
    if reader.count is None:
        print("\n⚠ STATUS 응답이 없다. 자동 주행(!)이 안 된다.")
        print("   아두이노 전원과 USB 를 확인할 것. 그래도 명령은 보낼 수 있다.\n")
    else:
        print("  ✓ STATUS 수신 — 현재 카운트 %d, 상태 %s\n"
              % (reader.count, reader.state))

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
            if cmd.startswith("!"):
                body = cmd[1:].replace(",", " ").split()
                stage, secs = 2, 10.0
                try:
                    if len(body) >= 1:
                        stage = int(body[0])
                    if len(body) >= 2:
                        secs = float(body[1])
                except ValueError:
                    print("[!] 예:  !  또는  !2,10  또는  !-1,5")
                    continue
                was = reader.show_status
                reader.show_status = False          # 표가 흐르지 않게
                timed_run(ser, reader, stage, secs, args.cpm)
                reader.show_status = was
                continue
            if not cmd:
                continue

            send(ser, cmd)
            print("[보냄] %s" % cmd)
    except KeyboardInterrupt:
        pass
    finally:
        #  ★ 나갈 때는 반드시 세운다. 콘솔을 닫았는데 차가 굴러가면 안 된다.
        try:
            send(ser, "1.00")
            time.sleep(0.2)
        except Exception:
            pass
        poller.alive = False
        reader.alive = False
        time.sleep(0.1)
        try:
            ser.close()
        except Exception:
            pass
        print("\n정지 명령을 보내고 종료했다. (STATUS %d줄 수신)"
              % reader.status_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
