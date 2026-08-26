#!/usr/bin/env python3
"""
mcu_bridge_node.py  —  T870 Arduino v28 시리얼 브릿지 (대회용 v2)

v1 대비 수정 사항
-----------------
[치명] 1. STATUS 파싱 실패로 텔레메트리 전멸
   protocol.parse_status 가 fault 필드를 숫자로만 읽어서, v28 이 출력하는
   "NONE" 문자열에서 예외 → 모든 STATUS 라인 폐기.
   /drive, /rpm, /arduino/raw_status, /odom 이 전혀 안 나왔고
   펌웨어 fault 기반 E-stop 래치도 동작하지 않았다.
   → protocol.py 에서 수정. 개별 필드가 깨져도 라인 전체를 버리지 않는다.

[중대] 2. 재연결 시 노드 2초 프리즈
   open_serial() 안의 time.sleep(2.0) 이 타이머 콜백을 블록해서,
   재연결 시도마다 구독/발행/워치독이 전부 멈췄다.
   → 논블로킹 연결 상태머신(CLOSED / WAIT_RESET / READY)으로 분리.

[중대] 3. 시작 시 조향 상태 불명
   wheel_dirty=False 로 시작해 조향 명령을 한 번도 보내지 않았다.
   펌웨어 조향은 시간 기반이라 이전 세션 위치를 그대로 물고 있어,
   부팅 직후 실제 조향각을 아무도 모르는 상태였다.
   → center_steer_on_connect (기본 true) 로 연결 시 W0 1회 전송.

[중대] 4. 코스팅 거리 누락
   cmd_stage==0 이면 엔코더 delta 를 0 으로 버렸다. 이 차량은 브레이크가
   없어 정지 명령 후에도 상당 거리를 굴러가는데(감속 램프만 0.5초),
   그 거리가 odom 에서 통째로 사라졌다.
   → coasting_policy 로 선택. 기본은 직전 진행 방향을 유지해 적산.

[보완] 5. 수신 버퍼 무한 증가 방어, 파라미터 타입 완화,
        시리얼 통신 두절 감시(/arduino/telemetry_ok) 추가.
"""

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

try:
    from tf2_ros import TransformBroadcaster
    TF_OK = True
except ImportError:
    TF_OK = False

import serial

from .protocol import (
    parse_drive_stage,
    drive_serial_command,
    valid_wheel_deg,
    wheel_serial_command,
    parse_status,
)

# 연결 상태
CONN_CLOSED = 0        # 포트 닫힘
CONN_WAIT_RESET = 1    # 포트 열림, 아두이노 USB 리셋 대기 중
CONN_READY = 2         # 통신 가능

MAX_RX_BUF = 8192      # 개행 없는 쓰레기 유입 시 메모리 방어


# ==========================================================
# 포트 자동 탐색
#
#  USB 를 꽂는 순서에 따라 /dev/ttyACM 번호가 바뀐다.
#  GPS 를 같이 꽂으면 아두이노가 ACM0 이 되기도 ACM1 이 되기도 한다.
#  실제로 이것 때문에 반나절을 날린 적이 있으므로 번호를 믿지 않고
#  USB 장치 정보(제조사/시리얼)로 아두이노를 직접 찾는다.
# ==========================================================

ARDUINO_HINTS = ("arduino", "mega", "ch340", "ch910", "wch", "usb2.0-serial")
NOT_ARDUINO_HINTS = ("u-blox", "u_blox", "gnss", "gps")


def find_arduino_port(logger=None):
    """/dev/serial/by-id 를 훑어 아두이노로 보이는 포트를 고른다.

    반환: 포트 경로. 못 찾으면 None.
    """
    base = "/dev/serial/by-id"
    if not os.path.isdir(base):
        return None

    candidates = []
    for name in sorted(os.listdir(base)):
        low = name.lower()
        if any(bad in low for bad in NOT_ARDUINO_HINTS):
            continue                       # GPS 등은 제외
        score = 0
        if "arduino" in low or "mega" in low:
            score = 100                    # 정품 아두이노 우선
        elif any(h in low for h in ARDUINO_HINTS):
            score = 50                     # 클론 칩
        if score:
            real = os.path.realpath(os.path.join(base, name))
            candidates.append((score, real, name))

    if not candidates:
        return None
    candidates.sort(reverse=True)
    if logger is not None and len(candidates) > 1:
        logger.warn("아두이노 후보가 여럿이다. 첫 번째 사용: %s"
                    % [c[1] for c in candidates])
    return candidates[0][1]


class McuBridge(Node):

    def __init__(self):
        super().__init__("mcu_bridge")

        # ---------- 파라미터 ----------
        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("send_hz", 10.0)
        self.declare_parameter("drive_timeout_s", 0.5)
        self.declare_parameter("wheel_timeout_s", 0.5)
        self.declare_parameter("status_poll_hz", 5.0)
        self.declare_parameter("max_steer_deg", 27.0)
        self.declare_parameter("steer_limit_ms", 440)
        self.declare_parameter("steer_cmd_mode", "W")
        self.declare_parameter("wheel_timeout_policy", "hold_last")

        # ---------- 직진 보정 ----------
        # 차량이 한쪽으로 치우쳐 갈 때 모든 조향 명령에 더하는 고정 각도.
        # 원인이 조향 영점이든 뒷축 틀어짐이든 결과적으로 같은 방식으로 보정된다.
        # +는 우, -는 좌.  예) 오른쪽으로 흘러가면 음수를 넣는다.
        self.declare_parameter("steer_offset_deg", 0.0)
        self.declare_parameter("latch_on_firmware_fault", True)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("arduino_reset_wait_s", 2.0)

        # 연결/재연결 시 조향을 0도로 한 번 맞출지.
        # 펌웨어 조향은 시간 기반이라 이전 세션 위치가 남아 있다.
        self.declare_parameter("center_steer_on_connect", True)

        # 텔레메트리 두절 판정 [초]. 0 이면 감시 안 함.
        self.declare_parameter("telemetry_timeout_s", 2.0)

        # 코스팅 처리:
        #   last_direction = 정지 명령 후에도 직전 진행 방향으로 적산 (권장)
        #   drop           = 정지 중 delta 폐기 (v1 동작)
        self.declare_parameter("coasting_policy", "last_direction")

        # ---------- 급정거 ----------
        # /mcu_stop=true 일 때 보낼 시리얼 명령.
        # v28 에는 램프를 건너뛰는 즉시정지 명령이 아직 없어 기본값은 "1.00"(램프 정지).
        # 펌웨어에 즉시정지(다이내믹 브레이킹) 명령을 추가하면 여기만 바꾸면 된다.
        self.declare_parameter("estop_serial_command", "1.00")
        # 급정거 신호 유효시간. 이보다 오래되면 해제로 본다.
        self.declare_parameter("stop_timeout_s", 0.5)

        # 오도메트리
        self.declare_parameter("counts_per_meter", 0.0)
        self.declare_parameter("wheelbase_m", 0.73)
        self.declare_parameter("encoder_signed", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        gp = lambda name: self.get_parameter(name).value
        # port: "auto" 면 USB 장치 정보로 아두이노를 직접 찾는다.
        #       고정하려면 "/dev/ttyACM1" 처럼 경로를 직접 넣으면 된다.
        port_param = str(gp("port")).strip()
        if port_param.lower() in ("auto", "", "none"):
            found = find_arduino_port(self.get_logger())
            if found:
                self.port = found
                self.get_logger().info("포트 자동 탐색: %s" % found)
            else:
                self.port = "/dev/ttyACM0"
                self.get_logger().warn(
                    "아두이노를 못 찾았다. /dev/ttyACM0 으로 시도한다. "
                    "USB 연결을 확인하거나 port:= 로 직접 지정하라.")
            self.auto_port = True
        else:
            self.port = port_param
            self.auto_port = False
        self.baud = int(gp("baud"))
        send_hz = float(gp("send_hz"))
        if send_hz <= 0.0:
            raise ValueError("send_hz must be > 0")
        self.send_period = 1.0 / send_hz
        self.drive_timeout = float(gp("drive_timeout_s"))
        self.wheel_timeout = float(gp("wheel_timeout_s"))
        status_hz = float(gp("status_poll_hz"))
        self.status_period = 1.0 / status_hz if status_hz > 0 else 0.0
        self.max_deg = float(gp("max_steer_deg"))
        self.steer_limit_ms = int(gp("steer_limit_ms"))
        self.steer_mode = str(gp("steer_cmd_mode")).upper()
        self.wheel_timeout_policy = str(gp("wheel_timeout_policy")).lower()
        self.steer_offset = float(gp("steer_offset_deg"))
        self.latch_on_firmware_fault = bool(gp("latch_on_firmware_fault"))
        self.reconnect_delay = float(gp("reconnect_delay_s"))
        self.reset_wait = float(gp("arduino_reset_wait_s"))
        self.center_on_connect = bool(gp("center_steer_on_connect"))
        self.telemetry_timeout = float(gp("telemetry_timeout_s"))
        self.coasting_policy = str(gp("coasting_policy")).lower()
        self.estop_cmd = str(gp("estop_serial_command")).strip()
        self.stop_timeout = float(gp("stop_timeout_s"))
        self.cpm = float(gp("counts_per_meter"))
        self.wheelbase = float(gp("wheelbase_m"))
        self.encoder_signed = bool(gp("encoder_signed"))
        self.odom_frame = str(gp("odom_frame"))
        self.base_frame = str(gp("base_frame"))
        self.publish_tf = bool(gp("publish_tf"))

        if self.wheel_timeout_policy not in ("hold_last", "center"):
            raise ValueError("wheel_timeout_policy must be hold_last or center")
        if self.coasting_policy not in ("last_direction", "drop"):
            raise ValueError("coasting_policy must be last_direction or drop")
        if self.max_deg <= 0:
            raise ValueError("max_steer_deg must be > 0")
        self.ms_per_deg = self.steer_limit_ms / self.max_deg

        # ---------- 연결 상태 ----------
        self.ser = None
        self.ser_lock = threading.Lock()
        self.conn_state = CONN_CLOSED
        self._last_connect_attempt = 0.0
        self._port_opened_at = 0.0

        # ---------- 명령 상태 ----------
        self.cmd_stage = 0
        self.cmd_deg = 0
        self.last_drive_rx = 0.0
        self.last_wheel_rx = 0.0
        self.have_drive = False
        self.have_wheel = False
        self.wheel_dirty = False
        self.last_status_req = 0.0
        self.drive_timeout_warned = False
        self.wheel_timeout_warned = False

        # ---------- 안전 래치 ----------
        self.hard_stop = False          # /mcu_stop 최신값
        self.hard_stop_rx = 0.0         # 마지막 /mcu_stop 수신 시각
        self.hard_stop_active = False   # 실제 적용 중인지
        self.ros_estop_asserted = False
        self.estop_latched = False
        self.firmware_fault = 0
        self.firmware_fault_text = ""

        # ---------- 텔레메트리 ----------
        self.last_status_rx = 0.0
        self.telemetry_ok = False
        self._telemetry_warned = False

        # ---------- 오도메트리 ----------
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.distance_m = 0.0
        self.prev_count = None
        self.prev_count_t = None
        self.last_motion_dir = 0      # 코스팅 시 사용할 직전 진행 방향

        # ---------- ROS I/O ----------
        self.declare_parameter("input_drive_topic", "/mcu/cmd_drive")
        self.declare_parameter("input_wheel_topic", "/mcu/cmd_wheel")
        self.declare_parameter("input_stop_topic", "/mcu/cmd_stop")
        _dt = str(self.get_parameter("input_drive_topic").value)
        _wt = str(self.get_parameter("input_wheel_topic").value)
        _st = str(self.get_parameter("input_stop_topic").value)
        self.sub_drive = self.create_subscription(Float32, _dt, self.cb_drive, 10)
        self.sub_wheel = self.create_subscription(Int32, _wt, self.cb_wheel, 10)
        self.sub_estop = self.create_subscription(Bool, "/estop_lock", self.cb_estop, 10)
        self.sub_stop = self.create_subscription(Bool, _st, self.cb_stop, 10)

        # ---------- 발행 토픽 (전부 파라미터) ----------
        # 기본값은 카메라팀이 이미 쓰고 있는 이름 규약을 따른다.
        # 그쪽 브릿지를 이 노드로 교체해도 구독 코드를 안 고쳐도 되게 하기 위함.
        pubdefs = [
            ("pub_topic_encoder",     "/inpulse"),
            ("pub_topic_steer_angle", "/steer_angle"),
            ("pub_topic_rpm",         "/rpm"),
            ("pub_topic_steer_a0",    "/steer_a0"),
            ("pub_topic_steer_ms",    "/steer_position_ms"),
            ("pub_topic_speed_mps",   "/vehicle/speed_mps"),
            ("pub_topic_speed_kph",   "/vehicle/speed_kph"),
            ("pub_topic_distance",    "/vehicle/distance_m"),
            ("pub_topic_speed_valid", "/vehicle/speed_valid"),
            ("pub_topic_connected",   "/arduino/connected"),
            ("pub_topic_status",      "/arduino/status"),
            ("pub_topic_raw_status",  "/arduino/raw_status"),
            ("pub_topic_feedback_valid", "/arduino/feedback_valid"),
            ("pub_topic_fault",       "/arduino/fault"),
            ("pub_topic_fault_text",  "/arduino/fault_text"),
            ("pub_topic_ready",       "/vehicle_interface/ready"),
            ("pub_topic_iface_status","/vehicle_interface/status"),
            ("pub_topic_estop",       "/mcu/estop_latched"),
            ("pub_topic_hard_stop",   "/mcu/hard_stop_active"),
            ("pub_topic_odom",        "/odom"),
        ]
        for name, default in pubdefs:
            self.declare_parameter(name, default)
        pt = lambda n: str(self.get_parameter(n).value)

        # 구버전 이름으로도 같이 발행할지 (/drive, /wheel, /arduino/telemetry_ok)
        self.declare_parameter("publish_legacy_names", True)
        self.legacy = bool(self.get_parameter("publish_legacy_names").value)

        self.pub_conn = self.create_publisher(Bool, pt("pub_topic_connected"), 10)
        self.pub_status = self.create_publisher(String, pt("pub_topic_status"), 10)
        self.pub_raw = self.create_publisher(String, pt("pub_topic_raw_status"), 10)
        self.pub_fault = self.create_publisher(Int32, pt("pub_topic_fault"), 10)
        self.pub_fault_text = self.create_publisher(String, pt("pub_topic_fault_text"), 10)
        self.pub_tele_ok = self.create_publisher(
            Bool, pt("pub_topic_feedback_valid"), 10)
        self.pub_drive = self.create_publisher(Int32, pt("pub_topic_encoder"), 10)
        self.pub_wheel = self.create_publisher(Float32, pt("pub_topic_steer_angle"), 10)
        self.pub_rpm = self.create_publisher(Float32, pt("pub_topic_rpm"), 10)
        self.pub_a0 = self.create_publisher(Int32, pt("pub_topic_steer_a0"), 10)
        self.pub_steer_ms = self.create_publisher(Int32, pt("pub_topic_steer_ms"), 10)
        self.pub_estop = self.create_publisher(Bool, pt("pub_topic_estop"), 10)
        self.pub_hard_stop = self.create_publisher(Bool, pt("pub_topic_hard_stop"), 10)
        self.pub_speed = self.create_publisher(Float32, pt("pub_topic_speed_mps"), 10)
        self.pub_speed_kph = self.create_publisher(Float32, pt("pub_topic_speed_kph"), 10)
        self.pub_distance = self.create_publisher(Float32, pt("pub_topic_distance"), 10)
        self.pub_speed_valid = self.create_publisher(
            Bool, pt("pub_topic_speed_valid"), 10)
        self.pub_iface_ready = self.create_publisher(Bool, pt("pub_topic_ready"), 10)
        self.pub_iface_status = self.create_publisher(
            String, pt("pub_topic_iface_status"), 10)

        # 구버전 호환 (우리 문서·도구에서 쓰던 이름)
        if self.legacy:
            self.pub_drive_legacy = self.create_publisher(Int32, "/drive", 10)
            self.pub_wheel_legacy = self.create_publisher(Float32, "/wheel", 10)
            self.pub_tele_legacy = self.create_publisher(
                Bool, "/arduino/telemetry_ok", 10)
        else:
            self.pub_drive_legacy = None
            self.pub_wheel_legacy = None
            self.pub_tele_legacy = None
        self.pub_odom = self.create_publisher(Odometry, pt("pub_topic_odom"), 10)
        self.tf_bc = TransformBroadcaster(self) if (TF_OK and self.publish_tf) else None

        self.reset_srv = self.create_service(Trigger, "/mcu/reset_estop", self.cb_reset_estop)

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_timer = self.create_timer(self.send_period, self.tx_tick)
        self.conn_timer = self.create_timer(0.2, self.conn_tick)

        self.get_logger().info(
            "mcu_bridge v2: %s @ %d, send=%.1fHz, drive_timeout=%.2fs, "
            "wheel_timeout=%.2fs, 1deg=%.2fms, 직진보정=%+.1f도, cpm=%s"
            % (self.port, self.baud, send_hz, self.drive_timeout,
               self.wheel_timeout, self.ms_per_deg, self.steer_offset,
               ("%.2f" % self.cpm) if self.cpm > 0 else "미측정(odom 비활성)")
        )
        if self.cpm <= 0:
            self.get_logger().warn(
                "counts_per_meter=0 → /odom, /vehicle/speed_mps, "
                "/vehicle/distance_m 비활성. 실측 후 설정하세요.")

    # ============================================================
    # 구독 콜백
    # ============================================================

    def cb_drive(self, msg: Float32):
        stage = parse_drive_stage(float(msg.data))
        now = time.monotonic()
        if stage is None:
            self.get_logger().error(
                "invalid /mcu_drive=%r; 허용값 -1,0,1,2,3. 거부하고 정지." % msg.data)
            self.cmd_stage = 0
            self.have_drive = True
            self.last_drive_rx = now
            return
        self.cmd_stage = stage
        self.have_drive = True
        self.last_drive_rx = now
        self.drive_timeout_warned = False

    def cb_wheel(self, msg: Int32):
        deg = int(msg.data)
        if not valid_wheel_deg(deg, self.max_deg):
            self.get_logger().error(
                "invalid /mcu_wheel=%d; 허용범위 ±%d. 거부." % (deg, int(self.max_deg)))
            return
        if deg != self.cmd_deg:
            self.wheel_dirty = True
        self.cmd_deg = deg
        self.have_wheel = True
        self.last_wheel_rx = time.monotonic()
        self.wheel_timeout_warned = False

    def cb_stop(self, msg: Bool):
        """급정거. E-stop 과 달리 래치하지 않는다.

        장애물이 치워지면 다시 출발해야 하므로, 요청이 사라지면 자동 복귀.
        """
        was = self.hard_stop
        self.hard_stop = bool(msg.data)
        self.hard_stop_rx = time.monotonic()
        if self.hard_stop and not was:
            self.get_logger().warn("급정거 요청 수신 → 즉시 정지 (cmd=%s)" % self.estop_cmd)
        elif was and not self.hard_stop:
            self.get_logger().info("급정거 해제")

    def cb_estop(self, msg: Bool):
        self.ros_estop_asserted = bool(msg.data)
        if msg.data and not self.estop_latched:
            self.get_logger().error("ROS E-STOP asserted: latch set")
        if msg.data:
            self.estop_latched = True
        # false 는 의도적으로 래치를 풀지 않는다. /mcu/reset_estop 필요.

    def cb_reset_estop(self, _request, response):
        if self.ros_estop_asserted:
            response.success = False
            response.message = "reset denied: /estop_lock is still true"
            return response
        if self.firmware_fault != 0:
            response.success = False
            response.message = "reset denied: Arduino fault=%s" % self.firmware_fault_text
            return response
        self.estop_latched = False
        self.get_logger().info("E-stop latch cleared")
        response.success = True
        response.message = "E-stop latch cleared; fresh ROS commands are still required"
        return response

    # ============================================================
    # 연결 (논블로킹 상태머신)
    # ============================================================

    def _try_open(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < self.reconnect_delay:
            return
        self._last_connect_attempt = now

        # 자동 모드면 재연결 때마다 다시 찾는다.
        # USB 를 뽑았다 꽂으면 번호가 바뀔 수 있다.
        if self.auto_port:
            found = find_arduino_port(None)
            if found and found != self.port:
                self.get_logger().info(
                    "포트 변경 감지: %s -> %s" % (self.port, found))
                self.port = found

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as exc:
            msg = str(exc)
            if "busy" in msg.lower() or "denied" in msg.lower():
                self.get_logger().error(
                    "포트 점유/권한: %s | sudo chmod 666 %s | sudo fuser -k %s"
                    % (msg, self.port, self.port))
            else:
                self.get_logger().warn("serial unavailable: %s (재시도 중)" % msg)
            return
        with self.ser_lock:
            self.ser = ser
        self._port_opened_at = now
        self.conn_state = CONN_WAIT_RESET

        # 재연결 정책: 연결 이전 명령은 전부 폐기하고 정지부터 시작
        self.cmd_stage = 0
        self.cmd_deg = 0
        self.have_drive = False
        self.have_wheel = False
        self.wheel_dirty = False
        self.last_drive_rx = 0.0
        self.last_wheel_rx = 0.0
        self.prev_count = None          # 엔코더 기준점 재설정
        self.last_motion_dir = 0
        self.get_logger().info(
            "포트 열림: %s — 아두이노 리셋 %.1fs 대기" % (self.port, self.reset_wait))

    def _finish_connect(self):
        self.conn_state = CONN_READY
        self.send_line("1.00")
        if self.center_on_connect:
            cmd = wheel_serial_command(
                self.steer_offset, self.max_deg,
                self.steer_limit_ms, self.steer_mode)
            self.send_line(cmd)
            self.get_logger().info(
                "시리얼 준비 완료: STOP + 조향중앙(%s) 전송, 새 ROS 명령 대기" % cmd)
        else:
            self.get_logger().info(
                "시리얼 준비 완료: STOP 전송, 새 ROS 명령 대기 "
                "(조향은 이전 위치 유지 — center_steer_on_connect=false)")

    def close_serial(self):
        with self.ser_lock:
            ser = self.ser
            self.ser = None
        self.conn_state = CONN_CLOSED
        self.telemetry_ok = False
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def send_line(self, line: str) -> bool:
        with self.ser_lock:
            ser = self.ser
            if ser is None:
                return False
            try:
                ser.write((line + "\n").encode("ascii"))
                return True
            except serial.SerialException:
                pass
        self.get_logger().error("serial TX 실패; 연결 해제")
        self.close_serial()
        return False

    def conn_tick(self):
        now = time.monotonic()

        if self.conn_state == CONN_CLOSED:
            self._try_open()
        elif self.conn_state == CONN_WAIT_RESET:
            if now - self._port_opened_at >= self.reset_wait:
                self._finish_connect()

        # 텔레메트리 감시: 연결돼 있는데 STATUS 가 안 오면 경고
        if self.telemetry_timeout > 0 and self.conn_state == CONN_READY:
            fresh = (self.last_status_rx > 0.0
                     and now - self.last_status_rx <= self.telemetry_timeout)
            if fresh != self.telemetry_ok:
                if fresh:
                    self.get_logger().info("텔레메트리 복구")
                    self._telemetry_warned = False
                self.telemetry_ok = fresh
            if not fresh and not self._telemetry_warned and self.last_status_rx > 0.0:
                self.get_logger().error(
                    "STATUS 수신 두절 %.1fs — 엔코더/상태 갱신 없음. "
                    "구동 명령은 계속 전송 중이므로 주의." % self.telemetry_timeout)
                self._telemetry_warned = True

        m = Bool(); m.data = (self.conn_state == CONN_READY); self.pub_conn.publish(m)
        m = Bool(); m.data = self.estop_latched; self.pub_estop.publish(m)
        m = Bool(); m.data = self.telemetry_ok; self.pub_tele_ok.publish(m)
        if self.pub_tele_legacy:
            self.pub_tele_legacy.publish(m)

    # ============================================================
    # 송신
    # ============================================================

    def tx_tick(self):
        if self.conn_state != CONN_READY:
            return
        now = time.monotonic()

        drive_fresh = self.have_drive and (now - self.last_drive_rx <= self.drive_timeout)
        wheel_fresh = self.have_wheel and (now - self.last_wheel_rx <= self.wheel_timeout)

        # 급정거는 최우선. 오래된 true 는 해제로 본다(장애물이 치워졌을 수 있음).
        stop_fresh = self.hard_stop and (now - self.hard_stop_rx <= self.stop_timeout)
        if stop_fresh != self.hard_stop_active:
            self.hard_stop_active = stop_fresh
        if stop_fresh:
            self.send_line(self.estop_cmd)
            m = Bool(); m.data = True; self.pub_hard_stop.publish(m)
            if self.status_period > 0 and now - self.last_status_req >= self.status_period:
                if self.send_line("S"):
                    self.last_status_req = now
            return

        m = Bool(); m.data = False; self.pub_hard_stop.publish(m)

        if self.estop_latched:
            stage_to_send = 0
        elif drive_fresh:
            stage_to_send = self.cmd_stage
        else:
            stage_to_send = 0
            if self.have_drive and not self.drive_timeout_warned:
                self.get_logger().warn(
                    "/mcu_drive 타임아웃 → 구동 정지 (조향은 영향 없음)")
                self.drive_timeout_warned = True

        # 펌웨어 워치독(2초) 급식. 매 주기 전송.
        self.send_line(drive_serial_command(stage_to_send))

        # 조향 타임아웃은 구동을 절대 멈추지 않는다.
        if self.have_wheel and not wheel_fresh and not self.wheel_timeout_warned:
            self.get_logger().warn(
                "/mcu_wheel 타임아웃 → policy=%s; 구동은 독립적으로 계속"
                % self.wheel_timeout_policy)
            self.wheel_timeout_warned = True
            if self.wheel_timeout_policy == "center" and self.cmd_deg != 0:
                self.cmd_deg = 0
                self.wheel_dirty = True

        # E-stop 중에는 조향을 현재 위치에 동결한다 (자동 복귀 없음).
        if not self.estop_latched and self.wheel_dirty:
            # 직진 보정을 더한 뒤 물리 한계로 클램프한다.
            deg = self.cmd_deg + self.steer_offset
            deg = max(-self.max_deg, min(self.max_deg, deg))
            cmd = wheel_serial_command(
                deg, self.max_deg, self.steer_limit_ms, self.steer_mode)
            if self.send_line(cmd):
                self.wheel_dirty = False

        if self.status_period > 0 and now - self.last_status_req >= self.status_period:
            if self.send_line("S"):
                self.last_status_req = now

    # ============================================================
    # 수신
    # ============================================================

    def rx_loop(self):
        buf = b""
        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser
            if ser is None:
                buf = b""
                time.sleep(0.1)
                continue
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                self.close_serial()
                time.sleep(0.1)
                continue
            except Exception:
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk

            # 개행 없는 쓰레기가 계속 들어오는 경우 메모리 방어
            if len(buf) > MAX_RX_BUF:
                buf = buf[-1024:]

            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                try:
                    self.handle_line(raw.decode("utf-8", "replace").strip())
                except Exception as exc:
                    self.get_logger().warn("STATUS 처리 오류: %s" % exc)

    def handle_line(self, line: str):
        status = parse_status(line)
        if status is None:
            return

        self.last_status_rx = time.monotonic()

        m = String(); m.data = status["raw"]; self.pub_raw.publish(m)
        m = String(); m.data = status["state"]; self.pub_status.publish(m)
        m = Int32(); m.data = status["fault"]; self.pub_fault.publish(m)
        m = String(); m.data = status["fault_text"]; self.pub_fault_text.publish(m)
        m = Int32(); m.data = status["adc"]; self.pub_a0.publish(m)
        m = Float32(); m.data = status["rpm"]; self.pub_rpm.publish(m)
        m = Int32(); m.data = status["encoder_count"]; self.pub_drive.publish(m)
        if self.pub_drive_legacy:
            self.pub_drive_legacy.publish(m)
        m = Int32(); m.data = status["steer_ms"]; self.pub_steer_ms.publish(m)

        steer_deg = status["steer_ms"] / self.ms_per_deg
        m = Float32(); m.data = float(round(steer_deg, 1)); self.pub_wheel.publish(m)
        if self.pub_wheel_legacy:
            self.pub_wheel_legacy.publish(m)

        # 차량 인터페이스 상태 (카메라팀 규약)
        r = Bool()
        r.data = (self.conn_state == CONN_READY and not self.estop_latched
                  and status["fault"] == 0)
        self.pub_iface_ready.publish(r)
        s2 = String(); s2.data = status["state"]; self.pub_iface_status.publish(s2)

        self.update_odom(status["encoder_count"], status["steer_ms"])

        if self.firmware_fault != status["fault"]:
            if status["fault"] != 0:
                self.get_logger().error(
                    "Arduino fault=%s (state=%s)"
                    % (status["fault_text"], status["state"]))
            else:
                self.get_logger().info("Arduino fault 해소: %s" % status["fault_text"])
        self.firmware_fault = status["fault"]
        self.firmware_fault_text = status["fault_text"]

        if self.latch_on_firmware_fault and self.firmware_fault != 0:
            if not self.estop_latched:
                self.get_logger().error(
                    "Arduino fault=%s: E-stop 래치" % self.firmware_fault_text)
            self.estop_latched = True

    # ============================================================
    # 오도메트리
    # ============================================================

    def update_odom(self, count: int, steer_ms: int):
        valid = Bool(); valid.data = self.cpm > 0; self.pub_speed_valid.publish(valid)
        if self.cpm <= 0:
            return

        now = time.monotonic()
        if self.prev_count is None:
            self.prev_count = count
            self.prev_count_t = now
            return

        delta = count - self.prev_count
        dt = now - self.prev_count_t
        self.prev_count = count
        self.prev_count_t = now
        if dt <= 0.0:
            return

        # 부호 결정. 펌웨어 카운트가 무부호면 진행 방향을 브릿지가 붙인다.
        if not self.encoder_signed:
            if self.cmd_stage > 0:
                self.last_motion_dir = 1
                delta = abs(delta)
            elif self.cmd_stage < 0:
                self.last_motion_dir = -1
                delta = -abs(delta)
            else:
                # 정지 명령 상태. 브레이크가 없어 실제로는 코스팅 중일 수 있다.
                if self.coasting_policy == "last_direction" and self.last_motion_dir != 0:
                    delta = abs(delta) * self.last_motion_dir
                else:
                    delta = 0
        else:
            if delta > 0:
                self.last_motion_dir = 1
            elif delta < 0:
                self.last_motion_dir = -1

        steer_deg = steer_ms / self.ms_per_deg
        steer_rad = math.radians(steer_deg)

        # 엔코더가 앞축(조향축)에 있으므로 뒤축 이동거리로 환산: d_rear = d_front·cos(δ)
        d_front = delta / self.cpm
        d = d_front * math.cos(steer_rad)
        speed = d / dt
        self.distance_m += abs(d)

        dtheta = d * math.tan(steer_rad) / self.wheelbase
        if abs(dtheta) > 1e-9:
            radius = d / dtheta
            self.x += radius * (math.sin(self.th + dtheta) - math.sin(self.th))
            self.y -= radius * (math.cos(self.th + dtheta) - math.cos(self.th))
        else:
            self.x += d * math.cos(self.th)
            self.y += d * math.sin(self.th)
        self.th = math.atan2(math.sin(self.th + dtheta), math.cos(self.th + dtheta))

        m = Float32(); m.data = float(speed); self.pub_speed.publish(m)
        m = Float32(); m.data = float(speed * 3.6); self.pub_speed_kph.publish(m)
        m = Float32(); m.data = float(self.distance_m); self.pub_distance.publish(m)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.th * 0.5)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = speed * math.tan(steer_rad) / self.wheelbase
        self.pub_odom.publish(odom)

        if self.tf_bc:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = odom.pose.pose.orientation.z
            tf.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_bc.send_transform(tf)

    # ============================================================

    def shutdown(self):
        try:
            self.send_line("1.00")
            time.sleep(0.05)
        finally:
            self.close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = McuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
