#!/usr/bin/env bash
# ==========================================================
#  T870 MCU 설치 스크립트
#
#  포트 권한과 udev 규칙을 한 번에 잡는다.
#  포트 번호(/dev/ttyACM0, ACM1 ...)는 USB 를 꽂는 순서에 따라
#  매번 바뀌므로, 번호 대신 장치 정보로 고정한다.
#
#  사용:
#      chmod +x setup.sh
#      ./setup.sh
# ==========================================================
set -e

echo "=========================================================="
echo "  T870 MCU 설치"
echo "=========================================================="

# ---------- 1. 의존 패키지 ----------
echo ""
echo "[1/5] 의존 패키지"
sudo apt-get update -qq
sudo apt-get install -y python3-serial python3-colcon-common-extensions >/dev/null
echo "      python3-serial, colcon 확인"

# ---------- 2. dialout 그룹 ----------
echo ""
echo "[2/5] 시리얼 포트 권한"
if groups "$USER" | grep -qw dialout; then
    echo "      이미 dialout 그룹에 속해 있다"
else
    sudo usermod -aG dialout "$USER"
    echo "      dialout 그룹에 추가했다"
    echo "      ★ 로그아웃 후 다시 로그인해야 적용된다"
    echo "        (지금 바로 쓰려면 아래 chmod 를 쓴다)"
fi

# ---------- 3. udev 규칙 ----------
echo ""
echo "[3/5] udev 규칙 — 포트 이름 고정"
sudo tee /etc/udev/rules.d/99-t870.rules > /dev/null << 'RULES'
# T870 자율주행 차량 — 포트 고정
#
# 꽂는 순서와 무관하게 항상 같은 이름을 쓴다.
#   /dev/t870_mcu    아두이노 Mega (하위제어기)
#   /dev/t870_gps    u-blox GNSS
#   /dev/t870_lidar  RPLIDAR (CP2102)
#
# 벤더 ID 기준이라 어느 컴퓨터에서도 동일하게 동작한다.

# 아두이노 (정품: 2341 / 클론 CH340: 1a86)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666", SYMLINK+="t870_mcu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2a03", MODE="0666", SYMLINK+="t870_mcu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", SYMLINK+="t870_mcu"

# u-blox GNSS
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", MODE="0666", SYMLINK+="t870_gps"

# CP2102 (RPLIDAR 등)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", SYMLINK+="t870_lidar"
RULES

sudo udevadm control --reload-rules
sudo udevadm trigger
echo "      /etc/udev/rules.d/99-t870.rules 적용"

# ---------- 4. 현재 포트에 즉시 권한 ----------
echo ""
echo "[4/5] 현재 연결된 포트에 즉시 권한 부여"
found=0
for p in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$p" ] || continue
    sudo chmod 666 "$p"
    echo "      $p"
    found=1
done
[ $found -eq 0 ] && echo "      연결된 시리얼 포트가 없다"

# ---------- 5. 확인 ----------
echo ""
echo "[5/5] 결과"
echo ""
if [ -e /dev/t870_mcu ]; then
    echo "      /dev/t870_mcu  ->  $(readlink -f /dev/t870_mcu)"
else
    echo "      /dev/t870_mcu 없음 — 아두이노가 연결되지 않았거나"
    echo "      USB 를 뽑았다 다시 꽂아야 규칙이 적용된다"
fi
[ -e /dev/t870_gps ]   && echo "      /dev/t870_gps    ->  $(readlink -f /dev/t870_gps)"
[ -e /dev/t870_lidar ] && echo "      /dev/t870_lidar  ->  $(readlink -f /dev/t870_lidar)"

echo ""
echo "=========================================================="
echo "  설치 완료"
echo "=========================================================="
cat << 'NEXT'

  빌드:
      cd ~/mcu_ws && colcon build --symlink-install
      source install/setup.bash

  실행 (포트를 적을 필요가 없다. 자동으로 찾는다):
      ros2 launch t870_mcu t870_mcu.launch.py

  조종:
      python3 tools/drive_wasd.py

  포트가 안 잡히면:
      python3 tools/check_ports.py

NEXT
