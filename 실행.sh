#!/usr/bin/env bash
# ==========================================================
#  T870 MCU 실행
#
#  ★ 창 하나에 하나씩 띄운다. 섞어 쓰지 말 것.
#
#    창1)  ./실행.sh 브릿지    아두이노 연결 (시리얼)
#    창2)  ./실행.sh 중재      명령 중재기
#    창3)  ./실행.sh 조종      WASD 수동 운전
#    창4)  ./실행.sh 프레임    센서 위치 TF
#
#  한 창에 브릿지+중재를 같이 띄우려면:
#    ./실행.sh 전체
#
#  그 밖에
#    ./실행.sh 점검      안 될 때 제일 먼저
#    ./실행.sh 상태      지금 뭐가 도는지
#
#  source 도 export 도 이 스크립트가 알아서 한다.
# ==========================================================
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export T870_WS="$HERE"

if [ ! -f "$HERE/t870_env.sh" ]; then
  echo "t870_env.sh 가 없다. 설치가 덜 됐다."
  echo "  → ./설치.sh --apply 를 먼저 실행할 것"
  exit 1
fi
# shellcheck disable=SC1090
. "$HERE/t870_env.sh"

#  ★ set -u 는 반드시 ROS 소싱 "뒤"에 켠다.
#    ROS/colcon 의 setup.bash 는 정의되지 않은 변수를 참조한다
#    ($COLCON_TRACE 등). set -u 가 켜져 있으면 그 줄에서
#    "unbound variable" 로 스크립트가 그대로 죽는다.
#    (0829: 이 순서를 반대로 두어 실행이 안 됐다. 라이다팀이 잡아줌)
set -u

MODE="${1:-도움말}"

#  ★ 점검은 "안 될 때" 돌리는 것이라 빌드가 없어도 실행되어야 한다.
#    나머지 명령만 빌드를 요구한다.
case "$MODE" in
  점검|check|상태|status|도움말|help|-h|--h*) NEED_BUILD=0 ;;
  *) NEED_BUILD=1 ;;
esac
if [ "$NEED_BUILD" = "1" ] && [ ! -f "$HERE/install/setup.bash" ]; then
  echo "빌드가 안 되어 있다 (install/ 없음)."
  echo "  → cd $HERE && colcon build --symlink-install"
  echo "  또는  ./실행.sh 점검   으로 무엇이 문제인지 먼저 볼 것"
  exit 1
fi

echo "───────────────────────────────────────────"
echo " 워크스페이스 : $T870_WS"
echo " DOMAIN_ID    : ${ROS_DOMAIN_ID:-미설정}"
echo " RMW          : ${RMW_IMPLEMENTATION:-기본}"
echo "───────────────────────────────────────────"

case "$MODE" in
  점검|check)
    exec python3 "$HERE/tools/preflight.py" --ws "$HERE"
    ;;
  상태|status)
    echo; echo "== 돌고 있는 노드 =="
    ros2 node list
    echo; echo "== 중요 토픽 발행자 수 (1 이 아니면 문제) =="
    for t in /odom /tf /mcu/cmd_drive /drive_mode /mcu/ready; do
      n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/.*Publisher count: //p')
      printf "  %-18s %s\n" "$t" "${n:-없음}"
    done
    ;;
  브릿지|bridge)
    echo
    echo " 브릿지만 띄운다 (아두이노 시리얼). 끄려면 Ctrl-C."
    echo " 중재기는 다른 창에서:  ./실행.sh 중재"
    echo
    exec ros2 launch t870_mcu t870_mcu.launch.py manager:=false
    ;;
  중재|manager)
    echo
    echo " 중재기만 띄운다 (명령 우선순위·모드). 끄려면 Ctrl-C."
    echo " 브릿지는 다른 창에서:  ./실행.sh 브릿지"
    echo
    exec ros2 launch t870_mcu t870_mcu.launch.py bridge:=false
    ;;
  프레임|frames|tf)
    echo
    echo " 센서 위치 static TF 를 띄운다. 끄려면 Ctrl-C."
    echo " 확인:  ros2 run tf2_ros tf2_echo base_link front_laser"
    echo
    exec ros2 launch t870_mcu t870_frames.launch.py
    ;;
  조종|drive)
    echo
    echo " ⚠ 브릿지가 다른 창에 떠 있어야 한다. 아니면 '아두이노 끊김' 이 뜬다."
    echo " ⚠ 창이 뜨면 0 키를 눌러 조종권을 잡아야 움직인다."
    echo "   (W A S D 주행 / F 급정거 / E E-Stop / Q 종료)"
    echo
    exec python3 "$HERE/tools/drive_wasd.py" --takeover
    ;;
  전체|all|실행|run)
    echo
    echo " 브릿지 + 중재기를 한 창에 띄운다. 끄려면 Ctrl-C."
    echo " 조종은 반드시 다른 창에서:  ./실행.sh 조종"
    echo
    exec ros2 launch t870_mcu t870_mcu.launch.py
    ;;
  도움말|help|-h|--help)
    echo
    echo "  창1)  ./실행.sh 브릿지    아두이노 연결 (시리얼)"
    echo "  창2)  ./실행.sh 중재      명령 중재기"
    echo "  창3)  ./실행.sh 조종      WASD 수동 운전"
    echo "  창4)  ./실행.sh 프레임    센서 위치 TF"
    echo
    echo "  ./실행.sh 전체   브릿지+중재를 한 창에"
    echo "  ./실행.sh 점검   안 될 때 제일 먼저"
    echo "  ./실행.sh 상태   지금 뭐가 도는지"
    echo
    ;;
  *)
    echo "모르는 명령: $MODE"
    echo
    echo "  창1)  ./실행.sh 브릿지    아두이노 연결"
    echo "  창2)  ./실행.sh 중재      명령 중재기"
    echo "  창3)  ./실행.sh 조종      WASD 수동 운전"
    echo "  창4)  ./실행.sh 프레임    센서 위치 TF"
    echo
    echo "  ./실행.sh 전체   브릿지+중재를 한 창에"
    echo "  ./실행.sh 점검   안 될 때 제일 먼저"
    echo "  ./실행.sh 상태   지금 뭐가 도는지"
    exit 1
    ;;
esac
