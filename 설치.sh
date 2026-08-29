#!/usr/bin/env bash
# ==========================================================
#  T870 MCU 설치 / 정리 스크립트   (2026-08-29)
#
#  하는 일
#    1. 지금 워크스페이스에 뭐가 있는지 훑고 충돌 후보를 찾는다
#    2. 옛날 t870_mcu 소스와 빌드 산출물을 백업 폴더로 "옮긴다"
#       (지우지 않는다. 문제 생기면 되돌릴 수 있게)
#    3. 이 폴더의 최신 파일을 넣는다
#    4. colcon build
#
#  쓰는 법
#    ./설치.sh              ← 뭘 할지 보여주기만 한다 (아무것도 안 건드림)
#    ./설치.sh --apply      ← 실제로 실행
#
#  워크스페이스 경로가 다르면
#    ./설치.sh --apply ~/내워크스페이스
# ==========================================================
set -u

APPLY=0
WS_DEFAULT="$HOME/T870_MCU"
WS=""

for arg in "$@"; do
  case "$arg" in
    --apply) APPLY=1 ;;
    -*)      echo "모르는 옵션: $arg"; exit 1 ;;
    *)       WS="$arg" ;;
  esac
done
[ -z "$WS" ] && WS="$WS_DEFAULT"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$HOME/t870_backup_$STAMP"

say()  { printf '%s\n' "$*"; }
head2() { printf '\n== %s ==\n' "$*"; }
run()  {
  if [ "$APPLY" -eq 1 ]; then
    say "  [실행] $*"
    "$@"
  else
    say "  [예정] $*"
  fi
}

if [ "$APPLY" -eq 0 ]; then
  say "########################################################"
  say "#  미리보기 모드 — 아무것도 바꾸지 않는다"
  say "#  실제로 하려면:  ./설치.sh --apply"
  say "########################################################"
fi

#  ★ 0830: 이 스크립트는 "zip 을 받은 팀원" 용이다.
#     git clone 안에서 돌리면 클론이 곧 워크스페이스인데도
#     ~/T870_MCU 라는 두 번째 워크스페이스를 새로 만들어 버린다.
#     그러면 "어느 쪽이 도는지 모르는" 상태가 된다.
if [ -d "$HERE/.git" ] && [ "$HERE" != "$WS" ]; then
  say ""
  say "########################################################"
  say "#  ⚠ 여기는 git 저장소다 ($HERE)"
  say "#"
  say "#  이 스크립트는 zip 을 받은 팀원용이다."
  say "#  git 으로 받았다면 클론 자체가 워크스페이스이므로"
  say "#  설치할 필요 없이 빌드만 하면 된다:"
  say "#"
  say "#      cd $HERE && colcon build --symlink-install"
  say "#"
  say "#  그래도 다른 워크스페이스에 설치하려면 경로를 직접 줄 것:"
  say "#      ./설치.sh --apply $HERE"
  say "########################################################"
  say ""
  if [ "$APPLY" -eq 1 ]; then
    say "  중단한다. 위 안내를 보고 다시 실행할 것."
    exit 1
  fi
fi

say ""
say "워크스페이스 : $WS"
say "새 파일 위치 : $HERE"
say "백업 폴더    : $BACKUP"

# ---------------------------------------------------------
head2 "0. 돌고 있는 노드 확인"
# ---------------------------------------------------------
#  ★ 패턴을 느슨하게 잡으면 이 스크립트 자신까지 잡힌다.
#    실제 ROS 노드 실행 형태만 좁게 본다.
FOUND_PROC=0
PROCS="$(pgrep -af 'ros2 (run|launch) t870_mcu|install/t870_mcu|mcu_bridge|mcu_manager|mission_manager wheel_odom' 2>/dev/null \
          | grep -v "설치.sh" | grep -v "^$$ " | grep -v "^$PPID ")"
if [ -n "$PROCS" ]; then
  printf '%s\n' "$PROCS" | sed 's/^/  /'
  FOUND_PROC=1
fi
if [ "$FOUND_PROC" -eq 1 ]; then
  say ""
  say "  ⚠ 위 노드들을 먼저 Ctrl-C 로 끄고 다시 실행할 것."
  say "    (시리얼 포트를 잡고 있으면 설치 후 브릿지가 안 뜬다)"
  [ "$APPLY" -eq 1 ] && { say ""; say "  중단한다."; exit 1; }
else
  say "  돌고 있는 노드 없음. 좋다."
fi

# ---------------------------------------------------------
head2 "1. t870_mcu 패키지 중복 찾기"
# ---------------------------------------------------------
#  ★ 워크스페이스 안에 옛 사본이 남아 있으면 colcon 빌드가 통째로 깨진다.
#    (백업을 워크스페이스 안에 만든 경우 등)
say "  워크스페이스 안 중복부터 본다 — 빌드가 여기서 깨진다"
WS_DUPES="$(find "$WS" -type d -name t870_mcu -not -path "$WS/src/t870_mcu*" 2>/dev/null)"
if [ -n "$WS_DUPES" ]; then
  printf '%s\n' "$WS_DUPES" | sed 's/^/    /'
  for d in $WS_DUPES; do
    holder="$(dirname "$d")"
    if [ ! -f "$holder/COLCON_IGNORE" ]; then
      say "    ⚠ $holder 에 COLCON_IGNORE 가 없다 → colcon 이 같이 잡아 빌드 실패"
      run touch "$holder/COLCON_IGNORE"
    fi
  done
else
  say "    워크스페이스 안 중복 없음"
fi
say ""

say "  홈 아래에서 t870_mcu 소스 폴더를 전부 찾는다..."
DUPES="$(find "$HOME" -maxdepth 5 -type d -name t870_mcu \
          -path "*/src/*" 2>/dev/null | sort)"
if [ -z "$DUPES" ]; then
  say "  없음 (처음 설치하는 것)"
else
  printf '%s\n' "$DUPES" | sed 's/^/  /'
  N=$(printf '%s\n' "$DUPES" | wc -l)
  if [ "$N" -gt 1 ]; then
    say ""
    say "  ⚠ 두 군데 이상이다. ROS 는 먼저 source 한 쪽을 쓴다."
    say "    이 스크립트는 '$WS' 것만 갱신한다."
    say "    나머지는 눈으로 확인하고 직접 정리할 것."
  fi
fi

# ---------------------------------------------------------
head2 "2. 백업"
# ---------------------------------------------------------
run mkdir -p "$BACKUP"

#  ★ colcon 은 폴더를 훑어 패키지를 찾는다. 백업 안의 옛 t870_mcu 가
#    같이 잡히면 "패키지 이름 중복" 으로 빌드가 통째로 실패한다.
#    (라이다팀이 실제로 이걸로 막혔다 — 0829)
#    COLCON_IGNORE 파일 하나면 colcon 이 그 폴더를 아예 안 본다.
run touch "$BACKUP/COLCON_IGNORE"

if [ -d "$WS/src/t870_mcu" ]; then
  say "  옛날 소스를 백업으로 옮긴다"
  run mv "$WS/src/t870_mcu" "$BACKUP/t870_mcu_src"
else
  say "  기존 소스 없음"
fi

for d in build install log; do
  if [ -d "$WS/$d" ]; then
    say "  빌드 산출물 $d/ 를 백업으로 옮긴다"
    say "    (여기에 옛날 코드가 남아 있으면 ros2 run 이 그걸 실행한다)"
    run mv "$WS/$d" "$BACKUP/$d"
  fi
done

if [ -d "$WS/tools" ]; then
  say "  옛날 tools/ 를 백업으로 옮긴다"
  run mv "$WS/tools" "$BACKUP/tools"
fi

# ---------------------------------------------------------
head2 "3. 새 파일 설치"
# ---------------------------------------------------------
run mkdir -p "$WS/src"
run cp -r "$HERE/src/t870_mcu" "$WS/src/t870_mcu"
run cp -r "$HERE/tools" "$WS/tools"

#  ★ 매번 손으로 export / source 하지 않도록 실행 스크립트를 같이 넣는다.
#    DOMAIN_ID·RMW 는 t870_env.sh 한 곳에만 있고, 팀 전체가 같은 값을 쓴다.
run cp "$HERE/t870_env.sh" "$WS/t870_env.sh"
run cp "$HERE/실행.sh"     "$WS/실행.sh"
run chmod +x "$WS/실행.sh" "$WS/t870_env.sh"
#  ★ 0830: 도구를 하나씩 적으면 새로 추가한 것이 빠진다.
#    실제로 odom_compare.py 가 빠져 있었다. 통째로 준다.
run chmod +x "$WS"/tools/*.py

# ---------------------------------------------------------
head2 "4. 빌드"
# ---------------------------------------------------------
if [ "$APPLY" -eq 1 ]; then
  #  ★ 0830: ROS 경로를 /opt/ros/jazzy 로 박아두지 않는다.
  #    배포판을 다르게 깐 사람이 있으면 여기서 바로 막힌다.
  ROS_SETUP=""
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
  else
    for d in /opt/ros/*/setup.bash; do
      [ -f "$d" ] && ROS_SETUP="$d"
    done
  fi

  if [ -z "$ROS_SETUP" ]; then
    say ""
    say "  ✗ ROS 2 를 못 찾았다 (/opt/ros/*/setup.bash 없음)."
    say "    ROS 2 Jazzy 를 먼저 설치할 것."
    exit 1
  fi

  say "  colcon build 시작 (1분쯤 걸린다) — $ROS_SETUP"
  #  🔴 0830: ROS 의 setup.bash 는 정의되지 않은 변수를 참조한다
  #     (AMENT_TRACE_SETUP_FILES, COLCON_TRACE 등).
  #     set -u 가 켜진 채로 source 하면 그 줄에서 unbound variable 로 죽는다.
  #     소싱하는 서브셸에서만 set +u 로 끈다.
  #     → 실행.sh 에서 이미 겪은 것과 같은 버그다. 여기에도 있었다.
  ( set +u; cd "$WS" && . "$ROS_SETUP" && colcon build --symlink-install ) \
    || { say ""; say "  ✗ 빌드 실패. 위 메시지를 SJ 한테 그대로 보낼 것."; exit 1; }
else
  say "  [예정] cd $WS && colcon build --symlink-install"
fi

# ---------------------------------------------------------
head2 "5. 정합성 감사"
#   소스와 yaml 이 서로 안 맞으면 노드가 죽거나 조용히 무시된다.
#   빌드 직후에 한 번 걸러낸다.
# ---------------------------------------------------------
if [ "$APPLY" -eq 1 ]; then
  if python3 "$WS/tools/audit.py" "$WS"; then
    say "  ✓ 정합성 문제 없음"
  else
    say ""
    say "  ✗ 위 항목을 SJ 한테 그대로 보낼 것. 이대로 두면 노드가 죽거나"
    say "    설정이 조용히 무시된다."
  fi
else
  say "  [예정] python3 $WS/tools/audit.py $WS"
fi

# ---------------------------------------------------------
head2 "6. 확인"
# ---------------------------------------------------------
if [ "$APPLY" -eq 1 ]; then
  say "  counts_per_meter (199.8 이어야 한다):"
  grep -n "counts_per_meter" "$WS/src/t870_mcu/config/t870_mcu.yaml" | sed 's/^/    /'
  say "  odom 토픽 / TF (/mcu/odom, false 여야 한다):"
  grep -n "pub_topic_odom\|publish_tf:" "$WS/src/t870_mcu/config/t870_mcu.yaml" | sed 's/^/    /'
  say ""
  say "  ✓ 끝. 백업은 여기 있다: $BACKUP"
  say ""
  say "  ─────────────────────────────────────────────"
  say "  이제부터는 이 세 개만 쓰면 된다."
  say "  source 도 export 도 스크립트가 알아서 한다."
  say ""
  say "    cd $WS"
  say "    ./실행.sh 점검      ← 지금 한 번 돌려볼 것"
  say "    ./실행.sh           브릿지 + 매니저"
  say "    ./실행.sh 조종      터미널로 직접 운전 (0 키로 조종권 획득)"
  say "  ─────────────────────────────────────────────"
else
  say ""
  say "  실제로 하려면:  ./설치.sh --apply"
fi
