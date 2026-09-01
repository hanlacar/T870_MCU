#!/usr/bin/env bash
# ==========================================================
#  T870 공통 환경 — 팀 전체가 같은 값이어야 한다
#
#  ROS 2 는 DOMAIN_ID 나 RMW 가 하나라도 다르면
#  **노드끼리 서로 존재조차 모른다.** 에러도 안 난다.
#  "나는 되는데 너는 안 되는" 상황의 1순위 원인이다.
#
#  ⚠ 이 파일은 팀에서 한 벌만 유지한다. 각자 고치지 말 것.
#    값을 바꿔야 하면 MCU 담당(SJ)에게 말하고 전원이 같이 받는다.
# ==========================================================

# ---- DOMAIN / RMW ----
#
#  ★ 0831 변경 — 도메인을 **강제하지 않는다.**
#
#    지금은 팀마다 다른 값으로 실험하는 단계다. 여기서 특정 값을 export 하면
#    그 터미널만 몰래 다른 도메인으로 떠서, 다른 팀 노드가 통째로 안 보인다.
#    (에러도 경고도 안 난다. 0831 라이다팀 SLAM 이 이것 때문에 하루 막혔다.)
#
#    0901 확정: 팀 전체가 **42** 를 쓴다.
#    이 파일을 source 하면 자동으로 42 로 맞춰진다.
#    바꿔야 하면 여기 한 줄만 고치고 전원이 다시 받는다.
T870_TEAM_DOMAIN="42"        # 0901 팀 확정값. 비우면 강제하지 않음.

if [ -n "$T870_TEAM_DOMAIN" ]; then
  export ROS_DOMAIN_ID="$T870_TEAM_DOMAIN"
fi
export T870_TEAM_DOMAIN

#  RMW 도 마찬가지로 강제하지 않는다. 이미 설정돼 있으면 그것을 쓴다.
#  (팀 PC 는 대부분 rmw_fastrtps_cpp 가 기본이다)

# 다른 PC 의 노드가 보여야 하므로 켜 두면 안 된다
unset ROS_LOCALHOST_ONLY

# ---- ROS 본체 ----
if [ -f /opt/ros/jazzy/setup.bash ]; then
  . /opt/ros/jazzy/setup.bash
fi

# ---- 이 워크스페이스 ----
#  T870_WS 가 미리 잡혀 있으면 그것을 쓰고, 없으면 이 파일 위치에서 찾는다.
if [ -z "${T870_WS:-}" ]; then
  T870_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
if [ -f "$T870_WS/install/setup.bash" ]; then
  . "$T870_WS/install/setup.bash"
fi
export T870_WS
