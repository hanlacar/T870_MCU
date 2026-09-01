#!/usr/bin/env python3
"""프레임(TF) 정의 검증 — 순수 함수. ROS 없이 pytest 로 돌아간다.

t870_frames.yaml 을 static TF 로 쏘기 전에 구조가 성립하는지 본다.
TF 는 잘못돼도 **에러가 안 난다.** 그냥 조용히 이상한 좌표가 나오거나,
트리가 두 조각으로 갈라져 slam_toolbox 가 스캔을 전부 버린다.
그러면 원인이 TF 라는 걸 알아내는 데만 하루가 걸린다. 그래서 미리 잡는다.

검사 항목
  1. 부모 누락    — parent 가 비었거나 자기 자신
  2. 순환         — a -> b -> a
  3. 뿌리 여러 개 — 트리가 여러 조각으로 갈라짐
  4. 중복 정의    — 같은 프레임을 두 번 정의
  5. 필드 누락/타입 — x/y/z/roll/pitch/yaw 가 숫자가 아님
"""

REQUIRED_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def validate_frames(frames, expected_root="base_link"):
    """(ok, 문제목록) 반환. 문제목록은 사람이 읽는 문자열 리스트."""
    problems = []

    if not isinstance(frames, dict) or not frames:
        return False, ["frames 가 비었거나 딕셔너리가 아니다"]

    for name, spec in frames.items():
        if not isinstance(spec, dict):
            problems.append("%s: 정의가 딕셔너리가 아니다" % name)
            continue

        parent = str(spec.get("parent", "")).strip()
        if not parent:
            problems.append("%s: parent 가 비었다" % name)
        elif parent == name:
            problems.append("%s: parent 가 자기 자신이다" % name)

        for field in REQUIRED_FIELDS:
            if field not in spec:
                problems.append("%s: %s 가 없다" % (name, field))
            elif isinstance(spec[field], bool) or not isinstance(
                    spec[field], (int, float)):
                problems.append("%s: %s 가 숫자가 아니다 (%r)"
                                % (name, field, spec[field]))

    # ---- 순환 검사 ----
    for name in frames:
        seen = [name]
        cur = name
        for _ in range(len(frames) + 1):
            spec = frames.get(cur)
            if not isinstance(spec, dict):
                break
            parent = str(spec.get("parent", "")).strip()
            if parent not in frames:
                break                       # 트리 밖 = 뿌리에 닿았다
            if parent in seen:
                problems.append("순환: %s" % " -> ".join(seen + [parent]))
                break
            seen.append(parent)
            cur = parent

    # ---- 뿌리 검사 ----
    roots = set()
    for spec in frames.values():
        if isinstance(spec, dict):
            parent = str(spec.get("parent", "")).strip()
            if parent and parent not in frames:
                roots.add(parent)
    if len(roots) > 1:
        problems.append(
            "뿌리가 여러 개다: %s — TF 트리가 갈라진다" % sorted(roots))
    elif expected_root and roots and expected_root not in roots:
        problems.append(
            "뿌리가 '%s' 가 아니라 %s 다" % (expected_root, sorted(roots)))

    # 중복 정의는 yaml 파서가 이미 마지막 것만 남기므로 여기서는 못 잡는다.
    # 대신 원본 텍스트에서 세는 검사를 audit.py 가 한다.

    return (not problems), problems
