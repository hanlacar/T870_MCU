#!/usr/bin/env python3
"""frame_contract 단위시험 + 실제 t870_frames.yaml 검증."""

import os
import yaml
import pytest

from t870_mcu.frame_contract import validate_frames

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAMES_YAML = os.path.join(HERE, "config", "t870_frames.yaml")


def _frame(parent="base_link", **kw):
    spec = {"parent": parent, "x": 0.0, "y": 0.0, "z": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    spec.update(kw)
    return spec


def test_valid_tree_passes():
    ok, problems = validate_frames({"laser": _frame(), "gps_link": _frame()})
    assert ok, problems


def test_missing_parent_caught():
    ok, problems = validate_frames({"laser": _frame(parent="")})
    assert not ok and any("parent" in p for p in problems)


def test_self_parent_caught():
    ok, problems = validate_frames({"laser": _frame(parent="laser")})
    assert not ok and any("자기 자신" in p for p in problems)


def test_cycle_caught():
    ok, problems = validate_frames({"a": _frame(parent="b"),
                                    "b": _frame(parent="a")})
    assert not ok and any("순환" in p for p in problems)


def test_split_tree_caught():
    """뿌리가 둘이면 TF 트리가 갈라진다 — slam 이 스캔을 버리는 원인."""
    ok, problems = validate_frames({"laser": _frame(parent="base_link"),
                                    "gps_link": _frame(parent="chassis")})
    assert not ok and any("뿌리" in p for p in problems)


def test_missing_field_caught():
    spec = _frame()
    del spec["pitch"]
    ok, problems = validate_frames({"laser": spec})
    assert not ok and any("pitch" in p for p in problems)


def test_non_numeric_field_caught():
    ok, problems = validate_frames({"laser": _frame(z="높음")})
    assert not ok and any("숫자가 아니다" in p for p in problems)


def test_bool_is_not_a_number():
    """True 는 파이썬에서 int 지만 좌표값으로는 오류다."""
    ok, problems = validate_frames({"laser": _frame(x=True)})
    assert not ok


def test_empty_frames_caught():
    ok, problems = validate_frames({})
    assert not ok


# ============================================================
# 실제 배포 yaml 검증 — 0901 실측값
# ============================================================

def _load():
    with open(FRAMES_YAML, encoding="utf-8") as handle:
        return yaml.safe_load(handle)["frames"]


def test_shipped_frames_yaml_is_valid():
    ok, problems = validate_frames(_load())
    assert ok, problems


def test_shipped_frames_measured_values():
    """0901 실측값이 그대로 들어 있는지 고정한다.

    원점 = 네 바퀴 정중앙, 지면. 바퀴 바깥 1.000 m 직사각형의 중심.
    """
    frames = _load()
    assert frames["front_laser"]["x"] == pytest.approx(0.730)
    assert frames["front_laser"]["z"] == pytest.approx(0.105)
    assert frames["rear_laser"]["x"] == pytest.approx(-0.680)
    assert frames["rear_laser"]["z"] == pytest.approx(0.155)
    assert frames["rear_laser"]["yaw"] == pytest.approx(3.14159265, abs=1e-6)
    assert frames["camera_link"]["x"] == pytest.approx(0.015)
    assert frames["camera_link"]["z"] == pytest.approx(0.970)
    assert frames["camera_link"]["pitch"] == pytest.approx(0.0872665, abs=1e-6)
    assert frames["gps_link"]["x"] == pytest.approx(0.450)
    assert frames["gps_link"]["z"] == pytest.approx(0.430)
    for name in ("front_laser", "rear_laser", "camera_link", "gps_link"):
        assert frames[name]["y"] == pytest.approx(0.0)


def test_legacy_laser_matches_front_laser():
    """구버전 이름 laser 는 front_laser 와 완전히 같은 자리여야 한다."""
    frames = _load()
    for field in ("x", "y", "z", "roll", "pitch", "yaw"):
        assert frames["laser"][field] == pytest.approx(
            frames["front_laser"][field])
