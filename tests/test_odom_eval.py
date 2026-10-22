"""stereo/odom_eval.py 的统计逻辑单测（纯 Python，不依赖 ROS）。"""
from __future__ import annotations

import math

import pytest

from stereo.odom_eval import OdomStats


def test_square_loop_closure() -> None:
    s = OdomStats()
    pts = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0.0, 0.02, 0)]
    for i, p in enumerate(pts):
        s.update(i * 0.1, p, False, 100 + i, 5.0)
    r = s.summary()
    assert r["path_m"] == pytest.approx(3.98)
    assert r["closure_m"] == pytest.approx(0.02)
    assert r["drift_pct"] == pytest.approx(100 * 0.02 / 3.98)
    assert r["max_from_start_m"] == pytest.approx(math.sqrt(2))
    assert r["rate_hz"] == pytest.approx(5 / 0.4)
    assert r["inliers_median"] == 102 and r["inliers_min"] == 100


def test_lost_frames_excluded_from_path() -> None:
    s = OdomStats()
    s.update(0.0, (0, 0, 0), False, 50, 5.0)
    s.update(0.1, (0, 0, 0), True, 0, 5.0)  # 丢失时 RTAB-Map 发空位姿，不能算成跳回原点
    s.update(0.2, (0.5, 0, 0), False, 50, 5.0)
    r = s.summary()
    assert r["lost"] == 1 and r["frames"] == 3
    assert r["path_m"] == pytest.approx(0.5)


def test_empty_summary() -> None:
    r = OdomStats().summary()
    assert r["frames"] == 0 and r["rate_hz"] == 0.0
    assert math.isnan(r["closure_m"]) and math.isnan(r["drift_pct"])
