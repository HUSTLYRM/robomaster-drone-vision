"""stereo/rect_check.py 单测：理想标定下行差为 0、方格边长等于真值；标定偏了能看出来。
需要 numpy/cv2/yaml（系统 python3 有，uv 环境下跳过）：python3 -m pytest tests/test_stereo_rect_check.py
"""
from __future__ import annotations

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

from stereo.calibrate import Board  # noqa: E402
from stereo.rect_check import RectCalib, board_metrics, summarize  # noqa: E402

BOARD = Board(11, 8, 0.03)
F, CX, CY, B = 500.0, 320.0, 240.0, 0.086


def ideal() -> RectCalib:
    k = np.array([[F, 0, CX], [0, F, CY], [0, 0, 1]])
    p1 = np.hstack([k, np.zeros((3, 1))])
    p2 = p1.copy()
    p2[0, 3] = -F * B
    return RectCalib(k, np.zeros(5), np.eye(3), p1, k.copy(), np.zeros(5), np.eye(3), p2)


def project(dy_right: float = 0.0):
    """棋盘放在 0.8 m 处略倾斜，投影到理想双目；dy_right 模拟右目整体上下错位。"""
    obj = BOARD.object_points().astype(np.float64)
    rvec, tvec = np.array([0.2, -0.3, 0.05]), np.array([-0.15, -0.1, 0.8])
    k = ideal().k1
    pl, _ = cv2.projectPoints(obj, rvec, tvec, k, None)
    pr, _ = cv2.projectPoints(obj, rvec, tvec - np.array([B, 0, 0]), k, None)
    pr = pr + np.array([0, dy_right])
    return pl.astype(np.float32), pr.astype(np.float32)


def test_ideal_rig() -> None:
    row, sq, z = board_metrics(ideal(), *project(), BOARD)
    assert row == pytest.approx(0.0, abs=1e-3)
    assert sq == pytest.approx(0.03, rel=1e-3)
    assert 0.6 < z < 1.0  # 棋盘中心在 0.8 m，略倾斜


def test_detects_vertical_shift() -> None:
    row, _, _ = board_metrics(ideal(), *project(dy_right=2.0), BOARD)
    assert row == pytest.approx(-2.0, abs=1e-3)


def test_wrong_baseline_shows_in_square_size() -> None:
    cal = ideal()
    cal.p2[0, 3] = -F * B * 1.05  # 标定基线比实际大 5%
    _, sq, _ = board_metrics(cal, *project(), BOARD)
    assert sq == pytest.approx(0.03 * 1.05, rel=1e-3)


def test_summary_verdict() -> None:
    assert "仍有效" in summarize([0.1, -0.1], [0.0301, 0.0299], 0.03, [0.8, 0.8])
    assert "重标" in summarize([-2.0, -2.1], [0.03, 0.03], 0.03, [0.8, 0.8])
    assert "重标" in summarize([0.0], [0.0315], 0.03, [0.8])
