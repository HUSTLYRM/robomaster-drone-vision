"""stereo/sync_check.py 的拟合单测（需要 numpy/cv2，uv 环境下跳过）：
    python3 -m pytest tests/test_stereo_sync.py
"""
from __future__ import annotations

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("yaml")

from stereo.sync_check import fit_dt  # noqa: E402


@pytest.mark.parametrize("dt_true", [0.0, 0.002, -0.005])
def test_fit_dt_recovers_offset(dt_true: float) -> None:
    rng = np.random.default_rng(0)
    t = np.arange(0, 3, 0.01)
    t = np.delete(t, [50, 51, 120])  # 模拟丢帧
    yl = 240 + 150 * np.sin(2 * np.pi * 1.2 * t)  # 上下晃，峰值速度约 1100 px/s
    vy = 150 * 2 * np.pi * 1.2 * np.cos(2 * np.pi * 1.2 * t)
    dy = 0.05 - vy * dt_true + rng.normal(0, 0.05, t.size)
    dt, a, vmed, n = fit_dt(t, yl, dy)
    assert dt == pytest.approx(dt_true, abs=2e-4)
    assert a == pytest.approx(0.05, abs=0.02)
    assert vmed > 300 and n > 250


def test_fit_dt_rejects_too_few() -> None:
    t = np.arange(0, 0.05, 0.01)
    with pytest.raises(ValueError):
        fit_dt(t, t * 100, np.zeros_like(t))
