"""stereo/calibrate.py 单测：用已知双目参数合成角点，检验标定能把参数算回来。

依赖 numpy/cv2/yaml（系统 python3 有，uv 环境没有），缺了就跳过：
    python3 -m pytest tests/test_stereo_calibrate.py
"""
from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

from stereo.calibrate import (  # noqa: E402
    Board,
    calibrate_stereo,
    coverage,
    is_new_pose,
    is_still,
    rectified_row_error,
    view_params,
    write_opencv_stereo,
    write_ros_yaml,
)
from stereo.frames import load_camera_info  # noqa: E402

BOARD = Board(11, 8, 0.03)
SIZE = (640, 480)
K_TRUE = np.array([[420.0, 0, 322.0], [0, 421.0, 238.0], [0, 0, 1]])
D_TRUE = np.array([-0.12, 0.05, 0.001, -0.0005, 0.0])
R_TRUE, _ = cv2.Rodrigues(np.array([0.004, -0.006, 0.002]))
T_TRUE = np.array([-0.025, 0.0003, -0.0002])  # 右目在左目右侧 25 mm


def synth_views(n: int, noise_px: float, seed: int = 0) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    rng = np.random.default_rng(seed)
    obj = BOARD.object_points().astype(np.float64)
    center = obj.mean(axis=0)
    left, right = [], []
    while len(left) < n:
        rvec = rng.uniform(-0.5, 0.5, 3)
        z = rng.uniform(0.35, 0.9)
        tvec = np.array([rng.uniform(-0.12, 0.12) * z, rng.uniform(-0.08, 0.08) * z, z])
        rmat, _ = cv2.Rodrigues(rvec)
        # 绕棋盘中心转，棋盘中心落在 tvec
        cam_l = (rmat @ (obj - center).T).T + tvec
        cam_r = (R_TRUE @ cam_l.T).T + T_TRUE
        pl, _ = cv2.projectPoints(cam_l, np.zeros(3), np.zeros(3), K_TRUE, D_TRUE)
        pr, _ = cv2.projectPoints(cam_r, np.zeros(3), np.zeros(3), K_TRUE, D_TRUE)
        pl, pr = pl.reshape(-1, 2), pr.reshape(-1, 2)
        inside = all(((p >= 5) & (p < np.array(SIZE) - 5)).all() for p in (pl, pr))
        if not inside:
            continue
        left.append((pl + rng.normal(0, noise_px, pl.shape)).reshape(-1, 1, 2).astype(np.float32))
        right.append((pr + rng.normal(0, noise_px, pr.shape)).reshape(-1, 1, 2).astype(np.float32))
    return left, right


def test_calibration_recovers_rig() -> None:
    left, right = synth_views(30, noise_px=0.1)
    cal = calibrate_stereo(BOARD, left, right, SIZE)
    assert cal.dropped == []
    assert cal.baseline * 1000 == pytest.approx(25.0, rel=0.01)
    assert cal.k1[0, 0] == pytest.approx(420.0, rel=0.01)
    assert cal.k2[1, 2] == pytest.approx(238.0, abs=2.0)
    assert cal.rms_stereo < 0.3
    mean_dy, max_dy = rectified_row_error(cal, left, right)
    # 每点 0.1 px 噪声，左右之差的平均绝对值理论下限 ≈ 0.1·√2·0.8 = 0.113 px
    assert mean_dy < 0.15 and max_dy < 0.8
    # ROS 双目约定：右目 P 的 Tx = -fx * 基线
    assert cal.p2[0, 3] == pytest.approx(-cal.p2[0, 0] * cal.baseline, rel=0.01)


def test_outlier_view_dropped() -> None:
    left, right = synth_views(25, noise_px=0.1, seed=1)
    rng = np.random.default_rng(9)
    right[7] = right[7] + rng.normal(0, 6.0, right[7].shape).astype(np.float32)
    cal = calibrate_stereo(BOARD, left, right, SIZE)
    assert cal.dropped == [7]
    assert cal.used == 24
    assert cal.baseline * 1000 == pytest.approx(25.0, rel=0.01)


def test_stereo_inconsistent_view_dropped() -> None:
    """右目整体平移几像素：两个单目各自仍是合法投影（单目误差小），但两目对不上，要靠双目误差剔掉。"""
    left, right = synth_views(25, noise_px=0.1, seed=5)
    right[11] = right[11] + np.array([[[4.0, 3.0]]], np.float32)
    cal = calibrate_stereo(BOARD, left, right, SIZE)
    assert 11 in cal.dropped
    assert cal.rms_stereo < 0.3
    assert cal.baseline * 1000 == pytest.approx(25.0, rel=0.01)


def test_view_params_and_selection() -> None:
    left, _ = synth_views(1, noise_px=0.0, seed=2)
    p = view_params(left[0], BOARD, SIZE)
    assert all(0.0 <= v <= 1.0 for v in p)
    assert not is_new_pose(p, [p])
    assert is_new_pose(p, [])
    assert coverage([]) == [0.0] * 4
    assert coverage([(0.0, 0.0, 0.1, 0.0), (1.0, 1.0, 0.9, 1.0)]) == [1.0] * 4


def test_is_still() -> None:
    a = np.zeros((88, 1, 2), np.float32)
    assert not is_still(None, a)
    assert is_still(a, a + 0.5)
    assert not is_still(a, a + 2.0)


def test_yaml_roundtrip(tmp_path: Path) -> None:
    left, right = synth_views(20, noise_px=0.1, seed=3)
    cal = calibrate_stereo(BOARD, left, right, SIZE)
    p = tmp_path / "right.yaml"
    write_ros_yaml(p, "stereo_right", cal.size, cal.k2, cal.d2, cal.r2, cal.p2)
    info = load_camera_info(p, *SIZE)
    assert info.p[3] == pytest.approx(float(cal.p2[0, 3]))
    assert len(info.d) == 5 and len(info.k) == 9 and len(info.r) == 9 and len(info.p) == 12


def test_opencv_stereo_roundtrip(tmp_path: Path) -> None:
    left, right = synth_views(20, noise_px=0.1, seed=4)
    cal = calibrate_stereo(BOARD, left, right, SIZE)
    p = tmp_path / "stereo.yml"
    write_opencv_stereo(p, cal)
    fs = cv2.FileStorage(str(p), cv2.FILE_STORAGE_READ)
    assert int(fs.getNode("image_width").real()) == 640
    assert np.allclose(fs.getNode("P2").mat(), cal.p2)
    assert np.allclose(fs.getNode("D1").mat().ravel(), cal.d1.ravel())
    assert fs.getNode("baseline_m").real() == pytest.approx(cal.baseline)
    fs.release()
