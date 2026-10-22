"""stereo/frames.py 单测。依赖 numpy/yaml（系统 python3 有，uv 环境没有），缺了就跳过：
    python3 -m pytest tests/test_stereo_frames.py
"""
from __future__ import annotations

from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("yaml")

from stereo.frames import DropCounter, load_camera_info, monotonic_to_wall_ns, split_stereo_y  # noqa: E402


def test_split_takes_y_and_halves() -> None:
    h, w = 4, 6
    raw = np.zeros((h, w, 2), np.uint8)
    raw[:, :3, 0] = 10  # 左目 Y
    raw[:, 3:, 0] = 200  # 右目 Y
    raw[..., 1] = 128  # UV 不该混进来
    left, right = split_stereo_y(raw)
    assert left.shape == right.shape == (h, 3)
    assert (left == 10).all() and (right == 200).all()
    assert left.flags["C_CONTIGUOUS"] and right.flags["C_CONTIGUOUS"]
    sl, sr = split_stereo_y(raw, swap=True)
    assert (sl == 200).all() and (sr == 10).all()


@pytest.mark.parametrize("shape", [(4, 6), (4, 6, 3), (4, 5, 2)])
def test_split_rejects_bad_shape(shape: tuple) -> None:
    with pytest.raises(ValueError):
        split_stereo_y(np.zeros(shape, np.uint8))


def test_monotonic_to_wall() -> None:
    # 缓冲区比现在早 10 ms，系统时间也应回推 10 ms
    assert monotonic_to_wall_ns(990_000_000, 1_000_000_000, 5_000_000_000) == 4_990_000_000


def test_drop_counter() -> None:
    dc = DropCounter(100)  # 周期 10 ms
    for t_ms in [0, 10, 20, 50, 60, 74]:  # 20→50 丢 2 帧；60→74 是 1.4 周期抖动，不算丢
        dc.update(t_ms * 1_000_000)
    assert dc.frames == 6
    assert dc.dropped == 2


CALIB_YAML = """\
image_width: 640
image_height: 480
camera_name: left
camera_matrix: {rows: 3, cols: 3, data: [500, 0, 320, 0, 500, 240, 0, 0, 1]}
distortion_model: plumb_bob
distortion_coefficients: {rows: 1, cols: 5, data: [0.1, -0.2, 0, 0, 0]}
rectification_matrix: {rows: 3, cols: 3, data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}
projection_matrix: {rows: 3, cols: 4, data: [500, 0, 320, -30, 0, 500, 240, 0, 0, 0, 1, 0]}
"""


def test_load_camera_info(tmp_path: Path) -> None:
    p = tmp_path / "left.yaml"
    p.write_text(CALIB_YAML, encoding="utf-8")
    info = load_camera_info(p, 640, 480)
    assert info.k[0] == 500.0 and info.k[2] == 320.0
    assert info.d == [0.1, -0.2, 0.0, 0.0, 0.0]
    assert info.p[3] == -30.0
    assert all(isinstance(v, float) for v in info.k + info.p)


def test_load_camera_info_rejects_wrong_resolution(tmp_path: Path) -> None:
    p = tmp_path / "left.yaml"
    p.write_text(CALIB_YAML, encoding="utf-8")
    with pytest.raises(ValueError, match="不符"):
        load_camera_info(p, 1280, 720)
