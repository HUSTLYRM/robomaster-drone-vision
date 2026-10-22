"""vision/ 识别加速的等价性测试：新实现必须与改造前的整帧算法逐像素一致。

依赖 numpy/cv2（系统 python3 有，uv 环境没有），缺了就跳过：
    python3 -m pytest tests/test_vision_detector.py
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision"))
import color_detector as cd  # noqa: E402
from frame_grabber import LatestFrameGrabber  # noqa: E402

W, H = 640, 480


# ---- 改造前的整帧算法（对照基准，逐字取自原 color_detector.detect_candidates） ----

def reference_masks(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv, cd.BLUE_HSV_LOW, cd.BLUE_HSV_HIGH)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, cd._MORPH_KERNEL)
    blue_mask = cv2.dilate(blue_mask, cd._MORPH_KERNEL, iterations=1)
    red_mask = cv2.inRange(hsv, cd.RED_HSV_LOW_1, cd.RED_HSV_HIGH_1) | cv2.inRange(
        hsv, cd.RED_HSV_LOW_2, cd.RED_HSV_HIGH_2
    )
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, cd._CLOSE_KERNEL)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, cd._MORPH_KERNEL)
    return blue_mask, red_mask


def reference_detect(frame_bgr: np.ndarray) -> list:
    blue, red = reference_masks(frame_bgr)
    return cd._extract_candidates(blue, "blue") + cd._extract_candidates(red, "red")


# ---- 合成场景 ----

def hsv_color(h: int, s: int, v: int) -> tuple[int, int, int]:
    bgr = cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def synthetic_scene(seed: int) -> np.ndarray:
    """灰色噪声背景上画灯点：标准蓝/红/橙、阈值附近的颜色、环形过曝灯、贴边、相邻、拖影。"""
    rng = np.random.default_rng(seed)
    img = rng.integers(20, 90, size=(H, W, 3), dtype=np.uint8)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    for _ in range(int(rng.integers(3, 12))):
        kind = rng.integers(0, 4)
        if kind == 0:  # 明显的蓝/红/橙
            color = [(255, 90, 10), (20, 20, 255), (0, 120, 255)][rng.integers(0, 3)]
        else:  # 阈值边界附近随机取色
            h = int(rng.choice([rng.integers(95, 136), rng.integers(0, 14), rng.integers(166, 180)]))
            color = hsv_color(h, int(rng.integers(60, 256)), int(rng.integers(120, 256)))
        cx, cy = int(rng.integers(-10, W + 10)), int(rng.integers(-10, H + 10))
        r = int(rng.integers(2, 25))
        shape = rng.integers(0, 3)
        if shape == 0:
            cv2.circle(img, (cx, cy), r, color, -1)
        elif shape == 1:  # 环形：中心过曝发白，外圈有色
            cv2.circle(img, (cx, cy), r + 4, color, -1)
            cv2.circle(img, (cx, cy), r, (255, 255, 255), -1)
        else:  # 运动拖影
            cv2.ellipse(img, (cx, cy), (r * 3, max(r // 3, 1)), float(rng.integers(0, 180)), 0, 360, color, -1)
    # 两个相距不到闭运算核宽的红点，检验能否被合并
    x, y = int(rng.integers(30, W - 60)), int(rng.integers(30, H - 30))
    cv2.circle(img, (x, y), 4, (20, 20, 255), -1)
    cv2.circle(img, (x + 13, y), 4, (20, 20, 255), -1)
    return img


def to_i420(bgr: np.ndarray) -> np.ndarray:
    """模拟相机 YU12 原始帧：OpenCV 关闭 RGB 转换后给出的是 (1, N) 的字节数组。"""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(1, -1)


def camera_bgr(i420: np.ndarray) -> np.ndarray:
    """改造前 OpenCV 在取帧时做的转换。"""
    return cv2.cvtColor(i420.reshape(H * 3 // 2, W), cv2.COLOR_YUV2BGR_I420)


# ---- 形态学 ----

def random_mask(rng: np.random.Generator, density: float) -> np.ndarray:
    m = (rng.random((H, W)) < density).astype(np.uint8) * 255
    for _ in range(int(rng.integers(0, 6))):
        cv2.circle(m, (int(rng.integers(-5, W + 5)), int(rng.integers(-5, H + 5))), int(rng.integers(1, 20)), 255,
                   int(rng.choice([-1, 2])))
    return m


@pytest.mark.parametrize("seed", range(12))
@pytest.mark.parametrize("density", [0.0, 0.0005, 0.02, 0.3])
def test_close_matches_full_frame_ellipse(seed: int, density: float) -> None:
    m = random_mask(np.random.default_rng(seed), density)
    want = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cd._CLOSE_KERNEL)
    assert np.array_equal(cd._close_by_rects(m), want)
    assert np.array_equal(cd._morph_in_roi(m, cd._close_by_rects, 15), want)


@pytest.mark.parametrize("seed", range(12))
def test_small_kernel_ops_in_roi_match_full_frame(seed: int) -> None:
    m = random_mask(np.random.default_rng(seed), 0.001)
    def open_(x):
        return cv2.morphologyEx(x, cv2.MORPH_OPEN, cd._MORPH_KERNEL)
    def dilate(x):
        return cv2.dilate(x, cd._MORPH_KERNEL, iterations=1)
    assert np.array_equal(cd._morph_in_roi(m, open_, 3), open_(m))
    assert np.array_equal(cd._morph_in_roi(m, dilate, 3), dilate(m))


def test_close_on_empty_and_corner_pixels() -> None:
    m = np.zeros((H, W), np.uint8)
    assert not cd._morph_in_roi(m, cd._close_by_rects, 15).any()
    for y, x in [(0, 0), (0, W - 1), (H - 1, 0), (H - 1, W - 1), (1, 7)]:
        m = np.zeros((H, W), np.uint8)
        m[y, x] = 255
        want = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cd._CLOSE_KERNEL)
        assert np.array_equal(cd._morph_in_roi(m, cd._close_by_rects, 15), want)


def test_rect_decomposition_rejects_non_staircase_kernel() -> None:
    k = np.ones((5, 5), np.uint8)
    k[2, 2] = 0  # 中间挖洞，不可能是矩形并集
    with pytest.raises(ValueError):
        cd._rect_decomposition(k)


# ---- 整条识别链路 ----

@pytest.mark.parametrize("seed", range(30))
def test_i420_detection_matches_reference(seed: int) -> None:
    i420 = to_i420(synthetic_scene(seed))
    bgr = camera_bgr(i420)
    want_masks = reference_masks(bgr)
    got_masks = cd._clean_masks(*cd._i420_masks(i420, W, H))
    assert np.array_equal(got_masks[0], want_masks[0])
    assert np.array_equal(got_masks[1], want_masks[1])
    assert cd.detect_candidates_i420(i420, W, H) == reference_detect(bgr)
    assert cd.detect_candidates(bgr) == reference_detect(bgr)


def test_synthetic_scenes_are_not_vacuous() -> None:
    """对照测试要有实际检出，否则「都是空列表」也会通过。"""
    counts = [len(reference_detect(camera_bgr(to_i420(synthetic_scene(s))))) for s in range(30)]
    assert sum(1 for c in counts if c > 0) >= 25
    colors = {c.color for s in range(30) for c in reference_detect(camera_bgr(to_i420(synthetic_scene(s))))}
    assert colors == {"blue", "red"}


@pytest.mark.parametrize("seed", range(5))
def test_i420_random_noise_matches_reference(seed: int) -> None:
    """全随机 Y/U/V：几乎每个 2×2 块的色度都不同，覆盖查表的大部分取值，且走整帧裁剪分支。"""
    i420 = np.random.default_rng(seed).integers(0, 256, size=(1, W * H * 3 // 2), dtype=np.uint8)
    bgr = camera_bgr(i420)
    got = cd._clean_masks(*cd._i420_masks(i420, W, H))
    want = reference_masks(bgr)
    assert np.array_equal(got[0], want[0]) and np.array_equal(got[1], want[1])
    assert want[1].any()


def test_i420_blank_frame_has_no_candidates() -> None:
    i420 = to_i420(np.full((H, W, 3), 128, np.uint8))
    assert cd.detect_candidates_i420(i420, W, H) == []


def test_chroma_lut_is_exact_per_uv() -> None:
    """查表为假的 (U, V)，所有亮度下都不能落进阈值；为真的至少有一个亮度落进阈值。"""
    lut = cd._chroma_lut()
    rng = np.random.default_rng(0)
    uv = rng.integers(0, 256, size=(400, 2))
    y_plane = np.arange(256, dtype=np.uint8).reshape(16, 16)  # 256 个亮度，色度全图都取 (u, v)
    h, w = y_plane.shape
    for u, v in uv:
        i420 = np.concatenate([y_plane.ravel(), np.full(h * w // 4, u, np.uint8), np.full(h * w // 4, v, np.uint8)])
        blue, red = cd._color_masks(cv2.cvtColor(i420.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_I420))
        assert bool((blue | red).any()) == bool(lut[u, v]), (u, v)


def test_gray_radius_prefilter_is_superset_of_lut() -> None:
    """粗筛「色度离灰色 ≥ 半径」必须覆盖查表为真的全部 (U, V)，否则会漏检。"""
    lut = cd._chroma_lut()
    u, v = np.meshgrid(np.arange(256), np.arange(256), indexing="ij")
    dist = np.maximum(np.abs(u - 128), np.abs(v - 128))
    assert (dist[lut] >= cd._gray_radius()).all()
    assert (dist[lut] == cd._gray_radius()).any()  # 取的是最小值，不是随便一个更小的数


# ---- 最新帧线程 ----

class FakeCap:
    """每 period 秒出一帧，图像内容即帧号；fail=True 时一直读失败。"""

    def __init__(self, fail: bool = False, period: float = 0.005) -> None:
        self.n = 0
        self.fail = fail
        self.period = period
        self.lock = threading.Lock()

    def read(self) -> tuple[bool, np.ndarray | None]:
        time.sleep(self.period)
        if self.fail:
            return False, None
        with self.lock:
            self.n += 1
            return True, np.array([self.n])

    def get(self, prop: int) -> float:
        return float(self.n) * 5.0


def test_grabber_always_returns_latest_frame() -> None:
    cap = FakeCap()
    grabber = LatestFrameGrabber(cap).start()
    try:
        last = 0
        seqs = []
        for _ in range(8):
            f = grabber.wait_newer(last)
            assert f is not None and f.seq > last
            assert int(f.image[0]) == f.seq  # 拿到的就是出队时的那一帧
            with cap.lock:
                assert cap.n - f.seq <= 1  # 不比相机最新帧落后超过 1 帧
            seqs.append(f.seq)
            last = f.seq
            time.sleep(0.03)  # 模拟处理比帧周期慢
        assert max(b - a for a, b in zip(seqs, seqs[1:])) >= 3  # 处理慢时旧帧被跳过
    finally:
        grabber.stop()


def test_grabber_waits_for_next_frame_instead_of_repeating() -> None:
    grabber = LatestFrameGrabber(FakeCap(period=0.3)).start()
    try:
        f = grabber.wait_newer(0)
        assert f is not None
        assert grabber.wait_newer(f.seq, timeout=0.05) is None  # 相机还没出下一帧，不能重复给旧帧
        nxt = grabber.wait_newer(f.seq, timeout=1.0)  # 要阻塞等到下一帧，而不是立刻返回空让主循环空转
        assert nxt is not None and nxt.seq == f.seq + 1
    finally:
        grabber.stop()


def test_grabber_times_out_when_camera_fails() -> None:
    grabber = LatestFrameGrabber(FakeCap(fail=True)).start()
    try:
        t0 = time.monotonic()
        assert grabber.wait_newer(0, timeout=0.2) is None
        assert time.monotonic() - t0 < 1.0
    finally:
        grabber.stop()
