"""颜色候选检测：从相机帧中提取蓝/红色发光候选点。

阈值形状过滤允许拉长的连通域——运动模糊会把圆形灯点拉成条状，过严的
圆度过滤会直接漏检。

蓝色饱和度下限设得比红色高：实测过曝场景下白色高光边缘会产生低饱和度的
蓝紫色散伪影（显示器反光、金属高光），S 阈值低于 100 时大量误检；真实蓝色
LED 饱和度明显更高，收紧到 140 可以把这类伪色过滤掉。红色暂未遇到同类问题，
维持较低阈值以兼顾运动模糊掉饱和度的情况。两者都需要用真实灯具实测复核。

红色在 HSV 色相环上处于 0°/180° 两端，需要拆成两段阈值再合并，
不能像蓝色那样用单段 inRange。

两个入口结果逐像素相同（tests/test_vision_detector.py 对照）：
- detect_candidates(frame_bgr)：BGR 输入，离线图片用。
- detect_candidates_i420(i420)：相机原始 I420（V4L2 YU12）输入，机载用。
  先用 (U, V) 查表找出「任何亮度下都不可能落进阈值」的 2×2 块直接跳过，
  只把剩下区域转成 BGR/HSV，省掉整帧颜色转换（鲁班猫上取帧转 BGR 约 5.5 ms）。

形态学只在掩码非零区域外扩一圈的范围内计算，15×15 椭圆闭运算拆成 5 个
可分离矩形核的并集；两者都与整帧 morphologyEx 结果严格相同（鲁班猫上整帧
椭圆闭运算约 20 ms）。
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Callable

import cv2
import numpy as np

BLUE_HSV_LOW = np.array([100, 140, 140], dtype=np.uint8)
BLUE_HSV_HIGH = np.array([130, 255, 255], dtype=np.uint8)
RED_HSV_LOW_1 = np.array([0, 80, 150], dtype=np.uint8)
RED_HSV_HIGH_1 = np.array([8, 255, 255], dtype=np.uint8)
RED_HSV_LOW_2 = np.array([172, 80, 150], dtype=np.uint8)
RED_HSV_HIGH_2 = np.array([180, 255, 255], dtype=np.uint8)

MIN_AREA = 8.0
MAX_AREA = 20000.0
MAX_ASPECT_RATIO = 8.0

_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
# 近距离过曝灯点常呈"环形"：中心过曝到饱和度不足而被阈值排除，外圈色环
# 被分割成好几个碎片轮廓。闭运算用更大的核把这些碎片连成一个整体斑点。
_CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))


def _rect_decomposition(kernel: np.ndarray) -> list[np.ndarray]:
    """把中心对称的「阶梯形」核（如椭圆核）拆成若干居中矩形核，其并集等于原核。

    平坦结构元满足 dilate(X, A∪B) = max(dilate(X, A), dilate(X, B))，erode 同理取 min，
    矩形核在 OpenCV 里按行列可分离计算，比非可分离的椭圆核快得多。
    """
    kh, kw = kernel.shape
    widths = sorted({int(np.count_nonzero(row)) for row in kernel})
    rects = []
    for w in widths:
        h = int(sum(np.count_nonzero(row) >= w for row in kernel))
        rects.append(cv2.getStructuringElement(cv2.MORPH_RECT, (w, h)))
    # 拼回原核做校验：任何不满足假设的核（不对称、非阶梯形）直接报错，不静默算错
    union = np.zeros_like(kernel)
    for r in rects:
        h, w = r.shape
        if (kh - h) % 2 or (kw - w) % 2:
            raise ValueError("矩形与核的中心不重合")
        y0, x0 = (kh - h) // 2, (kw - w) // 2
        union[y0:y0 + h, x0:x0 + w] = 1
    if not np.array_equal(union, (kernel > 0).astype(kernel.dtype)):
        raise ValueError("核不能拆成居中矩形的并集")
    return rects


_CLOSE_RECTS = _rect_decomposition(_CLOSE_KERNEL)


def _close_by_rects(mask: np.ndarray) -> np.ndarray:
    dilated = cv2.dilate(mask, _CLOSE_RECTS[0])
    for r in _CLOSE_RECTS[1:]:
        cv2.max(dilated, cv2.dilate(mask, r), dst=dilated)
    closed = cv2.erode(dilated, _CLOSE_RECTS[0])
    for r in _CLOSE_RECTS[1:]:
        cv2.min(closed, cv2.erode(dilated, r), dst=closed)
    return closed


def _morph_in_roi(mask: np.ndarray, op: Callable[[np.ndarray], np.ndarray], kernel_size: int) -> np.ndarray:
    """只在掩码非零像素的外接框外扩 2r+1（r 为核半径）范围内做形态学运算。

    开、闭、膨胀的结果都落在「非零像素外扩 r」以内，而计算这一圈需要再往外 r 的
    输入；外扩 2r+1 后裁剪边界外的像素在整帧计算里本来就是 0，与 OpenCV 默认边界
    （膨胀按最小值、腐蚀按最大值）等效，所以结果与整帧计算严格相同。
    """
    x, y, w, h = cv2.boundingRect(mask)
    out = np.zeros_like(mask)
    if w == 0:
        return out
    pad = 2 * (kernel_size // 2) + 1
    y0, y1 = max(y - pad, 0), min(y + h + pad, mask.shape[0])
    x0, x1 = max(x - pad, 0), min(x + w + pad, mask.shape[1])
    out[y0:y1, x0:x1] = op(mask[y0:y1, x0:x1])
    return out


@dataclass
class Candidate:
    cx: float
    cy: float
    area: float
    bbox: tuple[int, int, int, int]
    color: str


def _extract_candidates(mask: np.ndarray, color: str) -> list[Candidate]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[Candidate] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = max(w, h) / max(min(w, h), 1)
        if aspect_ratio > MAX_ASPECT_RATIO:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        cx = m["m10"] / m["m00"]
        cy = m["m01"] / m["m00"]
        candidates.append(Candidate(cx=cx, cy=cy, area=area, bbox=(x, y, w, h), color=color))
    return candidates


def _color_masks(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """HSV 阈值分割，返回形态学处理前的 (蓝, 红) 掩码。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, BLUE_HSV_LOW, BLUE_HSV_HIGH)
    red = cv2.inRange(hsv, RED_HSV_LOW_1, RED_HSV_HIGH_1) | cv2.inRange(hsv, RED_HSV_LOW_2, RED_HSV_HIGH_2)
    return blue, red


def _clean_masks(blue: np.ndarray, red: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    blue = _morph_in_roi(blue, lambda m: cv2.morphologyEx(m, cv2.MORPH_OPEN, _MORPH_KERNEL), 3)
    blue = _morph_in_roi(blue, lambda m: cv2.dilate(m, _MORPH_KERNEL, iterations=1), 3)
    red = _morph_in_roi(red, _close_by_rects, 15)
    red = _morph_in_roi(red, lambda m: cv2.morphologyEx(m, cv2.MORPH_OPEN, _MORPH_KERNEL), 3)
    return blue, red


@lru_cache(maxsize=1)
def _chroma_lut() -> np.ndarray:
    """256×256 查表：lut[U, V] 非 0 表示存在某个亮度 Y 使该像素落进蓝或红阈值。

    直接用 OpenCV 自己的 I420→BGR→HSV 把 256³ 种 (Y, U, V) 全部算一遍，
    不推导系数，换 OpenCV 版本也与运行时转换一致。鲁班猫上首次调用约 0.5 s。
    """
    u_plane = np.repeat(np.arange(256, dtype=np.uint8)[:, None], 256, axis=1)  # 行 = U
    v_plane = np.repeat(np.arange(256, dtype=np.uint8)[None, :], 256, axis=0)  # 列 = V
    lut = np.zeros((256, 256), dtype=bool)
    # 每个 2×2 块共用一组 (U, V)，四个像素放四个不同的 Y；64 轮覆盖全部 256 个 Y
    quad = np.array([[0, 1], [2, 3]], dtype=np.uint16)
    for k in range(64):
        y_plane = np.tile(quad + 4 * k, (256, 256)).astype(np.uint8)
        i420 = np.concatenate([y_plane.ravel(), u_plane.ravel(), v_plane.ravel()]).reshape(512 * 3 // 2, 512)
        blue, red = _color_masks(cv2.cvtColor(i420, cv2.COLOR_YUV2BGR_I420))
        hit = (blue | red).reshape(256, 2, 256, 2).any(axis=(1, 3))
        lut |= hit
    return lut


@lru_cache(maxsize=1)
def _gray_radius() -> int:
    """查表为真的 (U, V) 离灰色 (128, 128) 的最小 L∞ 距离。

    「色度离灰色 ≥ 该距离」是查表的超集，用两次 absdiff 就能整帧算完，
    先用它缩小范围，再只对范围内的块查表（鲁班猫上整帧查表约 1.8～2.6 ms）。
    """
    u, v = np.meshgrid(np.arange(256), np.arange(256), indexing="ij")
    return int(np.maximum(np.abs(u - 128), np.abs(v - 128))[_chroma_lut()].min())


def detect_candidates(frame_bgr: np.ndarray) -> list[Candidate]:
    blue, red = _clean_masks(*_color_masks(frame_bgr))
    return _extract_candidates(blue, "blue") + _extract_candidates(red, "red")


def _i420_masks(i420: np.ndarray, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """I420 帧的形态学前 (蓝, 红) 掩码，只转换查表判定可能有目标色的外接框。"""
    buf = i420.reshape(-1)
    cw, ch = width // 2, height // 2
    y = buf[: width * height].reshape(height, width)
    u = buf[width * height: width * height + cw * ch].reshape(ch, cw)
    v = buf[width * height + cw * ch: width * height + 2 * cw * ch].reshape(ch, cw)
    blue = np.zeros((height, width), dtype=np.uint8)
    red = np.zeros((height, width), dtype=np.uint8)
    gray = np.full_like(u, 128)
    far = cv2.compare(cv2.max(cv2.absdiff(u, gray), cv2.absdiff(v, gray)), _gray_radius(), cv2.CMP_GE)
    fx, fy, fw, fh = cv2.boundingRect(far)
    if fw == 0:
        return blue, red
    possible = _chroma_lut()[u[fy:fy + fh, fx:fx + fw], v[fy:fy + fh, fx:fx + fw]].view(np.uint8)
    bx, by, bw, bh = cv2.boundingRect(possible)
    if bw == 0:
        return blue, red
    bx, by = bx + fx, by + fy
    # 色度坐标 ×2 即像素坐标；I420→BGR 每个像素只取所在 2×2 块的色度，按偶数边界裁剪不改变结果
    sub = np.concatenate([
        y[2 * by: 2 * (by + bh), 2 * bx: 2 * (bx + bw)].ravel(),
        u[by: by + bh, bx: bx + bw].ravel(),
        v[by: by + bh, bx: bx + bw].ravel(),
    ]).reshape(2 * bh * 3 // 2, 2 * bw)
    b, r = _color_masks(cv2.cvtColor(sub, cv2.COLOR_YUV2BGR_I420))
    blue[2 * by: 2 * (by + bh), 2 * bx: 2 * (bx + bw)] = b
    red[2 * by: 2 * (by + bh), 2 * bx: 2 * (bx + bw)] = r
    return blue, red


def detect_candidates_i420(i420: np.ndarray, width: int, height: int) -> list[Candidate]:
    """i420：V4L2 YU12 原始帧（任意形状，按字节顺序为 Y、U、V 三个平面）。"""
    blue, red = _clean_masks(*_i420_masks(i420, width, height))
    return _extract_candidates(blue, "blue") + _extract_candidates(red, "red")
