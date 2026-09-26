"""颜色候选检测：从 BGR 帧中提取蓝/红色发光候选点。

阈值形状过滤允许拉长的连通域——运动模糊会把圆形灯点拉成条状，过严的
圆度过滤会直接漏检。

蓝色饱和度下限设得比红色高：实测过曝场景下白色高光边缘会产生低饱和度的
蓝紫色散伪影（显示器反光、金属高光），S 阈值低于 100 时大量误检；真实蓝色
LED 饱和度明显更高，收紧到 140 可以把这类伪色过滤掉。红色暂未遇到同类问题，
维持较低阈值以兼顾运动模糊掉饱和度的情况。两者都需要用真实灯具实测复核。

红色在 HSV 色相环上处于 0°/180° 两端，需要拆成两段阈值再合并，
不能像蓝色那样用单段 inRange。
"""
from __future__ import annotations

from dataclasses import dataclass

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


def detect_candidates(frame_bgr: np.ndarray) -> list[Candidate]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    blue_mask = cv2.inRange(hsv, BLUE_HSV_LOW, BLUE_HSV_HIGH)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, _MORPH_KERNEL)
    blue_mask = cv2.dilate(blue_mask, _MORPH_KERNEL, iterations=1)

    red_mask = cv2.inRange(hsv, RED_HSV_LOW_1, RED_HSV_HIGH_1) | cv2.inRange(
        hsv, RED_HSV_LOW_2, RED_HSV_HIGH_2
    )
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, _CLOSE_KERNEL)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, _MORPH_KERNEL)

    return _extract_candidates(blue_mask, "blue") + _extract_candidates(red_mask, "red")
