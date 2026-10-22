"""双目 UVC 相机（SunplusIT "USB Global Camera"，左右拼接输出）的帧处理纯函数，不依赖 ROS，便于单测。"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

# 实测 2026-10-22：只有这三种模式是左右拼接双目，1280x720 / 640x480 等只出单目
# 1280x480 每目是 16:9 画面中间裁 4:3 再缩放，视场比 2560x720 窄；换模式必须重新标定
STEREO_MODES: Dict[Tuple[int, int], int] = {
    (2560, 720): 60,
    (1280, 480): 100,
    (1280, 400): 120,
}


@dataclass
class CameraInfoData:
    """camera_calibration_parsers 的 YAML 内容；未标定时矩阵全 0（ROS 约定）。"""

    width: int
    height: int
    distortion_model: str = "plumb_bob"
    d: List[float] = field(default_factory=list)
    k: List[float] = field(default_factory=lambda: [0.0] * 9)
    r: List[float] = field(default_factory=lambda: [0.0] * 9)
    p: List[float] = field(default_factory=lambda: [0.0] * 12)


def split_stereo_y(yuyv: np.ndarray, swap: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    """从 YUYV 原始帧（h×w×2）取 Y 平面并劈成左右两目，返回 (左目, 右目) 连续内存的 mono8 图。

    传感器是黑白的（实测 RGB 三通道完全相等），只用 Y 不丢信息。
    swap=True 时拼接帧右半当左目：哪半是哪目取决于两路排线的接法，见 AGENTS.md 5.8。
    """
    if yuyv.ndim != 3 or yuyv.shape[2] != 2:
        raise ValueError(f"期望 YUYV 原始帧 h×w×2，实际 {yuyv.shape}")
    w = yuyv.shape[1]
    if w % 2:
        raise ValueError(f"拼接帧宽度必须为偶数，实际 {w}")
    y = yuyv[..., 0]
    half = w // 2
    a, b = np.ascontiguousarray(y[:, :half]), np.ascontiguousarray(y[:, half:])
    return (b, a) if swap else (a, b)


def monotonic_to_wall_ns(buf_mono_ns: int, now_mono_ns: int, now_wall_ns: int) -> int:
    """把驱动给的单调时钟缓冲区时间戳换算到系统时间，消除用户态读取延迟。"""
    return now_wall_ns - (now_mono_ns - buf_mono_ns)


def load_camera_info(path: Path, width: int, height: int) -> CameraInfoData:
    """读 camera_calibration 导出的 YAML；分辨率与当前模式不符直接报错，防止用错标定。"""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    w, h = int(doc["image_width"]), int(doc["image_height"])
    if (w, h) != (width, height):
        raise ValueError(f"{path} 标定分辨率 {w}x{h} 与当前每目 {width}x{height} 不符")
    return CameraInfoData(
        width=w,
        height=h,
        distortion_model=doc.get("distortion_model", "plumb_bob"),
        d=[float(v) for v in doc["distortion_coefficients"]["data"]],
        k=[float(v) for v in doc["camera_matrix"]["data"]],
        r=[float(v) for v in doc["rectification_matrix"]["data"]],
        p=[float(v) for v in doc["projection_matrix"]["data"]],
    )


class DropCounter:
    """按相邻缓冲区时间戳的间隔统计丢帧：间隔超过 1.5 个周期就算丢了 round(间隔/周期)-1 帧。"""

    def __init__(self, fps: float) -> None:
        self.period_ns = 1e9 / fps
        self.last_ns: int | None = None
        self.frames = 0
        self.dropped = 0

    def update(self, buf_ns: int) -> None:
        if self.last_ns is not None:
            gap = buf_ns - self.last_ns
            if gap > 1.5 * self.period_ns:
                self.dropped += round(gap / self.period_ns) - 1
        self.last_ns = buf_ns
        self.frames += 1
