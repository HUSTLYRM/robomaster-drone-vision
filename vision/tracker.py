"""单目标卡尔曼跟踪器：在候选点中关联、预测、丢失重捕获。

恒速运动模型，用于在检测漏帧（遮挡/过曝/运动模糊导致的瞬时漏检）时
仍能给出预测位置，并把下一帧的关联窗口收紧到预测点附近，减少高速运动下
误关联到背景同色噪声的概率。
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

import cv2
import numpy as np

from color_detector import Candidate


class TrackState(Enum):
    SEARCHING = auto()
    TRACKING = auto()


@dataclass
class TrackResult:
    state: TrackState
    cx: float = 0.0
    cy: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    color: str = ""
    lost_frames: int = 0


class DartTracker:
    def __init__(self, max_lost_frames: int = 10, gate_radius_px: float = 80.0) -> None:
        self._max_lost_frames = max_lost_frames
        self._gate_radius_px = gate_radius_px
        self._kf = self._build_kalman()
        self._state = TrackState.SEARCHING
        self._lost_frames = 0
        self._color = ""

    @staticmethod
    def _build_kalman() -> cv2.KalmanFilter:
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.array(
            [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32
        )
        kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-1
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 4.0
        return kf

    def _predict(self) -> tuple[float, float]:
        pred = self._kf.predict()
        return float(pred[0]), float(pred[1])

    def _pick_best_candidate(
        self, candidates: list[Candidate], ref_xy: tuple[float, float] | None
    ) -> Candidate | None:
        if not candidates:
            return None
        if ref_xy is None:
            return max(candidates, key=lambda c: c.area)
        rx, ry = ref_xy
        gate_sq = self._gate_radius_px**2
        gated = [c for c in candidates if (c.cx - rx) ** 2 + (c.cy - ry) ** 2 <= gate_sq]
        pool = gated if gated else candidates
        return min(pool, key=lambda c: (c.cx - rx) ** 2 + (c.cy - ry) ** 2)

    def update(self, candidates: list[Candidate]) -> TrackResult:
        if self._state == TrackState.SEARCHING:
            best = self._pick_best_candidate(candidates, None)
            if best is None:
                return TrackResult(state=TrackState.SEARCHING)
            self._kf.statePost = np.array([[best.cx], [best.cy], [0.0], [0.0]], dtype=np.float32)
            self._state = TrackState.TRACKING
            self._lost_frames = 0
            self._color = best.color
            return TrackResult(state=TrackState.TRACKING, cx=best.cx, cy=best.cy, color=best.color)

        pred_x, pred_y = self._predict()
        best = self._pick_best_candidate(candidates, (pred_x, pred_y))
        if best is None:
            self._lost_frames += 1
            if self._lost_frames > self._max_lost_frames:
                self._state = TrackState.SEARCHING
                return TrackResult(state=TrackState.SEARCHING)
            vx, vy = float(self._kf.statePost[2]), float(self._kf.statePost[3])
            return TrackResult(
                state=TrackState.TRACKING,
                cx=pred_x,
                cy=pred_y,
                vx=vx,
                vy=vy,
                color=self._color,
                lost_frames=self._lost_frames,
            )

        self._lost_frames = 0
        self._color = best.color
        measurement = np.array([[best.cx], [best.cy]], dtype=np.float32)
        corrected = self._kf.correct(measurement)
        vx, vy = float(corrected[2]), float(corrected[3])
        return TrackResult(
            state=TrackState.TRACKING,
            cx=float(corrected[0]),
            cy=float(corrected[1]),
            vx=vx,
            vy=vy,
            color=best.color,
        )
