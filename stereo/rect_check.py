#!/usr/bin/env python3
"""标定是否还有效：对着棋盘格采几十帧，算校正后左右行差（旋转是否变了）和双目三角化的方格边长（基线/尺度是否变了）。

支架不够刚性，碰过、拆装、换线后都该跑一次；行差中位 > 0.5 px 或边长偏差 > 1% 就该重标。
直接开相机（不经 ROS），相机不能被别的程序占用。

用法（仓库根目录）：
    python3 -m stereo.rect_check --cols 11 --rows 8 --square 0.03
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

from stereo.calibrate import Board, detect
from stereo.frames import split_stereo_y


@dataclass
class RectCalib:
    k1: np.ndarray
    d1: np.ndarray
    r1: np.ndarray
    p1: np.ndarray
    k2: np.ndarray
    d2: np.ndarray
    r2: np.ndarray
    p2: np.ndarray

    @property
    def baseline(self) -> float:
        return float(-self.p2[0, 3] / self.p2[0, 0])


def load_stereo_yml(path: Path) -> RectCalib:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(path)
    m = {k: fs.getNode(k).mat() for k in ("K1", "D1", "R1", "P1", "K2", "D2", "R2", "P2")}
    fs.release()
    return RectCalib(m["K1"], m["D1"], m["R1"], m["P1"], m["K2"], m["D2"], m["R2"], m["P2"])


def board_metrics(cal: RectCalib, cl: np.ndarray, cr: np.ndarray, board: Board) -> Tuple[float, float, float]:
    """一帧的 (左右行差中位数 px, 三角化出的相邻角点间距中位数 m, 板距中位数 m)。

    两目水平夹角变化只表现为固定视差偏移，行差看不出来，但方格边长会按 视差/(视差+偏移) 缩放，越远越明显。
    """
    ul = cv2.undistortPoints(cl, cal.k1, cal.d1, R=cal.r1, P=cal.p1).reshape(-1, 2)
    ur = cv2.undistortPoints(cr, cal.k2, cal.d2, R=cal.r2, P=cal.p2).reshape(-1, 2)
    fx, fy, cx, cy = cal.p1[0, 0], cal.p1[1, 1], cal.p1[0, 2], cal.p1[1, 2]
    z = fx * cal.baseline / (ul[:, 0] - ur[:, 0])
    pts = np.stack([(ul[:, 0] - cx) * z / fx, (ul[:, 1] - cy) * z / fy, z], axis=1).reshape(board.rows, board.cols, 3)
    # 横向和纵向相邻角点距离都算上
    d = np.concatenate([
        np.linalg.norm(pts[:, 1:] - pts[:, :-1], axis=2).ravel(),
        np.linalg.norm(pts[1:, :] - pts[:-1, :], axis=2).ravel(),
    ])
    return float(np.median(ul[:, 1] - ur[:, 1])), float(np.median(d)), float(np.median(z))


def summarize(rows: List[float], squares: List[float], square_true: float, dists: List[float]) -> str:
    r, s = np.array(rows), np.array(squares)
    row_med, sq_med = float(np.median(r)), float(np.median(s))
    sq_err = 100 * (sq_med / square_true - 1)
    verdict = "标定仍有效" if abs(row_med) <= 0.5 and abs(sq_err) <= 1.0 else "⚠️ 标定已偏，需要重标"
    return (
        f"{len(r)} 帧：行差中位 {row_med:+.3f} px（帧间范围 {r.min():+.2f}～{r.max():+.2f}），"
        f"三角化方格边长 {sq_med * 1000:.2f} mm（真值 {square_true * 1000:.1f}，偏差 {sq_err:+.2f}%，板距 {np.median(dists):.2f} m）→ {verdict}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="检查双目标定是否仍有效")
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--square", type=float, required=True, help="方格边长（米）")
    ap.add_argument("--calib", type=Path, default=Path("stereo/calib/1280x480/stereo.yml"))
    ap.add_argument("--device", default="/dev/video4")
    ap.add_argument("--swap-lr", action="store_true", help="拼接帧右半当左目（看当前接线）")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--wait", type=float, default=300.0, help="等棋盘出现的最长时间（秒）")
    a = ap.parse_args()
    board = Board(a.cols, a.rows, a.square)
    cal = load_stereo_yml(a.calib)

    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"打不开 {a.device}（被占用时 fuser -v {a.device} 查）")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    print(f"等棋盘格出现在两目画面里（最多 {a.wait:.0f} s），出现后保持静止……", flush=True)
    rows, squares, dists = [], [], []
    t_end = time.monotonic() + a.wait
    last_print = 0.0
    try:
        while len(rows) < a.frames and time.monotonic() < t_end:
            ok, raw = cap.read()
            if not ok:
                raise SystemExit("读帧失败")
            left, right = split_stereo_y(raw, a.swap_lr)
            cl, cr = detect(left, board), detect(right, board)
            if time.monotonic() - last_print > 1.0:
                # 每秒报一次进度：只有一目检测到通常是棋盘太近/太偏，85 mm 基线下两目视野差得多
                print(f"[{len(rows)}/{a.frames}] 左目{'✓' if cl is not None else '✗'} 右目{'✓' if cr is not None else '✗'}", flush=True)
                last_print = time.monotonic()
            if cl is None or cr is None:
                continue
            r, s, z = board_metrics(cal, cl, cr, board)
            rows.append(r)
            squares.append(s)
            dists.append(z)
    finally:
        cap.release()
    if not rows:
        raise SystemExit("等待超时，一直没在两目里同时检测到棋盘")
    print(summarize(rows, squares, a.square, dists))


if __name__ == "__main__":
    main()
