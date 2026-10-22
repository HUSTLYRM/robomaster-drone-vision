#!/usr/bin/env python3
"""双目曝光同步检查：上下晃棋盘格，录几秒左右图，离线算「校正后行差 dy」对「竖直速度 vy」的线性拟合，斜率即左右曝光时间差。

原理：校正后同一点左右应同行；若右目比左目晚 Δt 曝光，竖直运动的点在右图多走 vy·Δt，于是 dy = yL - yR ≈ -vy·Δt。
静止时 dy≈0 只能说明标定好，证明不了同步，所以必须动起来测。

用法（仓库根目录，先起 stereo.capture_node，已有标定 YAML）：
    source /opt/ros/humble/setup.bash
    python3 -m stereo.sync_check --cols 11 --rows 8 --calib stereo/calib/1280x480 --seconds 6
晃法：棋盘格两目都完整可见，以中等速度上下来回晃（太快会糊到检测不到角点），录制期间一直动。
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

from stereo.calibrate import Board, detect
from stereo.frames import load_camera_info


def rect_maps(calib: Path, size: Tuple[int, int]) -> dict:
    """读 left/right YAML，返回 undistortPoints 用的 K、D、R、P。"""
    out = {}
    for side in ("left", "right"):
        info = load_camera_info(calib / f"{side}.yaml", *size)
        out[side] = (
            np.array(info.k).reshape(3, 3),
            np.array(info.d),
            np.array(info.r).reshape(3, 3),
            np.array(info.p).reshape(3, 4),
        )
    return out


def fit_dt(t: np.ndarray, yl: np.ndarray, dy: np.ndarray) -> Tuple[float, float, float, int]:
    """yl: 每帧左目校正后平均行坐标；dy: 每帧左右行差。用中心差分求 vy，拟合 dy = a + b·vy，Δt = -b。

    只用相邻帧（间隔 < 1.5 个中位周期）都检测到的样本，避免跨丢帧的速度失真。
    返回 (Δt 秒, 截距 a 像素, |vy| 中位数 px/s, 参与拟合的样本数)。
    """
    period = float(np.median(np.diff(t)))
    vy, ds = [], []
    for i in range(1, len(t) - 1):
        if t[i + 1] - t[i - 1] < 3 * period:
            vy.append((yl[i + 1] - yl[i - 1]) / (t[i + 1] - t[i - 1]))
            ds.append(dy[i])
    vy_a, ds_a = np.array(vy), np.array(ds)
    if len(vy_a) < 10:
        raise ValueError(f"有效样本只有 {len(vy_a)} 个，检测不到角点或丢帧太多")
    b, a = np.polyfit(vy_a, ds_a, 1)
    return -float(b), float(a), float(np.median(np.abs(vy_a))), len(vy_a)


def record(seconds: float, board: Board, wait_s: float) -> List[Tuple[float, np.ndarray, np.ndarray]]:
    """先等到两目都检测到棋盘再开录，免得人还没就位就录完了。"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("stereo_sync_check")
    pending: dict = {}
    pairs: List[Tuple[float, np.ndarray, np.ndarray]] = []

    def on_img(side: str):
        def cb(m: Image) -> None:
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).copy()
            other = pending.pop(key, None)
            if other is None:
                pending[key] = img
                if len(pending) > 50:
                    pending.pop(next(iter(pending)))
                return
            li, ri = (img, other) if side == "left" else (other, img)
            pairs.append((key[0] + key[1] * 1e-9, li, ri))

        return cb

    # 队列放大，录制期间只收不算，尽量不丢帧
    node.create_subscription(Image, "/stereo/left/image_raw", on_img("left"), 100)
    node.create_subscription(Image, "/stereo/right/image_raw", on_img("right"), 100)
    print(f"等棋盘格出现在两目画面里（最多 {wait_s:.0f} s）……", flush=True)
    t_wait_end = time.monotonic() + wait_s
    while True:
        if time.monotonic() > t_wait_end:
            node.destroy_node()
            rclpy.try_shutdown()
            return []
        rclpy.spin_once(node, timeout_sec=0.01)
        if pairs:
            _, li, ri = pairs[-1]
            pairs.clear()
            if detect(li, board) is not None and detect(ri, board) is not None:
                break
    pairs.clear()
    print(f"检测到棋盘，开始录制 {seconds:.0f} s，上下晃……", flush=True)
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.try_shutdown()
    pairs.sort(key=lambda p: p[0])
    return pairs


def main() -> None:
    ap = argparse.ArgumentParser(description="双目曝光同步检查")
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--calib", type=Path, default=Path("stereo/calib/1280x480"))
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--wait", type=float, default=300.0, help="等棋盘出现的最长时间（秒）")
    a = ap.parse_args()
    board = Board(a.cols, a.rows, 0.0)  # 只用角点检测，边长无关

    pairs = record(a.seconds, board, a.wait)
    if not pairs:
        print("没录到图像：采集节点没在跑，或等待超时一直没检测到棋盘")
        return
    size = (pairs[0][1].shape[1], pairs[0][1].shape[0])
    span = pairs[-1][0] - pairs[0][0]
    print(f"录到 {len(pairs)} 对（{len(pairs) / max(span, 1e-6):.1f} 对/秒），检测角点中……", flush=True)
    m = rect_maps(a.calib, size)
    ts, yls, dys = [], [], []
    for t, li, ri in pairs:
        cl, cr = detect(li, board), detect(ri, board)
        if cl is None or cr is None:
            continue
        ul = cv2.undistortPoints(cl, m["left"][0], m["left"][1], R=m["left"][2], P=m["left"][3]).reshape(-1, 2)
        ur = cv2.undistortPoints(cr, m["right"][0], m["right"][1], R=m["right"][2], P=m["right"][3]).reshape(-1, 2)
        ts.append(t)
        yls.append(float(ul[:, 1].mean()))
        dys.append(float((ul[:, 1] - ur[:, 1]).mean()))
    print(f"两目都检测到角点的帧 {len(ts)} / {len(pairs)}")
    dt, a0, vmed, n = fit_dt(np.array(ts), np.array(yls), np.array(dys))
    print(f"拟合样本 {n}，竖直速度中位 {vmed:.0f} px/s，静态行差截距 {a0:+.3f} px")
    print(f"右目相对左目曝光时差 Δt = {dt * 1000:+.3f} ms（正 = 右目晚）")
    print(f"在 {vmed:.0f} px/s 下造成的行差 {abs(dt) * vmed:.2f} px")


if __name__ == "__main__":
    main()
