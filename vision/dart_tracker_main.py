#!/usr/bin/env python3
"""飞镖蓝/橙灯识别与跟踪主程序：采集 -> 颜色检测 -> 卡尔曼跟踪 -> 标注 -> MJPEG 推流。

用法（在鲁班猫上，用 ~/vio_venv 的解释器执行）：
    ~/vio_venv/bin/python3 dart_tracker_main.py --device /dev/video0 --port 8091
浏览器访问 http://<板子IP>:8091/ 查看标注画面；控制台输出跟踪状态日志。
"""
from __future__ import annotations

import argparse
import time

import cv2
from loguru import logger

from color_detector import detect_candidates
from stream_server import MjpegStreamServer
from tracker import DartTracker, TrackResult, TrackState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--gate-radius", type=float, default=80.0)
    parser.add_argument("--max-lost-frames", type=int, default=10)
    return parser.parse_args()


def draw_overlay(
    frame: cv2.typing.MatLike,
    candidates: list,
    track: TrackResult,
    fps: float,
) -> None:
    for c in candidates:
        color_bgr = (255, 120, 0) if c.color == "blue" else (0, 0, 255)
        x, y, w, h = c.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 1)

    if track.state == TrackState.TRACKING:
        color_bgr = (255, 120, 0) if track.color == "blue" else (0, 0, 255)
        cv2.drawMarker(
            frame,
            (int(track.cx), int(track.cy)),
            color_bgr,
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )
        label = f"{track.color} v=({track.vx:.1f},{track.vy:.1f})"
        if track.lost_frames > 0:
            label += f" lost={track.lost_frames}"
        cv2.putText(
            frame,
            label,
            (int(track.cx) + 15, int(track.cy) - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color_bgr,
            2,
        )
    else:
        cv2.putText(
            frame, "SEARCHING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
        )

    cv2.putText(
        frame,
        f"{fps:.1f} fps",
        (10, frame.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
    )


def main() -> None:
    args = parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError(f"打不开摄像头设备: {args.device}")

    tracker = DartTracker(max_lost_frames=args.max_lost_frames, gate_radius_px=args.gate_radius)
    server = MjpegStreamServer(port=args.port)
    server.serve_forever_in_background()
    logger.info(f"MJPEG 推流已启动: http://<板子IP>:{args.port}/")

    fps = 0.0
    last_t = time.monotonic()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                logger.warning("读帧失败，重试")
                time.sleep(0.05)
                continue

            candidates = detect_candidates(frame)
            track = tracker.update(candidates)

            now = time.monotonic()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            draw_overlay(frame, candidates, track, fps)
            server.update_frame(frame)

            if track.state == TrackState.TRACKING:
                logger.debug(
                    f"TRACK color={track.color} xy=({track.cx:.1f},{track.cy:.1f}) "
                    f"v=({track.vx:.1f},{track.vy:.1f}) lost={track.lost_frames}"
                )
    except KeyboardInterrupt:
        logger.info("收到中断，退出")
    finally:
        cap.release()
        server.shutdown()


if __name__ == "__main__":
    main()
