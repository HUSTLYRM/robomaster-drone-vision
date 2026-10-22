#!/usr/bin/env python3
"""飞镖蓝/橙灯识别与跟踪主程序：采集 -> 颜色检测 -> 卡尔曼跟踪 -> 标注 -> MJPEG 推流。

用法（在鲁班猫上，用 ~/vio_venv 的解释器执行）：
    ~/vio_venv/bin/python3 dart_tracker_main.py --device /dev/video0 --port 8091
浏览器访问 http://<板子IP>:8091/ 查看标注画面；控制台输出跟踪状态日志。

取帧不让 OpenCV 转 BGR，直接拿 V4L2 原始 YU12（I420）交给检测；后台线程只留最新帧。
BGR 只在识别完成后为推流画面转一次，不计入识别延迟。
运行期间把 CPU 最低频率抬到 --min-freq-mhz（默认 1800），退出时恢复，见 cpu_freq.py。
"""
from __future__ import annotations

import argparse
import signal
import time

import cv2
from loguru import logger

from color_detector import detect_candidates_i420
from cpu_freq import min_freq_locked
from frame_grabber import LatestFrameGrabber
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
    parser.add_argument("--min-freq-mhz", type=int, default=1800,
                        help="运行期间 CPU 最低频率，退出时恢复；0 = 不改，按系统调频")
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
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YU12"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if not cap.isOpened():
        raise RuntimeError(f"打不开摄像头设备: {args.device}")
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little")
    if fourcc != b"YU12":
        raise RuntimeError(f"相机没有协商成 YU12（I420），实际 {fourcc!r}")

    tracker = DartTracker(max_lost_frames=args.max_lost_frames, gate_radius_px=args.gate_radius)
    server = MjpegStreamServer(port=args.port)
    server.serve_forever_in_background()
    logger.info(f"MJPEG 推流已启动: http://<板子IP>:{args.port}/")

    # kill（SIGTERM）和 SSH 断线（SIGHUP）也走正常退出流程，保证最低频率被恢复；
    # 启动时已被忽略的信号（如 nohup 忽略 SIGHUP）保持忽略，不改变调用方的意图
    for sig in (signal.SIGTERM, signal.SIGHUP):
        if signal.getsignal(sig) != signal.SIG_IGN:
            signal.signal(sig, _raise_interrupt)
    with min_freq_locked(args.min_freq_mhz * 1000, logger.warning) as changed:
        for path, old, new in changed:
            logger.info(f"运行期间锁定 CPU 最低频率：{path} {old} → {new} kHz，退出时恢复")
        grabber = LatestFrameGrabber(cap).start()
        last_seq = 0
        fps = 0.0
        last_t = time.monotonic()
        try:
            while True:
                latest = grabber.wait_newer(last_seq)
                if latest is None:
                    logger.warning("1 s 内没有新帧，继续等待")
                    continue
                last_seq = latest.seq

                candidates = detect_candidates_i420(latest.image, args.width, args.height)
                track = tracker.update(candidates)
                frame = cv2.cvtColor(latest.image.reshape(args.height * 3 // 2, args.width), cv2.COLOR_YUV2BGR_I420)

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
            # 收尾期间不再响应第二个信号，免得打断频率恢复
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, signal.SIG_IGN)
            grabber.stop()
            cap.release()
            server.shutdown()
    if changed:
        logger.info("CPU 最低频率已恢复")


def _raise_interrupt(signum: int, frame: object) -> None:
    raise KeyboardInterrupt


if __name__ == "__main__":
    main()
