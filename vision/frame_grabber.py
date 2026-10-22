"""后台取帧线程：不停从相机出队，只保留最新一帧。

主循环处理一帧的时间接近帧周期时，直接 cap.read() 会一直拿到驱动队列里排着的旧帧
（鲁班猫实测：识别 31 ms、帧周期 33.5 ms 时，帧尾到识别完成从 46 ms 变成 78 ms）。
把出队放到后台线程、主线程只取最新帧，旧帧被直接丢弃，不再排队。
CAP_PROP_BUFFERSIZE=1 也能避免排队，但实测帧率掉到 10 fps，不可用。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Frame:
    image: np.ndarray
    timestamp_ms: float  # V4L2 缓冲区时间戳（CLOCK_MONOTONIC，毫秒；鲁班猫 rkisp 标为帧尾）
    seq: int  # 本线程出队计数，主线程据此统计跳过的帧数


class LatestFrameGrabber:
    def __init__(self, cap: cv2.VideoCapture) -> None:
        self._cap = cap
        self._cond = threading.Condition()
        self._latest: Frame | None = None
        self._seq = 0
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> LatestFrameGrabber:
        self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop:
            ok, image = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            frame = Frame(image, self._cap.get(cv2.CAP_PROP_POS_MSEC), self._seq + 1)
            with self._cond:
                self._seq = frame.seq
                self._latest = frame
                self._cond.notify_all()

    def wait_newer(self, last_seq: int, timeout: float = 1.0) -> Frame | None:
        """返回比 last_seq 新的最新一帧；超时（相机读不出帧）返回 None。"""
        with self._cond:
            self._cond.wait_for(
                lambda: self._stop or (self._latest is not None and self._latest.seq > last_seq), timeout
            )
            if self._latest is None or self._latest.seq <= last_seq:
                return None
            return self._latest

    def stop(self) -> None:
        with self._cond:
            self._stop = True
            self._cond.notify_all()
        self._thread.join(timeout=2.0)
