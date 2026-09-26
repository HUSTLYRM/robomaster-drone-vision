"""轻量 MJPEG over HTTP 推流服务，用于在浏览器实时查看标注画面。

仅为调试可视化设计，只支持单个客户端连接；多个客户端会共享同一个帧
就绪事件，互相清除会导致丢帧。调试完可视化后应关闭推流，避免占用带宽。
"""
from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class MjpegStreamServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 8091, jpeg_quality: int = 80) -> None:
        self._jpeg_quality = jpeg_quality
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._new_frame_event = threading.Event()
        server = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path != "/":
                    self.send_response(404)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        frame = server._wait_next_jpeg()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format: str, *args: object) -> None:
                pass

        self._httpd = ThreadingHTTPServer((host, port), Handler)

    def _wait_next_jpeg(self) -> bytes:
        self._new_frame_event.wait()
        with self._lock:
            frame = self._latest_jpeg
            self._new_frame_event.clear()
        assert frame is not None
        return frame

    def update_frame(self, frame_bgr: np.ndarray) -> None:
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = buf.tobytes()
        self._new_frame_event.set()

    def serve_forever_in_background(self) -> None:
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def shutdown(self) -> None:
        self._httpd.shutdown()
