"""鲁班猫 ov8858 实时预览。

板子上只跑 v4l2-ctl 把 NV12 裸帧写到 stdout（零依赖），经 SSH 管道传回本机，
本机做 NV12→RGB 转换并编码 JPEG，以 MJPEG over HTTP 输出给浏览器。
"""

from __future__ import annotations

import argparse
import io
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image

PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ov8858 preview</title>
<style>
  body{margin:0;background:#111;color:#ddd;font:14px/1.5 system-ui,sans-serif;
       display:flex;flex-direction:column;align-items:center;gap:12px;padding:16px}
  img{max-width:100%;border:1px solid #333;background:#000}
  #info{font-variant-numeric:tabular-nums}
</style></head>
<body>
  <img src="/stream" alt="camera">
  <div id="info">—</div>
  <script>
    setInterval(async () => {
      try {
        const r = await fetch('/stat');
        document.getElementById('info').textContent = await r.text();
      } catch (e) { /* 服务停了就保持上一次的显示 */ }
    }, 1000);
  </script>
</body></html>
""".encode("utf-8")


class FrameSource:
    """持有 SSH 抓帧子进程，把最新一帧解码结果暴露给 HTTP 线程。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.frame_size = args.width * args.height * 3 // 2  # NV12
        self.jpeg: bytes | None = None
        self.seq = 0
        self.fps = 0.0
        self.cond = threading.Condition()
        self.proc: subprocess.Popen[bytes] | None = None
        self.error: str | None = None
        # 显示层白平衡增益，仅修正预览观感，不改变板子输出的数据
        self.wb: tuple[float, float] | None = None
        if args.awb_gain:
            rg, bg = (float(x) for x in args.awb_gain.split(","))
            self.wb = (rg, bg)

    def _remote_cmd(self) -> list[str]:
        a = self.args
        v4l2 = (
            f"v4l2-ctl -d {a.device}"
            f" --set-fmt-video=width={a.width},height={a.height},pixelformat=NV12"
            f" --stream-mmap=4 --stream-to=-"
        )
        return [
            "ssh", "-T",
            "-o", "BatchMode=yes",
            "-o", "ServerAliveInterval=5",
            "-i", a.key,
            f"{a.user}@{a.host}",
            v4l2,
        ]

    def _drain_stderr(self, pipe) -> None:
        for line in iter(pipe.readline, b""):
            text = line.decode("utf-8", "replace").strip()
            # v4l2-ctl 会往 stderr 刷进度符号，只报告真正的错误
            if text and not set(text) <= {"<", ">", "#"}:
                print(f"[remote] {text}", file=sys.stderr)

    def _to_jpeg(self, buf: bytes) -> bytes:
        a = self.args
        y = np.frombuffer(buf, dtype=np.uint8, count=a.width * a.height)
        y = y.reshape(a.height, a.width)

        if a.gray:
            img = Image.fromarray(y, mode="L")
        else:
            uv = np.frombuffer(buf, dtype=np.uint8, offset=a.width * a.height)
            uv = uv.reshape(a.height // 2, a.width // 2, 2)
            # 最近邻上采样色度，再按 BT.601 反变换
            u = np.repeat(np.repeat(uv[:, :, 0], 2, 0), 2, 1).astype(np.float32) - 128.0
            v = np.repeat(np.repeat(uv[:, :, 1], 2, 0), 2, 1).astype(np.float32) - 128.0
            yf = y.astype(np.float32)
            rgb = np.stack(
                [yf + 1.402 * v, yf - 0.344136 * u - 0.714136 * v, yf + 1.772 * u], -1
            )
            if self.wb is not None:
                rgb[..., 0] *= self.wb[0]
                rgb[..., 2] *= self.wb[1]
            img = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8))

        out = io.BytesIO()
        img.save(out, format="JPEG", quality=self.args.quality)
        return out.getvalue()

    def run(self) -> None:
        cmd = self._remote_cmd()
        print(f"[local] 启动抓帧: {' '.join(cmd[-1:])}", file=sys.stderr)
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        threading.Thread(target=self._drain_stderr, args=(self.proc.stderr,), daemon=True).start()

        stdout = self.proc.stdout
        assert stdout is not None
        buf = bytearray(self.frame_size)
        view = memoryview(buf)
        count, t0 = 0, time.monotonic()

        while True:
            filled = 0
            while filled < self.frame_size:
                n = stdout.readinto(view[filled:])
                if not n:
                    self.error = "抓帧进程结束（SSH 断开或摄像头被占用）"
                    print(f"[local] {self.error}", file=sys.stderr)
                    with self.cond:
                        self.cond.notify_all()
                    return
                filled += n

            jpeg = self._to_jpeg(bytes(buf))
            count += 1
            dt = time.monotonic() - t0
            if dt >= 1.0:
                self.fps = count / dt
                count, t0 = 0, time.monotonic()

            with self.cond:
                self.jpeg = jpeg
                self.seq += 1
                self.cond.notify_all()

    def wait_frame(self, last_seq: int, timeout: float = 5.0) -> tuple[bytes | None, int]:
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            return self.jpeg, self.seq


class Handler(BaseHTTPRequestHandler):
    source: FrameSource  # 由 main 注入

    def log_message(self, fmt: str, *a) -> None:
        pass  # 静音逐请求日志，避免刷屏

    def do_GET(self) -> None:
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/stat":
            s = self.source
            body = f"{s.args.width}x{s.args.height}  {s.fps:.1f} fps  第 {s.seq} 帧"
            if s.error:
                body += f"  [{s.error}]"
            data = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while True:
                    jpeg, seq = self.source.wait_frame(seq)
                    if jpeg is None:
                        break
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # 浏览器关掉标签页，正常情况
        else:
            self.send_error(404)


def main() -> int:
    p = argparse.ArgumentParser(description="ov8858 实时预览")
    p.add_argument("--host", default="10.42.0.50",
                   help="板子地址；默认用 dispatcher 附加的固定地址，DHCP 地址会漂移")
    p.add_argument("--user", default="cat")
    p.add_argument("--key", default="/home/wanzi030/.ssh/id_ed25519_lubancat")
    p.add_argument("--device", default="/dev/video0")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--quality", type=int, default=80)
    p.add_argument("--gray", action="store_true", help="只传/显示 Y 平面，贴近 VIO 实际输入")
    p.add_argument("--awb-gain", default=None,
                   help="显示层白平衡增益 'R,B'，例如 1.427,1.085")
    p.add_argument("--port", type=int, default=8090)
    args = p.parse_args()

    source = FrameSource(args)
    Handler.source = source
    threading.Thread(target=source.run, daemon=True).start()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"[local] 预览地址 http://127.0.0.1:{args.port}", file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if source.proc:
            source.proc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
