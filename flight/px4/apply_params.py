#!/usr/bin/env python3
"""把参数文件写入 PX4 并逐项回读核对。

用法（仓库根目录，飞控 USB 接笔记本，QGC 须关闭以释放串口）：
    uv run python -m flight.px4.apply_params flight/px4/companion.params --dry-run   # 只看差异
    uv run python -m flight.px4.apply_params flight/px4/companion.params             # 写入并回读

PX4 的整型参数在 MAVLink 里按位模式塞进 float 字段（PARAM_ENCODE_BYTEWISE），
直接当浮点读会得到 1.4e-45 这类值，必须按 param_type 用 struct 重解释（硬件手册 7.3 踩过）。
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

mavlink = mavutil.mavlink


def encode_param(value: float, param_type: int) -> float:
    if param_type == mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type == mavlink.MAV_PARAM_TYPE_INT32:
        if float(value) != int(value):
            raise ValueError(f"整型参数不能取小数值 {value}")
        return struct.unpack("<f", struct.pack("<i", int(value)))[0]
    raise ValueError(f"不支持的 param_type={param_type}（PX4 只用 INT32/REAL32）")


def decode_param(raw: float, param_type: int) -> float:
    if param_type == mavlink.MAV_PARAM_TYPE_REAL32:
        return raw
    if param_type == mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", struct.pack("<f", raw))[0]
    raise ValueError(f"不支持的 param_type={param_type}")


def parse_params_file(path: str) -> list[tuple[str, float]]:
    params = []
    for lineno, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 2 or len(parts[0]) > 16:
            raise ValueError(f"{path}:{lineno} 格式应为「参数名 值」且参数名 ≤16 字符: {line!r}")
        params.append((parts[0], float(parts[1])))
    return params


def _read(conn, name: str, timeout_s: float = 2.0):  # noqa: ANN001, ANN202
    for _ in range(3):
        conn.mav.param_request_read_send(conn.target_system, conn.target_component, name.encode(), -1)
        deadline = time.monotonic() + timeout_s / 3
        while time.monotonic() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is not None and msg.param_id == name:
                return msg
    return None


def _set(conn, name: str, raw: float, param_type: int, timeout_s: float = 2.0):  # noqa: ANN001, ANN202
    for _ in range(3):
        conn.mav.param_set_send(conn.target_system, conn.target_component, name.encode(), raw, param_type)
        deadline = time.monotonic() + timeout_s / 3
        while time.monotonic() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is not None and msg.param_id == name:
                return msg
    return None


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("params_file")
    p.add_argument("--url", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args(argv)

    params = parse_params_file(args.params_file)
    conn = mavutil.mavlink_connection(args.url, baud=args.baud, source_system=255, source_component=190)
    # 先发心跳再等：PX4 的 UDP 链路以第一个来包地址为对端，用 udpout 连接时对方不会先发
    hb = None
    deadline = time.monotonic() + 10
    while hb is None and time.monotonic() < deadline:
        conn.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = conn.wait_heartbeat(timeout=1)
    if hb is None:
        print("10 s 内没有收到飞控心跳", file=sys.stderr)
        return 2

    failures = 0
    for name, target in params:
        cur = _read(conn, name)
        if cur is None:
            print(f"✗ {name}: 读取超时（参数名拼错或固件无此参数）")
            failures += 1
            continue
        current = decode_param(cur.param_value, cur.param_type)
        if abs(current - target) < 1e-6:
            print(f"= {name} = {current:g}（已是目标值）")
            continue
        if args.dry_run:
            print(f"~ {name}: {current:g} → {target:g}（dry-run 未写入）")
            continue
        echo = _set(conn, name, encode_param(target, cur.param_type), cur.param_type)
        back = _read(conn, name)  # 回声之外再独立读一次，确认真正生效
        if echo is None or back is None or abs(decode_param(back.param_value, back.param_type) - target) >= 1e-6:
            print(f"✗ {name}: 写入 {target:g} 后回读不一致")
            failures += 1
        else:
            print(f"✓ {name}: {current:g} → {target:g}")
    conn.close()
    if failures == 0 and not args.dry_run:
        # SITL 实测：写入后 2 s 内停掉 PX4，改动没有落盘（自动保存有延迟）
        print("全部写入并回读一致。PX4 延迟数秒自动保存：至少等 5 s 再断电/重启；EKF 等参数需重启生效。")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
