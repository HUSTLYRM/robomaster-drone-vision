#!/usr/bin/env python3
"""里程计质量统计：订阅 /stereo/odom 与 /stereo/odom_info，实时打印并在结束时汇总频率、丢失、内点、路程与首尾闭合误差。

用法（仓库根目录，vo.launch.py 已在跑）：
    source /opt/ros/humble/setup.bash
    python3 -m stereo.odom_eval                 # Ctrl+C 结束出汇总
    python3 -m stereo.odom_eval --seconds 30    # 静止漂移测试
闭环测试：从起点出发走一圈回到起点（同一位置同一朝向），闭合误差 / 路程 即漂移率。
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class OdomStats:
    t0: Optional[float] = None
    t_last: float = 0.0
    n: int = 0
    lost: int = 0
    inliers: List[int] = field(default_factory=list)
    est_ms: List[float] = field(default_factory=list)
    start: Optional[Vec3] = None
    last: Optional[Vec3] = None
    path: float = 0.0
    max_from_start: float = 0.0

    def update(self, t: float, pos: Vec3, lost: bool, inliers: int, est_ms: float) -> None:
        """丢失帧只计数，不进路程和位置（RTAB-Map 丢失时发的是空位姿）。"""
        if self.t0 is None:
            self.t0 = t
        self.t_last = t
        self.n += 1
        if lost:
            self.lost += 1
            return
        self.inliers.append(inliers)
        self.est_ms.append(est_ms)
        if self.start is None:
            self.start = pos
        if self.last is not None:
            self.path += _dist(pos, self.last)
        self.last = pos
        self.max_from_start = max(self.max_from_start, _dist(pos, self.start))

    def summary(self) -> dict:
        span = (self.t_last - self.t0) if self.t0 is not None else 0.0
        closure = _dist(self.last, self.start) if self.start and self.last else float("nan")
        return {
            "frames": self.n,
            "rate_hz": self.n / span if span > 0 else 0.0,
            "lost": self.lost,
            "inliers_median": _median(self.inliers),
            "inliers_min": min(self.inliers) if self.inliers else 0,
            "est_ms_median": _median(self.est_ms),
            "path_m": self.path,
            "closure_m": closure,
            "drift_pct": 100 * closure / self.path if self.path > 0 else float("nan"),
            "max_from_start_m": self.max_from_start,
        }


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _median(v: List[float]) -> float:
    if not v:
        return float("nan")
    s = sorted(v)
    m = len(s) // 2
    return float(s[m]) if len(s) % 2 else (s[m - 1] + s[m]) / 2


def format_summary(s: dict) -> str:
    return (
        f"帧 {s['frames']}（{s['rate_hz']:.1f} Hz），丢失 {s['lost']}，内点中位 {s['inliers_median']:.0f}（最少 {s['inliers_min']}），"
        f"估计耗时中位 {s['est_ms_median']:.1f} ms\n"
        f"路程 {s['path_m']:.3f} m，首尾距离 {s['closure_m']:.3f} m（{s['drift_pct']:.2f}% 路程），离起点最远 {s['max_from_start_m']:.3f} m"
    )


def main() -> None:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rtabmap_msgs.msg import OdomInfo

    ap = argparse.ArgumentParser(description="里程计质量统计")
    ap.add_argument("--seconds", type=float, default=0.0, help="统计时长，0 = 到 Ctrl+C")
    a = ap.parse_args()

    rclpy.init()
    node = Node("odom_eval")
    stats = OdomStats()
    # RTAB-Map 先发 odom 再发 odom_info（实测），按时间戳两边都到齐才统计
    odoms: dict = {}
    infos: dict = {}

    def pair(key: tuple) -> None:
        if key not in odoms or key not in infos:
            for d in (odoms, infos):
                if len(d) > 200:
                    d.pop(next(iter(d)))
            return
        m, info = odoms.pop(key), infos.pop(key)
        p = m.pose.pose.position
        stats.update(key[0] + key[1] * 1e-9, (p.x, p.y, p.z), info.lost, info.inliers, info.time_estimation * 1000)

    def on_info(m: OdomInfo) -> None:
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        infos[key] = m
        pair(key)

    def on_odom(m: Odometry) -> None:
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        odoms[key] = m
        pair(key)

    node.create_subscription(OdomInfo, "/stereo/odom_info", on_info, 50)
    node.create_subscription(Odometry, "/stereo/odom", on_odom, 50)
    print("等待 /stereo/odom ……", flush=True)
    t_end = time.monotonic() + a.seconds if a.seconds > 0 else math.inf
    last_print = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_print > 2.0 and stats.n:
                s = stats.summary()
                pos = stats.last or (0.0, 0.0, 0.0)
                print(f"[{s['frames']} 帧 {s['rate_hz']:.0f} Hz 丢 {s['lost']}] 内点 {stats.inliers[-1] if stats.inliers else 0} "
                      f"位置 ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) 路程 {s['path_m']:.2f} m", flush=True)
                last_print = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print("==== 汇总 ====")
    print(format_summary(stats.summary()))


if __name__ == "__main__":
    main()
