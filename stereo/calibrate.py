#!/usr/bin/env python3
"""双目标定：订阅采集节点的左右图像自动采样（棋盘静止 + 姿态够新才收），够了就用 OpenCV 标定并导出 ROS camera_info YAML。

替代 ROS camera_calibration 的 GUI（实测 2026-10-22 窗口在本机显示不全，按钮点不到）。

用法（仓库根目录，先起 stereo.capture_node）：
    source /opt/ros/humble/setup.bash
    python3 -m stereo.calibrate --cols 11 --rows 8 --square 0.03          # 在线采集 + 标定
    python3 -m stereo.calibrate --cols 11 --rows 8 --square 0.03 --images logs/stereo_calib/<时间>   # 用存下的样本离线重算

窗口按键：c = 样本够 15 张时立即标定；q / Esc = 放弃退出。
输出：--out 目录下 left.yaml / right.yaml（给 capture_node 的 left_info/right_info）、stereo.yml（OpenCV 格式，给 C++ VO）
和 stereo_report.yaml（外参与质量指标）；
样本图存 logs/stereo_calib/<时间>/（不进 git）。
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

# 与 camera_calibration 一致的覆盖度目标：X、Y、Size、Skew 四个维度的取值范围达到这些跨度算采够
PARAM_RANGES = (0.7, 0.7, 0.4, 0.5)
# 新样本与所有已收样本的四维 L1 距离超过此值才收（camera_calibration 也是 0.2）
MIN_PARAM_DIST = 0.2
# 相邻两次检测的角点平均位移小于此值（像素）算静止：防运动模糊，也规避左右曝光不同步的影响
STILL_PX = 1.0
MIN_SAMPLES_AUTO = 25
MIN_SAMPLES_MANUAL = 15
# 单张样本重投影误差超过「中位数 × 此倍数」且 > 1 px 视为离群，剔除后重算一次
OUTLIER_RATIO = 3.0
# 双目逐张误差的剔除下限（像素）：正常样本中位约 0.1～0.25
STEREO_OUTLIER_FLOOR_PX = 0.5


@dataclass(frozen=True)
class Board:
    cols: int  # 内角点列数
    rows: int  # 内角点行数
    square: float  # 方格边长，米

    def object_points(self) -> np.ndarray:
        g = np.zeros((self.rows * self.cols, 3), np.float32)
        g[:, :2] = np.mgrid[0 : self.cols, 0 : self.rows].T.reshape(-1, 2) * self.square
        return g


@dataclass
class StereoCalib:
    size: Tuple[int, int]  # 每目 (宽, 高)
    k1: np.ndarray
    d1: np.ndarray
    k2: np.ndarray
    d2: np.ndarray
    r: np.ndarray
    t: np.ndarray
    r1: np.ndarray
    r2: np.ndarray
    p1: np.ndarray
    p2: np.ndarray
    rms_left: float
    rms_right: float
    rms_stereo: float
    used: int
    dropped: List[int]

    @property
    def baseline(self) -> float:
        return float(np.linalg.norm(self.t))


def detect(img: np.ndarray, board: Board) -> Optional[np.ndarray]:
    """找内角点，返回 (N,1,2) float32；找不到返回 None。SB 算法自带亚像素精度，对模糊和光照更稳。"""
    ok, corners = cv2.findChessboardCornersSB(img, (board.cols, board.rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners.astype(np.float32) if ok else None


def view_params(corners: np.ndarray, board: Board, size: Tuple[int, int]) -> Tuple[float, float, float, float]:
    """仿 camera_calibration 的姿态描述：(X, Y, Size, Skew)，都归一到 0~1。"""
    w, h = size
    pts = corners.reshape(-1, 2)
    ul, ur = pts[0], pts[board.cols - 1]
    dl, dr = pts[-board.cols], pts[-1]
    quad = np.array([ul, ur, dr, dl])
    area = 0.5 * abs(np.cross(quad[2] - quad[0], quad[3] - quad[1]))
    border = math.sqrt(area)
    x = float(np.clip((pts[:, 0].mean() - border / 2) / max(w - border, 1), 0, 1))
    y = float(np.clip((pts[:, 1].mean() - border / 2) / max(h - border, 1), 0, 1))
    s = math.sqrt(area / (w * h))
    a, b = ul - ur, dr - ur
    ang = math.acos(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))
    skew = min(1.0, 2.0 * abs(math.pi / 2 - ang))
    return x, y, s, skew


def coverage(params: Sequence[Tuple[float, float, float, float]]) -> List[float]:
    """每个维度已覆盖跨度 / 目标跨度，封顶 1。"""
    if not params:
        return [0.0] * 4
    arr = np.array(params)
    return [min(1.0, float(arr[:, i].max() - arr[:, i].min()) / PARAM_RANGES[i]) for i in range(4)]


def is_new_pose(p: Tuple[float, float, float, float], params: Sequence[Tuple[float, float, float, float]]) -> bool:
    return all(sum(abs(a - b) for a, b in zip(p, q)) > MIN_PARAM_DIST for q in params)


def is_still(prev: Optional[np.ndarray], cur: np.ndarray) -> bool:
    return prev is not None and prev.shape == cur.shape and float(np.linalg.norm(prev - cur, axis=-1).mean()) < STILL_PX


def calibrate_stereo(board: Board, left: List[np.ndarray], right: List[np.ndarray], size: Tuple[int, int]) -> StereoCalib:
    """先各自标内参（固定 k3，样本少时 k3 易过拟合），按单目误差剔离群；再固定内参标外参，按双目误差再剔一轮；最后极线校正。"""
    obj = board.object_points()
    keep = list(range(len(left)))
    dropped: List[int] = []
    flags = cv2.CALIB_FIX_K3
    for attempt in range(2):
        objs = [obj] * len(keep)
        lp, rp = [left[i] for i in keep], [right[i] for i in keep]
        rms_l, k1, d1, _, _, _, _, err_l = cv2.calibrateCameraExtended(objs, lp, size, None, None, flags=flags)
        rms_r, k2, d2, _, _, _, _, err_r = cv2.calibrateCameraExtended(objs, rp, size, None, None, flags=flags)
        if attempt == 1:
            break
        err = np.maximum(err_l.ravel(), err_r.ravel())
        bad = [keep[i] for i, e in enumerate(err) if e > max(OUTLIER_RATIO * float(np.median(err)), 1.0)]
        if not bad or len(keep) - len(bad) < MIN_SAMPLES_MANUAL:
            break
        dropped = bad
        keep = [i for i in keep if i not in bad]
    # 再按双目逐张误差剔一轮：单目各自都好、两目对不上的样本（拿板时支架晃了等）单目那轮剔不掉，
    # 实测 2026-10-22 一张 2.5 px 的坏样本把双目 RMS 从 0.22 拉到 0.51
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-6)
    for attempt in range(2):
        objs = [obj] * len(keep)
        lp, rp = [left[i] for i in keep], [right[i] for i in keep]
        rms_s, k1, d1, k2, d2, r, t, _, _, pve = cv2.stereoCalibrateExtended(
            objs, lp, rp, k1, d1, k2, d2, size, np.eye(3), np.zeros((3, 1)), criteria=crit, flags=cv2.CALIB_FIX_INTRINSIC
        )
        if attempt == 1:
            break
        err = pve.max(axis=1)
        bad = [keep[i] for i, e in enumerate(err) if e > max(OUTLIER_RATIO * float(np.median(err)), STEREO_OUTLIER_FLOOR_PX)]
        if not bad or len(keep) - len(bad) < MIN_SAMPLES_MANUAL:
            break
        dropped = sorted(dropped + bad)
        keep = [i for i in keep if i not in bad]
    # alpha=0：校正后只保留有效像素，不留黑边
    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(k1, d1, k2, d2, size, r, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    return StereoCalib(size, k1, d1, k2, d2, r, t, r1, r2, p1, p2, rms_l, rms_r, rms_s, len(keep), dropped)


def rectified_row_error(cal: StereoCalib, left: List[np.ndarray], right: List[np.ndarray]) -> Tuple[float, float]:
    """校正后同一角点左右行坐标差（像素）：(平均绝对值, 最大值)。理想为 0，静态样本 < 0.5 算合格。"""
    dys = []
    for lp, rp in zip(left, right):
        ul = cv2.undistortPoints(lp, cal.k1, cal.d1, R=cal.r1, P=cal.p1).reshape(-1, 2)
        ur = cv2.undistortPoints(rp, cal.k2, cal.d2, R=cal.r2, P=cal.p2).reshape(-1, 2)
        dys.append(np.abs(ul[:, 1] - ur[:, 1]))
    allv = np.concatenate(dys)
    return float(allv.mean()), float(allv.max())


def _mat(m: np.ndarray) -> dict:
    m = np.atleast_2d(m)
    return {"rows": int(m.shape[0]), "cols": int(m.shape[1]), "data": [float(v) for v in m.ravel()]}


def write_ros_yaml(path: Path, name: str, size: Tuple[int, int], k: np.ndarray, d: np.ndarray, r: np.ndarray, p: np.ndarray) -> None:
    """camera_calibration_parsers 格式，stereo.frames.load_camera_info 可读。"""
    doc = {
        "image_width": size[0],
        "image_height": size[1],
        "camera_name": name,
        "camera_matrix": _mat(k),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": _mat(d.reshape(1, -1)),
        "rectification_matrix": _mat(r),
        "projection_matrix": _mat(p),
    }
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")


def write_opencv_stereo(path: Path, cal: StereoCalib) -> None:
    """OpenCV FileStorage 格式，给 C++ VO（stereo/vo/）用 cv::FileStorage 直接读。"""
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(cal.size[0]))
    fs.write("image_height", int(cal.size[1]))
    for name, m in (("K1", cal.k1), ("D1", cal.d1), ("R1", cal.r1), ("P1", cal.p1),
                    ("K2", cal.k2), ("D2", cal.d2), ("R2", cal.r2), ("P2", cal.p2)):
        fs.write(name, np.asarray(m, np.float64))
    fs.write("baseline_m", cal.baseline)
    fs.release()


def save_results(cal: StereoCalib, out: Path, row_err: Tuple[float, float], measured_baseline_mm: Optional[float]) -> None:
    out.mkdir(parents=True, exist_ok=True)
    write_ros_yaml(out / "left.yaml", "stereo_left", cal.size, cal.k1, cal.d1, cal.r1, cal.p1)
    write_ros_yaml(out / "right.yaml", "stereo_right", cal.size, cal.k2, cal.d2, cal.r2, cal.p2)
    write_opencv_stereo(out / "stereo.yml", cal)
    report = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "image_size": list(cal.size),
        "samples_used": cal.used,
        "samples_dropped_as_outlier": cal.dropped,
        "rms_px": {"left": cal.rms_left, "right": cal.rms_right, "stereo": cal.rms_stereo},
        "rectified_row_error_px": {"mean": row_err[0], "max": row_err[1]},
        "baseline_mm": cal.baseline * 1000,
        "baseline_measured_mm": measured_baseline_mm,
        "R_right_from_left": _mat(cal.r),
        "T_right_from_left_m": _mat(cal.t.reshape(1, 3)),
        "rectified_fx_px": float(cal.p1[0, 0]),
    }
    (out / "stereo_report.yaml").write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True), encoding="utf-8")


def print_summary(cal: StereoCalib, row_err: Tuple[float, float], measured_baseline_mm: Optional[float]) -> None:
    print(f"样本 {cal.used} 张（剔除离群 {len(cal.dropped)} 张：{cal.dropped}）")
    print(f"重投影 RMS：左 {cal.rms_left:.3f} px，右 {cal.rms_right:.3f} px，双目 {cal.rms_stereo:.3f} px（目标 < 0.3～0.5）")
    print(f"校正后行差：平均 {row_err[0]:.3f} px，最大 {row_err[1]:.3f} px（目标平均 < 0.5）")
    line = f"基线 {cal.baseline * 1000:.2f} mm"
    if measured_baseline_mm:
        line += f"，尺量 {measured_baseline_mm:.1f} mm，差 {100 * (cal.baseline * 1000 / measured_baseline_mm - 1):+.1f}%"
    print(line)
    print(f"校正后焦距 fx = {cal.p1[0, 0]:.1f} px")
    if cal.t[0] > 0:
        print("⚠️ Tx 为正：左右目接反了（右目在左目左侧），切换 capture_node 的 swap_lr 后重采，或对调样本 _L/_R 离线重算；此结果不可用")


def load_samples(d: Path, board: Board) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    left, right = [], []
    size = (0, 0)
    for lp in sorted(d.glob("*_L.png")):
        li = cv2.imread(str(lp), cv2.IMREAD_GRAYSCALE)
        ri = cv2.imread(str(lp.with_name(lp.name.replace("_L.png", "_R.png"))), cv2.IMREAD_GRAYSCALE)
        cl, cr = detect(li, board), detect(ri, board)
        if cl is None or cr is None:
            print(f"跳过 {lp.name}：角点检测失败")
            continue
        left.append(cl)
        right.append(cr)
        size = (li.shape[1], li.shape[0])
    return left, right, size


def _draw(img: np.ndarray, corners: Optional[np.ndarray], board: Board) -> np.ndarray:
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if corners is not None:
        cv2.drawChessboardCorners(vis, (board.cols, board.rows), corners, True)
    return vis


def collect_live(board: Board, max_samples: int, sample_dir: Path, gui: bool) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    """订阅 /stereo/{left,right}/image_raw，按时间戳精确配对（采集节点保证左右同戳），取最新一对处理。"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("stereo_calib")
    pending: dict = {}
    latest: List[Optional[Tuple[np.ndarray, np.ndarray]]] = [None]

    def on_img(side: str):
        def cb(m: Image) -> None:
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).copy()
            other = pending.pop(key, None)
            if other is None:
                pending[key] = (side, img)
                if len(pending) > 20:  # 另一目丢了，清掉陈旧的半对
                    pending.pop(next(iter(pending)))
            else:
                latest[0] = (img, other[1]) if side == "left" else (other[1], img)

        return cb

    node.create_subscription(Image, "/stereo/left/image_raw", on_img("left"), 5)
    node.create_subscription(Image, "/stereo/right/image_raw", on_img("right"), 5)

    sample_dir.mkdir(parents=True, exist_ok=True)
    left: List[np.ndarray] = []
    right: List[np.ndarray] = []
    params: List[Tuple[float, float, float, float]] = []
    prev: Optional[np.ndarray] = None
    size = (0, 0)
    last_print = 0.0
    print(f"等待图像……样本存到 {sample_dir}")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if latest[0] is None:
                continue
            li, ri = latest[0]
            latest[0] = None
            size = (li.shape[1], li.shape[0])
            cl, cr = detect(li, board), detect(ri, board)
            status = "未检测到棋盘（两目都要完整看到）"
            if cl is not None and cr is not None:
                p = view_params(cl, board, size)
                if not is_still(prev, cl):
                    status = "棋盘在动，停住"
                elif not is_new_pose(p, params):
                    status = "姿态太接近已有样本，换个位置/角度/距离"
                else:
                    idx = len(left)
                    left.append(cl)
                    right.append(cr)
                    params.append(p)
                    cv2.imwrite(str(sample_dir / f"{idx:03d}_L.png"), li)
                    cv2.imwrite(str(sample_dir / f"{idx:03d}_R.png"), ri)
                    status = f"收下第 {idx + 1} 张"
                prev = cl
            else:
                prev = None
            cov = coverage(params)
            bars = " ".join(f"{n}:{c * 100:3.0f}%" for n, c in zip(("X", "Y", "Size", "Skew"), cov))
            done = (len(left) >= MIN_SAMPLES_AUTO and min(cov) >= 1.0) or len(left) >= max_samples
            now = time.monotonic()
            if status.startswith("收下") or now - last_print > 2.0:
                print(f"[{len(left):2d} 张] {bars} | {status}", flush=True)
                last_print = now
            if gui:
                vis = np.hstack([_draw(li, cl, board), _draw(ri, cr, board)])
                vis = cv2.resize(vis, None, fx=0.75, fy=0.75)
                bar = np.zeros((70, vis.shape[1], 3), np.uint8)
                # 窗口文字只用 ASCII：OpenCV putText 不支持中文
                cv2.putText(bar, f"samples {len(left)}  {bars}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                hint = "STILL+NEW -> sample | c: calibrate now | q: quit"
                cv2.putText(bar, hint, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.imshow("stereo_calib", np.vstack([vis, bar]))
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord("q")):
                    raise KeyboardInterrupt
                if k == ord("c") and len(left) >= MIN_SAMPLES_MANUAL:
                    done = True
            if done:
                break
    finally:
        if gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
    return left, right, size


def main() -> None:
    ap = argparse.ArgumentParser(description="双目标定（自动采样 + OpenCV）")
    ap.add_argument("--cols", type=int, required=True, help="内角点列数")
    ap.add_argument("--rows", type=int, required=True, help="内角点行数")
    ap.add_argument("--square", type=float, required=True, help="方格边长（米）")
    ap.add_argument("--out", type=Path, default=Path("stereo/calib/1280x480"), help="YAML 输出目录")
    ap.add_argument("--images", type=Path, help="离线重算：样本目录（*_L.png / *_R.png）")
    ap.add_argument("--max-samples", type=int, default=50)
    ap.add_argument("--baseline-mm", type=float, help="尺量基线，用于核对")
    ap.add_argument("--no-gui", action="store_true")
    a = ap.parse_args()
    board = Board(a.cols, a.rows, a.square)

    if a.images:
        left, right, size = load_samples(a.images, board)
    else:
        sample_dir = Path("logs/stereo_calib") / time.strftime("%Y%m%d_%H%M%S")
        try:
            left, right, size = collect_live(board, a.max_samples, sample_dir, not a.no_gui)
        except KeyboardInterrupt:
            print("已放弃，未标定")
            return
    if len(left) < MIN_SAMPLES_MANUAL:
        print(f"样本只有 {len(left)} 张，至少要 {MIN_SAMPLES_MANUAL} 张，未标定")
        return
    print(f"开始标定（{len(left)} 张，{size[0]}x{size[1]}）……")
    cal = calibrate_stereo(board, left, right, size)
    keep = [i for i in range(len(left)) if i not in cal.dropped]
    row_err = rectified_row_error(cal, [left[i] for i in keep], [right[i] for i in keep])
    save_results(cal, a.out, row_err, a.baseline_mm)
    print_summary(cal, row_err, a.baseline_mm)
    print(f"已写入 {a.out}/left.yaml、right.yaml、stereo_report.yaml")


if __name__ == "__main__":
    main()
