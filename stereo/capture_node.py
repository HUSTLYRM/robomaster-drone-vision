#!/usr/bin/env python3
"""双目 UVC 相机 ROS2 采集节点：V4L2 读 YUYV 拼接帧 -> 劈左右 -> 发布 mono8 图像与 camera_info。

话题（左右同一时间戳，时间戳取驱动缓冲区时间换算到系统时间）：
    /stereo/left/image_raw   /stereo/left/camera_info
    /stereo/right/image_raw  /stereo/right/camera_info

用法（本机，系统 python3 + ROS2 Humble，仓库根目录）：
    source /opt/ros/humble/setup.bash
    python3 -m stereo.capture_node --ros-args -p exposure:=50 -p gain:=64
    # 标定后：-p left_info:=calib/left.yaml -p right_info:=calib/right.yaml

参数：
    device      /dev/video4          设备节点（插拔后可能变，用 v4l2-ctl --list-devices 核对）
    width/height 1280x480            拼接帧尺寸，只能是 frames.STEREO_MODES 里的双目模式
    exposure    0                    0 = 自动曝光；>0 写 exposure_time_absolute（单位未实测确认）
    gain        -1                   -1 = 不改；>=0 写 gain（0~255）
    frame_id    stereo_optical       左右共用，基线由右目 P 矩阵的 Tx 表达（ROS 双目约定）
    swap_lr     false                true = 拼接帧右半当左目。由两路排线插哪个口决定（与相机朝向无关），
                                     当前 85 mm 支架接法不对调；重新接线后用 calibrate.py 复核（Tx 为正即反了）
    left_info / right_info ""        camera_calibration 导出的 YAML；空 = 发未标定（全 0）的 camera_info
"""
from __future__ import annotations

import array
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from stereo.frames import STEREO_MODES, CameraInfoData, DropCounter, load_camera_info, monotonic_to_wall_ns, split_stereo_y

STATS_PERIOD_S = 5.0


def set_v4l2_ctrls(device: str, exposure: int, gain: int) -> None:
    """用 v4l2-ctl 写曝光/增益；OpenCV 的 CAP_PROP_EXPOSURE 在 V4L2 后端的映射不可靠。"""
    ctrls = ["auto_exposure=3"] if exposure <= 0 else ["auto_exposure=1", f"exposure_time_absolute={exposure}"]
    if gain >= 0:
        ctrls.append(f"gain={gain}")
    subprocess.run(["v4l2-ctl", "-d", device, f"--set-ctrl={','.join(ctrls)}"], check=True, capture_output=True)


def to_camera_info_msg(info: CameraInfoData, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = info.width
    msg.height = info.height
    msg.distortion_model = info.distortion_model
    msg.d = info.d
    msg.k = info.k
    msg.r = info.r
    msg.p = info.p
    return msg


class StereoCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_capture")
        self.device: str = self.declare_parameter("device", "/dev/video4").value
        width: int = self.declare_parameter("width", 1280).value
        height: int = self.declare_parameter("height", 480).value
        exposure: int = self.declare_parameter("exposure", 0).value
        gain: int = self.declare_parameter("gain", -1).value
        self.frame_id: str = self.declare_parameter("frame_id", "stereo_optical").value
        self.swap_lr: bool = self.declare_parameter("swap_lr", False).value
        left_info: str = self.declare_parameter("left_info", "").value
        right_info: str = self.declare_parameter("right_info", "").value

        if (width, height) not in STEREO_MODES:
            raise ValueError(f"{width}x{height} 不是双目拼接模式，可选：{sorted(STEREO_MODES)}")
        self.fps = STEREO_MODES[(width, height)]
        self.eye_w, self.eye_h = width // 2, height

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开 {self.device}（被占用时 fuser -v {self.device} 查占用进程）")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        # 不让 OpenCV 转 BGR，直接拿 h×w×2 的 YUYV 原始数据
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        got = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (width, height):
            raise RuntimeError(f"驱动没接受 {width}x{height}，实际 {got}")
        set_v4l2_ctrls(self.device, exposure, gain)

        self.info = {
            side: to_camera_info_msg(
                load_camera_info(Path(p), self.eye_w, self.eye_h) if p else CameraInfoData(self.eye_w, self.eye_h),
                self.frame_id,
            )
            for side, p in (("left", left_info), ("right", right_info))
        }
        if not (left_info and right_info):
            self.get_logger().warn("未给标定文件，camera_info 为未标定状态（全 0），只能用于标定")

        self.pub_img = {s: self.create_publisher(Image, f"/stereo/{s}/image_raw", 10) for s in ("left", "right")}
        self.pub_info = {s: self.create_publisher(CameraInfo, f"/stereo/{s}/camera_info", 10) for s in ("left", "right")}
        self.drops = DropCounter(self.fps)
        self.get_logger().info(
            f"{self.device} {width}x{height}@{self.fps} YUYV，每目 {self.eye_w}x{self.eye_h}，"
            f"曝光 {'自动' if exposure <= 0 else exposure}，增益 {'不改' if gain < 0 else gain}"
        )

    def spin_capture(self) -> None:
        stats_t0 = time.monotonic()
        stats_f0 = stats_d0 = 0
        while rclpy.ok():
            ok, raw = self.cap.read()
            now_mono, now_wall = time.monotonic_ns(), time.time_ns()
            if not ok:
                self.get_logger().error("读帧失败（设备拔出？）")
                break
            buf_ns = int(self.cap.get(cv2.CAP_PROP_POS_MSEC) * 1e6)
            self.drops.update(buf_ns)
            stamp_ns = monotonic_to_wall_ns(buf_ns, now_mono, now_wall)

            for side, img in zip(("left", "right"), split_stereo_y(raw, self.swap_lr)):
                msg = Image()
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                msg.header.frame_id = self.frame_id
                msg.height, msg.width = img.shape
                msg.encoding = "mono8"
                msg.step = msg.width
                # 必须给 array('B')：赋 bytes 会让 rclpy 逐元素做 Python 类型检查，实测只剩 19 fps
                msg.data = array.array("B", img.tobytes())
                info = self.info[side]
                info.header.stamp = msg.header.stamp
                self.pub_img[side].publish(msg)
                self.pub_info[side].publish(info)

            dt = time.monotonic() - stats_t0
            if dt >= STATS_PERIOD_S:
                n, d = self.drops.frames - stats_f0, self.drops.dropped - stats_d0
                lat_ms = (now_mono - buf_ns) / 1e6
                self.get_logger().info(f"{n / dt:.1f} fps，丢帧 {d}，读取延迟 {lat_ms:.1f} ms")
                stats_t0, stats_f0, stats_d0 = time.monotonic(), self.drops.frames, self.drops.dropped

    def destroy_node(self) -> None:
        self.cap.release()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = StereoCaptureNode()
    try:
        node.spin_capture()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
