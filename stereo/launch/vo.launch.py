"""双目 VO 一键启动：采集节点 -> image_proc 校正 -> RTAB-Map 双目里程计（只做 VO，不起 SLAM 建图，免得回环掩盖漂移）。

用法（仓库根目录）：
    source /opt/ros/humble/setup.bash
    ros2 launch stereo/launch/vo.launch.py                 # 带 rtabmap_viz 窗口
    ros2 launch stereo/launch/vo.launch.py viz:=false      # 无界面
另开终端看质量：python3 -m stereo.odom_eval

话题 /stereo/odom、/stereo/odom_info；TF stereo_odom -> stereo_base -> stereo_optical。
stereo_base 按 REP-103（x 前 y 左 z 上），暂定与左目光心重合；装机后按实际安装位姿改 stereo_base -> stereo_optical
的静态 TF，不要旋转图像（见 AGENTS.md 5.8）。
"""
from __future__ import annotations

import math
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

REPO = Path(__file__).resolve().parents[2]
CALIB = REPO / "stereo" / "calib" / "1280x480"

# 话题与坐标系一律带 stereo 前缀：本机同一 ROS 域里常有小车工程的仿真在跑（也发 /odom、odom→base_link），
# 实测混用会互相串话、TF 打架。两目时间戳逐帧相同，用精确同步
ODOM_PARAMS = {
    "frame_id": "stereo_base",
    "odom_frame_id": "stereo_odom",
    "subscribe_stereo": True,
    "approx_sync": False,
    "publish_tf": True,
    "wait_for_transform": 0.2,
}
# ~/.bashrc 把海康 MVS SDK 的库目录加进了 LD_LIBRARY_PATH，其中的旧 libusb 抢先加载，
# 实测 RTAB-Map 节点启动即崩（libpcl_io: undefined symbol libusb_set_option）。只对 RTAB-Map 节点剔除，不动全局配置。
RTABMAP_ENV = {
    "LD_LIBRARY_PATH": ":".join(p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p and not p.startswith("/opt/MVS")),
}
STEREO_REMAP = [
    ("left/image_rect", "/stereo/left/image_rect"),
    ("left/camera_info", "/stereo/left/camera_info"),
    ("right/image_rect", "/stereo/right/image_rect"),
    ("right/camera_info", "/stereo/right/camera_info"),
    ("odom", "/stereo/odom"),
    ("odom_info", "/stereo/odom_info"),
]


def rectify(side: str) -> ComposableNode:
    return ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name=f"rectify_{side}",
        remappings=[
            ("image", f"/stereo/{side}/image_raw"),
            ("camera_info", f"/stereo/{side}/camera_info"),
            ("image_rect", f"/stereo/{side}/image_rect"),
        ],
    )


def generate_launch_description() -> LaunchDescription:
    viz = LaunchConfiguration("viz")
    return LaunchDescription([
        DeclareLaunchArgument("viz", default_value="true", description="是否打开 rtabmap_viz"),
        ExecuteProcess(
            cmd=[
                "python3", "-m", "stereo.capture_node", "--ros-args",
                "-p", f"left_info:={CALIB / 'left.yaml'}",
                "-p", f"right_info:={CALIB / 'right.yaml'}",
            ],
            cwd=str(REPO),
            output="screen",
        ),
        ComposableNodeContainer(
            name="stereo_rectify",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[rectify("left"), rectify("right")],
            output="screen",
        ),
        # 光学系（z 前 x 右 y 下）相对机体系（x 前 y 左 z 上）
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", str(-math.pi / 2), "--pitch", "0", "--yaw", str(-math.pi / 2),
                "--frame-id", "stereo_base", "--child-frame-id", "stereo_optical",
            ],
        ),
        Node(
            package="rtabmap_odom",
            executable="stereo_odometry",
            output="screen",
            parameters=[ODOM_PARAMS],
            remappings=STEREO_REMAP,
            additional_env=RTABMAP_ENV,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=[{**ODOM_PARAMS, "subscribe_odom_info": True, "odometry_node_name": "stereo_odometry"}],
            remappings=STEREO_REMAP,
            additional_env=RTABMAP_ENV,
            condition=IfCondition(viz),
        ),
    ])
