#!/usr/bin/env bash
# PX4 v1.16.1 SITL（docker 内运行，宿主机零安装）。用法：
#   sim/sitl.sh src     # 浅克隆 PX4 源码到 PX4_SRC（约 1.7 GB，只需一次）
#   sim/sitl.sh image   # 构建镜像（只需一次，约 10~20 min）
#   sim/sitl.sh build   # 编译 px4_sitl_default（只需一次，约 15~30 min）
#   sim/sitl.sh sih     # 运行 SIH 四旋翼（PX4 内置物理，无 Gazebo，最轻；有 GPS）
#   sim/sitl.sh gz      # 运行 Gazebo x500_flow（光流 + 测距，与实机传感器配置一致），无界面
#   sim/sitl.sh reset   # 清除仿真持久化参数，回到默认
# 运行后在宿主机：uv run python -m flight.run_mission --url udpin:0.0.0.0:14540 --no-rc --mission ...
# ⚠ gz 在无 GPU 的容器里实时因子只有约 0.2，任务按墙钟计时会误判超时，目前只适合做静态检查（见 flight/README.md）
# PX4 源码默认在仓库同级 ../PX4-Autopilot（v1.16.1 + 子模块），可用 PX4_SRC 覆盖。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_SRC="${PX4_SRC:-$(cd "$HERE/../.." && pwd)/PX4-Autopilot}"
IMAGE="rm-px4-sitl:1.16.1"
NAME="rm-px4-sitl"
# 笔记本 16 GB 内存：限制容器内存与编译并行数，防止 OOM 拖死桌面
MEM_LIMIT="${MEM_LIMIT:-7g}"
JOBS="${JOBS:-4}"

GZ_EXTRA=()
run() {
    local tty=()
    [ -t 0 ] && tty=(-t)  # 交互终端里才分配 TTY，后台/脚本调用时不加，否则 docker 报 not a TTY
    docker run --rm -i "${tty[@]}" --name "$NAME" --network host --memory "$MEM_LIMIT" \
        --user "$(id -u):$(id -g)" -e HOME=/tmp -e HEADLESS=1 "${GZ_EXTRA[@]}" \
        -v "$PX4_SRC":/src/PX4-Autopilot -w /src/PX4-Autopilot "$IMAGE" "$@"
}

case "${1:-}" in
    src)
        git clone --branch v1.16.1 --depth 1 --recurse-submodules --shallow-submodules -j 8 \
            https://github.com/PX4/PX4-Autopilot.git "$PX4_SRC"
        # 浅克隆的 NuttX 子模块没有 tag，px_update_git_header.py 会 IndexError；
        # 取上游版本最高的 tag，与完整克隆时脚本选中的结果一致（只影响版本字符串）
        git -C "$PX4_SRC/platforms/nuttx/NuttX/nuttx" fetch --depth 1 origin \
            'refs/tags/nuttx-12.12.0:refs/tags/nuttx-12.12.0'
        ;;
    image) docker build -t "$IMAGE" -f "$HERE/Dockerfile" "$PX4_SRC/Tools/setup" ;;
    build) run make -j"$JOBS" px4_sitl_default ;;
    # 直接运行编译产物（等同 make px4_sitl <model>，但不再触发构建检查）。
    # -d：不启动 pxh 交互 shell——后台运行时 stdin 是 EOF，pxh 会死循环刷提示符（实测几十秒写出 40 MB 日志）
    sih) run env -C build/px4_sitl_default/rootfs PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ../bin/px4 -d ;;
    # GZ_PARTITION 必须显式设：容器以宿主机 uid 运行，/etc/passwd 里没有该用户，gz-transport 推不出默认分区
    # （hostname:username），PX4 与 gz 互相发现不了，一直卡在 "Waiting for Gazebo world"（2026-10-07 实测根因，
    # 与 lo 是否开组播无关）。world 用 sim/worlds/rm_noshadow.sdf：容器无 GPU，阴影软件渲染使实时因子跌到 0.013
    gz) GZ_EXTRA=(-e GZ_PARTITION=rm_px4_sitl -e PX4_GZ_WORLD=rm_noshadow
            -v "$HERE/worlds/rm_noshadow.sdf":/src/PX4-Autopilot/Tools/simulation/gz/worlds/rm_noshadow.sdf:ro)
        run env -C build/px4_sitl_default/rootfs PX4_SIM_MODEL=gz_x500_flow GZ_IP=127.0.0.1 ../bin/px4 -d ;;
    # 仿真参数会持久化在 rootfs 里（apply_params 写入的值下次仍在），需要回到出厂默认时执行
    reset) rm -f "$PX4_SRC"/build/px4_sitl_default/rootfs/parameters*.bson ;;
    *) sed -n '2,11p' "$0"; exit 1 ;;
esac
