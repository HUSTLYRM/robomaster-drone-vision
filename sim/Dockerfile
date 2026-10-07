# PX4 v1.16.1 SITL 构建/运行环境：Ubuntu 22.04 + PX4 官方 ubuntu.sh（含 Gazebo Harmonic），宿主机零安装。
# 构建上下文必须是 PX4 源码的 Tools/setup 目录（ubuntu.sh 要求 requirements.txt 同目录），见 sim/sitl.sh。
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai
# 本机网络下 http://archive.ubuntu.com 会被中间设备劫持返回假 404（2026-10-07 实测 22/923 个包失败），
# 与宿主机一致改用清华镜像；apt 仍校验 GPG 签名。其他网络环境可 --build-arg UBUNTU_MIRROR=... 覆盖
ARG UBUNTU_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu
RUN sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" /etc/apt/sources.list
# ubuntu.sh 靠 /.dockerenv 判断容器环境，docker build 阶段没有该文件，它自带的前置依赖不会装，这里手动补上
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates gnupg gosu lsb-release software-properties-common sudo wget tzdata \
    && rm -rf /var/lib/apt/lists/*
COPY ubuntu.sh requirements.txt /tmp/px4-setup/
# --no-nuttx：只做 SITL，不装飞控交叉编译链
RUN bash /tmp/px4-setup/ubuntu.sh --no-nuttx && rm -rf /var/lib/apt/lists/* /tmp/px4-setup
# ubuntu.sh 在 22.04 上用 pip --user 装到 /root/.local，而容器以宿主机 uid 运行（避免编译产物归 root），
# 看不到这些包，所以再装一份到系统路径
COPY requirements.txt /tmp/px4-req.txt
# PYTHONNOUSERSITE：否则 pip 认为 /root/.local 里已满足而跳过安装
RUN PYTHONNOUSERSITE=1 python3 -m pip install --no-cache-dir -r /tmp/px4-req.txt && rm /tmp/px4-req.txt
