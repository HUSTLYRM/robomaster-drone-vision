#!/usr/bin/env python3
"""机载自主飞行任务入口：连接飞控 → 请求消息流 → 20 Hz 运行任务状态机 → 写 CSV 遥测。

用法（仓库根目录执行，SSH 上务必在 tmux 里跑：SSH 断线 = SIGHUP = 请求降落）：
    # 只做自检，不解锁不起飞（装机后第一步，桌面可反复跑）
    python3 -m flight.run_mission --check-only
    # 实机飞任务（默认 /dev/ttyS8 @921600，即鲁班猫 UART8 ↔ 飞控 TELEM3）
    python3 -m flight.run_mission --mission flight/missions/hover_test.json
    # PX4 SITL（无遥控器）
    python3 -m flight.run_mission --url udpin:0.0.0.0:14540 --no-rc --mission ...

中止：Ctrl-C / SIGTERM / SIGHUP 一次 → 空中则请求降落、地面则确认上锁；再按一次 Ctrl-C →
立即退出程序（PX4 在 COM_OF_LOSS_T 后按 COM_OBL_RC_ACT 直接降落，SITL 实测约 1.5 s 进入 AUTO.LAND）。
真正的急停永远是遥控器 Kill 开关。
"""
from __future__ import annotations

import argparse
import csv
import json
import signal
import sys
import time
from dataclasses import fields
from pathlib import Path

from loguru import logger

from . import px4_modes
from .fc_link import MESSAGE_RATES_HZ, FcLink, mavlink
from .mission import AIRBORNE_PHASES, Command, Mission, MissionConfig, Phase, Request, Waypoint
from .state import VehicleState

LOOP_HZ = 20.0
REQUEST_INTERVAL_S = 0.5  # 同一请求未获 ACK 时的重发间隔
ACK_HOLD_S = 2.0  # 请求获 ACCEPTED 后，等心跳（1 Hz）反映结果的时间，期间不重发，避免覆盖飞手操作
EMERGENCY_LAND_S = 3.0

_REQUEST_COMMAND = {
    Request.OFFBOARD: mavlink.MAV_CMD_DO_SET_MODE,
    Request.LAND: mavlink.MAV_CMD_DO_SET_MODE,
    Request.ARM: mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    Request.DISARM: mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
}
# 程序掌握控制权的阶段：这些阶段里主循环崩溃，才由程序兜底请求降落
_IN_CONTROL_PHASES = AIRBORNE_PHASES + (Phase.ARM, Phase.DISARM, Phase.LAND)


def load_config(path: str | None) -> MissionConfig:
    cfg = MissionConfig()
    if path is None:
        return cfg
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    known = {f.name for f in fields(MissionConfig)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"任务文件含未知字段: {sorted(unknown)}")
    for key, value in data.items():
        if key == "waypoints":
            if not all(isinstance(wp, list) and len(wp) == 3 for wp in value):
                raise ValueError("waypoints 每项须为 [前, 右, 高] 三个数")
            value = [Waypoint(*wp) for wp in value]
        setattr(cfg, key, value)
    return cfg


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="/dev/ttyS8", help="串口设备或 pymavlink 连接串")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--mission", help="任务 JSON（字段同 MissionConfig），缺省为原地悬停")
    p.add_argument("--check-only", action="store_true", help="只循环打印自检结果，绝不解锁")
    p.add_argument("--no-rc", action="store_true", help="不要求遥控器在线（仅限拆桨桌面或 SITL）")
    p.add_argument("--no-battery-check", action="store_true", help="允许任务不设 min_battery_v（仅限 SITL）")
    p.add_argument("--log-dir", default="flight_logs")
    return p.parse_args(argv)


def setup_logging(log_dir: str) -> Path:
    # enqueue：日志写入放到独立线程，SSH 卡顿时写终端阻塞也不会拖住 20 Hz 主循环
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    path = Path(log_dir) / time.strftime("mission_%Y%m%d_%H%M%S.log")
    logger.remove()
    logger.add(sys.stderr, level="INFO", enqueue=True)
    logger.add(path, level="DEBUG", enqueue=True, encoding="utf-8")
    return path


class TelemetryCsv:
    COLUMNS = [
        "t", "phase", "mode", "armed", "landed", "x", "y", "z", "vx", "vy", "vz", "yaw",
        "range_m", "est_flags", "battery_v", "rc_ok", "sp_x", "sp_y", "sp_z", "request",
    ]

    def __init__(self, log_dir: str) -> None:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        self.path = Path(log_dir) / time.strftime("mission_%Y%m%d_%H%M%S.csv")
        # 行缓冲：程序崩溃时最后几秒的遥测恰恰最有用，不能留在缓冲区里丢掉
        self._f = self.path.open("w", newline="", encoding="utf-8", buffering=1)
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLUMNS)

    def write(self, mission: Mission, s: VehicleState, cmd: Command) -> None:
        sp = cmd.setpoint
        self._w.writerow([
            f"{s.now:.3f}", mission.phase.value, px4_modes.name(s.mode), int(s.armed), s.landed_state,
            f"{s.x:.3f}", f"{s.y:.3f}", f"{s.z:.3f}", f"{s.vx:.3f}", f"{s.vy:.3f}", f"{s.vz:.3f}",
            f"{s.yaw:.4f}", f"{s.range_m:.3f}", s.est_flags,
            "" if s.battery_v is None else f"{s.battery_v:.2f}", int(s.rc_ok),
            "" if sp is None else f"{sp.x:.3f}", "" if sp is None else f"{sp.y:.3f}",
            "" if sp is None else f"{sp.z:.3f}", "" if cmd.request is None else cmd.request.value,
        ])

    def close(self) -> None:
        self._f.close()


class RequestSender:
    """请求去抖：未获 ACK 每 0.5 s 重发；获 ACCEPTED 后 2 s 内不再重发。

    心跳只有 1 Hz，状态机看到结果最多滞后 1 s。若期间一直重发，飞手刚拨走的模式会被
    重发的 AUTO.LAND/ARM 覆盖回来。
    """

    def __init__(self, link: FcLink) -> None:
        self._link = link
        self._sent_t: dict[Request, float] = {}
        self._acked_t: dict[Request, float] = {}

    def handle(self, req: Request, now: float) -> None:
        sent = self._sent_t.get(req)
        if sent is not None:
            ack = self._link.last_ack(_REQUEST_COMMAND[req])
            if ack is not None and ack[1] >= sent and ack[0] == mavlink.MAV_RESULT_ACCEPTED:
                self._acked_t[req] = ack[1]
            acked = self._acked_t.get(req)
            if acked is not None and acked >= sent and now - acked < ACK_HOLD_S:
                return
            if now - sent < REQUEST_INTERVAL_S:
                return
        self._sent_t[req] = now
        if req == Request.OFFBOARD:
            self._link.request_mode(px4_modes.OFFBOARD)
        elif req == Request.LAND:
            self._link.request_mode(px4_modes.LAND)
        elif req == Request.ARM:
            self._link.request_arm()
        elif req == Request.DISARM:
            self._link.request_disarm()


def check_only(link: FcLink, mission: Mission) -> int:
    logger.info("自检模式：不会解锁，Ctrl-C 退出")
    try:
        while True:
            s = link.snapshot()
            problems = mission.preflight_problems(s)
            logger.info(
                f"模式={px4_modes.name(s.mode)} 解锁={s.armed} 位置=({s.x:.2f},{s.y:.2f},{-s.z:.2f}) "
                f"测距={s.range_m:.2f} 估计器=0x{s.est_flags:x} 电压={s.battery_v} 遥控={s.rc_ok}"
            )
            logger.info("自检通过" if not problems else "未通过: " + "; ".join(problems))
            link.send_heartbeat()
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


def _emergency_land(link: FcLink) -> None:
    deadline = time.monotonic() + EMERGENCY_LAND_S
    while time.monotonic() < deadline:
        try:
            link.request_mode(px4_modes.LAND)
        except Exception as e:  # 串口都写不进去时只能交给 PX4 失效保护
            logger.error(f"紧急降落请求发送失败: {e!r}")
            return
        time.sleep(0.3)


def run(link: FcLink, mission: Mission, log_dir: str) -> int:
    telemetry = TelemetryCsv(log_dir)
    logger.info(f"遥测写入 {telemetry.path}")

    # 信号处理里只记标志：处理函数可能插在 step() 中途，直接改阶段会被 step 后续的赋值覆盖
    pending = {"n": 0, "signum": 0}

    def on_signal(signum: int, frame: object) -> None:
        pending["n"] += 1
        pending["signum"] = signum
        if pending["n"] >= 2 and signum == signal.SIGINT:
            raise KeyboardInterrupt

    handled = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    old_handlers = {sig: signal.signal(sig, on_signal) for sig in handled}

    sender = RequestSender(link)
    aborted = False
    last_hb = 0.0
    last_phase: Phase | None = None
    last_reason = ""
    last_problems: list[str] = []
    period = 1.0 / LOOP_HZ
    next_t = time.monotonic()
    try:
        while True:
            if pending["n"] and not aborted:
                aborted = True
                name = signal.Signals(pending["signum"]).name
                logger.warning(f"收到 {name}：请求中止（再按一次 Ctrl-C 立即退出）")
                mission.abort(name)

            s = link.snapshot()
            cmd = mission.step(s)

            if cmd.setpoint is not None:
                link.send_setpoint(cmd.setpoint)
            if cmd.request is not None:
                sender.handle(cmd.request, s.now)
            if s.now - last_hb >= 1.0:
                last_hb = s.now
                link.send_heartbeat()

            # 连续航段同属 GOTO 阶段，所以原因变化也要记，否则日志里只看得到第一段
            if mission.phase != last_phase or mission.reason != last_reason:
                logger.info(f"[{mission.phase.value}] {mission.reason}")
                last_phase, last_reason = mission.phase, mission.reason
            if mission.phase == Phase.WAIT_READY and mission.problems != last_problems:
                logger.info("等待自检: " + ("; ".join(mission.problems) or "通过，保持中"))
                last_problems = list(mission.problems)
            telemetry.write(mission, s, cmd)

            # 被接管时程序不再发指令，但继续记录直到上锁，方便事后分析。
            # 终态的上锁结论已在状态机里用"晚于解锁请求的心跳"确认过
            if mission.finished and not s.armed:
                break

            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # 落后了就不补帧
    except KeyboardInterrupt:
        logger.error("强制退出：已停止发送设定点，由 PX4 失效保护接管")
        return 2
    except Exception:
        logger.exception(f"主循环异常（阶段 {mission.phase.value}）")
        if mission.phase in _IN_CONTROL_PHASES:
            logger.error("程序掌握控制权期间崩溃：直接请求 AUTO.LAND")
            _emergency_land(link)
        return 3
    finally:
        telemetry.close()
        for sig, h in old_handlers.items():
            signal.signal(sig, h)

    logger.info(f"任务结束: {mission.phase.value} — {mission.reason}")
    return 0 if mission.phase == Phase.DONE else 1


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_path = setup_logging(args.log_dir)
    logger.info(f"日志写入 {log_path}")
    cfg = load_config(args.mission)
    if args.no_rc:
        cfg.require_rc = False
        logger.warning("已关闭遥控器在线检查：只允许拆桨桌面测试或 SITL")
    if cfg.min_battery_v is None and not (args.check_only or args.no_battery_check):
        logger.error("任务未设 min_battery_v（PX4 默认低电只告警不动作），拒绝起飞")
        return 2
    mission = Mission(cfg)  # 构造时校验配置，配置错误在连接飞控前就报出

    link = FcLink(args.url, baud=args.baud)
    try:
        link.connect()
        failed = link.request_message_rates()
        if failed:
            logger.warning(f"以下消息频率请求未被确认: {failed}（共 {len(MESSAGE_RATES_HZ)} 项），自检会据实判断")
        if args.check_only:
            return check_only(link, mission)
        return run(link, mission, args.log_dir)
    finally:
        link.close()
        logger.complete()


if __name__ == "__main__":
    sys.exit(main())
