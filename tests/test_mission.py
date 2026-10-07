"""任务状态机：用 Px4Sim 逐帧驱动，覆盖正常流程、每个中止条件和外部接管。

请求经过 run_mission.RequestSender（与实机同一套去抖逻辑）再交给替身，
替身的心跳是 1 Hz，解锁状态和模式按真 PX4 一样滞后上报。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import pytest

from flight import px4_modes
from flight.fc_link import mavlink
from flight.mission import Command, Mission, MissionConfig, Phase, Waypoint
from flight.run_mission import RequestSender
from tests.px4_sim import Px4Sim

DT = 0.05


class SimLink:
    """把 RequestSender 的发送接到替身上，并按替身的接受与否生成 COMMAND_ACK。"""

    def __init__(self, sim: Px4Sim) -> None:
        self.sim = sim
        self.t = 0.0
        self._acks: dict[int, tuple[int, float]] = {}

    def last_ack(self, command: int) -> tuple[int, float] | None:
        return self._acks.get(command)

    def _ack(self, command: int, ok: bool) -> None:
        if self.sim.link_dead:
            return  # 链路断了连 ACK 都收不到
        result = mavlink.MAV_RESULT_ACCEPTED if ok else mavlink.MAV_RESULT_TEMPORARILY_REJECTED
        self._acks[command] = (result, self.t)

    def request_mode(self, mode: px4_modes.Px4Mode) -> None:
        self._ack(mavlink.MAV_CMD_DO_SET_MODE, self.sim.request_mode(self.t, mode))

    def request_arm(self) -> None:
        self._ack(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, self.sim.request_arm(self.t))

    def request_disarm(self) -> None:
        self._ack(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, self.sim.request_disarm(self.t))


@dataclass
class Trace:
    t: list[float] = field(default_factory=list)
    phase: list[Phase] = field(default_factory=list)
    alt: list[float] = field(default_factory=list)
    xy: list[tuple[float, float]] = field(default_factory=list)
    armed: list[bool] = field(default_factory=list)
    cmds: list[Command] = field(default_factory=list)


def fast_cfg(**kw: object) -> MissionConfig:
    base = dict(
        takeoff_alt=0.8, hover_s=3.0, ready_hold_s=0.5, climb_rate=0.5, xy_speed=0.5,
        settle_s=0.3, wp_hold_s=0.5, max_alt=1.5, max_radius=3.0,
    )
    base.update(kw)
    return MissionConfig(**base)  # type: ignore[arg-type]


def drive(
    mission: Mission,
    sim: Px4Sim,
    duration: float = 60.0,
    hook: Callable[[float, Px4Sim, Mission], None] | None = None,
) -> Trace:
    tr = Trace()
    link = SimLink(sim)
    sender = RequestSender(link)  # type: ignore[arg-type]
    t = 0.0
    while t < duration:
        if hook is not None:
            hook(t, sim, mission)
        s = sim.state(t)
        cmd = mission.step(s)
        link.t = t
        if cmd.setpoint is not None:
            sim.setpoint(t, cmd.setpoint)
        if cmd.request is not None:
            sender.handle(cmd.request, t)
        sim.tick(t, DT)
        tr.t.append(t)
        tr.phase.append(mission.phase)
        tr.alt.append(sim.alt)
        tr.xy.append((sim.x, sim.y))
        tr.armed.append(sim.armed)
        tr.cmds.append(cmd)
        if mission.finished and not sim.armed:
            break
        t += DT
    return tr


def _in_phase(target: Phase, after_s: float, action: Callable[[Px4Sim], None]) -> Callable[[float, Px4Sim, Mission], None]:
    """进入 target 阶段 after_s 秒后执行一次 action。"""
    state: dict[str, float] = {}

    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        if m.phase == target and "t0" not in state:
            state["t0"] = t
        if "t0" in state and "done" not in state and t - state["t0"] >= after_s:
            action(sim)
            state["done"] = t

    return hook


# ---------- 正常流程 ----------


def test_hover_only_completes_and_lands() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    tr = drive(m, sim)
    assert m.phase == Phase.DONE, m.reason
    assert not sim.armed and sim.alt == 0.0
    # 悬停阶段持续 ≥ hover_s，且高度在目标附近
    hover = [a for a, p in zip(tr.alt, tr.phase) if p == Phase.HOVER]
    assert len(hover) * DT >= 3.0 - DT
    assert all(abs(a - 0.8) < 0.15 for a in hover)
    assert max(tr.alt) < 0.8 + 0.15
    assert "任务完成" in m.reason


def test_ground_const_pos_flag_does_not_block_takeoff() -> None:
    # PX4 落地静止时置 CONST_POS_MODE（替身在地面自动置位），不能因此拒飞
    m = Mission(fast_cfg())
    sim = Px4Sim()
    s = sim.state(0.0)
    assert s.est_flags & 128
    drive(m, sim)
    assert m.phase == Phase.DONE, m.reason


def test_square_mission_visits_waypoints_and_returns_home() -> None:
    wps = [Waypoint(1.0, 0.0, 0.8), Waypoint(1.0, 1.0, 1.0), Waypoint(0.0, 1.0, 0.8)]
    m = Mission(fast_cfg(waypoints=wps))
    sim = Px4Sim()
    tr = drive(m, sim, duration=120.0)
    assert m.phase == Phase.DONE, m.reason
    # yaw=0：前=北(x)，右=东(y)
    for wp in wps:
        assert min(math.hypot(x - wp.forward, y - wp.right) for x, y in tr.xy) < 0.15
    assert math.hypot(sim.x, sim.y) < 0.15  # 回到起点降落


def test_waypoints_are_relative_to_takeoff_heading() -> None:
    # 机头朝东（yaw=90°）：向前 1 m 应该是 +y
    m = Mission(fast_cfg(waypoints=[Waypoint(1.0, 0.0, 0.8)], return_home=False))
    sim = Px4Sim(yaw=math.pi / 2)
    tr = drive(m, sim)
    assert m.phase == Phase.DONE, m.reason
    assert min(math.hypot(x - 0.0, y - 1.0) for x, y in tr.xy) < 0.15


def test_setpoint_moves_at_limited_speed() -> None:
    cfg = fast_cfg(waypoints=[Waypoint(2.0, 0.0, 0.8)])
    m = Mission(cfg)
    tr = drive(m, Px4Sim(), duration=120.0)
    sps = [c.setpoint for c in tr.cmds if c.setpoint is not None]
    for a, b in zip(sps, sps[1:]):
        assert math.hypot(b.x - a.x, b.y - a.y) <= cfg.xy_speed * DT + 1e-6
        assert abs(b.z - a.z) <= cfg.climb_rate * DT + 1e-6


def test_offboard_requested_only_after_prestream() -> None:
    from flight.mission import Request

    m = Mission(fast_cfg())
    tr = drive(m, Px4Sim())
    first_sp = next(i for i, c in enumerate(tr.cmds) if c.setpoint is not None)
    first_ob = next(i for i, c in enumerate(tr.cmds) if c.request == Request.OFFBOARD)
    assert (first_ob - first_sp) * DT >= 1.0
    # 先切 OFFBOARD 再解锁
    first_arm = next(i for i, c in enumerate(tr.cmds) if c.request == Request.ARM)
    assert first_arm > first_ob


# ---------- 起飞前拒绝 ----------


def test_no_rc_blocks_takeoff() -> None:
    m = Mission(fast_cfg(ready_timeout_s=3.0))
    sim = Px4Sim()
    sim.rc_ok = False
    tr = drive(m, sim)
    assert m.phase == Phase.FAILED
    assert "遥控器" in m.reason
    assert all(c.setpoint is None and c.request is None for c in tr.cmds)
    assert not any(tr.armed)


def test_no_rc_allowed_when_disabled() -> None:
    m = Mission(fast_cfg(require_rc=False))
    sim = Px4Sim()
    sim.rc_ok = False
    drive(m, sim)
    assert m.phase == Phase.DONE, m.reason


@pytest.mark.parametrize(
    "flags, keyword",
    [(0, "估计器缺少"), (0b1111, "高度"), (0b1001111 | 2048, "加速度计")],
)
def test_bad_estimator_blocks_takeoff(flags: int, keyword: str) -> None:
    m = Mission(fast_cfg(ready_timeout_s=2.0))
    sim = Px4Sim()
    sim.est_flags = flags
    tr = drive(m, sim)
    assert m.phase == Phase.FAILED
    assert keyword in m.reason
    assert not any(tr.armed)


def test_low_battery_blocks_takeoff() -> None:
    m = Mission(fast_cfg(ready_timeout_s=2.0, min_battery_v=14.0))
    sim = Px4Sim()
    sim.battery_v = 13.5
    drive(m, sim)
    assert m.phase == Phase.FAILED and "电池" in m.reason


@pytest.mark.parametrize("field_name, keyword", [("landed_t", "EXTENDED_SYS_STATE"), ("sys_t", "SYS_STATUS")])
def test_stale_status_blocks_takeoff(field_name: str, keyword: str) -> None:
    class StaleSim(Px4Sim):
        def state(self, t: float):  # noqa: ANN201
            s = super().state(t)
            setattr(s, field_name, -10.0)  # 很久以前收到过一次，之后再无更新
            return s

    m = Mission(fast_cfg(ready_timeout_s=3.0))
    sim = StaleSim()
    tr = drive(m, sim)
    assert m.phase == Phase.FAILED and keyword in m.reason and not any(tr.armed)


def test_estimator_lost_before_offboard_fails_on_ground() -> None:
    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        if m.phase == Phase.PRESTREAM:
            sim.est_flags = 0

    m = Mission(fast_cfg())
    sim = Px4Sim()
    tr = drive(m, sim, hook=hook)
    assert m.phase == Phase.FAILED and not any(tr.armed)


# ---------- 解锁阶段：心跳滞后下的上锁确认 ----------


def test_abort_while_arm_in_flight_actively_disarms() -> None:
    # 解锁已在 PX4 生效、但 1 Hz 心跳还没反映时中止：必须主动上锁，不能看着旧心跳就宣布结束
    aborted: dict[str, float] = {}

    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        if m.phase == Phase.ARM and sim.armed and "t" not in aborted:
            assert not sim.state(t).armed  # 确认此刻心跳确实还显示未解锁
            m.abort("test")
            aborted["t"] = t

    m = Mission(fast_cfg())
    sim = Px4Sim()
    tr = drive(m, sim, hook=hook)
    assert m.phase == Phase.FAILED and "操作员中止" in m.reason
    assert not sim.armed
    disarm_t = next(t for t, a in zip(tr.t, tr.armed) if t > aborted["t"] and not a)
    assert disarm_t - aborted["t"] < 2.5  # 主动上锁，而不是等 Offboard 丢失降落或 10 s 预飞超时


def test_mode_switch_during_arm_disarms() -> None:
    hook = _in_phase(Phase.ARM, 0.0, lambda s: setattr(s, "mode", px4_modes.POSCTL))
    m = Mission(fast_cfg())
    sim = Px4Sim()
    tr = drive(m, sim, hook=hook)
    assert m.phase == Phase.PREEMPTED and "解锁阶段" in m.reason, m.reason
    # 在途的解锁在 POSCTL 下生效了，程序须主动上锁，而不是等 PX4 的 10 s 预飞超时
    switch_i = next(i for i, p in enumerate(tr.phase) if p == Phase.ARM)
    armed_ticks = sum(tr.armed[switch_i:])
    assert armed_ticks * DT < 2.5
    assert not sim.armed


def test_disarm_unconfirmed_times_out_with_warning() -> None:
    aborted: dict[str, bool] = {}

    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        if m.phase == Phase.ARM and sim.armed and not aborted:
            m.abort("test")
            aborted["x"] = True

    m = Mission(fast_cfg())
    sim = Px4Sim()
    sim.stuck_armed = True
    drive(m, sim, hook=hook, duration=15.0)
    assert m.phase == Phase.FAILED and "上锁未确认" in m.reason


# ---------- 外部接管 ----------


@pytest.mark.parametrize("phase", [Phase.TAKEOFF, Phase.HOVER, Phase.GOTO, Phase.LAND])
def test_external_mode_switch_is_respected(phase: Phase) -> None:
    switched: dict[str, float] = {}

    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        if "t" in switched:
            return
        # LAND：在 AUTO.LAND 刚被接受、心跳还没反映时飞手拨到 POSCTL，最考验请求去抖
        hit = sim.mode == px4_modes.LAND if phase == Phase.LAND else m.phase == phase
        if hit:
            sim.mode = px4_modes.POSCTL
            switched["t"] = t

    cfg = fast_cfg(waypoints=[Waypoint(1.0, 0.0, 0.8)], hover_s=1.0)
    m = Mission(cfg)
    sim = Px4Sim()
    tr = drive(m, sim, hook=hook, duration=30.0)
    assert m.phase == Phase.PREEMPTED, m.reason
    assert sim.mode == px4_modes.POSCTL  # 飞手的模式没有被程序覆盖回去
    i = tr.phase.index(Phase.PREEMPTED)
    assert tr.t[i] - switched["t"] <= sim.hb_period + 2 * DT  # 下一次心跳就发现
    assert all(c.setpoint is None and c.request is None for c in tr.cmds[i:])


def test_unexpected_disarm_in_air_is_preempted() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    drive(m, sim, hook=_in_phase(Phase.HOVER, 0.5, lambda s: setattr(s, "armed", False)))
    assert m.phase == Phase.PREEMPTED and "上锁" in m.reason


# ---------- 空中安全检查 ----------


def test_altitude_limit_triggers_land() -> None:
    # 漂移大于替身位置环最大纠正速度，模拟失控上升
    m = Mission(fast_cfg(max_alt=1.2))
    sim = Px4Sim()
    drive(m, sim, hook=_in_phase(Phase.HOVER, 0.5, lambda s: setattr(s, "drift", (0.0, 0.0, -5.0))))
    assert "高度" in m.reason


def test_radius_limit_triggers_land() -> None:
    m = Mission(fast_cfg(max_radius=1.0))
    sim = Px4Sim()
    drive(m, sim, hook=_in_phase(Phase.HOVER, 0.5, lambda s: setattr(s, "drift", (5.0, 0.0, 0.0))))
    assert "水平偏离" in m.reason


def test_range_sensor_loss_in_air_triggers_land() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    drive(m, sim, hook=_in_phase(Phase.HOVER, 0.5, lambda s: setattr(s, "range_dead", True)))
    assert m.phase == Phase.DONE
    assert "DISTANCE_SENSOR" in m.reason
    assert not sim.armed


def test_range_stuck_low_in_hover_triggers_land() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    drive(m, sim, hook=_in_phase(Phase.HOVER, 0.5, lambda s: setattr(s, "range_stuck", 0.0)))
    assert m.phase == Phase.DONE and "贴地" in m.reason


def test_range_brief_dip_does_not_land() -> None:
    # 测距短暂掉到 0.05（0.5 s，如反光毛刺）不应触发；机器人从下方经过（读数 0.3）也不应触发
    hover_start: dict[str, float] = {}

    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        if m.phase == Phase.HOVER:
            ht = t - hover_start.setdefault("t", t)
            sim.range_stuck = 0.05 if 0.5 <= ht < 1.0 else (0.3 if 1.5 <= ht < 2.5 else None)

    m = Mission(fast_cfg())
    drive(m, Px4Sim(), hook=hook)
    assert m.phase == Phase.DONE and "任务完成" in m.reason


def test_const_pos_mode_in_hover_triggers_land() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    drive(m, sim, hook=_in_phase(Phase.HOVER, 0.5, lambda s: setattr(s, "const_pos_in_air", True)))
    assert m.phase == Phase.DONE and "定点模式" in m.reason


def test_const_pos_mode_ignored_during_takeoff() -> None:
    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        # 起飞段光流刚开始融合，置位不应打断起飞；进入悬停后清除
        sim.const_pos_in_air = m.phase in (Phase.WAIT_READY, Phase.PRESTREAM, Phase.OFFBOARD, Phase.ARM, Phase.TAKEOFF)

    m = Mission(fast_cfg())
    drive(m, Px4Sim(), hook=hook)
    assert m.phase == Phase.DONE and "任务完成" in m.reason


def test_takeoff_timeout_lands() -> None:
    m = Mission(fast_cfg(leg_margin_s=2.0))
    sim = Px4Sim()
    sim.climb_ok = False
    drive(m, sim)
    assert m.phase == Phase.DONE
    assert "起飞超时" in m.reason


def test_battery_dip_is_debounced_but_sustained_low_lands() -> None:
    cfg = fast_cfg(min_battery_v=14.0, battery_persist_s=1.0, hover_s=5.0)
    hover_start: dict[str, float] = {}

    def dip(t: float, sim: Px4Sim, m: Mission) -> None:
        # 悬停中只掉 0.5 s：不应触发
        if m.phase == Phase.HOVER:
            hover_t = t - hover_start.setdefault("t", t)
            sim.battery_v = 13.0 if 1.0 <= hover_t < 1.5 else 16.0

    m = Mission(cfg)
    drive(m, Px4Sim(), hook=dip)
    assert "任务完成" in m.reason

    m2 = Mission(cfg)
    drive(m2, Px4Sim(), hook=_in_phase(Phase.HOVER, 1.0, lambda s: setattr(s, "battery_v", 13.0)))
    assert "电池电压持续低于" in m2.reason

    m3 = Mission(cfg)
    drive(m3, Px4Sim(), hook=_in_phase(Phase.HOVER, 1.0, lambda s: setattr(s, "battery_v", None)))
    assert "电压读数丢失" in m3.reason


# ---------- 降落交接给 PX4 失效保护 ----------


def test_link_loss_in_air_hands_off_to_px4_failsafe() -> None:
    cfg = fast_cfg()
    m = Mission(cfg)
    sim = Px4Sim()
    tr = drive(m, sim, duration=30.0, hook=_in_phase(Phase.HOVER, 1.0, lambda s: setattr(s, "link_dead", True)))
    # 高频消息 stale_s(0.5 s) 先于心跳超时触发
    assert "LOCAL_POSITION_NED 超时" in m.reason
    assert "失效保护" in m.reason
    # 链路断着无法确认上锁，程序停在 LAND 且不发任何指令
    assert m.phase == Phase.LAND
    land_i = tr.phase.index(Phase.LAND)
    tail = tr.cmds[land_i + int(cfg.mode_timeout_s / DT) + 2:]
    assert tail and all(c.setpoint is None and c.request is None for c in tail)
    # 替身：设定点中断 → AUTO.LAND 落地上锁
    assert sim.mode == px4_modes.LAND and sim.alt == 0.0 and not sim.armed


def test_land_request_ignored_hands_off() -> None:
    def hook(t: float, sim: Px4Sim, m: Mission) -> None:
        sim.ignore_land = True

    m = Mission(fast_cfg())
    sim = Px4Sim()
    drive(m, sim, hook=hook)
    # PX4 不接受 LAND 命令 → 停发设定点 → PX4 Offboard 丢失保护降落
    assert m.phase == Phase.DONE, m.reason
    assert m.reason.count("失效保护") == 1
    assert not sim.armed


# ---------- 操作员中止 ----------


def test_operator_abort_in_air_lands() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    drive(m, sim, hook=lambda t, s, mm: mm.abort("test") if mm.phase == Phase.HOVER else None)
    assert m.phase == Phase.DONE and "操作员中止" in m.reason
    assert not sim.armed


def test_operator_abort_on_ground_never_arms() -> None:
    m = Mission(fast_cfg())
    sim = Px4Sim()
    tr = drive(m, sim, hook=lambda t, s, mm: mm.abort("test") if mm.phase == Phase.PRESTREAM else None)
    assert m.phase == Phase.FAILED and not any(tr.armed)


# ---------- 配置校验 ----------


@pytest.mark.parametrize(
    "kw",
    [
        dict(takeoff_alt=2.0, max_alt=1.5),
        dict(takeoff_alt=0.1),
        dict(waypoints=[Waypoint(3.0, 3.0, 0.8)], max_radius=2.0),
        dict(waypoints=[Waypoint(0.5, 0.0, 1.8)], max_alt=1.5),
        dict(xy_speed=0.0),
        dict(wp_hold_s="2"),
        dict(return_home="false"),
        dict(min_battery_v=True),
        dict(waypoints=[Waypoint("1", 0.0, 0.8)]),
    ],
)
def test_invalid_config_rejected(kw: dict) -> None:
    with pytest.raises(ValueError):
        Mission(fast_cfg(**kw))
