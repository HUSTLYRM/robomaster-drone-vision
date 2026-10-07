"""自主飞行任务状态机（纯逻辑，无 IO）。

流程：等待就绪 → 预推设定点 → 切 Offboard → 解锁 → 起飞 → 悬停 → 依次飞航点 → 回起点 → 降落。

设计原则：
- 每个控制周期调用一次 step(state)，返回本周期要发的设定点和模式请求，便于 pytest 逐帧驱动。
- 不和人抢控制权：一旦已确认进入 Offboard，模式再被切走（遥控器拨杆、PX4 自身失效保护），
  立即进入 PREEMPTED 并停止发送任何指令。
- 空中出任何问题一律降落，不尝试"自救"。请求 AUTO.LAND 若迟迟不生效，就停发设定点，
  让 PX4 的 Offboard 丢失保护接手（COM_OF_LOSS_T 后直接执行 COM_OBL_RC_ACT=4 Land；
  该路径是模式回退，不经 COM_FAIL_ACT_T 的 Hold 延迟——SITL 实测与源码一致）。
- 解锁状态只能从 1 Hz 的心跳得知，最多滞后 1 s。凡是发过解锁请求后要判断"已上锁"，
  都必须以晚于最后一次解锁请求的心跳为准；没起飞就离开解锁阶段时主动发上锁命令。
- 航点用「起飞时机头方向」的前/右/上表示：飞控没有磁力计，NED 的"北"只是 EKF 初始化方向，
  对人没有意义。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, replace
from enum import Enum

from . import px4_modes
from .state import (
    EST_ACCEL_ERROR,
    EST_ATTITUDE,
    EST_CONST_POS_MODE,
    EST_POS_HORIZ_REL,
    EST_POS_VERT_ABS,
    EST_POS_VERT_AGL,
    EST_VELOCITY_HORIZ,
    EST_VELOCITY_VERT,
    LANDED_ON_GROUND,
    VehicleState,
)


class Phase(Enum):
    WAIT_READY = "WAIT_READY"
    PRESTREAM = "PRESTREAM"
    OFFBOARD = "OFFBOARD"
    ARM = "ARM"
    DISARM = "DISARM"  # 解锁阶段没起飞就退出：确认上锁后再进入终态
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    GOTO = "GOTO"
    LAND = "LAND"
    DONE = "DONE"
    FAILED = "FAILED"
    PREEMPTED = "PREEMPTED"


AIRBORNE_PHASES = (Phase.TAKEOFF, Phase.HOVER, Phase.GOTO)
TERMINAL_PHASES = (Phase.DONE, Phase.FAILED, Phase.PREEMPTED)


class Request(Enum):
    OFFBOARD = "OFFBOARD"
    ARM = "ARM"
    DISARM = "DISARM"
    LAND = "LAND"


@dataclass
class Setpoint:
    """LOCAL_NED 位置设定点，yaw 为弧度。"""

    x: float
    y: float
    z: float
    yaw: float


@dataclass
class Command:
    setpoint: Setpoint | None = None
    request: Request | None = None


@dataclass
class Waypoint:
    """相对起飞点、按起飞时机头方向定义：forward 前、right 右、up 高度，单位 m。"""

    forward: float
    right: float
    up: float


@dataclass
class MissionConfig:
    takeoff_alt: float = 0.8
    hover_s: float = 12.0  # 规则要求持续悬停 ≥10 s，留余量
    waypoints: list[Waypoint] = field(default_factory=list)
    wp_hold_s: float = 2.0
    return_home: bool = True

    climb_rate: float = 0.3  # m/s，设定点爬升/下降速度
    xy_speed: float = 0.3  # m/s，设定点水平移动速度
    reach_tol: float = 0.15  # m，到点判定半径
    settle_s: float = 1.0  # 到点后需持续在容差内的时间

    # 安全边界：越界立即降落
    max_alt: float = 1.5
    max_radius: float = 2.0
    # 悬停/航段中测距持续低于该值 = 已贴地或测距失效（SITL 实测：EKF 高度漂移时飞机沉到地面而估计仍显示 0.7 m）。
    # 不做"测距与估计高度一致性"检查：机器人从下方经过时测距会合法跳变，阈值只防贴地
    min_air_range_m: float = 0.1
    range_low_persist_s: float = 1.0
    min_battery_v: float | None = None  # 实飞必须设置（run_mission 会检查），按电芯数定
    battery_persist_s: float = 2.0  # 电压持续低于阈值多久才算低电（避免机动瞬间压降误触发）
    require_rc: bool = True  # 规则要求急停遥控；仅桌面/SITL 可关

    stale_s: float = 0.5  # 高频消息（位置/姿态 30 Hz、测距/估计器 10 Hz）超过这么久没更新视为失效
    link_timeout_s: float = 3.0  # 心跳 1 Hz 及低频消息（SYS_STATUS/EXTENDED_SYS_STATE）超时，容忍丢 2 包
    ready_hold_s: float = 2.0  # 自检需连续通过的时间
    ready_timeout_s: float = 30.0
    prestream_s: float = 1.5  # PX4 Offboard 文档：设定点流持续 >1 s 才接受切模式/解锁
    mode_timeout_s: float = 3.0  # 切模式/解锁的等待上限（心跳 1 Hz，确认最多滞后 1 s）
    disarm_timeout_s: float = 5.0  # 主动上锁的确认上限
    leg_margin_s: float = 10.0  # 起飞/航段超时 = 理论用时 + 该余量

    def validate(self) -> None:
        # JSON 里写成字符串（"2"、"false"）不会在比较前报错，到空中才炸，这里先拦住
        for f in fields(self):
            v = getattr(self, f.name)
            if f.name == "waypoints":
                if not all(isinstance(wp, Waypoint) for wp in v):
                    raise ValueError("waypoints 每项须为 Waypoint")
                for wp in v:
                    for c in (wp.forward, wp.right, wp.up):
                        if isinstance(c, bool) or not isinstance(c, (int, float)):
                            raise ValueError(f"航点坐标须为数值: {wp}")
            elif f.name in ("return_home", "require_rc"):
                if not isinstance(v, bool):
                    raise ValueError(f"{f.name} 须为 true/false，实际 {v!r}")
            elif f.name == "min_battery_v" and v is None:
                continue
            elif isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError(f"{f.name} 须为数值，实际 {v!r}")
        if not 0.2 <= self.takeoff_alt <= self.max_alt:
            raise ValueError(f"takeoff_alt={self.takeoff_alt} 须在 [0.2, max_alt={self.max_alt}] 内")
        for i, wp in enumerate(self.waypoints):
            if not 0.2 <= wp.up <= self.max_alt:
                raise ValueError(f"航点 {i} 高度 {wp.up} 须在 [0.2, max_alt={self.max_alt}] 内")
            if math.hypot(wp.forward, wp.right) > self.max_radius:
                raise ValueError(f"航点 {i} 超出 max_radius={self.max_radius}")
        if self.climb_rate <= 0 or self.xy_speed <= 0:
            raise ValueError("climb_rate / xy_speed 必须为正")


_EST_REQUIRED = EST_ATTITUDE | EST_VELOCITY_HORIZ | EST_VELOCITY_VERT | EST_POS_HORIZ_REL
_EST_VERT = EST_POS_VERT_ABS | EST_POS_VERT_AGL
ARM_SETTLE_S = 0.3  # 解锁/上锁命令在 PX4 内生效并反映到下一次心跳所需的余量


def estimator_problems(flags: int, const_pos_bad: bool) -> list[str]:
    """const_pos_bad：是否把 CONST_POS_MODE 当异常。落地静止时 PX4 也会置这一位
    （v1.16.1 ekf_helper.cpp: fake_pos || valid_fake_pos || vehicle_at_rest），
    所以只在空中才算；起飞段光流刚开始融合，同样不判。"""
    problems = []
    missing = _EST_REQUIRED & ~flags
    if missing:
        problems.append(f"估计器缺少有效解 flags=0x{flags:x} 缺 0x{missing:x}")
    if not flags & _EST_VERT:
        problems.append("估计器无有效高度")
    if flags & EST_ACCEL_ERROR:
        problems.append(f"估计器加速度计异常 flags=0x{flags:x}")
    if const_pos_bad and flags & EST_CONST_POS_MODE:
        problems.append(f"估计器退回定点模式（无光流融合）flags=0x{flags:x}")
    return problems


class Mission:
    def __init__(self, cfg: MissionConfig) -> None:
        cfg.validate()
        self.cfg = cfg
        self.phase = Phase.WAIT_READY
        self.reason = ""  # 最近一次状态变化/中止的原因，供日志
        self.problems: list[str] = []  # 最近一次检查发现的问题

        self._t0: float | None = None  # 任务开始时间
        self._phase_t = 0.0  # 进入当前阶段的时间
        self._last_now: float | None = None
        self._ready_since: float | None = None
        self._settle_since: float | None = None
        self._battery_low_since: float | None = None
        self._range_low_since: float | None = None
        self._offboard_confirmed = False
        self._land_requested_t: float | None = None
        self._land_handoff = False  # 已停发设定点，交给 PX4 失效保护
        self._last_arm_req_t: float | None = None
        self._after_disarm: tuple[Phase, str] = (Phase.FAILED, "")

        # 起飞点（自检通过那一刻捕获）
        self._home: Setpoint | None = None
        self._sp: Setpoint | None = None  # 当前正在发送的设定点
        self._targets: list[Setpoint] = []  # 待飞航段目标（含回起点）
        self._target: Setpoint | None = None
        self._leg_deadline = 0.0
        self._holding_since: float | None = None

    # ---------- 对外接口 ----------

    @property
    def finished(self) -> bool:
        return self.phase in TERMINAL_PHASES

    def abort(self, reason: str) -> None:
        """操作员中止（Ctrl-C 等），须在两次 step() 之间调用。空中 → 降落；地面 → 确保上锁后结束。"""
        now = self._last_now or 0.0
        if self.phase in AIRBORNE_PHASES:
            self._enter(Phase.LAND, f"操作员中止: {reason}", now)
        elif self.phase in (Phase.WAIT_READY, Phase.PRESTREAM, Phase.OFFBOARD, Phase.ARM):
            self._end_on_ground(Phase.FAILED, f"操作员中止: {reason}", now)

    def step(self, s: VehicleState) -> Command:
        now = s.now
        dt = 0.0 if self._last_now is None else max(0.0, now - self._last_now)
        self._last_now = now
        if self._t0 is None:
            self._t0 = now
            self._phase_t = now

        handler = {
            Phase.WAIT_READY: self._wait_ready,
            Phase.PRESTREAM: self._prestream,
            Phase.OFFBOARD: self._offboard,
            Phase.ARM: self._arm,
            Phase.DISARM: self._disarm,
            Phase.TAKEOFF: self._airborne,
            Phase.HOVER: self._airborne,
            Phase.GOTO: self._airborne,
            Phase.LAND: self._land,
        }.get(self.phase)
        if handler is None:
            return Command()
        return handler(s, dt)

    def _cmd(self, request: Request | None = None) -> Command:
        """带当前设定点的命令。返回拷贝，避免调用方持有的历史命令随 self._sp 原地变化。"""
        assert self._sp is not None
        return Command(setpoint=replace(self._sp), request=request)

    # ---------- 检查 ----------

    def _fresh(self, s: VehicleState, t: float | None) -> bool:
        return t is not None and s.now - t <= self.cfg.stale_s

    def _link_ok(self, s: VehicleState) -> bool:
        return s.heartbeat_t is not None and s.now - s.heartbeat_t <= self.cfg.link_timeout_s

    def _slow_fresh(self, s: VehicleState, t: float | None) -> bool:
        return t is not None and s.now - t <= self.cfg.link_timeout_s

    def _disarm_confirmed(self, s: VehicleState) -> bool:
        """上锁须由晚于最后一次解锁请求的心跳确认，否则可能是请求生效前的旧心跳。"""
        if s.armed or not self._link_ok(s):
            return False
        assert s.heartbeat_t is not None
        return self._last_arm_req_t is None or s.heartbeat_t >= self._last_arm_req_t + ARM_SETTLE_S

    def _common_problems(self, s: VehicleState, const_pos_bad: bool = False) -> list[str]:
        problems = []
        if not self._link_ok(s):
            problems.append("飞控心跳丢失")
        if not self._fresh(s, s.est_t):
            problems.append("ESTIMATOR_STATUS 超时")
        else:
            problems += estimator_problems(s.est_flags, const_pos_bad)
        if not self._fresh(s, s.pos_t):
            problems.append("LOCAL_POSITION_NED 超时")
        if not self._fresh(s, s.att_t):
            problems.append("ATTITUDE 超时")
        if not self._fresh(s, s.range_t):
            problems.append("DISTANCE_SENSOR 超时")
        if not self._slow_fresh(s, s.sys_t):
            problems.append("SYS_STATUS 超时（遥控/电压状态未知）")
        if self.cfg.require_rc and not s.rc_ok:
            problems.append("遥控器未连接（急停链路不可用）")
        return problems

    def _battery_low(self, s: VehicleState) -> bool:
        if self.cfg.min_battery_v is None or s.battery_v is None:
            return False
        if s.battery_v >= self.cfg.min_battery_v:
            self._battery_low_since = None
            return False
        if self._battery_low_since is None:
            self._battery_low_since = s.now
        return s.now - self._battery_low_since >= self.cfg.battery_persist_s

    def _range_low(self, s: VehicleState) -> bool:
        # 起飞段刚离地时测距本来就小，只在悬停/航段判
        if self.phase not in (Phase.HOVER, Phase.GOTO) or s.range_m >= self.cfg.min_air_range_m:
            self._range_low_since = None
            return False
        if self._range_low_since is None:
            self._range_low_since = s.now
        return s.now - self._range_low_since >= self.cfg.range_low_persist_s

    def preflight_problems(self, s: VehicleState) -> list[str]:
        problems = self._common_problems(s)
        if self.cfg.min_battery_v is not None:
            if s.battery_v is None:
                problems.append("无电池电压")
            elif s.battery_v < self.cfg.min_battery_v:
                problems.append(f"电池电压 {s.battery_v:.2f} V 低于 {self.cfg.min_battery_v} V")
        if not self._slow_fresh(s, s.landed_t):
            problems.append("EXTENDED_SYS_STATE 超时")
        if s.armed:
            problems.append("飞控已处于解锁状态")
        if s.landed_state != LANDED_ON_GROUND:
            problems.append(f"未判定为落地状态 landed_state={s.landed_state}")
        return problems

    def _airborne_problems(self, s: VehicleState) -> list[str]:
        problems = self._common_problems(s, const_pos_bad=self.phase != Phase.TAKEOFF)
        assert self._home is not None
        alt = self._home.z - s.z
        if alt > self.cfg.max_alt:
            problems.append(f"高度 {alt:.2f} m 超过 {self.cfg.max_alt} m")
        radius = math.hypot(s.x - self._home.x, s.y - self._home.y)
        if radius > self.cfg.max_radius:
            problems.append(f"水平偏离 {radius:.2f} m 超过 {self.cfg.max_radius} m")
        if self._range_low(s):
            problems.append(f"测距持续低于 {self.cfg.min_air_range_m} m（贴地或测距失效）")
        if self.cfg.min_battery_v is not None and s.battery_v is None:
            problems.append("电池电压读数丢失")
        elif self._battery_low(s):
            problems.append(f"电池电压持续低于 {self.cfg.min_battery_v} V")
        return problems

    # ---------- 阶段 ----------

    def _enter(self, phase: Phase, reason: str, now: float) -> None:
        self.phase = phase
        self.reason = reason
        self._phase_t = now
        self._settle_since = None
        self._holding_since = None

    def _wait_ready(self, s: VehicleState, dt: float) -> Command:
        self.problems = self.preflight_problems(s)
        if self.problems:
            self._ready_since = None
            assert self._t0 is not None
            if s.now - self._t0 > self.cfg.ready_timeout_s:
                self._enter(Phase.FAILED, "自检超时: " + "; ".join(self.problems), s.now)
            return Command()
        if self._ready_since is None:
            self._ready_since = s.now
        if s.now - self._ready_since < self.cfg.ready_hold_s:
            return Command()

        self._home = Setpoint(s.x, s.y, s.z, s.yaw)
        self._sp = Setpoint(s.x, s.y, s.z, s.yaw)
        self._targets = self._build_targets()
        self._enter(Phase.PRESTREAM, "自检通过", s.now)
        return self._cmd()

    def _build_targets(self) -> list[Setpoint]:
        home = self._home
        assert home is not None
        c, sn = math.cos(home.yaw), math.sin(home.yaw)
        targets = []
        for wp in self.cfg.waypoints:
            north = wp.forward * c - wp.right * sn
            east = wp.forward * sn + wp.right * c
            targets.append(Setpoint(home.x + north, home.y + east, home.z - wp.up, home.yaw))
        if self.cfg.return_home and targets:
            targets.append(Setpoint(home.x, home.y, home.z - self.cfg.takeoff_alt, home.yaw))
        return targets

    def _end_on_ground(self, phase: Phase, reason: str, now: float) -> None:
        """未起飞就结束。发过解锁请求的，先经 DISARM 确认上锁，避免在途的解锁让电机空转。"""
        if self._last_arm_req_t is None:
            self._enter(phase, reason, now)
        else:
            self._after_disarm = (phase, reason)
            self._enter(Phase.DISARM, reason, now)

    def _ground_guard(self, s: VehicleState) -> bool:
        """起飞前阶段的检查；出问题时结束任务（尚未离地，无需降落）。"""
        problems = self._common_problems(s)
        if problems:
            self.problems = problems
            self._end_on_ground(Phase.FAILED, "起飞前检查失败: " + "; ".join(problems), s.now)
            return False
        return True

    def _prestream(self, s: VehicleState, dt: float) -> Command:
        if not self._ground_guard(s):
            return Command()
        if s.now - self._phase_t >= self.cfg.prestream_s:
            self._enter(Phase.OFFBOARD, "设定点流已建立", s.now)
        return self._cmd()

    def _offboard(self, s: VehicleState, dt: float) -> Command:
        if not self._ground_guard(s):
            return Command()
        if s.mode == px4_modes.OFFBOARD:
            self._offboard_confirmed = True
            self._enter(Phase.ARM, "已进入 OFFBOARD", s.now)
            return self._cmd()
        if s.now - self._phase_t > self.cfg.mode_timeout_s:
            self._enter(Phase.FAILED, f"切 OFFBOARD 超时，当前 {px4_modes.name(s.mode)}", s.now)
            return Command()
        return self._cmd(request=Request.OFFBOARD)

    def _arm(self, s: VehicleState, dt: float) -> Command:
        if s.mode != px4_modes.OFFBOARD:
            self._end_on_ground(Phase.PREEMPTED, f"解锁阶段模式被切到 {px4_modes.name(s.mode)}", s.now)
            return Command()
        if s.armed:
            self._enter(Phase.TAKEOFF, "已解锁", s.now)
            self._leg_deadline = s.now + self.cfg.takeoff_alt / self.cfg.climb_rate + self.cfg.leg_margin_s
            return self._cmd()
        if not self._ground_guard(s):
            return Command()
        if s.now - self._phase_t > self.cfg.mode_timeout_s:
            self._end_on_ground(Phase.FAILED, "解锁超时（看 PX4 STATUSTEXT 的拒绝原因）", s.now)
            return Command()
        self._last_arm_req_t = s.now
        return self._cmd(request=Request.ARM)

    def _disarm(self, s: VehicleState, dt: float) -> Command:
        phase, reason = self._after_disarm
        if self._disarm_confirmed(s):
            self._enter(phase, reason, s.now)
            return Command()
        if s.now - self._phase_t > self.cfg.disarm_timeout_s:
            self._enter(phase, reason + "；⚠ 上锁未确认，务必人工确认电机已停", s.now)
            return Command()
        return Command(request=Request.DISARM)

    def _airborne(self, s: VehicleState, dt: float) -> Command:
        if not s.armed:
            self._enter(Phase.PREEMPTED, "空中阶段飞控意外上锁", s.now)
            return Command()
        if s.mode != px4_modes.OFFBOARD:
            self._enter(Phase.PREEMPTED, f"模式被切到 {px4_modes.name(s.mode)}，放弃控制", s.now)
            return Command()
        problems = self._airborne_problems(s)
        if problems:
            self.problems = problems
            return self._begin_land(s, "安全检查触发: " + "; ".join(problems))

        if self.phase == Phase.TAKEOFF:
            return self._takeoff(s, dt)
        if self.phase == Phase.HOVER:
            return self._hover(s, dt)
        return self._goto(s, dt)

    def _settled(self, s: VehicleState, target: Setpoint, vertical_only: bool = False) -> bool:
        if vertical_only:
            dist = abs(s.z - target.z)
        else:
            dist = math.sqrt((s.x - target.x) ** 2 + (s.y - target.y) ** 2 + (s.z - target.z) ** 2)
        if dist > self.cfg.reach_tol:
            self._settle_since = None
            return False
        if self._settle_since is None:
            self._settle_since = s.now
        return s.now - self._settle_since >= self.cfg.settle_s

    def _move_sp_toward(self, target: Setpoint, dt: float) -> None:
        """设定点按限速向目标推进，而不是直接跳到目标，避免 PX4 以最大速度猛冲。"""
        sp = self._sp
        assert sp is not None
        dx, dy = target.x - sp.x, target.y - sp.y
        dxy = math.hypot(dx, dy)
        step_xy = self.cfg.xy_speed * dt
        if dxy <= step_xy:
            sp.x, sp.y = target.x, target.y
        else:
            sp.x += dx / dxy * step_xy
            sp.y += dy / dxy * step_xy
        dz = target.z - sp.z
        step_z = self.cfg.climb_rate * dt
        sp.z = target.z if abs(dz) <= step_z else sp.z + math.copysign(step_z, dz)
        sp.yaw = target.yaw

    def _takeoff(self, s: VehicleState, dt: float) -> Command:
        home = self._home
        assert home is not None
        target = Setpoint(home.x, home.y, home.z - self.cfg.takeoff_alt, home.yaw)
        self._move_sp_toward(target, dt)
        # 起飞只判高度：光流定点在爬升段的水平晃动不该拖住起飞判定
        if self._settled(s, target, vertical_only=True):
            self._enter(Phase.HOVER, f"到达起飞高度 {self.cfg.takeoff_alt} m", s.now)
        elif s.now > self._leg_deadline:
            return self._begin_land(s, "起飞超时未到达目标高度")
        return self._cmd()

    def _hover(self, s: VehicleState, dt: float) -> Command:
        if s.now - self._phase_t >= self.cfg.hover_s:
            return self._next_leg(s)
        return self._cmd()

    def _next_leg(self, s: VehicleState) -> Command:
        if not self._targets:
            return self._begin_land(s, "任务完成")
        self._target = self._targets.pop(0)
        sp = self._sp
        assert sp is not None
        dist_xy = math.hypot(self._target.x - sp.x, self._target.y - sp.y)
        dist_z = abs(self._target.z - sp.z)
        expected = max(dist_xy / self.cfg.xy_speed, dist_z / self.cfg.climb_rate)
        self._leg_deadline = s.now + expected + self.cfg.leg_margin_s
        self._enter(Phase.GOTO, f"飞往 ({self._target.x:.2f}, {self._target.y:.2f}, {self._target.z:.2f})", s.now)
        return self._cmd()

    def _goto(self, s: VehicleState, dt: float) -> Command:
        target = self._target
        assert target is not None
        self._move_sp_toward(target, dt)
        if self._holding_since is not None:
            if s.now - self._holding_since >= self.cfg.wp_hold_s:
                return self._next_leg(s)
        elif self._settled(s, target):
            self._holding_since = s.now
        elif s.now > self._leg_deadline:
            return self._begin_land(s, "航段超时未到达")
        return self._cmd()

    def _begin_land(self, s: VehicleState, reason: str) -> Command:
        self._enter(Phase.LAND, reason, s.now)
        return self._land(s, 0.0)

    def _land(self, s: VehicleState, dt: float) -> Command:
        if self._disarm_confirmed(s):
            self._enter(Phase.DONE, f"已落地上锁（降落原因: {self.reason}）", s.now)
            return Command()
        if self._land_handoff:
            # 已交给 PX4 失效保护，只等上锁，不再发任何指令（期间模式是什么都不干预）
            return Command()
        if s.mode == px4_modes.LAND:
            self._land_requested_t = None
            return Command()
        if self._offboard_confirmed and self._link_ok(s) and s.mode != px4_modes.OFFBOARD:
            # 既非 OFFBOARD 也非 LAND：遥控接管或 PX4 失效保护切了别的模式，不再干预
            self._enter(Phase.PREEMPTED, f"降落阶段模式被切到 {px4_modes.name(s.mode)}", s.now)
            return Command()
        if self._land_requested_t is None:
            self._land_requested_t = s.now
        if s.now - self._land_requested_t > self.cfg.mode_timeout_s:
            # 降落请求迟迟不生效（链路断/命令被拒）：停发设定点，触发 PX4 Offboard 丢失保护
            if not self._land_handoff:
                self._land_handoff = True
                self.reason += "；AUTO.LAND 请求未生效，停发设定点交给 PX4 失效保护"
            return Command()
        return self._cmd(request=Request.LAND)
