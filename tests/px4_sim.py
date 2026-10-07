"""测试用的 PX4 运动学替身：按我们对 PX4 v1.16 规则的理解响应设定点和命令。

编码的规则（依据 PX4 v1.16.1 文档与源码）：
- 心跳固定 1 Hz（mavlink_main.cpp），解锁状态和模式只能经心跳得知，最多滞后 1 s；
- 设定点流持续 >1 s 后才接受切 OFFBOARD；
- OFFBOARD 中设定点中断超过 COM_OF_LOSS_T → 直接 AUTO.LAND（模式回退，不经 COM_FAIL_ACT_T Hold，SITL 实测）；
- AUTO.LAND 落地后自动上锁（COM_DISARM_LAND）；解锁后一直没起飞，COM_DISARM_PRFLT(10 s) 后自动上锁；
- 普通上锁命令在空中被拒；
- 落地时估计器置 CONST_POS_MODE（vehicle_at_rest / valid_fake_pos）。
这只是替身，通过测试不代表与真实 PX4 兼容，以 SITL 和实物为准。
"""
from __future__ import annotations

from flight import px4_modes
from flight.mission import Setpoint
from flight.state import (
    EST_ATTITUDE,
    EST_CONST_POS_MODE,
    EST_POS_HORIZ_REL,
    EST_POS_VERT_AGL,
    EST_VELOCITY_HORIZ,
    EST_VELOCITY_VERT,
    LANDED_IN_AIR,
    LANDED_ON_GROUND,
    VehicleState,
)

GOOD_EST_FLAGS = EST_ATTITUDE | EST_VELOCITY_HORIZ | EST_VELOCITY_VERT | EST_POS_HORIZ_REL | EST_POS_VERT_AGL


class Px4Sim:
    def __init__(self, yaw: float = 0.0) -> None:
        self.armed = False
        self.mode = px4_modes.POSCTL
        self.x = self.y = self.z = 0.0  # NED，地面 z=0
        self.yaw = yaw
        self.sp: Setpoint | None = None
        self.last_sp_t: float | None = None
        self.stream_since: float | None = None
        self.on_ground_since: float | None = None
        self.armed_t: float | None = None
        self.took_off = False
        self.of_loss_t = 1.0  # COM_OF_LOSS_T
        self.disarm_prflt = 10.0  # COM_DISARM_PRFLT
        self.hb_period = 1.0
        self.max_speed = 2.0
        self.land_speed = 0.5

        # 心跳快照：(时间, 解锁, 模式)
        self._hb: tuple[float, bool, px4_modes.Px4Mode] | None = None

        # 故障注入
        self.est_flags = GOOD_EST_FLAGS
        self.const_pos_in_air = False
        self.rc_ok = True
        self.battery_v: float | None = 16.0
        self.range_dead = False
        self.range_stuck: float | None = None  # 测距卡在固定读数
        self.link_dead = False
        self.climb_ok = True
        self.drift = (0.0, 0.0, 0.0)  # m/s，叠加在控制之外的漂移（NED）
        self.ignore_land = False
        self.stuck_armed = False  # 上锁命令被拒、自动上锁也不发生（卡死）
        self._frozen: VehicleState | None = None

    @property
    def alt(self) -> float:
        return -self.z

    @property
    def on_ground(self) -> bool:
        return self.z >= -0.02

    def _stream_alive(self, t: float) -> bool:
        return self.last_sp_t is not None and t - self.last_sp_t <= self.of_loss_t

    # ---------- 来自伴随计算机的消息 ----------

    def setpoint(self, t: float, sp: Setpoint) -> None:
        if self.link_dead:
            return
        if not self._stream_alive(t):
            self.stream_since = t
        self.last_sp_t = t
        self.sp = Setpoint(sp.x, sp.y, sp.z, sp.yaw)

    def request_mode(self, t: float, mode: px4_modes.Px4Mode) -> bool:
        if self.link_dead:
            return False
        if mode == px4_modes.OFFBOARD:
            if not (self._stream_alive(t) and self.stream_since is not None and t - self.stream_since > 1.0):
                return False
        if mode == px4_modes.LAND and self.ignore_land:
            return False
        self.mode = mode
        return True

    def request_arm(self, t: float) -> bool:
        if self.link_dead or self.armed:
            return False
        if self.mode == px4_modes.OFFBOARD and not self._stream_alive(t):
            return False
        self.armed = True
        self.armed_t = t
        self.took_off = False
        self.on_ground_since = None
        return True

    def request_disarm(self, t: float) -> bool:
        if self.link_dead or not self.on_ground or self.stuck_armed:
            return False
        self.armed = False
        return True

    # ---------- 仿真推进 ----------

    def tick(self, t: float, dt: float) -> None:
        if self.armed and self.mode == px4_modes.OFFBOARD and not self._stream_alive(t):
            self.mode = px4_modes.LAND

        if self.armed and self.mode == px4_modes.OFFBOARD and self.sp is not None:
            for axis in ("x", "y", "z"):
                err = getattr(self.sp, axis) - getattr(self, axis)
                if axis == "z" and err < 0 and not self.climb_ok:
                    err = 0.0
                step = max(-self.max_speed * dt, min(self.max_speed * dt, err * 4.0 * dt))
                setattr(self, axis, getattr(self, axis) + step)
            self.yaw = self.sp.yaw
        elif self.armed and self.mode == px4_modes.LAND:
            self.z += self.land_speed * dt
        if self.armed:
            self.x += self.drift[0] * dt
            self.y += self.drift[1] * dt
            self.z += self.drift[2] * dt
        if self.z < -0.1:
            self.took_off = True
        if self.z >= 0.0:
            self.z = 0.0
            if self.armed and self.mode == px4_modes.LAND:
                if self.on_ground_since is None:
                    self.on_ground_since = t
                elif t - self.on_ground_since >= 0.5 and not self.stuck_armed:
                    self.armed = False
        if (self.armed and not self.took_off and not self.stuck_armed and self.armed_t is not None
                and t - self.armed_t > self.disarm_prflt):
            self.armed = False

        if not self.link_dead and (self._hb is None or t - self._hb[0] >= self.hb_period):
            self._hb = (t, self.armed, self.mode)

    def state(self, t: float) -> VehicleState:
        if self.link_dead and self._frozen is not None:
            s = VehicleState(**vars(self._frozen))
            s.now = t
            return s
        hb_t, hb_armed, hb_mode = self._hb if self._hb is not None else (None, False, px4_modes.POSCTL)
        flags = self.est_flags
        if self.on_ground or self.const_pos_in_air:
            flags |= EST_CONST_POS_MODE
        s = VehicleState(
            now=t,
            heartbeat_t=hb_t,
            armed=hb_armed,
            mode=hb_mode,
            landed_t=t,
            landed_state=LANDED_ON_GROUND if self.on_ground else LANDED_IN_AIR,
            pos_t=t,
            x=self.x,
            y=self.y,
            z=self.z,
            att_t=t,
            yaw=self.yaw,
            est_t=t,
            est_flags=flags,
            range_t=t,
            range_m=max(0.02, self.alt) if self.range_stuck is None else self.range_stuck,
            sys_t=t,
            battery_v=self.battery_v,
            rc_ok=self.rc_ok,
        )
        if self.range_dead:
            s.range_t = self._frozen.range_t if self._frozen is not None else None
        self._frozen = s
        return s
