"""飞控状态快照：由 fc_link 的接收线程填充，任务状态机只读。

所有时间戳都是本机 time.monotonic() 秒（收到消息的时刻），None 表示从未收到。
坐标是 PX4 的 LOCAL_NED（北-东-地），z 向下为正，高度 = -z。
"""
from __future__ import annotations

from dataclasses import dataclass

from .px4_modes import Px4Mode

# MAV_LANDED_STATE，与 pymavlink 常量数值一致，这里写死以便纯逻辑模块不依赖 pymavlink
LANDED_UNDEFINED = 0
LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2
LANDED_TAKEOFF = 3
LANDED_LANDING = 4

# ESTIMATOR_STATUS_FLAGS（MAVLink common.xml）
EST_ATTITUDE = 1
EST_VELOCITY_HORIZ = 2
EST_VELOCITY_VERT = 4
EST_POS_HORIZ_REL = 8
EST_POS_HORIZ_ABS = 16
EST_POS_VERT_ABS = 32
EST_POS_VERT_AGL = 64
EST_CONST_POS_MODE = 128
EST_ACCEL_ERROR = 2048


@dataclass
class VehicleState:
    now: float = 0.0

    heartbeat_t: float | None = None
    armed: bool = False
    mode: Px4Mode = Px4Mode(0, 0)

    landed_t: float | None = None
    landed_state: int = LANDED_UNDEFINED

    pos_t: float | None = None
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    att_t: float | None = None
    yaw: float = 0.0

    est_t: float | None = None
    est_flags: int = 0

    range_t: float | None = None
    range_m: float = 0.0

    sys_t: float | None = None
    battery_v: float | None = None
    # SYS_STATUS 里 RC 接收机的 present & enabled & health 三位同时置位
    rc_ok: bool = False
