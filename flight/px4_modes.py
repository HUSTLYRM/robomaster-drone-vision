"""PX4 飞行模式编解码。

PX4 通过 HEARTBEAT.custom_mode 上报模式：main_mode 在 bit16-23，sub_mode 在 bit24-31。
数值对照 PX4 v1.16.1 `src/modules/commander/px4_custom_mode.h`。
切模式走 MAV_CMD_DO_SET_MODE：param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED，
param2 = main_mode，param3 = sub_mode。
"""
from __future__ import annotations

from typing import NamedTuple

# PX4_CUSTOM_MAIN_MODE
MAIN_MANUAL = 1
MAIN_ALTCTL = 2
MAIN_POSCTL = 3
MAIN_AUTO = 4
MAIN_ACRO = 5
MAIN_OFFBOARD = 6
MAIN_STABILIZED = 7

# PX4_CUSTOM_SUB_MODE_AUTO
SUB_AUTO_READY = 1
SUB_AUTO_TAKEOFF = 2
SUB_AUTO_LOITER = 3
SUB_AUTO_MISSION = 4
SUB_AUTO_RTL = 5
SUB_AUTO_LAND = 6


class Px4Mode(NamedTuple):
    main: int
    sub: int


OFFBOARD = Px4Mode(MAIN_OFFBOARD, 0)
LAND = Px4Mode(MAIN_AUTO, SUB_AUTO_LAND)
HOLD = Px4Mode(MAIN_AUTO, SUB_AUTO_LOITER)
POSCTL = Px4Mode(MAIN_POSCTL, 0)

_NAMES = {
    (MAIN_MANUAL, 0): "MANUAL",
    (MAIN_ALTCTL, 0): "ALTCTL",
    (MAIN_POSCTL, 0): "POSCTL",
    (MAIN_ACRO, 0): "ACRO",
    (MAIN_OFFBOARD, 0): "OFFBOARD",
    (MAIN_STABILIZED, 0): "STABILIZED",
    (MAIN_AUTO, SUB_AUTO_READY): "AUTO.READY",
    (MAIN_AUTO, SUB_AUTO_TAKEOFF): "AUTO.TAKEOFF",
    (MAIN_AUTO, SUB_AUTO_LOITER): "AUTO.LOITER",
    (MAIN_AUTO, SUB_AUTO_MISSION): "AUTO.MISSION",
    (MAIN_AUTO, SUB_AUTO_RTL): "AUTO.RTL",
    (MAIN_AUTO, SUB_AUTO_LAND): "AUTO.LAND",
}


def decode(custom_mode: int) -> Px4Mode:
    return Px4Mode((custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF)


def encode(mode: Px4Mode) -> int:
    return (mode.main << 16) | (mode.sub << 24)


def name(mode: Px4Mode) -> str:
    # 非 AUTO 模式的 sub 恒为 0，未知组合原样输出数字便于排查
    return _NAMES.get((mode.main, mode.sub), f"UNKNOWN({mode.main},{mode.sub})")
