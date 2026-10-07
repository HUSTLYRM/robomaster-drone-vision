"""PX4 模式编码：数值对照 PX4 v1.16.1 px4_custom_mode.h。"""
from __future__ import annotations

from flight import px4_modes


def test_known_custom_mode_values() -> None:
    # OFFBOARD 心跳里的 custom_mode 是 6<<16，AUTO.LAND 是 4<<16 | 6<<24
    assert px4_modes.encode(px4_modes.OFFBOARD) == 393216
    assert px4_modes.encode(px4_modes.LAND) == 0x06040000
    assert px4_modes.decode(393216) == px4_modes.OFFBOARD
    assert px4_modes.decode(0x06040000) == px4_modes.LAND


def test_roundtrip_and_names() -> None:
    for mode in (px4_modes.OFFBOARD, px4_modes.LAND, px4_modes.HOLD, px4_modes.POSCTL):
        assert px4_modes.decode(px4_modes.encode(mode)) == mode
    assert px4_modes.name(px4_modes.LAND) == "AUTO.LAND"
    assert px4_modes.name(px4_modes.Px4Mode(9, 9)) == "UNKNOWN(9,9)"
