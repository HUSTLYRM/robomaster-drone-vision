"""参数编码与参数文件：整型按位模式重解释，仓库内的参数文件可被解析。"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

from flight.px4.apply_params import decode_param, encode_param, mavlink, parse_params_file

INT32 = mavlink.MAV_PARAM_TYPE_INT32
REAL32 = mavlink.MAV_PARAM_TYPE_REAL32


@pytest.mark.parametrize("v", [0, 1, 4, 101, -1, 2**31 - 1])
def test_int32_bitwise_roundtrip(v: int) -> None:
    assert decode_param(encode_param(v, INT32), INT32) == v


def test_int32_is_bit_pattern_not_value() -> None:
    # 整数 1 的位模式按 float 解读是非规格化数 1.4e-45，而不是 1.0
    assert math.isclose(encode_param(1, INT32), 1.401298464324817e-45)


def test_real32_passthrough_and_int_rejects_fraction() -> None:
    assert decode_param(encode_param(0.5, REAL32), REAL32) == 0.5
    with pytest.raises(ValueError):
        encode_param(1.5, INT32)


def test_repo_params_file_parses() -> None:
    params = dict(parse_params_file("flight/px4/companion.params"))
    assert params == {"COM_OF_LOSS_T": 1.0, "COM_OBL_RC_ACT": 4.0, "NAV_RCL_ACT": 3.0}


def test_bad_line_rejected(tmp_path: Path) -> None:
    f = tmp_path / "x.params"
    f.write_text("COM_OBL_RC_ACT\n", encoding="utf-8")
    with pytest.raises(ValueError):
        parse_params_file(str(f))
