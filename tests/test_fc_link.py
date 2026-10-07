"""FcLink + run_mission 端到端：经真实 MAVLink 编解码（UDP 回环）驱动假飞控跑完整任务。"""
from __future__ import annotations

import csv
import json
import socket
import time
from pathlib import Path
from typing import Iterator

import pytest

from flight import px4_modes, run_mission
from flight.fc_link import MESSAGE_RATES_HZ, FcLink
from tests.fake_px4 import FakePx4


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def fake_and_port() -> Iterator[tuple[FakePx4, int]]:
    port = _free_port()
    fake = FakePx4(port)
    fake.start()
    yield fake, port
    fake.stop()


def test_link_requests_rates_and_filters_sources(fake_and_port: tuple[FakePx4, int]) -> None:
    fake, port = fake_and_port
    link = FcLink(f"udpin:127.0.0.1:{port}")
    try:
        link.connect(timeout_s=5.0)
        assert link.target_system == 1
        assert link.request_message_rates() == []
        assert set(fake.rates) == set(MESSAGE_RATES_HZ)
        time.sleep(1.0)
        s = link.snapshot()
        assert s.mode == px4_modes.POSCTL  # 地面站心跳（mode=0）不能覆盖飞控模式
        assert s.range_m == pytest.approx(0.02)  # 光流模块 compid 88 的 5.55 m 被忽略
        assert s.rc_ok and s.battery_v == pytest.approx(16.0)
        assert s.est_flags == fake.sim.est_flags
        for t in (s.heartbeat_t, s.pos_t, s.att_t, s.est_t, s.range_t, s.landed_t, s.sys_t):
            assert t is not None and s.now - t < 1.0
    finally:
        link.close()


def test_full_mission_over_mavlink(fake_and_port: tuple[FakePx4, int], tmp_path: Path) -> None:
    fake, port = fake_and_port
    mission_file = tmp_path / "m.json"
    mission_file.write_text(
        json.dumps({
            "takeoff_alt": 0.5, "hover_s": 1.0, "waypoints": [[0.5, 0.0, 0.5]], "wp_hold_s": 0.3,
            "climb_rate": 1.0, "xy_speed": 1.0, "settle_s": 0.3, "ready_hold_s": 0.5,
            "min_battery_v": 14.0,
        }),
        encoding="utf-8",
    )
    log_dir = tmp_path / "logs"
    rc = run_mission.main(
        ["--url", f"udpin:127.0.0.1:{port}", "--mission", str(mission_file), "--log-dir", str(log_dir)]
    )
    assert rc == 0
    assert not fake.sim.armed and fake.sim.alt == 0.0
    assert 0.4 < fake.max_alt < 0.65
    assert fake.modes_seen[-2:] == ["OFFBOARD", "AUTO.LAND"]

    (csv_path,) = log_dir.glob("mission_*.csv")
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    phases = [r["phase"] for r in rows]
    for p in ("WAIT_READY", "PRESTREAM", "OFFBOARD", "ARM", "TAKEOFF", "HOVER", "GOTO", "LAND", "DONE"):
        assert p in phases
    assert max(float(r["x"]) for r in rows) > 0.4  # 确实飞到了 0.5 m 前方的航点


def test_mission_refuses_without_rc(fake_and_port: tuple[FakePx4, int], tmp_path: Path) -> None:
    fake, port = fake_and_port
    fake.sim.rc_ok = False
    mission_file = tmp_path / "m.json"
    mission_file.write_text(json.dumps({"ready_timeout_s": 2.0, "min_battery_v": 14.0}), encoding="utf-8")
    rc = run_mission.main(
        ["--url", f"udpin:127.0.0.1:{port}", "--mission", str(mission_file), "--log-dir", str(tmp_path)]
    )
    assert rc == 1
    assert fake.modes_seen == ["POSCTL"] and not fake.sim.armed


def test_refuses_mission_without_battery_threshold(tmp_path: Path) -> None:
    # 不设 min_battery_v 时连飞控都不连，直接拒绝（PX4 默认低电只告警不动作）
    rc = run_mission.main(["--url", "udpin:127.0.0.1:1", "--log-dir", str(tmp_path)])
    assert rc == 2


def test_mission_file_type_errors_rejected(tmp_path: Path) -> None:
    f = tmp_path / "m.json"
    f.write_text(json.dumps({"waypoints": [[1.0, 0.0]]}), encoding="utf-8")
    with pytest.raises(ValueError):
        run_mission.load_config(str(f))
    f.write_text(json.dumps({"hover_s": "12"}), encoding="utf-8")
    with pytest.raises(ValueError):
        run_mission.Mission(run_mission.load_config(str(f)))
