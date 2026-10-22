"""vision/cpu_freq.py：运行期间锁定 CPU 最低频率、退出时恢复。用临时目录模拟 sysfs。"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision"))
import cpu_freq  # noqa: E402
from cpu_freq import min_freq_locked, write_sysfs  # noqa: E402


def make_policy(base: Path, name: str, cur_min: int, max_: int, hw_min: int = 408000) -> Path:
    p = base / name
    p.mkdir(parents=True)
    (p / "scaling_min_freq").write_text(f"{cur_min}\n")
    (p / "scaling_max_freq").write_text(f"{max_}\n")
    (p / "cpuinfo_min_freq").write_text(f"{hw_min}\n")
    return p / "scaling_min_freq"


def read(path: Path) -> int:
    return int(path.read_text())


def plain_write(path: Path, value: int) -> None:
    path.write_text(str(value))


def test_locks_inside_and_restores_after(tmp_path: Path) -> None:
    a = make_policy(tmp_path, "policy0", 408000, 1800000)
    b = make_policy(tmp_path, "policy4", 600000, 1200000)  # 最高频率低于目标时取最高频率
    warns: list[str] = []
    with min_freq_locked(1800000, warns.append, base=tmp_path, write=plain_write) as changed:
        assert (read(a), read(b)) == (1800000, 1200000)
        assert changed == [(a, 408000, 1800000), (b, 600000, 1200000)]
    assert (read(a), read(b)) == (408000, 600000)
    assert warns == []


def test_restores_when_body_raises(tmp_path: Path) -> None:
    a = make_policy(tmp_path, "policy0", 408000, 1800000)
    with pytest.raises(KeyboardInterrupt):
        with min_freq_locked(1800000, print, base=tmp_path, write=plain_write):
            raise KeyboardInterrupt
    assert read(a) == 408000


def test_zero_means_untouched(tmp_path: Path) -> None:
    a = make_policy(tmp_path, "policy0", 408000, 1800000)
    calls: list[tuple[Path, int]] = []
    with min_freq_locked(0, print, base=tmp_path, write=lambda p, v: calls.append((p, v))) as changed:
        assert changed == []
    assert calls == [] and read(a) == 408000


def test_failure_midway_rolls_back_and_keeps_running(tmp_path: Path) -> None:
    a = make_policy(tmp_path, "policy0", 408000, 1800000)
    b = make_policy(tmp_path, "policy4", 408000, 1800000)

    def flaky(path: Path, value: int) -> None:
        if path == b and value == 1800000:
            raise subprocess.CalledProcessError(1, "sudo")
        plain_write(path, value)

    warns: list[str] = []
    with min_freq_locked(1800000, warns.append, base=tmp_path, write=flaky) as changed:
        assert changed == []
        assert read(a) == 408000  # 已改的那个立刻回滚
    assert len(warns) == 1 and "失败" in warns[0]


def test_warns_about_leftover_lock(tmp_path: Path) -> None:
    a = make_policy(tmp_path, "policy0", 1800000, 1800000)  # 上次被 kill -9，停在锁定值
    warns: list[str] = []
    with min_freq_locked(1800000, warns.append, base=tmp_path, write=plain_write):
        pass
    assert read(a) == 1800000
    assert len(warns) == 1 and "遗留" in warns[0]


def test_restore_failure_is_reported_not_raised(tmp_path: Path) -> None:
    a = make_policy(tmp_path, "policy0", 408000, 1800000)
    state = {"locked": False}

    def write_once(path: Path, value: int) -> None:
        if state["locked"]:
            raise OSError("设备忙")
        plain_write(path, value)
        state["locked"] = True

    warns: list[str] = []
    with min_freq_locked(1800000, warns.append, base=tmp_path, write=write_once):
        pass
    assert read(a) == 1800000
    assert len(warns) == 1 and "恢复" in warns[0]


@pytest.mark.skipif(os.geteuid() == 0, reason="root 写只读文件不会报 PermissionError")
def test_write_sysfs_falls_back_to_sudo(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    f = tmp_path / "scaling_min_freq"
    f.write_text("408000")
    f.chmod(0o444)
    calls = []
    monkeypatch.setattr(cpu_freq.subprocess, "run", lambda *a, **k: calls.append((a, k)))
    write_sysfs(f, 1800000)
    assert calls[0][0][0] == ["sudo", "-n", "tee", str(f)]
    assert calls[0][1]["input"] == "1800000" and calls[0][1]["check"] is True
