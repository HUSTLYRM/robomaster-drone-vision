"""运行期间锁定 CPU 最低频率，退出时恢复原值；不改调频策略，不写任何持久配置。

鲁班猫 interactive 调频在等帧的空闲里会降到 408～816 MHz，识别从 7.6 ms 变成 15 ms
（2026-10-23 实测，帧尾到识别完成中位 26.3 → 17.2 ms）。只把 scaling_min_freq 抬到
1800 MHz 即可，程序退出后系统回到原来的动态调频。

sysfs 写入需要 root：先直接写，没权限再走 `sudo -n tee`（板上用户 cat 有免密 sudo）。
SIGKILL / 断电时来不及恢复，最低频率会停在锁定值直到重启（sysfs 不持久）或手动写回。
"""
from __future__ import annotations

import subprocess
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Iterator

CPUFREQ_DIR = Path("/sys/devices/system/cpu/cpufreq")


def write_sysfs(path: Path, value: int) -> None:
    try:
        path.write_text(str(value))
    except PermissionError:
        subprocess.run(["sudo", "-n", "tee", str(path)], input=str(value), text=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=True, timeout=5)


@contextmanager
def min_freq_locked(
    freq_khz: int,
    warn: Callable[[str], None],
    base: Path = CPUFREQ_DIR,
    write: Callable[[Path, int], None] = write_sysfs,
) -> Iterator[list[tuple[Path, int, int]]]:
    """在 with 块内把每个 cpufreq policy 的最低频率抬到 freq_khz（不超过该 policy 的最高频率）。

    产出实际改动的 (文件, 原值, 新值) 列表；freq_khz <= 0 或改不了时产出空列表并照常运行，
    不因锁频失败中断识别。
    """
    changed: list[tuple[Path, int, int]] = []
    if freq_khz > 0:
        try:
            for policy in sorted(base.glob("policy*")):
                min_path = policy / "scaling_min_freq"
                old = int(min_path.read_text())
                new = min(freq_khz, int((policy / "scaling_max_freq").read_text()))
                cpuinfo_min = int((policy / "cpuinfo_min_freq").read_text())
                if old == new and old != cpuinfo_min:
                    warn(f"{min_path} 启动前已是 {old} kHz（硬件最低 {cpuinfo_min}），可能是上次异常退出遗留；"
                         f"退出时只能恢复成 {old}，需要时手动写回")
                write(min_path, new)
                changed.append((min_path, old, new))
        except (OSError, ValueError, subprocess.SubprocessError) as e:
            warn(f"锁定 CPU 最低频率失败，按系统调频运行：{e}")
            _restore(changed, warn, write)
            changed = []
    try:
        yield changed
    finally:
        _restore(changed, warn, write)


def _restore(
    changed: list[tuple[Path, int, int]], warn: Callable[[str], None], write: Callable[[Path, int], None]
) -> None:
    for path, old, _ in reversed(changed):
        try:
            write(path, old)
        except (OSError, subprocess.SubprocessError) as e:
            warn(f"恢复 {path} = {old} 失败：{e}")
