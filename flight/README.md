# flight — 机载自主飞行程序

鲁班猫上运行的 Python 程序，经 MAVLink 让 PX4 完成「自检 → 起飞 → 悬停 → 航点 → 回起点 → 降落」。
PX4 固件本身不改；本程序是飞控的上位机，不需要烧录。

```
鲁班猫 UART8 /dev/ttyS8 (40pin 8/10/6 脚) ──921600──  飞控 U4 = TELEM3 = /dev/ttyS3
```

## 文件

| 路径 | 作用 |
| --- | --- |
| `run_mission.py` | 入口：连接、请求消息流、20 Hz 主循环、CSV 遥测（`flight_logs/`） |
| `mission.py` | 任务状态机与安全检查，纯逻辑无 IO |
| `fc_link.py` | pymavlink 收发；只采信飞控本体 compid 1 的消息 |
| `missions/*.json` | 任务配置，字段同 `MissionConfig`；航点 `[前, 右, 高]`，单位 m，相对起飞点和起飞时机头方向。`min_battery_v` 按 2S 设 7.0 V（带载 3.5 V/节，持续 2 s 才触发），不设则程序拒绝起飞 |
| `px4/extras.txt` | 放飞控 SD 卡 `/etc/extras.txt`，开机在 TELEM3 起 mavlink |
| `px4/companion.params` | 失效保护参数；`px4/apply_params.py` 写入并回读 |

## 安全行为（全部有 pytest 覆盖，`tests/test_mission.py`）

- 自检（估计器有效解、各消息新鲜、遥控在线、落地且未解锁、电压）连续通过 2 s 才起飞，30 s 不过则退出，不解锁。
  落地静止时 PX4 会置估计器 `CONST_POS_MODE` 位，这是正常现象，只在空中（起飞段除外）才算异常。
- 空中任一检查失败、越高度/半径边界、低电压持续 2 s 或电压读数丢失、起飞或航段超时 → 请求 AUTO.LAND。
- AUTO.LAND 请求 3 s 未生效 → 停发设定点交给 PX4：`COM_OF_LOSS_T`(1 s) 后按 `COM_OBL_RC_ACT=4` 直接切 AUTO.LAND。
  Offboard 丢失走 PX4 的模式回退，**不经过** `COM_FAIL_ACT_T` 的 Hold 延迟（SITL 实测：程序被 kill -9 后
  1.0 s 触发、1.5 s 进入 AUTO.LAND、11 s 落地上锁；v1.16.1 `failsafe.cpp` `checkModeFallback` 源码一致）。
- 模式被遥控或 PX4 切走、飞控意外上锁 → 下一次心跳（1 Hz）发现后立即停发一切指令（PREEMPTED）。
  模式/解锁请求获 ACCEPTED 后 2 s 内不重发，避免在心跳滞后窗口里把飞手刚拨走的模式覆盖回来。
- 心跳只有 1 Hz，解锁状态最多滞后 1 s：没起飞就退出（中止、超时、被接管）时，若发过解锁请求，
  会主动发上锁命令，并要等到晚于解锁请求的心跳确认已上锁才结束；5 s 确认不了会告警。
- 主循环出未捕获异常：若程序正掌握控制权，先连发 3 s AUTO.LAND 再退出。
- Ctrl-C / SIGTERM / **SIGHUP（SSH 断线）** 一次 = 中止（空中降落、地面上锁）；Ctrl-C 两次 = 立即退出。
  **真正的急停永远是遥控器 Kill 开关。**

## 验证状态（2026-10-07）

| 层级 | 状态 |
| --- | --- |
| 状态机逻辑（运动学替身，1 Hz 心跳滞后） | ✅ pytest，Python 3.8 / 3.10 均通过；25 处安全逻辑做过变异测试，逐一删掉后测试均变红 |
| MAVLink 收发（UDP 回环假飞控） | ✅ 端到端跑完整任务 |
| 独立审查 | ✅ 对照 PX4 v1.16.1 源码审查，必须修与建议项已修复（2026-10-07） |
| 真 PX4 SIH（PX4 v1.16.1 SITL，`sim/`） | ✅ 2026-10-07：自检、参数写入回读、悬停任务、航点任务（`sim/sih_square.json`）、kill -9 / SIGINT / SIGHUP 中止，全部按预期降落上锁 |
| 真 PX4 Gazebo x500_flow（光流+测距） | ⚠️ 能启动，地面静态检查通过；容器无 GPU，实时因子约 0.2，飞行任务未跑，见下 |
| 实物飞控 | ❌ 未做（飞控当时未连接） |

假飞控只编码了我们对 PX4 行为的理解，通过测试不代表与真 PX4 兼容。

SITL 实测得到、已据此修正的事实：
- 落地静止时估计器 flags=0x3ff，含 `CONST_POS_MODE`（修复前自检永远不过）。
- 测距高度参考必须和实机一致（`EKF2_HGT_REF=2`）。SIH 默认用 GPS 测高时，估计高度漂了 0.7 m，飞机在「悬停」中实际沉到了地面，
  因此新增「悬停/航段中测距持续 1 s 低于 0.1 m → 降落」。
- Offboard 丢失：1.0 s 触发，1.5 s 进入 AUTO.LAND，没有 Hold。
- PX4 参数延迟自动保存，写入后 2 s 内重启会丢失改动；`apply_params` 已提示至少等 5 s。
- SIH 用模拟 GPS 定位，水平误差达 0.7 m，0.15 m 的到点容差在 SIH 上达不到。SIH 专用任务放宽到 0.6 m；
  实机容差等光流首飞的悬停数据出来再定。

Gazebo（`sim/sitl.sh gz`）的情况：
- 起初一直卡在 "Waiting for Gazebo world"，根因是 `GZ_PARTITION`。容器以宿主机 uid 运行，`/etc/passwd` 里没有这个用户，
  gz-transport 推不出默认分区，PX4 和 gz 互相发现不了。显式设置分区后立即就绪。
  前面一度怀疑是宿主机 lo 没开组播，实测关掉组播照样能用，与组播无关。
- 只靠光流加测距停在地面时，估计器 flags=0x16f，含「水平相对位置有效」，地面自检判定为通过。
- 容器里没有 GPU，光流相机和 gpu_lidar 靠 CPU 软件渲染。关掉阴影（`sim/worlds/rm_noshadow.sdf`）后，
  实时因子从 0.013 提到约 0.2，但任务程序按墙钟计时，在这个速度下心跳和各种超时都会误判，所以飞行任务还没在光流仿真里跑过。
  要跑需要给容器接上 T500：装 nvidia-container-toolkit 并重启 docker，这会影响本机其他容器，需要人工确认。

已知未处理的风险：
- EKF 位置重置（光流开始融合、高度参考切换）时设定点不会跟着平移，飞机可能突然窜动。可用 ODOMETRY 的
  `reset_counter` 检测，待 SITL/实物看到实际现象再做。
- 消息频率请求的 ACK 只按命令号匹配，迟到的 ACK 可能张冠李戴；自检会再看消息新鲜度兜底。

## 部署到鲁班猫

```bash
# 本机：同步代码（板子上不需要 tests/）
rsync -av --exclude __pycache__ flight pyproject.toml lubancat:~/rm-drone/
# 板子：Python 3.8，pymavlink 钉 2.4.47（最后一个带 cp38 wheel 的版本）
ssh lubancat '~/vio_venv/bin/pip install "pymavlink==2.4.47" "loguru>=0.7"'
ssh lubancat 'groups | grep -q dialout || echo "需要 sudo usermod -aG dialout cat 后重新登录"'
```

在板子 `~/rm-drone` 目录下、**tmux 会话里**运行：`~/vio_venv/bin/python3 -m flight.run_mission ...`
（SSH 断线会发 SIGHUP，程序按中止处理：空中会降落。用 tmux 才能在断线后继续跑完任务。）
日志与 CSV 遥测写在 `flight_logs/`。

## 装机检查清单

按顺序执行，**上桨之前每一项都要拆桨完成**。标「待核实」的是没有在本机或手册里确认过的内容。

### 一、机体（QGC，飞控 USB 接笔记本）

1. **机架**：Airframe 选四旋翼 X 型。Actuators 页逐个电机测试，对照 PX4 Quad X 图核对**电机顺序和转向**。
2. **电调协议**：在 Actuators 页设置（DShot 或 PWM；用 PWM 时要做电调行程校准）。具体型号等电调到货后确认。
3. **传感器校准**：加速度计、陀螺、Level Horizon。DM-FC01 规格没有列磁力计。如果 QGC 报磁力计缺失，设 `SYS_HAS_MAG=0`（v1.16.1 源码确认存在该参数）。
4. **安装位置参数**：以飞控 IMU 为原点量取后填写 `EKF2_OF_POS_X/Y/Z`、`EKF2_RNG_POS_X/Y/Z`，以及光流朝向 `SENS_FLOW_ROT`。
5. **电池**：Power 页设置电芯数与分压，核对 QGC 显示的电压与万用表一致。任务文件里设 `min_battery_v`。

### 二、遥控与急停（规则强制）

6. 对频接收机后，`COM_RC_IN_MODE` 改回 `0`（当前为 4，摇杆输入全部禁用，遥控丢失保护也随之失效）。
7. 遥控校准；设 Kill 开关（`RC_MAP_KILL_SW`），并把一个模式开关映射到 **Position** 模式，作为飞手的接管手段（程序检测到模式被切走会立即停手）。
8. 拆桨实测：解锁后拨 Kill 开关，电机立即停转；关闭遥控器后，飞控进入遥控丢失失效保护。

### 三、伴随计算机链路

9. 接线三根：飞控 U4 口 TX→鲁班猫 10 脚 RXD，RX→8 脚 TXD，GND→6 脚。**不接 5V。**
10. 把 `px4/extras.txt` 拷到飞控 SD 卡 `/etc/extras.txt` 后重启飞控。在 QGC MAVLink Console 执行 `mavlink status`，应能看到 `/dev/ttyS3` 实例。
    若出厂固件已在 ttyS3 上跑着 mavlink，第二次 start 会失败，此时先 `mavlink stop -d /dev/ttyS3`（**待上板确认**）。
11. 写参数：`uv run python -m flight.px4.apply_params flight/px4/companion.params`，所有项都应显示 ✓ 或 =。
12. 板子上运行 `python3 -m flight.run_mission --check-only`：消息流齐全、估计器 flags 正常、测距随高度变化，预期显示「自检通过」。
    921600 波特率在 RK3566 上是否稳定**待实测**。若不稳定，`extras.txt` 和 `--baud` 一起改成 460800 或 115200。

### 四、拆桨全流程

13. 拆桨运行 `run_mission --mission missions/hover_test.json`。
    没桨飞不起来，PX4 会在约 10 s（`COM_DISARM_PRFLT`）后因未起飞自动上锁。所以预期流程是：解锁 → TAKEOFF →
    程序报「空中阶段飞控意外上锁」进入 PREEMPTED，退出码 1。这是拆桨时的**正常结果**。
    这一步验证的是 Offboard 切换和解锁链路，CSV 里可以看到每个阶段。中途按一次 Ctrl-C，可以顺便验证中止流程。

### 五、上桨首飞

14. **先手飞**：Position 模式手动悬停，看是否稳定、振动是否正常（拿 ulog 看）。小机架大概率要调 PID，不稳就不要进下一步。
15. 运行 `hover_test.json`，飞手手放在模式开关和 Kill 开关上。
16. 悬停稳定后再运行 `square_1m.json`。
