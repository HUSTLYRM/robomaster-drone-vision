# AGENTS.md — 竞赛无人机视觉项目必读

更新日期：2026-10-22

本文是接手本项目的 agent **必读入口**，只记「不看就会做错」的事实与结论。
详细数据见 `docs/`，两处不重复维护：

- `docs/项目计划书.md` — 目标、选型基线、阶段验收
- `docs/硬件接线与配置手册.md` — 实测确认的接线、引脚、参数（硬件事实 SSOT）
- `docs/新机QGC配置指南.md` — 新机从刷固件到可以手飞的 QGC 操作步骤（引用手册事实，不另存硬件数据）
- `docs/PX4参数中文手册.md` — MicoAir743 上改过和常调的 PX4 参数中文说明（含义译自官方元数据，原因引用硬件手册）
- `docs/飞控电调一体板需求.md` — 自研「飞控 + 电调 + V2H 核心板载板」一体板的需求、核心板接口事实与待确认项
- `docs/飞控电调一体板设计.md` — 一体板的实现：调研、引脚、选型、决策记录 D1~D33、PCB 规则；进度见 `hardware/fc-esc/晨报.md`

结论标注来源：**实测**（本会话或历史验证过）、**手册**（厂家文档）、**待确认**（尚无依据）。

---

## 1. 连接机载计算板

```bash
ssh lubancat          # 主通道：网线直连（推荐）
ssh lubancat-usb      # 备用通道：USB OTG，网线不通时救场
```

**实测 2026-09-21**，两条通道并存，互不依赖：

| 通道 | 本机 | 板子 | 说明 |
| --- | --- | --- | --- |
| **网线（主）** | `enp0s31f6` = `192.168.50.1/24` | `eth0` = `192.168.50.2/24` | 千兆全双工，静态 IP，**地址永不变** |
| OTG（备） | `enx*` = `10.42.0.1/24` | `usb0` = `10.42.0.50` + DHCP | 提供板子外网；DHCP 地址每次漂移，**只用 .50** |

两端都用 NetworkManager 配置且 `autoconnect yes`，**重启持久化已实测**：板子重启后约
26 秒网线通道自动恢复。已实测断开 OTG 后网线可独立工作。

⚠️ **板子的外网依赖 OTG**。`eth0` 配了 `ipv4.never-default`，不提供默认路由；笔记本也没
为 `192.168.50.0/24` 做 NAT。只插网线时板子能 SSH 但**上不了外网**（apt 装包会失败），
需要外网时插上 OTG 即可。

⚠️ **不要用 `ip link set <iface> down` 停接口**。这样 NetworkManager 会把连接标记为
disconnected，之后单纯 `ip link set up` **不会**恢复 IP 配置，必须
`sudo nmcli con up "<连接名>"` 或 `sudo nmcli dev connect <iface>`。本会话踩过。

板子上的连接名：`Wired connection 1` = eth0，`Wired connection 2` = usb0。
登录用户 `cat`，密钥 `~/.ssh/id_ed25519_lubancat`。

> 历史遗留：板子 `/etc/NetworkManager/dispatcher.d/50-usb0-fixed-ip` 给 usb0 附加固定地址
> `10.42.0.50`（因为 RNDIS 的 MAC 每次上电随机，DHCP 地址会漂移）。该脚本只管 usb0，
> 不影响 eth0。

---

## 2. 摄像头：已接通，但选型不匹配目标

### 2.1 当前硬件状态（实测 2026-09-21）

在手的是**野火官方配套 ov8858 模组**，已完整验证可用：

| 项 | 值 |
| --- | --- |
| 驱动识别 | `Detected OV008858 sensor, REVISION 0xb2` |
| 分辨率 / 帧率 | 1632×1224 @ **29.87 fps**（实测） |
| I2C | sensor `0x36`，对焦马达 dw9714 `0x0c`（均在 i2c-1） |
| MIPI | 2 lane，720 Mbps/lane，pixel_rate 144 MHz |
| 设备节点 | `/dev/video0`（rkisp_mainpath），sensor subdev `/dev/v4l-subdev3` |
| 启用方式 | `/boot/uEnv/uEnv.txt` 第 45 行 overlay，**改完必须重启** |

```
dtoverlay=/dtb/overlay/rk356x-lubancat-csi0-ov8858-1632x1224-30fps-overlay.dtbo
```

⚠️ **这一行不要注释掉**，注释了摄像头就认不到。原文件备份在
`/boot/uEnv/uEnv.txt.bak-*`（板子上）。

### 2.2 用途已变更：不做 VIO，做飞镖拦截

**这是最容易搞错的一条。** VIO 定位由 **MTF-02P 光流 + 飞控** 承担，相机不参与。
相机的任务是**识别并跟踪飞镖上的灯，用于跟踪拦截**。

因此：**白平衡、色差一类的问题对本项目无意义，不要在上面投入时间。**
（曾实测出白纸场景 R/G=0.701 的偏色，但那个测法本身有缺陷——单一颜色铺满画面是 AWB
的病态输入，数据不可直接采信；且识别自发光灯点靠的是亮度阈值，与白平衡无关。）

### 2.3 ov8858 为何不胜任（实测，三条硬伤）

| 硬伤 | 实测依据 | 能否修 |
| --- | --- | --- |
| **卷帘快门** | ov8858 规格 | ❌ 物理特性，高速目标必然斜切变形 |
| **30 fps 封顶** | `vertical_blanking` 已压到最小 26，`horizontal_blanking` 616 只读 | ❌ 当前 sensor 模式无余量 |
| **曝光被锁 10 ms** | iqfile 里 `AecAntiFlicker` = 50Hz 开启 | ⭕ 改 iqfile 可解，但解了也没用 |

补充两条容易误判的：

- **降分辨率不提高帧率**。实测设 640×480 仍是 29.9 fps——那是 ISP 在缩放，sensor 端照样
  按 1632×1224 @ 30fps 读出，缩放只省带宽不省时间。
- **MIPI 链路带宽只用了不到一半**（2 lane × 720 Mbps ≈ 72 fps 理论上限），卡住帧率的是
  驱动里写死的 sensor 时序表，不是物理传输能力。要突破得改内核驱动 + OmniVision 寄存器表
  （NDA），且驱动是 `CONFIG_VIDEO_OV8858=y` 编译进内核的，改完要重编刷机。不划算。

---

## 3. 候选替代方案（尚未决策）

目标需求：**全局快门**（高速目标不变形）+ **高帧率**（≥120fps，越高越好）+ **短曝光**。
分辨率不重要——飞镖的灯在画面里就是个亮点，640×480 足够定位。

| 方案 | 板子支持度 | 关键参数 | 阻塞项 |
| --- | --- | --- | --- |
| **A. SC132GS** MIPI 模组 | **最高**：内核驱动 ✓ overlay ✓ dts 节点 ✓ | 1280×1080 @120fps，1/4"，5~10 g | **缺 iqfile**；dts 只配 1 lane（约限 50fps） |
| **B. OV9281** MIPI 模组 | 内核**无**驱动 | 640×480 **@180fps**，1280×800@120fps，单色 | 需移植驱动 + 重编内核 |
| **C. OV7251** | 内核驱动 ✓，无 overlay / dts 节点 | 640×480 @~100fps | 帧率优势不明显，不优先 |
| **D. USB3 工业相机** | 板子**有** USB3 控制器 | 可达 200fps+ | Type-C 的 USB3 差分线是否物理引出**待确认** |

### 关键实测细节

**方案 A（SC132GS）**：dtbo 已存在（`rk356x-lubancat-csi0-sc132gs-overlay.dtbo`），但
`uEnv.txt` 的摄像头段落里**没有列出这一行**，需手动添加。设备树节点在主 dtb 里已定义
（`smartsens,sc132gs`，I2C `0x30`，`data-lanes = <0x01>`，status=disabled），走
`csi2-dphy0/port@0/endpoint@5`——**与 ov8858 共用同一 dphy，同样走 ISP 通路**，因此同样
依赖 3A server 和 iqfile，而 `/etc/iqfiles/` 里没有 sc132gs 的标定文件。

**方案 D（USB3）**：`lsusb -t` 显示 `Bus 06: xhci-hcd, 5000M`，控制器 `fcc00000.dwc3`
当前是 device 模式（就是跑我们这条 RNDIS 的）。带宽账：640×480 mono 8bit @200fps ≈
492 Mbps，USB2（实际~320Mbps）不够，**USB3（实际~3.2Gbps）绰绰有余**。切 host 模式后
SSH 会断，替代管理通道有两条：`eth0`（RTL8211F 千兆 PHY 已识别，未接线验证）和已启用的
UART8 串口。板子**无 WiFi**。

---

## 4. 板子的硬约束（踩过的坑）

1. **ISP 离不开 `rkaiq_3A_server`**。实测停掉该服务后，抓帧永久阻塞，dmesg 报
   `rkisp-vir0: waiting on params stream on event timeout`，输出文件 0 字节。
   RK 的 ISP 必须由 3A server 持续通过 `/dev/video6` 下发参数才能出流——**这与普通 UVC
   相机的习惯完全不同，不要试图"关掉自动曝光"**。
   恢复：`sudo systemctl start rkaiq_3A`（unit 名 `rkaiq_3A`，由 `/etc/init.d/rkaiq_3A.sh` 生成）。

2. **`pkill -f <模式>` 会匹配到执行它的 bash 命令行自身**，把自己杀掉（退出码 144），
   后续命令静默不执行。本会话踩了三次。
   注意 `[p]review.py` 这类方括号写法**只在命令行里不含目标字面串时才有效**——若同一条命令
   里还写了 `tools/preview.py`（比如"先停再启动"），照样自杀。
   **最稳的做法**：`kill $(pgrep -f "...")` 先取 PID，或把停止和启动拆成两条独立命令。

3. **`cd dir && nohup cmd &` 里 `$!` 不是 `cmd` 的 PID**。bash 后台化一个 `&&` 复合命令列表时
   会额外 fork 一层子 shell 去执行整条列表，`$!` 拿到的是那层子 shell 的 PID，真正的目标进程
   PID 要另外 `pgrep -f` 才能拿到；对着 `$!` 直接 `kill` 只会杀掉 wrapper，目标进程（连同它占
   着的摄像头设备、监听端口）原样健在。本会话在起 `vision/dart_tracker_main.py` 时踩过。
   **最稳的做法**：起后台任务写成单条命令（不要 `cd &&` 前缀，用绝对路径代替 `cd`），
   `nohup /path/to/python3 /path/to/script.py ... > log 2>&1 < /dev/null & echo $!`，
   这样 `$!` 才是真正的目标 PID。

4. **编译 ROS 工作空间必须限并行**：`colcon build --parallel-workers 4`（板子 3.8GB 内存）。

5. 其余（供电、40pin 短路风险、ROS 版本不匹配）见 `docs/硬件接线与配置手册.md` 第 4.4 节。

---

## 5. 待确认事项（阻塞选型决策）

| # | 待确认 | 找谁 / 怎么确认 | 阻塞什么 |
| --- | --- | --- | --- |
| 1 | 野火是否提供 SC132GS 配套模组和 **iqfile** | 问野火官方 / 社区 | 方案 A 能否成立 |
| 2 | 鲁班猫 Zero 的 Type-C **USB3 差分线是否引出** | 野火原理图，看 SSTX/SSRX 是否连到 SoC | 方案 D 能否成立 |
| 3 | 飞镖**实际飞行速度** | 队里实测或规则上限 | 反推所需 fps |
| 4 | 计划在**多远距离**开始跟踪 | 战术方案 | 反推视场角与镜头焦距 |
| 5 | `eth0` 有无**物理 RJ45 座** | 看板子 / 原理图 | 切 USB host 后的管理通道 |

⚠️ 此前所有「帧间位移」的估算都基于**飞镖 25 m/s 的假设**，该数字无依据，仅用于说明量级。
拿到第 3 项真实数据前，不要据此做选型定论。

---

## 5.5 机载自主飞行程序 `flight/`

鲁班猫上的 pymavlink 程序，让 PX4 走 Offboard 完成自检、起飞、悬停、航点和降落。用法、安全行为和**装机首飞检查清单**
见 `flight/README.md`，任务进度见计划书 7.1。必须知道的几点：

- ⚠️ **2026-10-11 飞控已由 DM-FC01 换成 MicoAir743 V2，10-12 刷入上游 PX4 v1.16.1**。**最终选型是微空飞控电调一体板（AIO，推测 MicoAir743v2-AIO-35A），固件目标 `micoair_h743-v2` 是对的**，资料在 `MicoAir743-AIO/`。串口映射、校准和待重做的配置见硬件手册 2.0 节。下面这条链路和 `vehicle.params` 都是按 DM-FC01 写的，**不能直接套用**。
- 链路为鲁班猫 `/dev/ttyS8` ↔ 飞控 U4/TELEM3 `/dev/ttyS3`，波特率 921600（**待实测**），飞控侧靠 SD 卡 `extras.txt` 启动 mavlink。
- 代码须兼容 **Python 3.8**（板子），pymavlink 钉死 2.4.47；本机测试用 `uv run pytest`。
- 已在 PX4 v1.16.1 SIH SITL 上跑通，包括故障注入。Gazebo 光流仿真能启动，但容器里没有 GPU，只能做静态检查；实物还没跑。SITL 用法见 `sim/sitl.sh`，PX4 源码放在仓库同级的 `../PX4-Autopilot`。
- 容器里跑 gz 必须设 `GZ_PARTITION`：容器内的 uid 没有对应用户名，gz 推不出默认分区，会一直卡在等待 world。
- 默认要求遥控在线、任务文件设了 `min_battery_v` 才起飞；`--no-rc` 只用于拆桨桌面测试或 SITL。
- PX4 心跳固定 1 Hz，解锁状态/模式最多滞后 1 s——改状态机时别用「心跳显示未解锁」直接判定已上锁。
- 板子上要在 tmux 里运行：SSH 断线 = SIGHUP = 程序中止（空中降落）。

---

## 5.6 数字孪生仿真 `sim/`（2026-10-12）

目标是**仿真与实机一致**：同一份 PX4（带本地补丁）、同一份实机参数、同一条 MTF-02P MAVLink 传感器路径。
画面逼真不是目标。实机事故几乎都出在传感器/EKF 配置层。

```bash
uv run python tools/fetch_ulogs.py get                               # 从飞控拉 ulog 到 logs/（MAVLink FTP）
uv run sim/fit_twin.py fit logs/sess12[0-2]/*.ulg                    # 拟合 → sim/mtf02p_model.json、sim/twin.params
sim/twin.sh                                                          # 一键起孪生 + Rerun 3D 窗口（真值绿 / EKF 估计橙）
uv run python -m flight.run_mission --url udpin:0.0.0.0:14540 --no-rc --mission sim/sih_square.json
sim/calibrate.sh [覆盖.params]                                        # 单轮校准：飞一次并输出仿真/实机指标比值
sim/calibrate.sh --mount-rot 2 sim/faults/r69_flow_rot.params        # 复现历史故障（对照组 r69_fixed.params）
uv run sim/fit_twin.py flowcheck <日志>.ulg                           # 光流方向检查
uv run sim/twin_view.py ulog <实飞或仿真>.ulg ...                     # 回放
```

必须知道的：
- **SITL 必须带 `sim/px4_patches/0001-sih-rm-twin.patch`**：① SIH 振动噪声随推力缩放（`SIH_ACC_NOISE`/`SIH_GYR_NOISE`）；② 着地静摩擦（`SIH_GND_FRIC`，上游落地后残余转速会让飞机在地上一直转，第二次起飞偏航差 78°）。
  上游 SIH 一解锁就加满额 IMU 噪声，实测让 EKF 在地面以为在爬升、控制器不给油，仿真飞机离不了地。
  实机怠速几乎无振动（加速度 σ 0.03～0.07）。PX4 源码改动后要 `sim/sitl.sh build`。
- 仿真参数**走容器内 `px4-param`** 写，别用 MAVLink：SITL 的 UDP 链路会被最后一个发包方抢走，
  Rerun 窗口连着时 `apply_params` 收不到心跳。
- 每次试验从干净启动开始（`twin.sh` 会 reset）：一次失败的飞行会把 EKF 高度/测距一致性带坏，后面全不可信。
- PX4 v1.16.1 的 HIL_STATE_QUATERNION `time_usec` 恒为 0，仿真器改用同链路 HIL_ACTUATOR_CONTROLS 的时钟。
- 实机 MTF-02P 的 sysid 是 **1**（device_id 9699590 解码；10-16 起改接 TELEM2，设备号变为 9699598，`EKF2_RNG_DEVID` 已同步），光流相对飞控陀螺**滞后约 40 ms**（PX4 只按 `EKF2_OF_DELAY=20` 补偿）。
- 校准现状（第 2 轮，sess120～122 基准）：悬停推力、IMU 噪声、水平阻力、横滚角速度环的比值 0.9～1.16。
  偏航未校准（方形任务不激励），俯仰不作目标（实机俯仰环 R² 仅 0.31，陀螺 FFT 在 31～47 Hz 有峰，实机问题）。
  `SIH_MASS` 未称重，取 0.25 kg（只有比值影响结果）。
- 3D 里的机体外形在 `sim/drone_mesh.py`，尺寸是 2026-10-13 按照片（以电池为比例尺）估的，机头方向假设沿电池长边，**均未实测**；只影响画面，不进仿真。
- `twin.sh` 收尾：终端面板关标签页会杀掉整个进程树（trap、setsid 都来不及），容器靠 `systemd-run --user` 看门狗按 ID 清理；Rerun 查看器是独立进程，会记住上次的布局和镜头，所以代码里用 `send_blueprint(make_active=True)` 每次强制启用。
- 3D 影子 = EKF 估计：水平用估计位置，高度用飞控定高实际用的 `dist_bottom`（EKF 的 z 以开机零点为基准，比真实高度低约 0.13 m），PX4 判定落地时画在地面（贴地时 dist_bottom 被 `EKF2_MIN_RNG` 夹成 0.1 m）。
- **flowcheck 在故障工况下不可信**：EKF 速度已经被错的光流带偏，比较对象本身就错了。

## 5.7 飞控电调一体板 `hardware/fc-esc/`（2026-10-22）

嘉立创EDA 专业版工程「New Project_2026-10-22_01-33-02」，全部由 `hardware/fc-esc/design/` 下的脚本经 easyeda-api 桥接
（127.0.0.1:49620，单次请求 30 s 超时）生成。数据源是 `design/spec.py`，改原理图先改它。流程与每步作用见设计文档第 0 节。

必须知道的（都已实测，细节见设计文档 D18~D30）：
- **嘉立创EDA 的 DRC（API）在 8 层下不报间距**，以 `design/drc.py` 为准（shapely：`uv run --no-project --with shapely python3 drc.py --fresh`）。
- **填充图元存盘后丢内孔**；经 API 建的铺铜无法重建。平面用 `pour.py` 自算并切成无孔块写入，任何改动后要重跑。
- **经 API 放置的器件焊盘没有阻焊开窗**，靠 `mask.py` 补；**USB-C 外壳通孔脚丢失**，靠 `pcb.py usbc` 补（单位 bug）。
- 「从原理图导入变更」会卡在独立窗口的确认框：别点「应用」，器件已由脚本直接建在 PCB 上。
- 焊盘角度可能存成 89.99，判断 90° 一律用 `round()`；多层焊盘的 `getState_Pad()` 回读值不可信。
- **原理图换图框尺寸**：用 `lib_Device.search('Drawing-Symbol_A3')` 的结果传给 `sch_PrimitiveComponent.create(..., 0, 0)`，EDA 会替换该页旧图框（调用会超过 30 s，要后台跑再轮询）。**不要用 `modifySchematicPageTitleBlock` 改尺寸**：会把图框引用写成只有名字，原理图从此保存失败。
- 布线用 Freerouting 2.5.0 jar（`~/tools/freerouting/`，便携 JRE 25），**默认开遥测**，`route.py` 已加参数关掉；原生 CLI 遇平面会崩。

## 5.8 笔记本双目 VO `stereo/`（2026-10-22）

USB 双目相机接在笔记本上做纯视觉双目里程计（开发评估阶段，机载部署未定）。采集节点 `stereo/capture_node.py`，用法见其文件头。

已实测：
- 设备 `SunplusIT "USB Global Camera"`（0bda:9281，USB3 5 Gbps，UVC），本机 `/dev/video4`（插拔可能变）。**无 IMU**。**黑白传感器**（RGB 三通道完全相等），只取 YUYV 的 Y。
- **只有 `2560×720@60`、`1280×480@100`、`1280×400@120` 是左右拼接双目**；`1280×720`、`640×480` 等只出单目。`1280×480` 每目是 16:9 中间裁 4:3 再缩放，视场更窄。**每个模式要单独标定**。YUYV 各模式都能跑满标称帧率。
- 曝光/增益只有一组控制项，左右同步响应，同场景亮度差约 5%；`exposure_time_absolute` 单位未确认，改它不影响帧率。
- 节点 1280×480 实测 99.3 Hz，左右时间戳逐帧相同（来自同一拼接帧），端到端延迟约 10 ms，偶发丢 1 帧（约 0.1%～0.6%）。
- **rclpy 给 `Image.data` 赋 `bytes` 会逐元素做 Python 类型检查，实测从 100 掉到 19 fps**，必须赋 `array.array('B', ...)`。
- **两颗传感器经排线接到同一桥接芯片，装在丸子换的支架上**（2026-10-22 晚起基线约 85 mm）。拼接帧哪半是哪目**由排线接法决定**，与相机朝向无关；
  `capture_node` 参数 `swap_lr`（默认 `false`，对应当前接法）。**重新接线/动过支架必须重标**，`calibrate.py` 遇 Tx>0（左右反了）会报警。
  支架刚性直接决定外参，松动即失效。倒装时**不要在软件里旋转图像**（会再次互换左右、改变主点，标定作废），朝向写进相机→机体外参。
- 标定用 `stereo/calibrate.py`（自动采样，不用 ROS camera_calibration：其 GUI 在本机显示不全、按钮点不到）。样本存 `logs/stereo_calib/`，可 `--images` 离线重算；
  接反时可把样本 `_L`/`_R` 对调后离线重算，不必重采。
  **样本要远近拉开**：15 张、Size 覆盖 39% 时 RMS 照样很小，但 Tz 错出 5.7 mm、基线子集标准差 0.54 mm——小 RMS 是过拟合，不代表准。
  除单目误差外还**按双目逐张误差剔样本**：单目各自都好、两目对不上的样本（拿板时支架晃了）单目那轮剔不掉，实测一张 2.5 px 坏样本把双目 RMS 从 0.22 拉到 0.51。
  同时输出 `stereo.yml`（OpenCV FileStorage，给 C++ VO）。
- **标定是否还有效用 `python3 -m stereo.rect_check --cols 11 --rows 8 --square 0.03`**（直开相机，对着棋盘采 30 帧）：
  行差查旋转，三角化方格边长查基线/尺度；行差中位 > 0.5 px 或边长偏差 > 1% 即需重标。
  **两目水平夹角变化行差看不出来，只表现为固定视差偏移 → 尺度随距离变**：f 565 px、基线 86 mm 下夹角变 0.1° ≈ 1 px 视差，1 m 处尺度偏 2%、3 m 处偏 6%。
- **当前标定**（`stereo/calib/1280x480/`，2026-10-22 23:0x，丸子改**刚性安装**后重标，27 张，棋盘 11×8 内角点 / 30 mm 厂家专业标定板）：
  双目 RMS 0.17 px，校正后行差均值 0.16 px（最大 0.81），T = [−85.56, −3.12, −0.85] mm，**基线 85.61 mm**（子集标准差 0.08 mm），校正后 fx 558。
  标后 rect_check：行差 +0.18 px、1.15 m 处方格 +0.64% → 有效。（测前是否拨过排线未确认。）
  之前非刚性支架两次标定都在几分钟～半小时内失效（行差 −1.6 px / 尺度 ±2.3%），说明正常拿取就能让外参变 ~0.1°。
  作废的标定备份：24 mm 支架 `backup_baseline24mm_20261022_215659/`，85 mm 非刚性 `backup_85mm_invalid_20261022_224404/`、
  `backup_85mm_flexible_20261022_230333/`（均在 `logs/stereo_calib/`）。
- 单测 `tests/test_stereo_*.py` 需要 numpy/cv2/yaml，用系统 `python3 -m pytest` 跑；`uv run pytest` 下会跳过。
- **两目硬件同步曝光**（24 mm 支架上测）：`stereo/sync_check.py` 上下晃棋盘（240 个拟合样本，竖直速度中位 547 px/s）测得时差 −0.014 ms，
  若有 1 ms 时差会产生 0.55 px 行差、远超噪声，结论可信。同步由桥接芯片决定，换支架后大概率不变，但**85 mm 支架尚未复测**。

- **机载 VO 用 C++ 轻量版 `stereo/vo/`**（丸子定：要轻量、用 C++）：单进程单线程、无 ROS，只依赖 OpenCV（只用 4.2 起就有的接口，照顾板上 apt 版）。
  关键帧 + KLT + 只校正特征点 + 双目三角化 + PnP RANSAC/LM，不建图不做 BA；V4L2 直读、排空队列只处理最新帧。
  构建/测试：`cmake -S stereo/vo -B stereo/vo/build && cmake --build stereo/vo/build -j4 && stereo/vo/build/test_vo`；
  运行：`stereo/vo/build/stereo_vo [--csv x.csv] [--seconds N] [--exposure N]`，`--record DIR` 录 PGM 序列，`--replay DIR [--skip 3]` 离线回放。
  输出位姿为 FLU（前左上），原点 = 首个关键帧左目光心。MAVLink 输出与上板未做。
  - gtest 7 个（合成双目，变异测试确认能抓尺度错、位姿乘反、遮挡点不过滤、每帧换关键帧；另测 PnP 合理性检查）：最频繁换关键帧时最坏误差 0.79% 路程、0.10°。
  - **遮挡边界点**左右看到不同东西，前后向 KLT 检查拦不住但残差大，深度可错 33%：按左右 KLT 残差自适应剔除（> 4×中位数且 > 2）。
  - **换关键帧用相对阈值**（内点 < 建帧点数 × 0.6 或 < 30）：曾写死「< 80」，弱纹理场景每帧换关键帧，静止 3 s 漂 15 cm。
  - **合成测试纹素投影必须 > 1 px**：曾缩小渲染出混叠，把测试假象当成 VO 漂移（误差显示 12.8%）。
  - 真机（笔记本，2026-10-22，刚性安装后）：
    - 纹理丰富场景、固定曝光（`--exposure 50 --gain 96`）静止 30 s：无丢失、1 个关键帧、内点 96，位置标准差 0.18 mm、最大 0.74 mm，3.6 ms/帧。
      （首次静止测试丢 2 次是相机刚放下还在晃：录像相位相关显示整幅平移 3～50 px。判断 VO 前先确认真静止。）
      弱纹理（天花板/白墙）静止时内点约 35、横向游走 1～5 cm，属场景问题。
    - **手持闭环 222 s / 14 m：丢失 24 次（20% 帧无位姿），首尾 0.57 m，不合格**。多数丢失在慢速（12～16°/s）：
      跟踪点衰减、该换关键帧时凑不够 40 个双目点换不成；156～170 s 角速度 190～660°/s（5 ms 曝光下拖影约 24 px，超出 KLT 范围），
      纯视觉无解，要靠飞控陀螺预测。双目约一半点因右目看不到（85 mm 基线左缘无重叠、右目前有遮挡物）被前后向检查剔掉；
      左右亮度对齐实测无效（未加）。已改：特征 200→300（静止内点 98→122）、建关键帧下限 40→30。
    - **慢速闭环数据集 `logs/stereo_seq/loop1`**（66 s、约 6.3 m、6481 对 PGM、3.8 GB，固定曝光；离线回放 `--replay logs/stereo_seq/loop1 --skip 3`）：
      默认参数、33 fps 处理：有效 99.3%，首尾 7.1 cm（**1.13% 路程**），2.4 ms/帧，唯一一次丢失是手指挡住左镜头（50.2 s）。
      回放发现 **PnP 错解**：RANSAC 只看重投影，收敛到部分内点在相机背后的解（121 内点、跳 9.4 m / 82°，下一帧跳回，路程被算成 30.8 m）。
      已加合理性检查（内点深度全正 + 相邻有效位姿平移 ≤ 0.5 m、转角 ≤ 45°），不合格帧丢弃不重置，连续 5 帧才算丢失；加后无 > 5 cm 单帧跳变。
      200/40 与 300/30 两组参数在此数据集上首尾 0.7%～1.7% 互有胜负，单个数据集分不出优劣。
    - 相机侧丢帧约 17%（VO 才 3.6 ms/帧，原因未查）。
- 笔记本对照基线（不上机）：`ros2 launch stereo/launch/vo.launch.py`（采集 → image_proc 校正 → RTAB-Map `stereo_odometry`，只做 VO 不起 SLAM），
  质量统计 `python3 -m stereo.odom_eval`。apt 装了 `ros-humble-rtabmap-ros`、`ros-humble-stereo-image-proc`（RTAB-Map 0.23.13）。
  - **话题/坐标系全带 stereo 前缀**（`/stereo/odom`、`/stereo/odom_info`，TF `stereo_odom → stereo_base → stereo_optical`）：
    本机同一 ROS 域常有小车工程（`~/工创大赛小车`）的仿真在跑，也发 `/odom`、`base_link`，实测混用会串话。
  - **`~/.bashrc` 把海康 MVS SDK 的 `/opt/MVS/lib/64` 加进了 `LD_LIBRARY_PATH`**，其旧 libusb 让 RTAB-Map 节点启动即崩
    （`libpcl_io.so: undefined symbol: libusb_set_option`）。launch 里只对 RTAB-Map 节点剔除该路径，未改全局。
  - RTAB-Map 先发 `odom` 后发 `odom_info`，同戳配对要等两边到齐。`rtabmap_viz` 报找不到 `rtabmap` 节点的服务是因为没起 SLAM，无害。
  - 2026-10-22 静止 15 s（85 mm 支架）：无丢失，内点中位 305，首尾 0.000 m。**速度未定论**：当时机器负载 25（8 线程，小车仿真 + QQ），
    实测 116 ms/帧、7 Hz，需空载复测；RTAB-Map 警告约 85% 双目对应点被拒，多在弱纹理区，待纹理丰富场景评估。

待确认：更多闭环数据集（含较快转弯）验证参数、陀螺辅助、刚性安装后的同步复测、相机侧丢帧 17% 的原因、机载板选型与板上帧率、MAVLink 输出、传感器型号与是否真为全局快门。

## 6. 调试工具

**实时预览** `tools/preview.py`（板子侧零依赖，本机解码）：

```bash
python3 -X utf8 tools/preview.py            # 默认连 10.42.0.50:8090
#   --gray        只传 Y 平面，灰度，省 1/3 带宽
#   --awb-gain R,B  显示层白平衡增益（仅改预览观感，不碰板子配置）
```

原理：板子上只跑 `v4l2-ctl` 把 NV12 裸帧写 stdout，经 SSH 管道传回，本机做
NV12→RGB→JPEG 并以 MJPEG over HTTP 输出。实测 640×480 满帧 29.9fps 无掉帧，
USB RNDIS 带宽够用。

⚠️ 板子上**没有** gstreamer / ffmpeg / OpenCV / numpy / PIL，Python 是裸的；本机没有
mpv / vlc / ffplay，只有 firefox。写工具时别假设这些存在。

⚠️ 预览运行时独占 `/dev/video0`，其他程序打不开摄像头（EBUSY）。调试前先停预览，并
**进程 + 端口双侧核验**（本机 `ss -tln | grep 8090`，板子 `pgrep -af "[v]4l2-ctl"`）。

浏览器打开 http://127.0.0.1:8090 看实时画面与帧率。
