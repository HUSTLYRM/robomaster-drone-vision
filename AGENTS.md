# AGENTS.md — 竞赛无人机视觉项目必读

更新日期：2026-09-21

本文是接手本项目的 agent **必读入口**，只记「不看就会做错」的事实与结论。
详细数据见 `docs/`，两处不重复维护：

- `docs/项目计划书.md` — 目标、选型基线、阶段验收
- `docs/硬件接线与配置手册.md` — 实测确认的接线、引脚、参数（硬件事实 SSOT）

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
