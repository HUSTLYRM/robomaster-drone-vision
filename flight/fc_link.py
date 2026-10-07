"""与 PX4 的 MAVLink 链路：后台线程收消息填 VehicleState，主线程发设定点和命令。

约定：
- 所有发送都在主线程（pymavlink 的序号计数不是线程安全的），接收线程只读不写。
- 模式切换/解锁是"发出即返回"，不阻塞等 ACK：主循环必须保持 ≥2 Hz 的设定点流，
  阻塞 1 s 等 ACK 就可能触发 Offboard 丢失保护。是否生效由状态机看 HEARTBEAT 判断，
  ACK 被拒的原因由接收线程记日志。
- 只采信飞控本体（compid 1）的消息；光流模块等其他组件（如 MTF-02P 的 compid 88）忽略。
"""
from __future__ import annotations

import copy
import os
import threading
import time

from loguru import logger

# 必须在导入 pymavlink 之前设置，强制 MAVLink2（PX4 USB 口默认 v1，串口按对端切换）
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from . import px4_modes  # noqa: E402
from .mission import Setpoint  # noqa: E402
from .state import VehicleState  # noqa: E402

mavlink = mavutil.mavlink

AUTOPILOT_COMPID = mavlink.MAV_COMP_ID_AUTOPILOT1
ONBOARD_COMPID = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER

_RC_BIT = mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
# 只控制位置和航向，忽略速度、加速度和航向角速度
_POSITION_ONLY_MASK = (
    mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# 任务需要的消息及频率（Hz）。PX4 minimal/onboard 模式下默认流不一定包含这些，统一显式请求
MESSAGE_RATES_HZ = {
    mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 30.0,
    mavlink.MAVLINK_MSG_ID_ATTITUDE: 30.0,
    mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS: 10.0,
    mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10.0,
    mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 5.0,
    mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
}


class FcLink:
    def __init__(
        self,
        url: str,
        baud: int = 921600,
        source_system: int = 1,
        source_component: int = ONBOARD_COMPID,
    ) -> None:
        """url 例：'/dev/ttyS8'（实机串口）、'udpin:0.0.0.0:14540'（PX4 SITL 的 offboard 端口）。"""
        self._conn = mavutil.mavlink_connection(
            url, baud=baud, source_system=source_system, source_component=source_component
        )
        self._lock = threading.Lock()
        self._state = VehicleState()
        self._acks: dict[int, tuple[int, float]] = {}  # command -> (result, 收到时间)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._t_start = time.monotonic()
        self.target_system = 0

    # ---------- 生命周期 ----------

    def connect(self, timeout_s: float = 10.0) -> None:
        """等到飞控心跳，确定 target_system，然后启动接收线程。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            # 对方可能要先收到我们的消息才开始发（mavlink -w），等待期间持续发心跳
            self.send_heartbeat()
            msg = self._conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg is None or msg.get_srcComponent() != AUTOPILOT_COMPID:
                continue
            if msg.type == mavlink.MAV_TYPE_GCS:
                continue
            self.target_system = msg.get_srcSystem()
            self._handle(msg)
            logger.info(f"已连接飞控 sysid={self.target_system} 模式={px4_modes.name(self._state.mode)}")
            self._thread = threading.Thread(target=self._rx_loop, name="fc_rx", daemon=True)
            self._thread.start()
            return
        raise TimeoutError(f"{timeout_s} s 内未收到飞控心跳")

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._conn.close()

    def snapshot(self) -> VehicleState:
        with self._lock:
            s = copy.copy(self._state)
        s.now = time.monotonic()
        return s

    # ---------- 接收 ----------

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception as e:  # 串口瞬断等，不能让接收线程死掉
                logger.warning(f"接收异常: {e!r}")
                time.sleep(0.05)
                continue
            if msg is not None:
                self._handle(msg)

    def _handle(self, msg) -> None:  # noqa: ANN001 - pymavlink 消息类型是动态生成的
        if msg.get_srcSystem() != self.target_system or msg.get_srcComponent() != AUTOPILOT_COMPID:
            return
        t = time.monotonic()
        mtype = msg.get_type()
        with self._lock:
            s = self._state
            if mtype == "HEARTBEAT":
                if msg.type == mavlink.MAV_TYPE_GCS:
                    return
                s.heartbeat_t = t
                s.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if msg.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                    s.mode = px4_modes.decode(msg.custom_mode)
            elif mtype == "LOCAL_POSITION_NED":
                s.pos_t = t
                s.x, s.y, s.z = msg.x, msg.y, msg.z
                s.vx, s.vy, s.vz = msg.vx, msg.vy, msg.vz
            elif mtype == "ATTITUDE":
                s.att_t = t
                s.yaw = msg.yaw
            elif mtype == "ESTIMATOR_STATUS":
                s.est_t = t
                s.est_flags = msg.flags
            elif mtype == "DISTANCE_SENSOR":
                # 只要朝下的测距；PX4 用 cm 上报
                if msg.orientation == mavlink.MAV_SENSOR_ROTATION_PITCH_270:
                    s.range_t = t
                    s.range_m = msg.current_distance / 100.0
            elif mtype == "EXTENDED_SYS_STATE":
                s.landed_t = t
                s.landed_state = msg.landed_state
            elif mtype == "SYS_STATUS":
                s.sys_t = t
                s.battery_v = None if msg.voltage_battery == 0xFFFF else msg.voltage_battery / 1000.0
                s.rc_ok = bool(
                    msg.onboard_control_sensors_present
                    & msg.onboard_control_sensors_enabled
                    & msg.onboard_control_sensors_health
                    & _RC_BIT
                )
            elif mtype == "COMMAND_ACK":
                self._acks[msg.command] = (msg.result, t)
        # 日志放到锁外
        if mtype == "COMMAND_ACK" and msg.result != mavlink.MAV_RESULT_ACCEPTED:
            logger.warning(f"命令 {msg.command} 被拒，result={msg.result}")
        elif mtype == "STATUSTEXT":
            logger.info(f"[PX4] {msg.text}")

    def last_ack(self, command: int) -> tuple[int, float] | None:
        with self._lock:
            return self._acks.get(command)

    # ---------- 发送（只在主线程调用） ----------

    def send_heartbeat(self) -> None:
        self._conn.mav.heartbeat_send(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE
        )

    def send_setpoint(self, sp: Setpoint) -> None:
        self._conn.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self._t_start) * 1000) & 0xFFFFFFFF,
            self.target_system,
            AUTOPILOT_COMPID,
            mavlink.MAV_FRAME_LOCAL_NED,
            _POSITION_ONLY_MASK,
            sp.x, sp.y, sp.z,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            sp.yaw, 0.0,
        )

    def send_command(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._conn.mav.command_long_send(self.target_system, AUTOPILOT_COMPID, command, 0, *p)

    def request_mode(self, mode: px4_modes.Px4Mode) -> None:
        self.send_command(
            mavlink.MAV_CMD_DO_SET_MODE,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode.main,
            mode.sub,
        )

    def request_arm(self) -> None:
        self.send_command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def request_disarm(self) -> None:
        # 不带强制魔数 21196：PX4 在空中会拒绝普通上锁，这正是我们要的
        self.send_command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def request_message_rates(self, timeout_s: float = 3.0) -> list[int]:
        """起飞前调用（此时还没有设定点流，可以阻塞）。返回未被接受的消息 ID。"""
        failed = []
        for msg_id, hz in MESSAGE_RATES_HZ.items():
            accepted = False
            for _ in range(3):
                sent_t = time.monotonic()
                self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
                deadline = time.monotonic() + timeout_s / 3
                while time.monotonic() < deadline:
                    ack = self.last_ack(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)
                    if ack is not None and ack[1] >= sent_t:
                        accepted = ack[0] == mavlink.MAV_RESULT_ACCEPTED
                        break
                    time.sleep(0.02)
                else:
                    continue
                break
            if not accepted:
                failed.append(msg_id)
        return failed
