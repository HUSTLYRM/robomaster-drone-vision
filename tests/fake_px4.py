"""MAVLink 层假飞控：把 Px4Sim 包在真实的 MAVLink 收发外面，供 FcLink 端到端测试。

只发送被 SET_MESSAGE_INTERVAL 请求过的消息（心跳除外），以验证消息频率请求确实生效；
另外故意混入光流模块（compid 88）和地面站心跳，验证 FcLink 的来源过滤。
"""
from __future__ import annotations

import os
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from flight import px4_modes  # noqa: E402
from flight.mission import Setpoint  # noqa: E402
from tests.px4_sim import Px4Sim  # noqa: E402

mavlink = mavutil.mavlink
_RC = mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER


class FakePx4:
    def __init__(self, port: int) -> None:
        self.sim = Px4Sim()
        self.conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1, source_component=1)
        self.rates: dict[int, float] = {}  # msg_id -> Hz
        self._next_send: dict[int, float] = {}
        self.max_alt = 0.0
        self.modes_seen: list[str] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.conn.close()

    def _ack(self, command: int, ok: bool) -> None:
        result = mavlink.MAV_RESULT_ACCEPTED if ok else mavlink.MAV_RESULT_TEMPORARILY_REJECTED
        self.conn.mav.command_ack_send(command, result)

    def _handle(self, msg, t: float) -> None:  # noqa: ANN001
        mtype = msg.get_type()
        if mtype == "SET_POSITION_TARGET_LOCAL_NED":
            self.sim.setpoint(t, Setpoint(msg.x, msg.y, msg.z, msg.yaw))
        elif mtype == "COMMAND_LONG":
            if msg.command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                self.rates[int(msg.param1)] = 1e6 / msg.param2
                self._ack(msg.command, True)
            elif msg.command == mavlink.MAV_CMD_DO_SET_MODE:
                mode = px4_modes.Px4Mode(int(msg.param2), int(msg.param3))
                self._ack(msg.command, self.sim.request_mode(t, mode))
            elif msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.param1 == 1:
                    ok = self.sim.request_arm(t)
                    if not ok:
                        self.conn.mav.statustext_send(mavlink.MAV_SEVERITY_CRITICAL, b"Arming denied")
                else:
                    ok = self.sim.request_disarm(t)
                self._ack(msg.command, ok)

    def _due(self, msg_id: int, t: float) -> bool:
        hz = self.rates.get(msg_id)
        if not hz:
            return False
        if t >= self._next_send.get(msg_id, 0.0):
            self._next_send[msg_id] = t + 1.0 / hz
            return True
        return False

    def _send_streams(self, t: float) -> None:
        sim, mav = self.sim, self.conn.mav
        if t >= self._next_send.get(-1, 0.0):  # 心跳与真 PX4 一样固定 1 Hz，不受频率请求控制
            self._next_send[-1] = t + 1.0
            base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if sim.armed else 0)
            mav.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_PX4, base,
                px4_modes.encode(sim.mode), mavlink.MAV_STATE_ACTIVE,
            )
            # 干扰：地面站心跳（不同模式值）和光流模块的测距，FcLink 都应忽略
            mav.srcComponent = 190
            mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            mav.srcComponent = 88
            mav.distance_sensor_send(0, 2, 600, 555, 0, 0, mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0)
            mav.srcComponent = 1
        ms = int(t * 1000) & 0xFFFFFFFF
        if self._due(mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, t):
            mav.local_position_ned_send(ms, sim.x, sim.y, sim.z, 0.0, 0.0, 0.0)
        if self._due(mavlink.MAVLINK_MSG_ID_ATTITUDE, t):
            mav.attitude_send(ms, 0.0, 0.0, sim.yaw, 0.0, 0.0, 0.0)
        if self._due(mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, t):
            mav.estimator_status_send(int(t * 1e6), sim.est_flags, 0, 0, 0, 0, 0, 0, 0, 0)
        if self._due(mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, t):
            mav.distance_sensor_send(
                ms, 2, 600, int(max(0.02, sim.alt) * 100), 0, 0, mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0
            )
        if self._due(mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, t):
            landed = mavlink.MAV_LANDED_STATE_ON_GROUND if sim.z >= -0.02 else mavlink.MAV_LANDED_STATE_IN_AIR
            mav.extended_sys_state_send(0, landed)
        if self._due(mavlink.MAVLINK_MSG_ID_SYS_STATUS, t):
            rc = _RC if sim.rc_ok else 0
            mv = 0xFFFF if sim.battery_v is None else int(sim.battery_v * 1000)
            mav.sys_status_send(_RC, _RC, rc, 0, mv, -1, -1, 0, 0, 0, 0, 0, 0)

    def _loop(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic()
            while True:
                msg = self.conn.recv_match(blocking=False)
                if msg is None:
                    break
                self._handle(msg, t)
            self.sim.tick(t, t - last)
            last = t
            self.max_alt = max(self.max_alt, self.sim.alt)
            name = px4_modes.name(self.sim.mode)
            if not self.modes_seen or self.modes_seen[-1] != name:
                self.modes_seen.append(name)
            self._send_streams(t)
            time.sleep(0.005)
