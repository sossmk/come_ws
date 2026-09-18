#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import PortHandler, PacketHandler

# =================== 고정 설정 (여기만 수정하세요) ===================
DEVICE         = "/dev/ttyUSB0"   # 포트
BAUDRATE       = 57600            # 보드레이트

ID_LEFT        = 8                # 왼쪽 구동
ID_RIGHT       = 9                # 오른쪽 구동
ID_STEER       = 7                # 조향

# 구동 최대속도 제한(LSB) — 네 모터가 ±200 까지 허용
VEL_LSB_LIMIT  = 200

# 로봇 치수
WHEEL_RADIUS   = 0.0325           # m
WHEEL_BASE     = 0.24             # m

# 조향(위치) 튜닝
STEER_CENTER   = 800             # 센터 틱(12-bit 기준 예시)
STEER_MIN      = 300             # 최소 틱
STEER_MAX      = 1300             # 최대 틱
MAX_STEER_DEG  = 25.0             # 최대 조향 각도(도)
STEER_DIR      = -1               # 좌우 뒤집힘 보정(+1 또는 -1)
STEER_GAIN     = 1.0              # 조향 감도 스케일(1.0 기본)
USE_CUR_AS_CENTER = False         # True면 시작시 현재 위치를 센터로 잡음

# 안전: /cmd_vel이 끊기면 이 시간(s) 뒤 정지
CMD_TIMEOUT_S  = 0.5

# 오른쪽 바퀴 전진은 음수(-), 왼쪽 바퀴 전진은 양수(+)로 “고정”
# =====================================================================

# ===== Control Table (X-series, Protocol 2.0) =====
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_VELOCITY    = 104      # int32
ADDR_GOAL_POSITION    = 116      # uint32
ADDR_PRESENT_VELOCITY = 128      # int32
ADDR_PRESENT_POSITION = 132      # uint32

MODE_VELOCITY = 1
MODE_POSITION = 3
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

RPM_PER_LSB   = 0.229
TICKS_PER_DEG = 1.0 / 0.088      # ≈ 11.36 tick/deg

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v


class DynaCmdVel(Node):
    def __init__(self):
        super().__init__('dyna_cmd_vel')

        # ---- 포트 열기 ----
        self.port   = PortHandler(DEVICE)
        self.packet = PacketHandler(2.0)

        if not self.port.openPort():
            self.get_logger().error(f'포트 열기 실패: {DEVICE}')
            raise RuntimeError('openPort failed')
        if not self.port.setBaudRate(BAUDRATE):
            self.get_logger().error(f'보드레이트 설정 실패: {BAUDRATE}')
            raise RuntimeError('setBaudRate failed')

        # ---- 모드 설정 ----
        self._set_mode(ID_LEFT,  MODE_VELOCITY)
        self._set_mode(ID_RIGHT, MODE_VELOCITY)
        self._set_mode(ID_STEER, MODE_POSITION)

        # ---- 초기값 ----
        self._write_vel(ID_LEFT,  0)
        self._write_vel(ID_RIGHT, 0)

        # 상태 점검
        omL = self._read1(ID_LEFT,  ADDR_OPERATING_MODE)
        omR = self._read1(ID_RIGHT, ADDR_OPERATING_MODE)
        omS = self._read1(ID_STEER, ADDR_OPERATING_MODE)
        teL = self._read1(ID_LEFT,  ADDR_TORQUE_ENABLE)
        teR = self._read1(ID_RIGHT, ADDR_TORQUE_ENABLE)
        teS = self._read1(ID_STEER, ADDR_TORQUE_ENABLE)
        ppS = self._read4(ID_STEER,  ADDR_PRESENT_POSITION)
        self.get_logger().info(
            f'STATUS OM L/R/S={omL}/{omR}/{omS}, TE L/R/S={teL}/{teR}/{teS}, PP S={ppS}')

        # 센터 설정
        center = int(ppS) if USE_CUR_AS_CENTER else STEER_CENTER
        self._write_pos(ID_STEER, center)
        self.steer_center = center

        # ---- ROS 구독/타이머 ----
        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f'dyna_cmd_vel_node: L={ID_LEFT} R={ID_RIGHT} S={ID_STEER} '
            f'dev={DEVICE} baud={BAUDRATE} limit={VEL_LSB_LIMIT} center={self.steer_center}'
        )

    # ===== 저수준 I/O (진단 포함) =====
    def _write1(self, dxl_id, addr, val):
        comm, dxl_err = self.packet.write1ByteTxRx(self.port, dxl_id, addr, int(val))
        if comm != 0 or dxl_err != 0:
            self.get_logger().warn(f'W1 FAIL id={dxl_id} addr={addr} val={val} comm={comm} dxl_err={dxl_err}')

    def _write4(self, dxl_id, addr, val):
        comm, dxl_err = self.packet.write4ByteTxRx(self.port, dxl_id, addr, int(val) & 0xFFFFFFFF)
        if comm != 0 or dxl_err != 0:
            self.get_logger().warn(f'W4 FAIL id={dxl_id} addr={addr} val={val} comm={comm} dxl_err={dxl_err}')

    def _read1(self, dxl_id, addr):
        data, comm, dxl_err = self.packet.read1ByteTxRx(self.port, dxl_id, addr)
        if comm != 0 or dxl_err != 0:
            self.get_logger().warn(f'R1 FAIL id={dxl_id} addr={addr} comm={comm} dxl_err={dxl_err}')
        return data

    def _read4(self, dxl_id, addr):
        data, comm, dxl_err = self.packet.read4ByteTxRx(self.port, dxl_id, addr)
        if comm != 0 or dxl_err != 0:
            self.get_logger().warn(f'R4 FAIL id={dxl_id} addr={addr} comm={comm} dxl_err={dxl_err}')
        return data

    def _set_mode(self, dxl_id, mode):
        self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self._write1(dxl_id, ADDR_OPERATING_MODE, mode)
        self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def _write_vel(self, dxl_id, lsb):
        lsb = int(clamp(lsb, -VEL_LSB_LIMIT, VEL_LSB_LIMIT))
        self._write4(dxl_id, ADDR_GOAL_VELOCITY, lsb)

    def _write_pos(self, dxl_id, pos):
        pos = int(clamp(pos, STEER_MIN, STEER_MAX))
        self._write4(dxl_id, ADDR_GOAL_POSITION, pos)

    # ===== 콜백 =====
    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        v  = float(msg.linear.x)
        wz = float(msg.angular.z)

        # ---- 조향 (정지에서도 조향 가능) ----
        if abs(wz) < 1e-4:
            delta_deg = 0.0
        else:
            if abs(v) < 1e-4:
                delta_deg = clamp(STEER_GAIN * wz * MAX_STEER_DEG, -MAX_STEER_DEG, MAX_STEER_DEG)
            else:
                delta = math.atan(WHEEL_BASE * wz / v)
                delta_deg = clamp(STEER_GAIN * math.degrees(delta), -MAX_STEER_DEG, MAX_STEER_DEG)

        steer_ticks = int(self.steer_center + STEER_DIR * delta_deg * TICKS_PER_DEG)
        self._write_pos(ID_STEER, steer_ticks)

        # ---- 구동: 8은 +가 전진, 9는 -가 전진(고정) ----
        if abs(v) < 0.05:   # ~5cm/s 이하는 데드존
            lsbL = 0
            lsbR = 0
        else:
            wheel_rpm = (v / (2 * math.pi * WHEEL_RADIUS)) * 60.0
            vel_lsb   = wheel_rpm / RPM_PER_LSB
            lsbL = int( vel_lsb)   # 좌측(+)
            lsbR = int(-vel_lsb)   # 우측(-)

        self._write_vel(ID_LEFT,  lsbL)
        self._write_vel(ID_RIGHT, lsbR)

        # 상태 로그(현재속도/현재위치)
        pvL = self._read4(ID_LEFT,  ADDR_PRESENT_VELOCITY)
        pvR = self._read4(ID_RIGHT, ADDR_PRESENT_VELOCITY)
        ppS = self._read4(ID_STEER,  ADDR_PRESENT_POSITION)
        self.get_logger().info(
            f'cmd v={v:.2f} w={wz:.2f} → steer={delta_deg:.1f}°({steer_ticks}), '
            f'LSB L={lsbL} R={lsbR}, PV L={pvL} R={pvR}, PP S={ppS}'
        )

    def on_timer(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > CMD_TIMEOUT_S:
            self._write_vel(ID_LEFT,  0)
            self._write_vel(ID_RIGHT, 0)

    def destroy_node(self):
        try:
            self._write_vel(ID_LEFT, 0)
            self._write_vel(ID_RIGHT, 0)
            self._write1(ID_LEFT,  ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._write1(ID_RIGHT, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._write1(ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            try:
                self.port.closePort()
            except Exception:
                pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynaCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


