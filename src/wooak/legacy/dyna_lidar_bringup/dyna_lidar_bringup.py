#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Dynamixel SDK
from dynamixel_sdk import *  # pip install dynamixel-sdk

# ===== DXL Control Table (X/MX 2.0 공통) =====
ADDR_TORQUE_ENABLE   = 64
ADDR_OPERATING_MODE  = 11
ADDR_GOAL_VELOCITY   = 104    # 4 bytes, unit: 0.229 rpm/LSB (X-series 공식 단위)
ADDR_GOAL_POSITION   = 116    # 4 bytes
ADDR_PRESENT_POS     = 132    # 4 bytes

# 운영모드 값 (X-series)
MODE_VELOCITY = 1
MODE_POSITION = 3

class LidarGapSteer(Node):
    def __init__(self):
        super().__init__('lidar_gap_steer')

        # ------------ 파라미터 ------------
        self.declare_parameter('device', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('protocol_version', 2.0)

        self.declare_parameter('left_id', 7)
        self.declare_parameter('right_id', 9)
        self.declare_parameter('steer_id', 8)

        self.declare_parameter('reverse_left', False)
        self.declare_parameter('reverse_right', False)

        # 조향 서보 맵핑
        self.declare_parameter('steer_center', 3200)     # 서보 센터 위치(틱)
        self.declare_parameter('steer_range_ticks', 800) # ±조향범위(틱) → 총 1600틱이 최대 조향
        self.declare_parameter('steer_max_deg', 30.0)    # ±최대 조향각(도) : 로봇 실제 휠 꺾임 한계

        # 속도/안전
        self.declare_parameter('base_speed', 50)        # 기본 구동 속도(Goal Velocity LSB단위) 양수=전진
        self.declare_parameter('min_speed', 20)         # 장애물 근접 시 최저 속도
        self.declare_parameter('stop_dist', 0.35)       # 정지 임계(전방) [m]
        self.declare_parameter('slow_dist', 0.7)        # 감속 시작 거리 [m]

        # FTG(격자) 파라미터
        self.declare_parameter('range_clip_min', 0.05)
        self.declare_parameter('range_clip_max', 3.5)
        self.declare_parameter('smoothing_window', 5)    # 이동평균 윈도우(홀수)
        self.declare_parameter('bubble_radius', 0.25)    # 가장 가까운 물체 주변 버블 반경[m]
        self.declare_parameter('front_fov_deg', 180.0)   # 처리할 전방 FOV(도) (예: 180도)

        # 읽기
        self.port_name       = self.get_parameter('device').value
        self.baudrate        = self.get_parameter('baudrate').value
        self.pv              = float(self.get_parameter('protocol_version').value)
        self.left_id         = self.get_parameter('left_id').value
        self.right_id        = self.get_parameter('right_id').value
        self.steer_id        = self.get_parameter('steer_id').value
        self.rev_left        = self.get_parameter('reverse_left').value
        self.rev_right       = self.get_parameter('reverse_right').value

        self.center_ticks    = int(self.get_parameter('steer_center').value)
        self.steer_ticks     = int(self.get_parameter('steer_range_ticks').value)
        self.max_steer_deg   = float(self.get_parameter('steer_max_deg').value)

        self.base_speed      = int(self.get_parameter('base_speed').value)
        self.min_speed       = int(self.get_parameter('min_speed').value)
        self.stop_dist       = float(self.get_parameter('stop_dist').value)
        self.slow_dist       = float(self.get_parameter('slow_dist').value)

        self.rmin            = float(self.get_parameter('range_clip_min').value)
        self.rmax            = float(self.get_parameter('range_clip_max').value)
        self.win             = int(self.get_parameter('smoothing_window').value)
        self.bubble_r        = float(self.get_parameter('bubble_radius').value)
        self.front_fov_deg   = float(self.get_parameter('front_fov_deg').value)

        # ------------ DXL 연결 ------------
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(self.pv)
        if not self.port.openPort():
            self.get_logger().fatal(f'Failed to open {self.port_name}')
            raise SystemExit(1)
        if not self.port.setBaudRate(self.baudrate):
            self.get_logger().fatal(f'Failed to set baudrate {self.baudrate}')
            raise SystemExit(1)

        # 모터 초기화: 좌/우=Velocity, 조향=Position
        for mid in (self.left_id, self.right_id, self.steer_id):
            self._write1(mid, ADDR_TORQUE_ENABLE, 0)
        self._write1(self.left_id,  ADDR_OPERATING_MODE, MODE_VELOCITY)
        self._write1(self.right_id, ADDR_OPERATING_MODE, MODE_VELOCITY)
        self._write1(self.steer_id, ADDR_OPERATING_MODE, MODE_POSITION)
        for mid in (self.left_id, self.right_id, self.steer_id):
            self._write1(mid, ADDR_TORQUE_ENABLE, 1)

        # 조향을 센터로
        self._set_steer_ticks(self.center_ticks)

        # ------------ LiDAR 구독 ------------
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        self.get_logger().info('LidarGapSteer started. FTG + DXL direct control.')

    # ---------------- DXL helper ----------------
    def _dxl_err(self, cr, er):
        if cr != COMM_SUCCESS:
            self.get_logger().error(self.packet.getTxRxResult(cr))
        if er != 0:
            self.get_logger().error(self.packet.getRxPacketError(er))

    def _write1(self, mid, addr, val):
        cr, er = self.packet.write1ByteTxRx(self.port, mid, addr, int(val))
        self._dxl_err(cr, er)

    def _write4(self, mid, addr, val):
        cr, er = self.packet.write4ByteTxRx(self.port, mid, addr, int(val) & 0xFFFFFFFF)
        self._dxl_err(cr, er)

    def _set_drive_velocity(self, goal_vel):  # goal_vel: LSB
        # 방향 반전 처리
        vl = goal_vel * (-1 if self.rev_left  else 1)
        vr = goal_vel * (-1 if self.rev_right else 1)
        # X-series Goal Velocity는 부호 있는 4바이트 (2's complement)
        # dynamixel_sdk는 정수로 그대로 써주면 됨.
        self._write4(self.left_id,  ADDR_GOAL_VELOCITY,  int(vl))
        self._write4(self.right_id, ADDR_GOAL_VELOCITY,  int(vr))

    def _set_steer_ticks(self, pos_ticks):   # pos_ticks: 0~4095(모델별)
        self._write4(self.steer_id, ADDR_GOAL_POSITION, int(pos_ticks))

    # ---------------- LiDAR callback ----------------
    def on_scan(self, msg: LaserScan):
        # 1) 전방 FOV만 사용할 인덱스 범위 선택
        total = len(msg.ranges)
        fov = math.radians(self.front_fov_deg)
        # 중앙을 0도라 가정하고 좌/우로 fov/2 씩
        # LaserScan은 angle_min에서 angle_increment로 증가
        angles = msg.angle_min + np.arange(total) * msg.angle_increment
        mask = np.abs(angles) <= (fov / 2.0)
        rng = np.array(msg.ranges, dtype=np.float32)[mask]
        ang = angles[mask]

        if rng.size < 5:
            return

        # 2) 범위 클리핑 + NaN/inf 처리
        rng = np.clip(rng, self.rmin, self.rmax)
        rng[~np.isfinite(rng)] = self.rmax

        # 3) 이동평균(간단 스무딩)
        if self.win > 1 and self.win % 2 == 1:
            k = self.win
            ker = np.ones(k, dtype=np.float32) / k
            rng = np.convolve(rng, ker, mode='same')

        # 4) 가장 가까운 물체 찾고 버블(안전 반경) 비우기
        idx_min = int(np.argmin(rng))
        min_dist = float(rng[idx_min])
        # 버블: 거리->각도상 반경을 개략적으로 근사(작은 각 근사)
        # 각 해상도에 따른 인덱스 반경
        ang_inc = abs(msg.angle_increment)
        if ang_inc == 0:
            ang_inc = math.radians(1.0)
        # 버블 반경을 각도 기준으로 환산: theta ≈ bubble_r / min_dist
        # (근사: 가까울수록 더 큰 각 범위를 막음)
        bubble_ang = min(1.2, self.bubble_r / max(min_dist, 1e-3))
        bubble_idx = max(1, int(bubble_ang / ang_inc))
        rng[max(0, idx_min - bubble_idx): min(len(rng), idx_min + bubble_idx + 1)] = self.rmin

        # 5) 가장 긴 gap(연속으로 큰 거리) 탐색
        thresh = 0.6 * self.rmax  # 넉넉한 gap 기준
        good = rng > thresh
        # run-length로 가장 긴 True 구간 찾기
        best_len, best_s, best_e = 0, 0, -1
        s = None
        for i, g in enumerate(good):
            if g and s is None:
                s = i
            if (not g or i == len(good)-1) and s is not None:
                e = i if not g else i  # [s, e)
                length = e - s
                if length > best_len:
                    best_len, best_s, best_e = length, s, e
                s = None
        if best_len == 0:
            # gap이 없으면 가장 먼 방향으로
            target_idx = int(np.argmax(rng))
        else:
            target_idx = (best_s + best_e) // 2

        target_ang = float(ang[target_idx])  # 라디안 (좌=+, 우=- 로 보통 들어옴)
        # 6) 속도 결정(가깝게 있으면 감속/정지)
        front_min = float(np.min(rng[len(rng)//2 - 5 : len(rng)//2 + 5]))  # 중심부 최소
        if front_min < self.stop_dist:
            goal_v = 0
        elif front_min < self.slow_dist:
            # 선형 스케일링
            alpha = (front_min - self.stop_dist) / max(self.slow_dist - self.stop_dist, 1e-3)
            goal_v = int(self.min_speed + (self.base_speed - self.min_speed) * alpha)
        else:
            goal_v = int(self.base_speed)

        # 7) 조향각 → 서보 틱 맵핑
        # 제한: ±steer_max_deg
        max_rad = math.radians(self.max_steer_deg)
        target_ang = float(np.clip(target_ang, -max_rad, max_rad))
        # 라디안 → 틱 : 0 rad -> center, +max_rad -> center + steer_ticks
        pos_ticks = int(self.center_ticks + (target_ang / max_rad) * self.steer_ticks)

        # 8) 모터 명령
        self._set_steer_ticks(pos_ticks)
        self._set_drive_velocity(goal_v)

        # 디버그 로그 (간헐적으로)
        if (self.get_clock().now().nanoseconds // 1e9) % 1 < 0.02:
            self.get_logger().info(
                f"min={min_dist:.2f}m front={front_min:.2f}m  steer={math.degrees(target_ang):.1f}deg  vel={goal_v}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = LidarGapSteer()
    try:
        rclpy.spin(node)
    finally:
        # 정지 및 토크 OFF
        try:
            node._set_drive_velocity(0)
            for mid in (node.left_id, node.right_id, node.steer_id):
                node._write1(mid, ADDR_TORQUE_ENABLE, 0)
            node.port.closePort()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
