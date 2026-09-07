#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR /scan -> /cmd_vel 변환 노드 (ROS2 Humble)
- 시야각(ROI)을 -FOV/2 ~ +FOV/2 로 제한
- 중앙/좌/우 구간 평균거리로 회피/추종하는 간단한 규칙 기반 제어
- 안전 정지, NaN/inf 정리, 저역통과(smoothing) 포함

토픽:
  sub: /scan (sensor_msgs/msg/LaserScan)
  pub: /cmd_vel (geometry_msgs/msg/Twist)

파라미터(ros2 param set 또는 launch override로 변경):
  fov_deg:             전방 시야각(도), 예: 90 → -45°~+45°
  center_width_deg:    중앙 구간 폭(도), 예: 30 → -15°~+15°
  stop_range:          정지 임계 거리(m). 이보다 가까우면 정지
  cruise_speed:        평시 직진 속도(m/s)
  max_ang_vel:         최대 회전 속도(rad/s)
  min_valid_ratio:     ROI 내 유효 샘플 비율이 이보다 낮으면 정지
  smooth_alpha_lin:    선속 저역통과(0~1, 클수록 반응 빠름)
  smooth_alpha_ang:    각속 저역통과(0~1)
  scan_topic:          입력 스캔 토픽명
  cmd_vel_topic:       출력 cmd_vel 토픽명
  rate_hz:             제어 루프 주기(Hz)
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class ScanToCmdVel(Node):
    def __init__(self):
        super().__init__('scan_to_cmdvel')

        # ===== 파라미터 선언 =====
        self.declare_parameter('fov_deg',           90.0)
        self.declare_parameter('center_width_deg',  30.0)
        self.declare_parameter('stop_range',         0.35)   # m
        self.declare_parameter('cruise_speed',       0.20)   # m/s
        self.declare_parameter('max_ang_vel',        1.20)   # rad/s
        self.declare_parameter('min_valid_ratio',    0.25)   # 유효샘플 비율
        self.declare_parameter('smooth_alpha_lin',   0.4)
        self.declare_parameter('smooth_alpha_ang',   0.5)
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('cmd_vel_topic',     '/cmd_vel')
        self.declare_parameter('rate_hz',           20.0)

        # 파라미터 로드
        self.fov_deg          = float(self.get_parameter('fov_deg').value)
        self.center_width_deg = float(self.get_parameter('center_width_deg').value)
        self.stop_range       = float(self.get_parameter('stop_range').value)
        self.cruise_speed     = float(self.get_parameter('cruise_speed').value)
        self.max_ang_vel      = float(self.get_parameter('max_ang_vel').value)
        self.min_valid_ratio  = float(self.get_parameter('min_valid_ratio').value)
        self.smooth_alpha_lin = float(self.get_parameter('smooth_alpha_lin').value)
        self.smooth_alpha_ang = float(self.get_parameter('smooth_alpha_ang').value)
        self.scan_topic       = self.get_parameter('scan_topic').value
        self.cmd_vel_topic    = self.get_parameter('cmd_vel_topic').value
        self.rate_hz          = float(self.get_parameter('rate_hz').value)

        # 입력 QoS (센서 QoS 권장)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 상태 변수
        self.latest_scan: LaserScan | None = None
        self.y_lin = 0.0  # 저역통과된 선속
        self.y_ang = 0.0  # 저역통과된 각속

        # 제어 타이머
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_step)

        self.get_logger().info(
            f"ScanToCmdVel started: scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, "
            f"FOV={self.fov_deg}deg, center={self.center_width_deg}deg, stop_range={self.stop_range}m"
        )

    # ===== 스캔 콜백 =====
    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    # ===== 인덱스 범위 계산 유틸 =====
    def angle_to_index(self, msg: LaserScan, angle_rad: float) -> int:
        # angle_min ~ angle_max 사이에서 가장 가까운 인덱스
        idx = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
        return clamp(idx, 0, len(msg.ranges) - 1)

    def roi_indices(self, msg: LaserScan, deg_lo: float, deg_hi: float) -> Tuple[int, int]:
        # deg → rad
        a_lo = math.radians(deg_lo)
        a_hi = math.radians(deg_hi)
        i_lo = self.angle_to_index(msg, a_lo)
        i_hi = self.angle_to_index(msg, a_hi)
        if i_lo > i_hi:
            i_lo, i_hi = i_hi, i_lo
        return i_lo, i_hi

    # ===== 거리 벡터에서 유효값만 추출 후 평균 =====
    def filtered_mean(self, values: List[float]) -> Tuple[float, float]:
        valid = [v for v in values if math.isfinite(v) and v > 0.0]
        if not valid:
            return float('nan'), 0.0
        return sum(valid) / len(valid), len(valid) / len(values)

    # ===== 제어 스텝 =====
    def control_step(self):
        if self.latest_scan is None:
            # 스캔이 아직 없음 → 정지
            self.publish_cmd(0.0, 0.0)
            return

        msg = self.latest_scan

        # ROI 설정: -FOV/2 ~ +FOV/2 (전방)
        half = self.fov_deg * 0.5
        i_lo, i_hi = self.roi_indices(msg, -half, +half)
        roi = msg.ranges[i_lo:i_hi + 1]

        # 중앙/좌/우 구간 나누기
        c_half = self.center_width_deg * 0.5
        i_c_lo, i_c_hi = self.roi_indices(msg, -c_half, +c_half)
        i_l_lo, i_l_hi = self.roi_indices(msg, -half, -c_half)
        i_r_lo, i_r_hi = self.roi_indices(msg, +c_half, +half)

        left  = msg.ranges[i_l_lo:i_l_hi + 1] if i_l_hi >= i_l_lo else []
        center= msg.ranges[i_c_lo:i_c_hi + 1] if i_c_hi >= i_c_lo else []
        right = msg.ranges[i_r_lo:i_r_hi + 1] if i_r_hi >= i_r_lo else []

        mean_l, ratio_l = self.filtered_mean(left)   if left  else (float('nan'), 0.0)
        mean_c, ratio_c = self.filtered_mean(center) if center else (float('nan'), 0.0)
        mean_r, ratio_r = self.filtered_mean(right)  if right else (float('nan'), 0.0)

        # ROI 전체 유효율
        _, ratio_roi = self.filtered_mean(roi)

        # ===== 안전 조건 =====
        safe_stop = False
        reason = ""

        # 1) 유효 데이터 부족
        if ratio_roi < self.min_valid_ratio:
            safe_stop = True
            reason = f"low valid ratio ({ratio_roi:.2f} < {self.min_valid_ratio:.2f})"

        # 2) 중앙 매우 근접 → 즉시 정지
        if not safe_stop and math.isfinite(mean_c) and mean_c < self.stop_range:
            safe_stop = True
            reason = f"obstacle ahead < stop_range ({mean_c:.2f} < {self.stop_range:.2f})"

        if safe_stop:
            self.publish_cmd(0.0, 0.0)
            if reason:
                self.get_logger().debug(f"SAFE STOP: {reason}")
            return

        # ===== 규칙 기반 제어 =====
        # 선속: 중앙 평균거리에 따라 선형 스케일링(멀수록 크루즈 속도, 가까울수록 감속)
        # 간단히: 중앙이 stop_range*2 보다 멀면 cruise_speed, 그 사이에서는 선형 보간
        lin = self.cruise_speed
        if math.isfinite(mean_c):
            near = self.stop_range * 2.0
            if mean_c < near:
                lin = self.cruise_speed * clamp((mean_c - self.stop_range) / (near - self.stop_range), 0.0, 1.0)

        # 각속: 좌/우 평균거리 차이를 이용
        # 오른쪽이 더 멀면 오른쪽으로 공간 → 음수 회전(오른쪽 회전), 반대면 양수(왼쪽 회전)
        ang = 0.0
        if math.isfinite(mean_l) and math.isfinite(mean_r):
            diff = (mean_l - mean_r)  # >0 이면 왼쪽이 더 멀다(오른쪽이 가깝다) → 왼쪽으로 회전(ang>0)
            # 거리 차이를 [-max_ang_vel, +max_ang_vel]로 스케일 (간단한 비례)
            # 스케일 팩터는 경험적으로 1.0/1.0m 로 두고 clamp
            ang = clamp(diff * 1.0, -self.max_ang_vel, self.max_ang_vel)

        # ===== 저역통과 필터(부드럽게) =====
        self.y_lin = self.smooth_alpha_lin * lin + (1.0 - self.smooth_alpha_lin) * self.y_lin
        self.y_ang = self.smooth_alpha_ang * ang + (1.0 - self.smooth_alpha_ang) * self.y_ang

        self.publish_cmd(self.y_lin, self.y_ang)

        # 디버깅은 필요할 때만 주석 해제
        # self.get_logger().info(
        #     f"L:{mean_l:.2f} C:{mean_c:.2f} R:{mean_r:.2f} | lin:{self.y_lin:.2f} ang:{self.y_ang:.2f}"
        # )

    def publish_cmd(self, lin: float, ang: float):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ScanToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

