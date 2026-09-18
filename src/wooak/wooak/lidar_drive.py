#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32

from wooak.lidar_common import (STEER_CENTER, STEER_STEP, N_MIN, N_MAX, idx,
                                valid_ranges, zone_min, zone_argmin, clamp_steer)

class LidarDriveNode(Node):
    def __init__(self):
        super().__init__('lidar_drive_node')

        # LiDAR 배열 구독
        self.sub = self.create_subscription(Float32MultiArray, '/lidar_array', self.lidar_callback, 10)

        # 조향 명령 발행
        self.angle_pub = self.create_publisher(Float32, '/lidar_steer', 10)

        # ✅ 추가: cone 주행 완료 신호 퍼블리셔
        self.cone_done_pub = self.create_publisher(Int32, '/cone_done', 10)

        # ✅ 추가: finish 모드용 방향 신호 퍼블리셔
        self.finish_dir_pub = self.create_publisher(Int32, '/finish_dir', 10)

        # ✅ 중복 발행 방지용 플래그
        self.cone_done_flag = False

        self.get_logger().info("✅ LidarDriveNode started (Publishing /lidar_steer, /cone_done & /finish_dir)")

    def lidar_callback(self, msg):
        if not msg.data:
            return

        ranges = msg.data
        n = len(ranges)

        # ===== YDLIDAR G6 데이터 유효성 =====
        if n < N_MIN or n > N_MAX:
            self.get_logger().warn(f"⚠️ 예상 인덱스(약 1860)와 다름: {n}")
            return

        # ===== 구역 설정 (0~1860 인덱스 기준) =====
        right_indices  = idx((1450, 1610))
        center_indices = idx((1610, 1860), (0, 250))
        left_indices   = idx((250, 410))

        # ===== 거리 유효성 검증 =====
        ranges_valid = valid_ranges(ranges)

        # ===== 각 영역 최소값 =====
        right_min_val  = zone_min(ranges_valid, right_indices)
        center_min_val = zone_min(ranges_valid, center_indices)
        left_min_val   = zone_min(ranges_valid, left_indices)

        # ===== 최소값 인덱스 =====
        right_min_idx  = zone_argmin(ranges_valid, right_indices)
        left_min_idx   = zone_argmin(ranges_valid, left_indices)

        # ===== 기본 설정 =====
        steer_pos = STEER_CENTER
        state = "🟢 Straight"

        # ===== 전방 하위 영역 =====
        front_left_indices  = idx((1610, 1860))
        front_right_indices = idx((0, 250))
        front_left_min  = zone_min(ranges_valid, front_left_indices)
        front_right_min = zone_min(ranges_valid, front_right_indices)

        # ===== 주행 제어 로직 =====
        if (right_min_val < 0.4 or center_min_val < 0.60 or left_min_val < 0.4):
            # 전방(center) 장애물 가까움
            if center_min_val < 0.6:
                if front_left_min > front_right_min:
                    steer_pos = 300
                    state = "⬅️ Hard Left"
                elif front_left_min < front_right_min:
                    steer_pos = 1300
                    state = "➡️ Hard Right"

            elif left_min_val < right_min_val and left_min_val < 0.4:
                offset = (410 - left_min_idx) * STEER_STEP
                steer_pos = STEER_CENTER - offset
                state = "↩️ Turn left"

            elif right_min_val < left_min_val and right_min_val < 0.4:
                offset = (right_min_idx - 1449) * STEER_STEP
                steer_pos = STEER_CENTER + offset
                state = "↪️ Turn right"

            else:
                steer_pos = STEER_CENTER
                state = "🟢 Straight Drive"

        else:
            steer_pos = STEER_CENTER
            state = "🟢 Straight Drive"

        # ===== 조향 한계 제한 =====
        steer_pos = clamp_steer(steer_pos)

        # ===== 발행 =====
        self.angle_pub.publish(Float32(data=steer_pos * 1.0))

        # ✅ 추가 2: finish 모드용 방향 신호
        if front_left_min < front_right_min:
            self.finish_dir_pub.publish(Int32(data=1))   # 왼쪽이 더 가까움
        elif front_right_min < front_left_min:
            self.finish_dir_pub.publish(Int32(data=0))   # 오른쪽이 더 가까움


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriveNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
