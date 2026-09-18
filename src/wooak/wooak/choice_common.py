#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# choice.py / choice1.py / choice2.py 공통 노드
# 세 파일은 토픽명, 전방 구간, 최소 유효거리, 판단 거리만 달랐음

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray

from wooak.lidar_common import valid_ranges, zone_min


class LidarObstacleCheckNode(Node):
    def __init__(self, topic, center_indices, min_r, near_max, tag=''):
        # 원래 세 노드 이름이 모두 같아서 동시에 띄우면 이름 충돌 → tag 로 구분
        super().__init__('lidar_obstacle_check_node' + tag)
        self.center_indices = center_indices
        self.min_r = min_r          # 이 값 이하는 노이즈로 보고 무시
        self.near_max = near_max    # 이 거리 안에 있으면 정면 장애물
        self.tag = tag
        self.sub = self.create_subscription(Float32MultiArray, '/lidar_array', self.lidar_callback, 10)
        self.pub = self.create_publisher(Int32, topic, 10)
        self.get_logger().info(f"✅ LidarObstacleCheckNode started ({topic})")

    def lidar_callback(self, msg):
        if not msg.data:
            return

        ranges = msg.data
        if len(ranges) < 1800:
            return

        # ===== 거리 유효성 보정 =====
        ranges_valid = valid_ranges(ranges, self.min_r)

        # ===== 최소 거리 계산 =====
        center_min = zone_min(ranges_valid, self.center_indices)

        # ===== 단순 판단 =====
        if center_min < self.near_max and center_min > 0.2:
            result = 1  # 정면 장애물 → 왼쪽 차선으로
            state = f"🚧 Front obstacle{self.tag} ({center_min:.2f} m) → output=1"
        else:
            result = 0  # 오른쪽 장애물 → 오른쪽 차선으로
            state = f"🚧 No obstacle{self.tag} ({center_min:.2f} m) → output=0"

        # ===== 퍼블리시 =====
        self.pub.publish(Int32(data=result))
        self.get_logger().info(state)


def run(**kwargs):
    rclpy.init()
    node = LidarObstacleCheckNode(**kwargs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
