#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 왼쪽차선만 보고 주행하는경우
# 공통 부분(카메라 노출, 구독, ROI 시각화, 대표직선 계산)은 lane_common.py

import rclpy
import cv2
import numpy as np
import math
# 다른 부분에서 저장된 델타값 사용할 수 있도록
from std_msgs.msg import Float32

from wooak.lane_common import LaneNodeBase, calculate_slope


class LaneDetectionNode(LaneNodeBase):
    def __init__(self):
        super().__init__('left_lane_detection_node', '/left_lane_detection/processed_image',
                         roi_center_width=200)

        # 다른 부분에서 저장된 델타값 사용할 수 있도록
        self.Left_Delta_pub = self.create_publisher(Float32, '/Left_lane_detection/Left_Delta_x', 10)

        # x_left_top if문 밖에서 사용하기 위해
        self.prev_x_left_top = 100

    def lane_detect(self, image):
        # 왼쪽 ROI 설정
        roi_start_x = 50  # 50, ROI 시작범위 설정
        roi_end_x = self.WIDTH // 2   # 전체 가로의 절반까지만
        roi_img = image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x]

        # 노란색 강조 (R+G-1.6B)
        b, g, r = cv2.split(roi_img)
        yellow_enhanced = cv2.add(r, g)
        yellow_enhanced = cv2.subtract(yellow_enhanced, cv2.multiply(b, 1.6))
        yellow_enhanced = np.clip(yellow_enhanced, 0, 255).astype(np.uint8)
        blur_mask_y = cv2.GaussianBlur(yellow_enhanced, (5, 5), 0)
        edge_img_y = cv2.Canny(blur_mask_y, 100, 140)

        # Hough 변환 노란차선
        all_lines = cv2.HoughLinesP(edge_img_y, 1, math.pi/180, 30, 30, 15)
        if all_lines is None:
            # 차선 검출에 관계없이 ROI 박스/텍스트가 나타나도록함
            self.visualize_lanes(image, roi_img, roi_start_x, [])
            return False

        # 선분 필터링 및 분류
        left_lines = []
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = float(y2-y1) / float(x2-x1)
            if abs(slope) <= 0.3:  # 수평에 가까운선(기울기 작음)은 차선이 아닐 가능성 커서 제거
                continue

            # ROI 영역 안에서 왼쪽 차선 분류
            if (x1 < roi_end_x and x2 < roi_end_x):
                left_lines.append(line[0])

        # 대표직선 계산
        m_left = calculate_slope(left_lines)

        # Roi의 상단과 차선의 교점에서 x좌표 구하기
        if m_left[0] != 0.0:
            x_left_top = int((0 - m_left[1]) / m_left[0]) + roi_start_x
        else:
            x_left_top = self.prev_x_left_top

        self.prev_x_left_top = x_left_top

        # 왼쪽 차선의 위치와 오른쪽 차선(= 왼쪽 + 420)의 중간 위치
        x_midpoint = (x_left_top + (x_left_top+420)) // 2
        # 화면의 중앙값 = 카메라 위치
        View_Center = roi_end_x
        # delta_x
        Left_Delta_x_raw = float(x_midpoint - View_Center)
        # delta_x 너무 작은 값은 0으로 처리
        Left_Delta_x = 0.0 if abs(Left_Delta_x_raw) < 7 else Left_Delta_x_raw  # 10
        # 다른 부분에서 저장된 델타값 사용할 수 있도록
        self.Left_Delta_pub.publish(Float32(data=Left_Delta_x))

        # 노란색 네모(x_midpoint)
        cv2.rectangle(image, (x_midpoint-5, self.roi_start_y-5), (x_midpoint+5, self.roi_start_y+5), (0, 255, 255), 4)
        # 갈색 네모(View_Center)
        cv2.rectangle(image, (View_Center-5, self.roi_start_y-5), (View_Center+5, self.roi_start_y+5), (19, 69, 139), 4)
        # delta x 표시
        cv2.putText(image, f"Delta_x={int(Left_Delta_x)}",
                    (roi_end_x - 150, self.roi_start_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        # x_left_top 표시 (빨간색 - 왼쪽 차선과 같은 색)
        cv2.circle(image, (x_left_top, self.roi_start_y), 8, (0, 0, 255), -1)
        cv2.putText(image, f"L:{x_left_top}",
                    (x_left_top - 30, self.roi_start_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 시각화
        self.visualize_lanes(image, roi_img, roi_start_x, [(m_left, (0, 0, 255))])
        return True


def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()

    try:
        rclpy.spin(lane_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        lane_detection_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
