#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 양쪽 차선 보고 주행하는 경우
# 공통 부분(카메라 노출, 구독, ROI 시각화, 대표직선 계산)은 lane_common.py

import rclpy
from sensor_msgs.msg import Image
import cv2
import math
# 다른 부분에서 저장된 델타값 사용할 수 있도록
from std_msgs.msg import Float32

from wooak.lane_common import LaneNodeBase, calculate_slope


class LaneDetectionNode(LaneNodeBase):
    def __init__(self):
        super().__init__('lane_detection_node', '/lane_detection/processed_image',
                         roi_center_width=640)  # 중앙 640픽셀(=전체) 사용

        ### 성민 수정 부분2
        self.blur_pub = self.create_publisher(Image, '/debug/blur', 10)
        self.edge_pub = self.create_publisher(Image, '/debug/edge', 10)
        ### 여기까지

        # 다른 부분에서 저장된 델타값 사용할 수 있도록
        self.Delta_pub = self.create_publisher(Float32, '/lane_detection/Delta_x', 10)

        # x_left_top, x_right_top if문 밖에서 사용하기 위해
        self.prev_x_left_top = 100
        self.prev_x_right_top = 540

    def lane_detect(self, image):
        # 중앙 ROI 설정
        roi_start_x = (self.WIDTH - self.roi_center_width) // 2
        roi_end_x = roi_start_x + self.roi_center_width
        roi_img = image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x]

        gray_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray_mask = cv2.adaptiveThreshold(gray_img, 255,
                                          cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv2.THRESH_BINARY, 45, -3)

        blur_mask_y = cv2.GaussianBlur(gray_mask, (5, 5), 0)
        edge_img_y = cv2.Canny(blur_mask_y, 100, 140)

        try:
            self.blur_pub.publish(self.bridge.cv2_to_imgmsg(blur_mask_y, "mono8"))
            self.edge_pub.publish(self.bridge.cv2_to_imgmsg(edge_img_y, "mono8"))
        except Exception as e:
            self.get_logger().error(f"Error publishing debug images: {e}")

        # Hough 변환 노란차선
        all_lines = cv2.HoughLinesP(edge_img_y, 1, math.pi/180, 30, 30, 15)
        if all_lines is None:
            # 차선 검출에 관계없이 ROI 박스/텍스트가 나타나도록함
            self.visualize_lanes(image, roi_img, roi_start_x, [])
            return False

        margin = 1
        center_x = self.roi_center_width // 2
        left_min, left_max = 0, center_x - margin
        right_min, right_max = center_x + margin, self.roi_center_width - 1

        # 선분 필터링 및 분류
        left_lines, right_lines = [], []
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = float(y2-y1) / float(x2-x1)
            if abs(slope) <= 0.4:  # 수평에 가까운선(기울기 작음)은 차선이 아닐 가능성 커서 제거
                continue

            left_region = (left_min <= x1 <= left_max)
            right_region = (right_min <= x2 <= right_max)

            # 너무 짧은(가로 폭 5픽셀 미만) 선분은 제외
            gap = abs(x2 - x1)

            if left_region and gap >= 5:
                left_lines.append(line[0])
            if right_region and gap >= 5:
                right_lines.append(line[0])

        # 대표직선 계산
        m_left = calculate_slope(left_lines)
        m_right = calculate_slope(right_lines)

        # Roi의 상단과 차선의 교점에서 x좌표 구하기 (ROI 좌표를 전체 이미지 좌표로 변환)
        if m_left[0] != 0.0:
            x_left_top = int((0 - m_left[1]) / m_left[0]) + roi_start_x
        else:
            x_left_top = self.prev_x_left_top
        if m_right[0] != 0.0:
            x_right_top = int((0 - m_right[1]) / m_right[0]) + roi_start_x
        else:
            x_right_top = self.prev_x_right_top

        # 한쪽만 보이면 차선 폭(380px)으로 반대쪽 추정
        if m_left[0] != 0.0 and m_right[0] == 0.0:
            x_right_top = x_left_top + 380
        elif m_left[0] == 0.0 and m_right[0] != 0.0:
            x_left_top = x_right_top - 380

        self.prev_x_left_top = x_left_top
        self.prev_x_right_top = x_right_top

        # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치
        x_midpoint = (x_left_top + x_right_top) // 2
        # 화면의 중앙값 = 카메라 위치
        # ※ roi_start_x 가 0 일 때만 화면 중앙과 같음 (보고서 참고, 튜닝값이라 유지)
        View_Center = (roi_start_x + self.roi_center_width) // 2
        # delta_x
        Delta_x_raw = float(x_midpoint - View_Center)
        # delta_x 너무 작은 값은 0으로 처리
        Delta_x = 0.0 if abs(Delta_x_raw) < 10 else Delta_x_raw

        # 다른 부분에서 저장된 델타값 사용할 수 있도록
        self.Delta_pub.publish(Float32(data=Delta_x))
        # 노란색 네모(x_midpoint)
        cv2.rectangle(image, (x_midpoint-5, self.roi_start_y-5), (x_midpoint+5, self.roi_start_y+5), (0, 255, 255), 4)
        # 갈색 네모(View_Center)
        cv2.rectangle(image, (View_Center-5, self.roi_start_y-5), (View_Center+5, self.roi_start_y+5), (19, 69, 139), 4)
        # delta x 표시
        cv2.putText(image, f"Delta_x={int(Delta_x)}",
                    (roi_end_x - 150, self.roi_start_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        # x_left_top 표시 (빨간색 - 왼쪽 차선과 같은 색)
        cv2.circle(image, (x_left_top, self.roi_start_y), 8, (0, 0, 255), -1)
        cv2.putText(image, f"L:{x_left_top}",
                    (x_left_top - 30, self.roi_start_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # x_right_top 표시 (초록색 - 오른쪽 차선과 같은 색)
        cv2.circle(image, (x_right_top, self.roi_start_y), 8, (0, 255, 0), -1)
        cv2.putText(image, f"R:{x_right_top}",
                    (x_right_top + 10, self.roi_start_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 시각화 (왼쪽 빨강, 오른쪽 초록)
        self.visualize_lanes(image, roi_img, roi_start_x, [(m_left, (0, 0, 255)), (m_right, (0, 255, 0))])
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
