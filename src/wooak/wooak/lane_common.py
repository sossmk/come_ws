#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 차선 인식 노드 3개(양쪽/왼쪽/오른쪽)가 공통으로 쓰던 코드 모음
# 각 노드에는 자기만의 lane_detect() 만 남김

import os

import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# 카메라 노출값 조정을 위한 세팅
# auto_exposure: 1 = 수동, 3 = 자동(aperture priority)
def cam_exposure(value=60, auto_exposure=1):
    os.system(f'v4l2-ctl -d /dev/video0 -c auto_exposure={auto_exposure}')
    os.system('v4l2-ctl -d /dev/video0 -c exposure_dynamic_framerate=0')
    os.system(f'v4l2-ctl -d /dev/video0 -c exposure_time_absolute={value}')
    print(f"[INFO] Camera exposure set to manual ({value})")


# 선분들의 평균 기울기/절편 → 대표직선 [m, b]  (검출 없으면 [0.0, 0.0])
def calculate_slope(lines):
    if len(lines) == 0:
        return [0.0, 0.0]

    x_sum = y_sum = m_sum = 0.0
    for line in lines:
        x1, y1, x2, y2 = line
        x_sum += x1 + x2
        y_sum += y1 + y2
        if x2 != x1:
            m_sum += float(y2-y1)/float(x2-x1)

    size = len(lines)
    x_avg = x_sum / (size*2)
    y_avg = y_sum / (size*2)
    m = m_sum / size
    b = y_avg - m * x_avg
    return [m, b]


class LaneNodeBase(Node):
    """/image_raw 구독, 해상도 자동 감지, ROI 시각화/결과 발행 공통 부분"""

    def __init__(self, node_name, processed_topic, roi_center_width, exposure=60, auto_exposure=1):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} Started!')

        # 카메라 노출 고정 (ROS 시작 시 한 번만 실행)
        cam_exposure(exposure, auto_exposure)
        self.get_logger().info(f'Camera exposure locked ({exposure}, auto_exposure={auto_exposure})')

        # OpenCV Bridge 초기화
        self.bridge = CvBridge()

        # 구독자/발행자
        self.image_subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.processed_image_pub = self.create_publisher(Image, processed_topic, 10)

        # 파라미터
        self.WIDTH = 640
        self.HEIGHT = 480

        # ROI 설정
        self.roi_center_width = roi_center_width
        self.set_roi()

    def set_roi(self):
        self.roi_start_y = int(self.HEIGHT * 0.6)
        self.roi_end_y = int(self.HEIGHT * 0.8)
        self.roi_height = self.roi_end_y - self.roi_start_y
        self.L_ROW = int(self.roi_height * 0.25)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 이미지 크기 자동 감지
            actual_height, actual_width = cv_image.shape[:2]
            if actual_width != self.WIDTH or actual_height != self.HEIGHT:
                self.WIDTH = actual_width
                self.HEIGHT = actual_height
                self.roi_center_width = min(800, self.WIDTH)  # 화면보다 작게
                self.set_roi()
                self.get_logger().info(f'Resolution: {self.WIDTH}x{self.HEIGHT}, ROI width: {self.roi_center_width}')

            # 차선 검출
            self.lane_detect(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def lane_detect(self, image):
        raise NotImplementedError

    def visualize_lanes(self, image, roi_img, roi_start_x, lanes):
        """lanes: [(m_b, BGR색), ...] 대표직선들을 ROI 에 그려서 원본에 합성"""
        line_draw_img = roi_img.copy()

        for m, color in lanes:
            if m[0] != 0.0:
                x1 = int((0.0 - m[1]) / m[0])
                x2 = int((self.roi_height - m[1]) / m[0])
                cv2.line(line_draw_img, (x1, 0), (x2, self.roi_height), color, 3)

        # ROI 영역을 원본 이미지에 합성
        roi_width = roi_img.shape[1]
        roi_end_x = roi_start_x + roi_width
        image[self.roi_start_y:self.roi_end_y, roi_start_x:roi_end_x] = line_draw_img

        # ROI 영역 테두리 표시 (녹색 사각형)
        cv2.rectangle(image,
                      (roi_start_x, self.roi_start_y),
                      (roi_end_x, self.roi_end_y),
                      (0, 255, 0), 2)

        # ROI 정보 텍스트 표시
        roi_text = f"ROI: {roi_width}x{self.roi_height}"
        cv2.putText(image, roi_text,
                    (roi_start_x, self.roi_start_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        self.publish_result_image(image)

    def publish_result_image(self, image):
        if self.processed_image_pub.get_subscription_count() > 0:
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.processed_image_pub.publish(processed_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {str(e)}')
