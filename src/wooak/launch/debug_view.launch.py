#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
디버깅 전용: rqt_image_view만 띄운다.
competition.launch.py를 먼저 실행한 뒤, 영상 확인이 필요할 때만 이 파일을 추가로 실행할 것.
(rqt_image_view는 GUI 렌더링 + 이미지 디코딩 비용이 꽤 커서 대회 본 주행에는 절대 같이 띄우지 말 것)

사용법:
    ros2 launch wooak competition.launch.py
    # 다른 터미널에서, 확인이 필요할 때만:
    ros2 launch wooak debug_view.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            output='screen',
        ),
    ])
