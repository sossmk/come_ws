#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
대회용 통합 launch 파일.

기존에는 camall.launch.py / lidarall.launch.py 두 개로 나뉘어 있었고,
m2ct(모터 구동), choice/choice1/choice2(라이다 장애물 판단), main_driver(주행 FSM)는
어느 launch 파일에도 포함되어 있지 않아 터미널을 여러 개 열어 수동으로 띄워야 했다.
이 파일 하나로 카메라→비전→라이다→FSM→모터까지 전 구간을 한 번에 띄운다.

실행 방법:
    ros2 launch wooak competition.launch.py
    ros2 launch wooak competition.launch.py start_mode:=1 camera_fps:=15.0

주의:
    - rqt_image_view 같은 GUI 뷰어는 포함하지 않았다 (CPU/GPU 낭비 + 대회 중 불필요).
      디버깅이 필요하면 별도로 debug_view.launch.py 를 사용할 것.
    - 각 비전 노드(/image_raw 구독)는 BEST_EFFORT + depth=1 QoS를 쓰므로
      usb_cam 쪽 QoS와 상관없이 매칭된다 (RELIABLE 발행자 ↔ BEST_EFFORT 구독자는 호환).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_mode_arg = DeclareLaunchArgument(
        'start_mode', default_value='1',
        description='main_drive_node 시작 모드 (1~9). 출발 위치가 바뀌는 미션에 맞춰 조정.'
    )
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps', default_value='15.0',
        description='USB 카메라 프레임레이트. 비전 노드 4개가 동시에 같은 프레임을 처리하므로 '
                    'Jetson Orin Nano에서는 15fps 권장 (필요시 올려서 테스트).'
    )

    start_mode = LaunchConfiguration('start_mode')
    camera_fps = LaunchConfiguration('camera_fps')

    # 1) 카메라
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': camera_fps,
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
            'io_method': 'mmap',
            'autoexposure': False,
            'exposure': 60,
            'auto_white_balance': False,
        }],
    )

    # 2) 라이다
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baudrate': 512000,
            'frame_id': 'laser',
            'resolution_fixed': True,
            'angle_min': -180.0,
            'angle_max': 180.0,
            'range_min': 0.05,
            'range_max': 12.0,
            'ignore_array': '',
            'reversion': False,
            'auto_reconnect': True,
            'single_channel': False,
        }],
    )

    # 3) 비전 노드 4개 (/image_raw 구독) — 전부 wooak 콘솔 스크립트로 등록된 실제 ROS 노드.
    #    (이전에는 ExecuteProcess로 python3 스크립트를 직접 띄웠는데, 그러면 launch가
    #     프로세스를 제대로 관리/재시작 못하고 --ros-args remap도 매번 손으로 맞춰야 했음)
    vision_nodes = [
        Node(package='wooak', executable='lane_detection_node', name='lane_detection_node', output='screen'),
        Node(package='wooak', executable='left_lane_detection_node', name='left_lane_detection_node', output='screen'),
        Node(package='wooak', executable='right_lane_detection_node', name='right_lane_detection_node', output='screen'),
        Node(package='wooak', executable='stopline_find', name='stopline_find', output='screen'),
    ]

    # 4) 라이다 2차 처리 + 장애물/미로 판단 노드들
    lidar_logic_nodes = [
        Node(package='wooak', executable='lidar_array_node', name='lidar_array_node', output='screen'),
        Node(package='wooak', executable='lidar_drive', name='lidar_drive_node', output='screen'),
        Node(package='wooak', executable='maze_drive', name='maze_drive_node', output='screen'),
        Node(package='wooak', executable='obstacle_check', name='obstacle_check_node', output='screen'),
        Node(package='wooak', executable='obstacle_check1', name='obstacle_check_node1', output='screen'),
        Node(package='wooak', executable='obstacle_check2', name='obstacle_check_node2', output='screen'),
    ]

    # 5) 주행 FSM (두뇌)
    main_drive_node = Node(
        package='wooak',
        executable='main_driver',
        name='main_drive_node',
        output='screen',
        parameters=[{'start_mode': start_mode}],
    )

    # 6) 모터 구동 (다이나믹셀 실제 액추에이터) — 이게 빠지면 차가 안 움직임.
    motor_driver_node = Node(
        package='wooak',
        executable='m2ct',
        name='motor_driver_node',
        output='screen',
    )

    return LaunchDescription([
        start_mode_arg,
        camera_fps_arg,
        usb_cam_node,
        ydlidar_node,
        *vision_nodes,
        *lidar_logic_nodes,
        main_drive_node,
        motor_driver_node,
    ])
