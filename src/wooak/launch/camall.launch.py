from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1️⃣ USB 카메라 노드
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,              # ✅ 추가!
                'pixel_format': 'mjpeg2rgb',    # ✅ 추가!
                'image_width': 640,             # ✅ 추가!
                'image_height': 480,            # ✅ 추가!
                'io_method': 'mmap',            # ✅ 추가!
                'autoexposure': False,          # ✅ 추가!
                'exposure': 60,                 # ✅ 추가!
                'auto_white_balance': False     # ✅ 추가!
            }]
        ),

        # 2️⃣ 차선 인식 / 정지선 노드
        #    (원래 /home/kkk/... 절대경로를 python3 로 직접 실행 → 다른 PC 에서 안 돌아감)
        Node(
            package='wooak',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen'
        ),
        
        Node(
            package='wooak',
            executable='stopline_find',
            name='stopline_find',
            output='screen'
        ),

        Node(
            package='wooak',
            executable='left_lane_detection_node',
            name='left_lane_detection_node',
            output='screen'
        ),
        
        Node(
            package='wooak',
            executable='right_lane_detection_node',
            name='right_lane_detection_node',
            output='screen'
        ),
        
        

        # 3️⃣ 이미지 뷰어
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            output='screen'
        ),

    ])

