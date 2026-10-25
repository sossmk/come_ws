from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
                'auto_white_balance': False,    # ✅ 추가!
                'brightness': -1                # usb_cam 기본값 50이 화면을 하얗게 띄움, -1 = 카메라 값 유지
            }]
        ),

        # usb_cam에 해당 파라미터가 없어 v4l2-ctl로 직접 설정 (역광 보정이 유광 시트 반사를 키움)
        ExecuteProcess(
            cmd=['v4l2-ctl', '-d', '/dev/video0', '-c', 'backlight_compensation=0'],
            output='screen'
        ),

        # 2️⃣ 차선 인식 파이썬 노드 (직접 실행)
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/lane_detection_node.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/stopline_find.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/left_lane_detection_node.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                'python3',
                '/home/kkk/come_ws/src/wooak/wooak/right_lane_detection_node.py',
                '--ros-args',
                '--remap', '/camera/image_raw:=/image_raw'
            ],
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

