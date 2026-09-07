from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1️⃣ LiDAR 데이터를 /lidar_array 로 변환
        Node(
            package='wooak',   # lidar_array_node 가 들어있는 패키지명
            executable='lidar_array_node',
            name='lidar_array_node',
            output='screen',
        ),

        # 2️⃣ 라이다 데이터 기반으로 조향/속도 명령 생성
        Node(
            package='wooak',   # lidar_drive_node 가 들어있는 패키지명
            executable='lidar_drive',
            name='lidar_drive',
            output='screen',
        ),

    ])

