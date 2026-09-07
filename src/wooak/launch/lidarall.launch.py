from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # =============================
        # 1. YDLIDAR ROS2 드라이버 노드
        # =============================
        Node(
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
                'single_channel': False
            }]
        ),

        # =============================
        # 2. Lidar Array Node
        # =============================
        Node(
            package='wooak',
            executable='lidar_array.py',
            name='lidar_array',
            output='screen'
        ),

        # =============================
        # 3. Lidar Drive Node
        # =============================
        Node(
            package='wooak',
            executable='lidar_drive.py',
            name='lidar_drive',
            output='screen'
        ),

        # =============================
        # 4. Maze Drive Node
        # =============================
        Node(
            package='wooak',
            executable='maze_drive.py',
            name='maze_drive',
            output='screen'
        )
    ])

