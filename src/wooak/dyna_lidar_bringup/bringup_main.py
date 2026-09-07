from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_ld():
    ld = LaunchDescription()

    # ★ rplidar 런치파일을 절대경로로 직접 include
    lidar_launch = "/home/kangsanmaru/ros2_ws/src/rplidar_ros-ros2/launch/view_rplidar_a2m12_launch.py"

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '256000',
                'frame_id': 'laser',
            }.items()
        )
    )

    # 모터 노드
    ld.add_action(
        Node(
            package='dyna_lidar_bringup',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{
                'left_id': 7,
                'right_id': 9,
                'steer_id': 8,
                'port': '/dev/ttyUSB1',
                'baudrate': 57600
            }]
        )
    )
    return ld

def main():
    ls = LaunchService()
    ls.include_launch_description(generate_ld())
    ls.run()
