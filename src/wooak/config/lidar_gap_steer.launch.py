from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    u2d2_port   = LaunchConfiguration('u2d2_port')
    lidar_port  = LaunchConfiguration('lidar_port')
    lidar_baud  = LaunchConfiguration('lidar_baud')
    frame_id    = LaunchConfiguration('frame_id')

    pkg_share = get_package_share_directory('dyna_lidar_bringup')
    params = os.path.join(pkg_share, 'config', 'lidar_gap_steer.yaml')

    # RPLIDAR 포함 실행 (네가 평소 쓰던 launch 그대로 포함)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'),
                         'launch', 'view_rplidar_a2m12_launch.py')
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id': frame_id
        }.items()
    )

    ctrl_node = Node(
        package='dyna_lidar_bringup',
        executable='lidar_gap_steer',
        name='lidar_gap_steer',
        output='screen',
        parameters=[params, {'device': u2d2_port}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('u2d2_port',  default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='256000'),
        DeclareLaunchArgument('frame_id',   default_value='laser'),
        rplidar_launch,
        ctrl_node
    ])
