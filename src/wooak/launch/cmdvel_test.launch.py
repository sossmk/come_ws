from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ---------- Args ----------
    # LiDAR 관련
    lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1'
    )
    lidar_baud = DeclareLaunchArgument('lidar_baud', default_value='256000')

    # Dynamixel 관련
    u2d2_port = DeclareLaunchArgument(
        'u2d2_port',
        default_value='/dev/ttyUSB0'
    )
    dxl_baud  = DeclareLaunchArgument('dxl_baud',  default_value='57600')
    left_id   = DeclareLaunchArgument('left_id',   default_value='7')
    right_id  = DeclareLaunchArgument('right_id',  default_value='9')
    steer_id  = DeclareLaunchArgument('steer_id',  default_value='8')

    # ---------- LaunchConfigurations ----------
    lc_lidar_port = LaunchConfiguration('lidar_port')
    lc_lidar_baud = LaunchConfiguration('lidar_baud')
    lc_u2d2_port  = LaunchConfiguration('u2d2_port')
    lc_dxl_baud   = LaunchConfiguration('dxl_baud')
    lc_left_id    = LaunchConfiguration('left_id')
    lc_right_id   = LaunchConfiguration('right_id')
    lc_steer_id   = LaunchConfiguration('steer_id')

    # ---------- Nodes ----------
    # 1) LiDAR 노드
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': lc_lidar_port,
            'serial_baudrate': lc_lidar_baud,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        output='screen'
    )

    # 2) /scan → /cmd_vel 변환 노드
    scan_to_cmd_node = Node(
        package='dyna_lidar_bringup',
        executable='scan_to_cmdvel',
        name='scan_to_cmdvel',
        parameters=[{
            'fov_deg': 90.0,
            'center_width_deg': 30.0,
            'stop_range': 0.35,
            'cruise_speed': 0.20,
            'max_ang_vel': 1.2,
            'min_valid_ratio': 0.25,
            'smooth_alpha_lin': 0.4,
            'smooth_alpha_ang': 0.5,
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            'rate_hz': 20.0,
        }],
        output='screen'
    )

    # 3) Dynamixel 제어 노드
    cmdvel_node = Node(
        package='dyna_lidar_bringup',
        executable='dyna_cmd_vel_node',
        name='dyna_cmd_vel',
        parameters=[{
            'port': lc_u2d2_port,
            'baudrate': lc_dxl_baud,
            'left_id': lc_left_id,
            'right_id': lc_right_id,
            'steer_id': lc_steer_id,
            'wheel_radius': 0.0325,
            'wheel_base': 0.24,
            'steer_center': 3200,
            'steer_min': 2700,
            'steer_max': 3700,
            'cmd_timeout': 0.5,
            'vel_lsb_limit': 300,
            'max_steer_deg': 25.0,
        }],
        output='screen'
    )

    # LiDAR가 먼저 뜨고 → 2초 뒤 나머지 노드 실행
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[scan_to_cmd_node, cmdvel_node]
    )

    return LaunchDescription([
        LogInfo(msg='[Launch] CMDVEL TEST: LiDAR + CMD_VEL + Dynamixel starting...'),
        lidar_port, lidar_baud, u2d2_port, dxl_baud, left_id, right_id, steer_id,
        lidar_node,
        delayed_nodes
    ])

