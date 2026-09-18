#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === Launch args ===
    serial_port      = LaunchConfiguration('serial_port',      default='/dev/ttyUSB0')
    serial_baudrate  = LaunchConfiguration('serial_baudrate',  default='256000')  # A2M12는 115200/256000 둘 다 시도 가능
    frame_id         = LaunchConfiguration('frame_id',         default='laser')
    inverted         = LaunchConfiguration('inverted',         default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode',        default='Sensitivity')

    # 라이다 센서의 base_link 상대 위치(필요 시 수정)
    laser_x = LaunchConfiguration('laser_x', default='0.20')
    laser_y = LaunchConfiguration('laser_y', default='0.00')
    laser_z = LaunchConfiguration('laser_z', default='0.25')
    laser_roll  = LaunchConfiguration('laser_roll',  default='0.0')
    laser_pitch = LaunchConfiguration('laser_pitch', default='0.0')
    laser_yaw   = LaunchConfiguration('laser_yaw',   default='0.0')

    # 오도메 스크립트 경로 (네 odom_from_dxl.py 위치로 바꿔도 됨)
    odom_script = LaunchConfiguration(
        'odom_script',
        default=str(PathJoinSubstitution([
            '/', 'home', 'kangsanmaru', 'ros2_ws', 'src',
            'dyna_lidar_bringup', 'scripts', 'odom_from_dxl.py'   # ← 여기!
        ]))
    )

    # SLAM 파라미터 파일 (slam_toolbox 기본 설정)
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=str(PathJoinSubstitution([
            get_package_share_directory('slam_toolbox'),
            'config', 'mapper_params_online_async.yaml'
        ]))
    )

    # (옵션) RViz 설정 파일
    rviz_config = LaunchConfiguration('rviz_config', default='')

    # === Nodes ===

    # 1) RPLIDAR 드라이버
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'serial',
            'serial_port':      serial_port,
            'serial_baudrate':  serial_baudrate,
            'frame_id':         frame_id,
            'inverted':         inverted,
            'angle_compensate': angle_compensate,
            'scan_mode':        scan_mode,
        }]
    )

    # 2) laser 프레임을 base_link에 고정 (설치 위치 반영)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_link_tf',
        arguments=[
            laser_x, laser_y, laser_z,
            laser_roll, laser_pitch, laser_yaw,
            'base_link', frame_id
        ],
        output='screen'
    )

    # 3) 오도메 노드 (DynamixelSDK 기반 Python 스크립트 직접 실행)
    odom_proc = ExecuteProcess(
        cmd=['python3', odom_script],
        output='screen'
    )

    # 4) SLAM Toolbox (online async)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(PathJoinSubstitution([
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ]))
        ),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    # 5) (옵션) RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=None  # 빈 값이면 그냥 기본 RViz 뜨거나, 경로 주면 해당 config로 뜸
    )

    # === Declare args ===
    declares = [
        DeclareLaunchArgument('serial_port',      default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate',  default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id',         default_value=frame_id),
        DeclareLaunchArgument('inverted',         default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode',        default_value=scan_mode),

        DeclareLaunchArgument('laser_x', default_value=laser_x),
        DeclareLaunchArgument('laser_y', default_value=laser_y),
        DeclareLaunchArgument('laser_z', default_value=laser_z),
        DeclareLaunchArgument('laser_roll',  default_value=laser_roll),
        DeclareLaunchArgument('laser_pitch', default_value=laser_pitch),
        DeclareLaunchArgument('laser_yaw',   default_value=laser_yaw),

        DeclareLaunchArgument('odom_script',     default_value=odom_script),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file),
        DeclareLaunchArgument('rviz_config',      default_value=rviz_config),
    ]

    return LaunchDescription(declares + [
        rplidar_node,
        static_tf,
        odom_proc,
        slam_launch,
        rviz_node,
    ])
