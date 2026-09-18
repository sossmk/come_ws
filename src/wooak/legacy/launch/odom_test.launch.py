# dyna_lidar_bringup/launch/full_bringup.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node
import os

def generate_launch_description():
    rviz_config_path = os.path.expanduser('~/ros2_ws/rviz/default_view.rviz')

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_e2085ebc65c7654c9a900fe6a5f7c902-if00-port0',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'scan_mode': 'Sensitivity'
        }],
    )

    odom = Node(
        package='dyna_lidar_bringup',
        executable='odom_node',
        name='odom_from_dxl',
        parameters=[{
            'device': '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAA0974-if00-port0',
            'baudrate': 57600,
            'left_id': 7,
            'right_id': 9,
            'wheel_radius': 0.0325,
            'wheel_base': 0.24,
        }],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0.10', '0.0', '0.12', '0.0', '0.0', '0.0', '1.0', 'base_link', 'laser'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],  # ← 여기서 설정파일 지정
        output='screen'
    )

    delayed_nodes = TimerAction(period=2.0, actions=[odom, static_tf])
    delayed_rviz = TimerAction(period=5.0, actions=[rviz])

    debug = LogInfo(msg='[INFO] Bringup: LIDAR + ODOM + TF + RViz (auto-config)')

    return LaunchDescription([
        debug,
        lidar,
        delayed_nodes,
        delayed_rviz
    ])

