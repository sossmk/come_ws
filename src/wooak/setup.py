from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wooak'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'legacy', 'legacy.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkk',
    maintainer_email='kkk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ===== 메인 / 모터 =====
            'main_driver = wooak.main_drive_node:main',
            'm2ct = wooak.m2ct:main',
            'motor_control_node = wooak.motor_control_node:main',
            # ===== 카메라 =====
            'lane_detection_node = wooak.lane_detection_node:main',
            'left_lane_detection_node = wooak.left_lane_detection_node:main',
            'right_lane_detection_node = wooak.right_lane_detection_node:main',
            'stopline_find = wooak.stopline_find:main',
            # ===== 라이다 =====
            # (lidarall.launch.py 가 쓰는 '.py' 붙은 이름 그대로 유지)
            'lidar_array.py = wooak.lidar_array_node:main',   # 원래 wooak.lidar_array(없는 모듈)
            'lidar_array_node = wooak.lidar_array_node:main', # 원래 1440bin 레거시 버전을 가리켰음
            'lidar_drive.py = wooak.lidar_drive:main',
            'maze_drive.py = wooak.maze_drive:main',
            'choice = wooak.choice:main',
            'choice1 = wooak.choice1:main',
            'choice2 = wooak.choice2:main',
        ],
    },
)
