from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'wooak'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            'main_driver = wooak.main_drive_node:main',
            'm2ct = wooak.m2ct:main',
            'stopline_find = wooak.stopline_find:main',
            'lane_detection_node = wooak.lane_detection_node:main',
            'left_lane_detection_node = wooak.left_lane_detection_node:main',
            'right_lane_detection_node = wooak.right_lane_detection_node:main',
            'lidar_array_node = wooak.lidar_array_node:main',
            'lidar_drive = wooak.lidar_drive:main',
            'maze_drive = wooak.maze_drive:main',
            'obstacle_check = wooak.choice:main',
            'obstacle_check1 = wooak.choice1:main',
            'obstacle_check2 = wooak.choice2:main',
        ],
    },
)
