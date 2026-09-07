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
        'console_scripts': ['bringup = dyna_lidar_bringup.bringup_main:main',
            'motor_node = dyna_lidar_bringup.motor_node:main',
            'odom_node = dyna_lidar_bringup.odom_from_dxl:main',
            'dyna_cmd_vel_node = dyna_lidar_bringup.dyna_cmd_vel_node:main',
            'quick_vel_test = dyna_lidar_bringup.quick_vel_test:main',
            'lidar_array_node = dyna_lidar_bringup.lidar_array_node:main',
            'scan_to_cmdvel = scan_to_cmdvel:main',
            'main_driver = wooak.main_drive_node:main',
            'm2ct = wooak.m2ct:main',
            'stopline_find = wooak.stopline_find:main',
            'lidar_array.py = wooak.lidar_array:main',
            'lidar_drive.py = wooak.lidar_drive:main',
            'maze_drive.py = wooak.maze_drive:main',
           
           
        ],
    },
)
