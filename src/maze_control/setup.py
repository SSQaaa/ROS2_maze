from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'maze_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    # 安装 ROS 2 资源文件
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # 安装 launch 文件
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        # 安装 Gazebo world 文件
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),

        (
        os.path.join('share', 'maze_control', 'rviz'),
        glob('rviz/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='ssq',
    maintainer_email='ssq@todo.todo',
    description='ROS2 maze robot simulation and control',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'open_loop_control = maze_control.open_loop_control:main',
            'point_control = maze_control.point_control:main',
            'lidar_process = maze_control.lidar_process:main',
            'obstacle_avoidance = maze_control.obstacle_avoidance:main',
            'wall_follow = maze_control.wall_follow:main',
            'wall_follow_optimized = maze_control.wall_follow_optimized:main',
        ],
    },
)