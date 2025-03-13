from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'color_object_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/best_copy.pt']),  # 确保安装模型
        ('share/' + package_name + '/resource', glob('resource/*')),  # 资源文件支持（可选）
    ],
    install_requires=['setuptools', 'ament_index_python'],  # 需要 ament_index_python
    zip_safe=True,
    maintainer='mscrobotics2425laptop41',
    maintainer_email='mscrobotics2425laptop41@todo.todo',
    description='ROS2 package for object detection using YOLOv8 and RealSense camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_object = color_object_detector.detect_object:main',
            'get_target_position = color_object_detector.get_target_position:main',
        ],
    },
)
