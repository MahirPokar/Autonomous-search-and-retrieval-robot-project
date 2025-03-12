from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'obj_gsp'  # 保持你的包名不变

setup(
    name=package_name,
    version='0.0.1',  # 版本号可以更新
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/best_copy.pt']),  #
        ('share/' + package_name + '/resource', glob('resource/*')),  # 资源文件支持（可选）
	(os.path.join('share', package_name, 'launch'), ['launch/leolaunch1.py']),
    ],
    install_requires=['setuptools', 'ament_index_python'],  # 需要 ament_index_python
    zip_safe=True,
    maintainer='mscrobotics2425laptop29',
    maintainer_email='mscrobotics2425laptop29@todo.todo',
    description='ROS 2 package for object detection and manipulator control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_control = obj_gsp.manipulator_control:main',
            'detect_object = obj_gsp.detect_object:main',  # 添加目标检测脚本
        ],
    },
)

