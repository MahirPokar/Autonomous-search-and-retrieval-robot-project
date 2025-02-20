from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'obj_gsp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mscrobotics2425laptop29',
    maintainer_email='mscrobotics2425laptop29@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_subscriber = obj_gsp.pose_subscriber:main',
            'manipulator_control = obj_gsp.manipulator_control:main',
            'publisher = obj_gsp.publisher:main',
        ],
    },
)
