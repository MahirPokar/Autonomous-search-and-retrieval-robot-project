from setuptools import find_packages, setup

package_name = 'color_object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/yolov5s.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mscrobotics2425laptop41',
    maintainer_email='mscrobotics2425laptop41@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_object = color_object_detector.detect_object:main',
            'get_target_position = color_object_detector.get_target_position:main',
        ],
    },
)
