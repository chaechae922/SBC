from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # 설정 파일
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='배달 로봇 센서 패키지 - LiDAR + 초음파 (즉시 발행)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_node = sensor.nodes.ultrasonic_node:main',
            'obstacle_detector_node = sensor.nodes.obstacle_detector_node:main',
        ],
    },
)
