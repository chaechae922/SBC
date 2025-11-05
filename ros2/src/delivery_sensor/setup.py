from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'delivery_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch 파일
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # 설정 파일 (YAML)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='배달 로봇 센서 패키지 - LiDAR + 초음파 기반 장애물 감지 (즉시 발행 + 시각화 + 로깅)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 초음파 센서 노드
            'ultrasonic_node = delivery_sensor.nodes.ultrasonic_node:main',
            # 장애물 감지 노드 (RViz2 + 로깅 포함)
            'obstacle_detector_node = delivery_sensor.nodes.obstacle_detector_node:main',
        ],
    },
)
