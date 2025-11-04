#!/usr/bin/env python3
"""
full_system.launch.py

배달 로봇 전체 시스템 런치 파일
- YDLidar X4 Pro
- 초음파 센서 (전방, 좌, 우)
- 장애물 감지 (즉시 발행)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """런치 설명 생성"""
    
    # 파라미터 선언
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ydlidar',
        description='YDLidar 포트'
    )
    
    # ========================================
    # 1. YDLidar 노드
    # ========================================
    
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ]),
        launch_arguments={
            'port': LaunchConfiguration('lidar_port'),
            'baudrate': '128000',  # X4 Pro
            'frame_id': 'laser_frame',
            'lidar_type': '1',  # TYPE_TRIANGLE
            'min_range': '0.12',
            'max_range': '10.0',
            'scan_frequency': '7.0',
        }.items()
    )
    
    # ========================================
    # 2. 초음파 센서 노드
    # ========================================
    
    ultrasonic_node = Node(
        package='sensor',
        executable='ultrasonic_node',
        name='ultrasonic_node',
        output='screen',
        parameters=[{
            'update_rate': 20.0,
        }]
    )
    
    # ========================================
    # 3. 장애물 감지 노드 (즉시 발행)
    # ========================================
    
    obstacle_detector_node = Node(
        package='sensor',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
    )
    
    # ========================================
    # 런치 설명 반환
    # ========================================
    
    return LaunchDescription([
        # 파라미터
        lidar_port_arg,
        
        # 노드
        ydlidar_launch,
        ultrasonic_node,
        obstacle_detector_node,
    ])
