#!/bin/bash
# run_ros2.sh
# ROS2 전체 장애물 감지 시스템 실행

echo "🚀 딜리봇 ROS2 장애물 감지 시스템 시작"
echo ""

# ROS 환경 설정
source /opt/ros/jazzy/setup.bash
source ~/delivery_bot/sbc/ros2/install/setup.bash

# ROS2 런치 실행
ros2 launch delivery_sensor full_system.launch.py
