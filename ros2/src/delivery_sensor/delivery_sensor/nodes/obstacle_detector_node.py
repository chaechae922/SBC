#!/usr/bin/env python3
"""
obstacle_detector_node.py

장애물 감지 ROS2 노드 (즉시 발행 방식)
정량 지표 기반 위험 판단:
- LiDAR 3m 이하 → 위험
- 초음파 1m 이하 → 위험
- 3.5m 이상 → 안전

입력:
- /scan (sensor_msgs/LaserScan)
- /ultrasonic/front (sensor_msgs/Range)
- /ultrasonic/left (sensor_msgs/Range)
- /ultrasonic/right (sensor_msgs/Range)

출력:
- /obstacle_status (std_msgs/String)
  형식: "level=<int>,dir=<str>,front=<float>,left=<float>,right=<float>"

특징:
- 센서 데이터 수신 시 즉시 처리 및 발행 (지연 최소화)
- 타이머 없음 (이벤트 기반)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
import numpy as np
import time

# 기존 설정 재사용
from sensor.lib import config


class ObstacleDetectorNode(Node):
    """장애물 감지 노드 (즉시 발행)"""
    
    def __init__(self):
        super().__init__('obstacle_detector_node')
        
        # 정량 지표
        self.LIDAR_DANGER = 3.0      # m
        self.ULTRA_DANGER = 1.0      # m
        self.SAFE_RESUME = 3.5       # m
        self.LIDAR_WARNING = 4.0     # m
        self.ULTRA_WARNING = 1.5     # m
        
        # 센서 데이터 저장
        self.lidar_data = None
        self.ultra_front = 999.0
        self.ultra_left = 999.0
        self.ultra_right = 999.0
        
        # 융합 결과
        self.fused_front = 999.0
        self.fused_left = 999.0
        self.fused_right = 999.0
        
        # 디버그용 카운터
        self.publish_count = 0
        self.last_log_time = time.time()
        
        # 발행자 (구독자보다 먼저 생성)
        self.status_pub = self.create_publisher(
            String,
            '/obstacle_status',
            10
        )
        
        # 구독자 - 각각 데이터 수신 시 즉시 처리
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.ultra_front_sub = self.create_subscription(
            Range,
            '/ultrasonic/front',
            self.ultra_front_callback,
            10
        )
        
        self.ultra_left_sub = self.create_subscription(
            Range,
            '/ultrasonic/left',
            self.ultra_left_callback,
            10
        )
        
        self.ultra_right_sub = self.create_subscription(
            Range,
            '/ultrasonic/right',
            self.ultra_right_callback,
            10
        )
        
        self.get_logger().info('장애물 감지 노드 시작 (즉시 발행 모드)')
        self.get_logger().info(f'정량지표: LiDAR {self.LIDAR_DANGER}m, 초음파 {self.ULTRA_DANGER}m, 재개 {self.SAFE_RESUME}m')
        self.get_logger().info('센서 데이터 수신 시 즉시 처리 및 발행')
    
    def lidar_callback(self, msg):
        """LiDAR 데이터 수신 → 즉시 처리"""
        self.lidar_data = msg
        self.process_and_publish()  # 즉시 발행
    
    def ultra_front_callback(self, msg):
        """전방 초음파 데이터 수신 → 즉시 처리"""
        self.ultra_front = msg.range
        self.process_and_publish()  # 즉시 발행
    
    def ultra_left_callback(self, msg):
        """좌측 초음파 데이터 수신 → 즉시 처리"""
        self.ultra_left = msg.range
        self.process_and_publish()  # 즉시 발행
    
    def ultra_right_callback(self, msg):
        """우측 초음파 데이터 수신 → 즉시 처리"""
        self.ultra_right = msg.range
        self.process_and_publish()  # 즉시 발행
    
    def analyze_lidar_sectors(self):
        """
        LiDAR 섹터 분석
        
        Returns:
            dict: {'front': float, 'left': float, 'right': float}
        """
        if self.lidar_data is None:
            return {'front': 999.0, 'left': 999.0, 'right': 999.0}
        
        ranges = np.array(self.lidar_data.ranges)
        num_points = len(ranges)
        
        # 유효 범위 필터링
        valid_mask = (ranges >= self.lidar_data.range_min) & (ranges <= self.lidar_data.range_max)
        ranges = np.where(valid_mask, ranges, np.inf)
        
        # 섹터 크기
        sector_size = num_points // 6
        
        # 전방 (중앙 ±30도)
        front_start = num_points // 2 - sector_size // 2
        front_end = num_points // 2 + sector_size // 2
        front_min = np.min(ranges[front_start:front_end])
        front_min = float(front_min) if front_min < np.inf else 999.0
        
        # 좌측 (90~180도)
        left_section = ranges[num_points // 4:num_points // 2]
        left_min = np.min(left_section)
        left_min = float(left_min) if left_min < np.inf else 999.0
        
        # 우측 (-90~-180도)
        right_section = ranges[0:num_points // 4]
        right_min = np.min(right_section)
        right_min = float(right_min) if right_min < np.inf else 999.0
        
        return {
            'front': front_min,
            'left': left_min,
            'right': right_min
        }
    
    def fuse_sensors(self):
        """센서 융합 (간단한 최소값)"""
        lidar_sectors = self.analyze_lidar_sectors()
        
        # 전방: 초음파와 LiDAR 최소값
        self.fused_front = min(self.ultra_front, lidar_sectors['front'])
        
        # 좌우: 초음파와 LiDAR 최소값
        self.fused_left = min(self.ultra_left, lidar_sectors['left'])
        self.fused_right = min(self.ultra_right, lidar_sectors['right'])
    
    def calculate_danger(self):
        """
        위험도 계산
        
        Returns:
            tuple: (danger_level, danger_direction)
                danger_level: 0=안전, 1=경고, 2=위험
                danger_direction: 'front', 'left', 'right', 'none'
        """
        # 레벨 2: 위험 (즉시 정지)
        if (self.fused_front <= self.LIDAR_DANGER or 
            self.ultra_front <= self.ULTRA_DANGER):
            return (2, 'front')
        
        if self.ultra_left <= self.ULTRA_DANGER:
            return (2, 'left')
        
        if self.ultra_right <= self.ULTRA_DANGER:
            return (2, 'right')
        
        # 레벨 1: 경고 (감속)
        if (self.fused_front <= self.LIDAR_WARNING or 
            self.ultra_front <= self.ULTRA_WARNING):
            return (1, 'front')
        
        if self.ultra_left <= self.ULTRA_WARNING:
            return (1, 'left')
        
        if self.ultra_right <= self.ULTRA_WARNING:
            return (1, 'right')
        
        # 레벨 0: 안전
        if self.fused_front >= self.SAFE_RESUME:
            return (0, 'none')
        else:
            # 재개 거리 미달
            return (1, 'front')
    
    def process_and_publish(self):
        """
        센서 융합 + 위험도 계산 + 즉시 발행
        
        이 함수는 센서 데이터가 들어올 때마다 호출됨
        """
        # 센서 융합
        self.fuse_sensors()
        
        # 위험도 계산
        danger_level, danger_direction = self.calculate_danger()
        
        # 상태 메시지 생성
        status_msg = String()
        status_msg.data = (
            f"level={danger_level},"
            f"dir={danger_direction},"
            f"front={self.fused_front:.3f},"
            f"left={self.fused_left:.3f},"
            f"right={self.fused_right:.3f}"
        )
        
        # 즉시 발행
        self.status_pub.publish(status_msg)
        self.publish_count += 1
        
        # 디버그 출력 (1초마다)
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:
            level_text = ['안전', '경고', '위험'][danger_level]
            self.get_logger().info(
                f"[{level_text}] 전방:{self.fused_front:.2f}m "
                f"좌:{self.fused_left:.2f}m 우:{self.fused_right:.2f}m "
                f"| 발행: {self.publish_count}회/초"
            )
            self.publish_count = 0
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
