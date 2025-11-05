#!/usr/bin/env python3
"""
obstacle_detector_node.py

장애물 감지 ROS2 노드 (즉시 발행 + RViz2 시각화 + CSV 로깅)
--------------------------------------------------------
- LiDAR + 초음파 센서 융합으로 위험도 판단
- level 0~2 (안전 / 경고 / 위험)
- 데이터 수신 시 즉시 처리 (타이머 없음)
- RViz2 MarkerArray로 시각화
- CSV 로그 파일 자동 생성 (logs/ros2_obstacle_*.csv)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
import os
import time
import csv
from datetime import datetime

# config.py 값 사용 (없으면 기본값)
try:
    from delivery_sensor.lib import config
except Exception:
    config = None


def _cfg(name, default):
    """config에 값이 있으면 사용, 없으면 default"""
    if config is None:
        return default
    return getattr(config, name, default)


class ObstacleDetectorNode(Node):
    """장애물 감지 노드 (즉시 발행 + 시각화 + 로깅)"""

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # === 정량 지표 ===
        self.LIDAR_DANGER = _cfg('LIDAR_DANGER_DISTANCE', 3.0)
        self.ULTRA_DANGER = _cfg('ULTRASONIC_DANGER_DISTANCE', 1.0)
        self.SAFE_RESUME = _cfg('SAFE_RESUME_DISTANCE', 3.5)
        self.LIDAR_WARNING = _cfg('LIDAR_WARNING_DISTANCE', 4.0)
        self.ULTRA_WARNING = _cfg('ULTRASONIC_WARNING_DISTANCE', 1.5)

        # === 센서 데이터 저장 ===
        self.lidar_data = None
        self.ultra_front = 999.0
        self.ultra_left = 999.0
        self.ultra_right = 999.0

        # 융합 결과
        self.fused_front = 999.0
        self.fused_left = 999.0
        self.fused_right = 999.0

        # === 상태 관리 ===
        self.publish_count = 0
        self.last_log_time = time.time()

        # === 발행자 ===
        self.status_pub = self.create_publisher(String, '/obstacle_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        # === 구독자 (이벤트 기반) ===
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.ultra_front_sub = self.create_subscription(Range, '/ultrasonic/front', self.ultra_front_callback, 10)
        self.ultra_left_sub = self.create_subscription(Range, '/ultrasonic/left', self.ultra_left_callback, 10)
        self.ultra_right_sub = self.create_subscription(Range, '/ultrasonic/right', self.ultra_right_callback, 10)

        # === 로깅 설정 ===
        os.makedirs("logs", exist_ok=True)
        self.log_name = datetime.now().strftime("logs/ros2_obstacle_%Y%m%d_%H%M%S.csv")
        try:
            self.log_file = open(self.log_name, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(["time", "front", "left", "right", "level", "direction"])
            self.start_time = time.time()
            self.get_logger().info(f"CSV 로깅 시작: {self.log_name}")
        except Exception as e:
            self.log_writer = None
            self.get_logger().warn(f"로그 파일 생성 실패: {e}")

        # === 초기 로그 ===
        self.get_logger().info("장애물 감지 노드 시작 (즉시 발행 모드)")
        self.get_logger().info(f"정량지표: LiDAR {self.LIDAR_DANGER}m / 초음파 {self.ULTRA_DANGER}m / 재개 {self.SAFE_RESUME}m")

    # ============================================================
    # 콜백 (데이터 수신 시 즉시 처리)
    # ============================================================

    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = msg
        self.process_and_publish()

    def ultra_front_callback(self, msg: Range):
        self.ultra_front = msg.range
        self.process_and_publish()

    def ultra_left_callback(self, msg: Range):
        self.ultra_left = msg.range
        self.process_and_publish()

    def ultra_right_callback(self, msg: Range):
        self.ultra_right = msg.range
        self.process_and_publish()

    # ============================================================
    # LiDAR 데이터 처리
    # ============================================================

    def analyze_lidar_sectors(self):
        """LiDAR 데이터를 전방/좌/우 3개 섹터로 나누어 최소 거리 계산"""
        if self.lidar_data is None:
            return {'front': 999.0, 'left': 999.0, 'right': 999.0}

        ranges = np.array(self.lidar_data.ranges, dtype=float)
        valid_mask = (ranges >= self.lidar_data.range_min) & (ranges <= self.lidar_data.range_max)
        ranges = np.where(valid_mask, ranges, np.inf)

        num_points = len(ranges)
        if num_points == 0:
            return {'front': 999.0, 'left': 999.0, 'right': 999.0}

        # 전방: 중앙 ±30도 근사
        center = num_points // 2
        sector = num_points // 6
        front_min = np.min(ranges[center - sector // 2:center + sector // 2])

        # 좌측: 90~180도
        left_min = np.min(ranges[num_points // 4:num_points // 2])
        # 우측: -90~-180도
        right_min = np.min(ranges[0:num_points // 4])

        # 무한대 방지
        def valid(x): return float(x) if x < np.inf else 999.0
        return {'front': valid(front_min), 'left': valid(left_min), 'right': valid(right_min)}

    # ============================================================
    # 센서 융합 및 위험도 계산
    # ============================================================

    def fuse_sensors(self):
        """초음파 + LiDAR 최소값 융합"""
        lidar = self.analyze_lidar_sectors()
        self.fused_front = min(lidar['front'], self.ultra_front)
        self.fused_left = min(lidar['left'], self.ultra_left)
        self.fused_right = min(lidar['right'], self.ultra_right)

    def calculate_danger(self):
        """융합 결과 기반 위험도 계산 (0=안전,1=경고,2=위험)"""
        # 위험
        if (self.fused_front <= self.LIDAR_DANGER or self.ultra_front <= self.ULTRA_DANGER):
            return (2, 'front')
        if self.ultra_left <= self.ULTRA_DANGER:
            return (2, 'left')
        if self.ultra_right <= self.ULTRA_DANGER:
            return (2, 'right')

        # 경고
        if (self.fused_front <= self.LIDAR_WARNING or self.ultra_front <= self.ULTRA_WARNING):
            return (1, 'front')
        if self.ultra_left <= self.ULTRA_WARNING:
            return (1, 'left')
        if self.ultra_right <= self.ULTRA_WARNING:
            return (1, 'right')

        # 안전
        if self.fused_front >= self.SAFE_RESUME:
            return (0, 'none')
        return (1, 'front')

    # ============================================================
    # RViz Marker 생성 및 발행
    # ============================================================

    def _make_marker(self, idx, x, y, color):
        """RViz2용 Marker 생성"""
        m = Marker()
        m.header.frame_id = _cfg('RVIZ_FRAME', 'base_link')
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "obstacle"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = Duration(sec=1)
        return m

    def publish_markers(self, front, left, right, level):
        """위험도에 따라 색상 변경하여 MarkerArray 발행"""
        colors = {
            0: (0.0, 1.0, 0.0, 0.6),  # Green
            1: (1.0, 1.0, 0.0, 0.8),  # Yellow
            2: (1.0, 0.0, 0.0, 1.0)   # Red
        }
        color = colors.get(level, (1.0, 1.0, 1.0, 0.5))
        ma = MarkerArray()
        ma.markers.append(self._make_marker(0, front, 0.0, color))
        ma.markers.append(self._make_marker(1, 0.0, left, color))
        ma.markers.append(self._make_marker(2, 0.0, -right, color))
        self.marker_pub.publish(ma)

    # ============================================================
    # 메인 처리 함수 (즉시 호출)
    # ============================================================

    def process_and_publish(self):
        """센서 융합 + 위험도 계산 + 상태 발행 + 시각화 + 로그"""
        self.fuse_sensors()
        level, direction = self.calculate_danger()

        # 상태 메시지 생성
        msg = String()
        msg.data = (
            f"level={level},dir={direction},"
            f"front={self.fused_front:.3f},left={self.fused_left:.3f},right={self.fused_right:.3f}"
        )
        self.status_pub.publish(msg)
        self.publish_markers(self.fused_front, self.fused_left, self.fused_right, level)

        # CSV 로그 기록
        if self.log_writer:
            elapsed = round(time.time() - self.start_time, 2)
            self.log_writer.writerow([elapsed, self.fused_front, self.fused_left, self.fused_right, level, direction])
            self.log_file.flush()

        # 1Hz 디버그 출력
        now = time.time()
        self.publish_count += 1
        if now - self.last_log_time >= 1.0:
            level_text = ['안전', '경고', '위험'][level]
            self.get_logger().info(
                f"[{level_text}] front:{self.fused_front:.2f}m left:{self.fused_left:.2f}m right:{self.fused_right:.2f}m | {self.publish_count}Hz"
            )
            self.publish_count = 0
            self.last_log_time = now

    def destroy_node(self):
        """노드 종료 시 로그 파일 닫기"""
        if getattr(self, "log_file", None):
            self.log_file.close()
            self.get_logger().info(f"CSV 로그 저장 완료: {self.log_name}")
        super().destroy_node()


# ============================================================
# 메인 진입점
# ============================================================

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


if __name__ == "__main__":
    main()
