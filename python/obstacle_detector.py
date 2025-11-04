"""
obstacle_detector.py

센서 융합 및 장애물 감지
- LiDAR + 초음파 데이터 통합
- 정량 지표 기반 위험도 판단
"""

import time
from threading import Lock

import config


class ObstacleDetector:
    def __init__(self, lidar_reader, ultrasonic_reader):
        self.lidar = lidar_reader
        self.ultrasonic = ultrasonic_reader
        
        # 상태
        self.lock = Lock()
        self.danger_level = 0  # 0:안전, 1:경고, 2:위험
        self.danger_direction = None
        
        # 융합 데이터
        self.fused_front_distance = 999.0
        self.fused_left_distance = 999.0
        self.fused_right_distance = 999.0
        
        # 연속 감지 카운터 (오감지 방지)
        self.danger_count = 0
        self.safe_count = 0
        
        print("[장애물 감지] 초기화 완료")
    
    def update(self):
        """센서 데이터 업데이트 및 위험도 계산"""
        # 센서 데이터 읽기
        lidar_front = self.lidar.get_front_distance()
        ultrasonic_dists = self.ultrasonic.get_distances()
        
        # 센서 융합
        self._fuse_sensors(lidar_front, ultrasonic_dists)
        
        # 위험도 계산
        self._calculate_danger_level()
    
    def _fuse_sensors(self, lidar_front, ultrasonic_dists):
        """센서 데이터 융합"""
        with self.lock:
            # 전방: LiDAR와 초음파 중 최소값 (보수적)
            ultra_front = ultrasonic_dists['front']
            
            if lidar_front < 900.0 and ultra_front < 900.0:
                # 둘 다 유효: 최소값
                self.fused_front_distance = min(lidar_front, ultra_front)
            elif lidar_front < 900.0:
                # LiDAR만 유효
                self.fused_front_distance = lidar_front
            elif ultra_front < 900.0:
                # 초음파만 유효
                self.fused_front_distance = ultra_front
            else:
                # 둘 다 무효
                self.fused_front_distance = 999.0
            
            # 좌우: 초음파만
            self.fused_left_distance = ultrasonic_dists['left']
            self.fused_right_distance = ultrasonic_dists['right']
    
    def _calculate_danger_level(self):
        """위험도 레벨 계산 (정량 지표 기반)"""
        with self.lock:
            front = self.fused_front_distance
            left = self.fused_left_distance
            right = self.fused_right_distance
            
            # ========================================
            # 레벨 2: 위험 (즉시 정지)
            # ========================================
            
            # LiDAR 3m 이하 OR 초음파 1m 이하
            if front <= config.LIDAR_DANGER_DISTANCE:
                self.danger_count += 1
                self.safe_count = 0
                
                if self.danger_count >= config.CONSECUTIVE_DETECTIONS:
                    self.danger_level = 2
                    self.danger_direction = 'front'
                    return
            
            # 측면 초음파 1m 이하
            elif left <= config.ULTRASONIC_DANGER_DISTANCE:
                self.danger_count += 1
                self.safe_count = 0
                
                if self.danger_count >= config.CONSECUTIVE_DETECTIONS:
                    self.danger_level = 2
                    self.danger_direction = 'left'
                    return
            
            elif right <= config.ULTRASONIC_DANGER_DISTANCE:
                self.danger_count += 1
                self.safe_count = 0
                
                if self.danger_count >= config.CONSECUTIVE_DETECTIONS:
                    self.danger_level = 2
                    self.danger_direction = 'right'
                    return
            
            # ========================================
            # 레벨 1: 경고 (감속)
            # ========================================
            
            elif front <= config.LIDAR_WARNING_DISTANCE:
                self.danger_count = 0
                self.safe_count = 0
                self.danger_level = 1
                self.danger_direction = 'front'
                return
            
            elif (left <= config.ULTRASONIC_WARNING_DISTANCE or 
                  right <= config.ULTRASONIC_WARNING_DISTANCE):
                self.danger_count = 0
                self.safe_count = 0
                self.danger_level = 1
                self.danger_direction = 'side'
                return
            
            # ========================================
            # 레벨 0: 안전
            # ========================================
            
            else:
                self.danger_count = 0
                
                # 재개 조건: 3.5m 이상
                if front >= config.SAFE_RESUME_DISTANCE:
                    self.safe_count += 1
                    
                    if self.safe_count >= config.CONSECUTIVE_DETECTIONS:
                        self.danger_level = 0
                        self.danger_direction = None
                else:
                    # 정지 상태 유지 (거리 부족)
                    if self.danger_level == 2:
                        pass  # 위험 레벨 유지
                    else:
                        self.danger_level = 1
    
    def get_status(self):
        """현재 상태 반환"""
        with self.lock:
            return {
                'danger_level': self.danger_level,
                'danger_direction': self.danger_direction,
                'front_distance': self.fused_front_distance,
                'left_distance': self.fused_left_distance,
                'right_distance': self.fused_right_distance
            }
    
    def is_safe(self):
        """안전 여부"""
        with self.lock:
            return self.danger_level == 0
    
    def should_stop(self):
        """긴급정지 필요 여부"""
        with self.lock:
            return self.danger_level == 2
    
    def should_slow(self):
        """감속 필요 여부"""
        with self.lock:
            return self.danger_level == 1
    
    def get_min_distance(self):
        """최소 거리 반환"""
        with self.lock:
            return min(
                self.fused_front_distance,
                self.fused_left_distance,
                self.fused_right_distance
            )


# ============================================================
# 테스트 코드
# ============================================================

if __name__ == '__main__':
    print("장애물 감지 테스트")
    
    # 더미 센서 리더
    class DummyLidar:
        def __init__(self):
            self.distance = 5.0
        def get_front_distance(self):
            return self.distance
    
    class DummyUltrasonic:
        def __init__(self):
            self.distances = {'front': 5.0, 'left': 5.0, 'right': 5.0}
        def get_distances(self):
            return self.distances
    
    lidar = DummyLidar()
    ultrasonic = DummyUltrasonic()
    detector = ObstacleDetector(lidar, ultrasonic)
    
    # 시나리오 테스트
    print("\n[시나리오 1] 안전 거리")
    lidar.distance = 5.0
    for i in range(3):
        detector.update()
        print(f"  {i+1}회: {detector.get_status()}")
    
    print("\n[시나리오 2] 경고 거리 (3.5m)")
    lidar.distance = 3.8
    for i in range(3):
        detector.update()
        print(f"  {i+1}회: {detector.get_status()}")
    
    print("\n[시나리오 3] 위험 거리 (2.5m)")
    lidar.distance = 2.5
    for i in range(3):
        detector.update()
        print(f"  {i+1}회: {detector.get_status()}")
    
    print("\n[시나리오 4] 재개 조건 (4.0m)")
    lidar.distance = 4.0
    for i in range(5):
        detector.update()
        print(f"  {i+1}회: {detector.get_status()}")
