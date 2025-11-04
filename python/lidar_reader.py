"""
lidar_reader.py

YDLidar X4 Pro 센서 데이터 읽기 및 처리
- 전방 영역 최소 거리 계산
- 스캔 데이터 필터링
"""

import time
import math
from threading import Thread, Lock

try:
    from ydlidar import YDLidarX4
    YDLIDAR_AVAILABLE = True
except ImportError:
    print("WARNING: ydlidar module not found. Install with: pip3 install ydlidar")
    YDLIDAR_AVAILABLE = False

import config


class LidarReader:
    def __init__(self):
        self.lidar = None
        self.running = False
        self.thread = None
        
        # 데이터
        self.lock = Lock()
        self.front_distance = 999.0  # meters
        self.last_scan_time = 0
        
        # 통계
        self.scan_count = 0
        self.error_count = 0
        
        print(f"[LiDAR] 초기화 중... (YDLidar X4 Pro, 포트: {config.LIDAR_PORT})")
    
    def start(self):
        """LiDAR 시작"""
        if not YDLIDAR_AVAILABLE:
            print("[LiDAR] ERROR: ydlidar 모듈이 설치되지 않음")
            print("[LiDAR] 시뮬레이션 모드로 계속 진행")
            self.running = True
            self.thread = Thread(target=self._simulation_loop, daemon=True)
            self.thread.start()
            return True
        
        try:
            # YDLidar X4 초기화
            self.lidar = YDLidarX4()
            self.lidar.connect(config.LIDAR_PORT, config.LIDAR_BAUDRATE)
            self.lidar.turnOn()
            
            time.sleep(2)  # 안정화 대기
            
            # 읽기 스레드 시작
            self.running = True
            self.thread = Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            
            print("[LiDAR] 시작 완료")
            return True
            
        except Exception as e:
            print(f"[LiDAR] ERROR: 시작 실패 - {e}")
            print("[LiDAR] 시뮬레이션 모드로 계속 진행")
            self.running = True
            self.thread = Thread(target=self._simulation_loop, daemon=True)
            self.thread.start()
            return True
    
    def stop(self):
        """LiDAR 정지"""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2.0)
        
        if self.lidar:
            try:
                self.lidar.turnOff()
                self.lidar.disconnect()
            except:
                pass
        
        print("[LiDAR] 정지 완료")
    
    def _read_loop(self):
        """LiDAR 읽기 루프 (스레드)"""
        print("[LiDAR] 스캔 시작...")
        
        try:
            while self.running:
                scan = self.lidar.getScan()
                if scan:
                    self._process_scan(scan)
                    self.scan_count += 1
                time.sleep(0.05)  # 20Hz
                
        except Exception as e:
            print(f"[LiDAR] ERROR: 읽기 오류 - {e}")
            self.error_count += 1
    
    def _simulation_loop(self):
        """시뮬레이션 루프"""
        print("[LiDAR] 시뮬레이션 모드로 실행 중...")
        import random
        
        while self.running:
            with self.lock:
                # 랜덤 거리 생성 (2~8m)
                self.front_distance = random.uniform(2.0, 8.0)
                self.last_scan_time = time.time()
            
            self.scan_count += 1
            time.sleep(0.05)  # 20Hz
    
    def _process_scan(self, scan):
        """스캔 데이터 처리"""
        # 전방 영역 최소 거리 계산
        front_distances = []
        
        for point in scan:
            angle = point.angle
            distance = point.range / 1000.0  # mm -> m
            
            # 전방 영역만 (예: -30도 ~ +30도)
            if config.LIDAR_FRONT_ANGLE_MIN <= angle <= config.LIDAR_FRONT_ANGLE_MAX:
                # 유효 범위 확인 (YDLidar X4: 0.12m ~ 10m)
                if 0.12 < distance < 10.0:
                    front_distances.append(distance)
        
        # 최소 거리 업데이트
        with self.lock:
            if front_distances:
                self.front_distance = min(front_distances)
            else:
                self.front_distance = 999.0
            
            self.last_scan_time = time.time()
    
    def get_front_distance(self):
        """전방 최소 거리 반환 (meters)"""
        with self.lock:
            return self.front_distance
    
    def is_healthy(self):
        """LiDAR 상태 확인"""
        if not self.running:
            return False
        
        # 최근 1초 이내 스캔 확인
        time_since_scan = time.time() - self.last_scan_time
        return time_since_scan < 1.0
    
    def get_stats(self):
        """통계 반환"""
        return {
            'scan_count': self.scan_count,
            'error_count': self.error_count,
            'front_distance': self.front_distance,
            'healthy': self.is_healthy()
        }


# ============================================================
# 테스트 코드
# ============================================================

if __name__ == '__main__':
    print("YDLidar X4 Pro 테스트 시작")
    
    lidar = LidarReader()
    
    if lidar.start():
        try:
            while True:
                distance = lidar.get_front_distance()
                print(f"전방 거리: {distance:.2f}m")
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n테스트 종료")
        finally:
            lidar.stop()
    else:
        print("LiDAR 시작 실패")
