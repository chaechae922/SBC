"""
lidar_reader.py

YDLidar X4 Pro 센서 데이터 읽기 및 처리
- 전방 영역 최소 거리 계산
- 스캔 데이터 필터링
- 시뮬레이션 모드: 패턴 기반 접근/후퇴 반복
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

        self.lock = Lock()
        self.front_distance = 999.0
        self.last_scan_time = 0
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
            self.lidar = YDLidarX4()
            self.lidar.connect(config.LIDAR_PORT, config.LIDAR_BAUDRATE)
            self.lidar.turnOn()
            time.sleep(2)
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
        """LiDAR 읽기 루프"""
        print("[LiDAR] 스캔 시작...")
        try:
            while self.running:
                scan = self.lidar.getScan()
                if scan:
                    self._process_scan(scan)
                    self.scan_count += 1
                time.sleep(0.05)
        except Exception as e:
            print(f"[LiDAR] ERROR: 읽기 오류 - {e}")
            self.error_count += 1

    def _simulation_loop(self):
        """시뮬레이션 루프 (패턴 기반 접근→정지→후퇴 반복)"""
        print("[LiDAR] 시뮬레이션 모드로 실행 중... (패턴 기반)")
        t = 0.0
        while self.running:
            # 거리 = 5 + 3*cos(2πt/10) → 10초 주기로 2~8m 반복
            simulated_distance = 5 + 3 * math.cos((t / 10.0) * 2 * math.pi)
            with self.lock:
                self.front_distance = simulated_distance
                self.last_scan_time = time.time()
            self.scan_count += 1
            t += 0.05
            time.sleep(0.05)

    def _process_scan(self, scan):
        """스캔 데이터 처리"""
        front_distances = []
        for point in scan:
            angle = point.angle
            distance = point.range / 1000.0
            if config.LIDAR_FRONT_ANGLE_MIN <= angle <= config.LIDAR_FRONT_ANGLE_MAX:
                if 0.12 < distance < 10.0:
                    front_distances.append(distance)
        with self.lock:
            self.front_distance = min(front_distances) if front_distances else 999.0
            self.last_scan_time = time.time()

    def get_front_distance(self):
        with self.lock:
            return self.front_distance

    def is_healthy(self):
        if not self.running:
            return False
        return (time.time() - self.last_scan_time) < 1.0

    def get_stats(self):
        return {
            'scan_count': self.scan_count,
            'error_count': self.error_count,
            'front_distance': self.front_distance,
            'healthy': self.is_healthy()
        }


if __name__ == '__main__':
    print("YDLidar X4 Pro 테스트 시작")
    lidar = LidarReader()
    if lidar.start():
        try:
            while True:
                print(f"전방 거리: {lidar.get_front_distance():.2f}m")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n테스트 종료")
        finally:
            lidar.stop()
    else:
        print("LiDAR 시작 실패")
