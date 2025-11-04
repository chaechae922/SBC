"""
ultrasonic_reader.py

HY-SRF05 초음파 센서 데이터 읽기
- 3개 센서 (전방, 좌측, 우측)
- 비동기 읽기로 블로킹 최소화
"""

import time
from threading import Thread, Lock

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("WARNING: RPi.GPIO module not found. Running in simulation mode.")
    GPIO_AVAILABLE = False
    # 시뮬레이션 모드용 더미 클래스
    class GPIO:
        BCM = 'BCM'
        OUT = 'OUT'
        IN = 'IN'
        @staticmethod
        def setmode(mode): pass
        @staticmethod
        def setup(pin, mode): pass
        @staticmethod
        def output(pin, value): pass
        @staticmethod
        def input(pin): return False
        @staticmethod
        def cleanup(): pass

import config


class UltrasonicReader:
    def __init__(self):
        self.running = False
        self.thread = None
        
        # 데이터
        self.lock = Lock()
        self.distances = {
            'front': 999.0,  # meters
            'left': 999.0,
            'right': 999.0
        }
        
        # 통계
        self.read_count = {'front': 0, 'left': 0, 'right': 0}
        self.error_count = {'front': 0, 'left': 0, 'right': 0}
        
        print("[초음파] 초기화 중...")
        
        if GPIO_AVAILABLE:
            # GPIO 설정
            GPIO.setmode(GPIO.BCM)
            
            for name, pins in config.ULTRASONIC_PINS.items():
                GPIO.setup(pins['trigger'], GPIO.OUT)
                GPIO.setup(pins['echo'], GPIO.IN)
                GPIO.output(pins['trigger'], False)
            
            time.sleep(0.1)
            print("[초음파] GPIO 설정 완료")
        else:
            print("[초음파] WARNING: 시뮬레이션 모드로 실행")
    
    def start(self):
        """초음파 센서 시작"""
        self.running = True
        self.thread = Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        
        print("[초음파] 시작 완료")
        return True
    
    def stop(self):
        """초음파 센서 정지"""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2.0)
        
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        
        print("[초음파] 정지 완료")
    
    def _read_loop(self):
        """초음파 센서 읽기 루프 (스레드)"""
        print("[초음파] 측정 시작...")
        
        sensors = ['front', 'left', 'right']
        sensor_idx = 0
        
        while self.running:
            try:
                # 순차적으로 센서 읽기 (간섭 방지)
                sensor_name = sensors[sensor_idx]
                distance = self._read_sensor(sensor_name)
                
                with self.lock:
                    if distance is not None:
                        self.distances[sensor_name] = distance
                        self.read_count[sensor_name] += 1
                    else:
                        self.distances[sensor_name] = 999.0
                        self.error_count[sensor_name] += 1
                
                # 다음 센서로
                sensor_idx = (sensor_idx + 1) % 3
                
                # 간격 (20ms × 3 = 60ms 주기)
                time.sleep(0.02)
                
            except Exception as e:
                if config.DEBUG_PRINT_SENSORS:
                    print(f"[초음파] ERROR: {sensor_name} - {e}")
                time.sleep(0.1)
    
    def _read_sensor(self, sensor_name):
        """개별 센서 읽기"""
        if not GPIO_AVAILABLE:
            # 시뮬레이션: 랜덤 값
            import random
            return random.uniform(0.5, 5.0)
        
        pins = config.ULTRASONIC_PINS[sensor_name]
        trigger_pin = pins['trigger']
        echo_pin = pins['echo']
        
        try:
            # 트리거 펄스
            GPIO.output(trigger_pin, True)
            time.sleep(0.00001)  # 10us
            GPIO.output(trigger_pin, False)
            
            # Echo 상승 대기
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > config.ULTRASONIC_TIMEOUT:
                    return None
            
            # Echo 하강 대기
            timeout_start = time.time()
            while GPIO.input(echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > config.ULTRASONIC_TIMEOUT:
                    return None
            
            # 거리 계산
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * config.ULTRASONIC_SPEED_OF_SOUND) / 2.0
            
            # 유효 범위 확인 (0.02m ~ 4m)
            if 0.02 < distance < 4.0:
                return distance
            else:
                return None
                
        except Exception as e:
            return None
    
    def get_distances(self):
        """모든 거리 반환 (meters)"""
        with self.lock:
            return self.distances.copy()
    
    def get_front_distance(self):
        """전방 거리 반환 (meters)"""
        with self.lock:
            return self.distances['front']
    
    def is_healthy(self):
        """센서 상태 확인"""
        with self.lock:
            # 모든 센서가 유효한 값을 가지고 있는지
            return all(d < 900.0 for d in self.distances.values())
    
    def get_stats(self):
        """통계 반환"""
        with self.lock:
            return {
                'distances': self.distances.copy(),
                'read_count': self.read_count.copy(),
                'error_count': self.error_count.copy(),
                'healthy': self.is_healthy()
            }


# ============================================================
# 테스트 코드
# ============================================================

if __name__ == '__main__':
    print("초음파 센서 테스트 시작")
    
    ultrasonic = UltrasonicReader()
    
    if ultrasonic.start():
        try:
            while True:
                distances = ultrasonic.get_distances()
                print(f"전방: {distances['front']:.2f}m  "
                      f"좌측: {distances['left']:.2f}m  "
                      f"우측: {distances['right']:.2f}m")
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n테스트 종료")
        finally:
            ultrasonic.stop()
    else:
        print("초음파 센서 시작 실패")
