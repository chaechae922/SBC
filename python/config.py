"""
config.py

딜리봇 시스템 설정 파일
- 정량 지표 정의
- 센서 설정
- MCU 통신 설정
"""

# ============================================================
# 정량 지표
# ============================================================

# 정량지표 2: 장애물(사람) 감지-정지 안전성
LIDAR_DANGER_DISTANCE = 3.0      # m - LiDAR 3m 이하 감지 → 즉시 정지
ULTRASONIC_DANGER_DISTANCE = 1.0 # m - 초음파 1m 이하 감지 → 즉시 정지
SAFE_RESUME_DISTANCE = 3.5       # m - 3.5m 이상 거리 확보 → 재주행

# 경고 단계 (감속)
LIDAR_WARNING_DISTANCE = 4.0     # m
ULTRASONIC_WARNING_DISTANCE = 1.5 # m

# ============================================================
# 센서 설정
# ============================================================

# LiDAR (YDLidar X4 Pro)
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 128000
LIDAR_TIMEOUT = 1.0

# LiDAR 스캔 영역
LIDAR_FRONT_ANGLE_MIN = -30  # degrees
LIDAR_FRONT_ANGLE_MAX = 30   # degrees

# 초음파 센서 (HY-SRF05)
ULTRASONIC_PINS = {
    'front': {'trigger': 23, 'echo': 24},  # GPIO 핀 번호
    'left': {'trigger': 17, 'echo': 27},
    'right': {'trigger': 22, 'echo': 10}
}

ULTRASONIC_TIMEOUT = 0.03  # seconds (30ms)
ULTRASONIC_SPEED_OF_SOUND = 343  # m/s (20°C)

# ============================================================
# 시스템 타이밍
# ============================================================

SENSOR_READ_INTERVAL = 0.05    # 20Hz (센서 업데이트)
STATUS_PRINT_INTERVAL = 1.0    # 1Hz (화면 출력)

# ============================================================
# 안전 설정
# ============================================================

# 최소 정지 시간 (긴급정지 후)
MIN_STOP_DURATION = 0.5  # seconds

# 연속 감지 횟수 (오감지 방지)
CONSECUTIVE_DETECTIONS = 2  # 2회 연속 감지 시 확정

# ============================================================
# 디버그 설정
# ============================================================

DEBUG_ENABLE = True
DEBUG_PRINT_SENSORS = False  # 센서 상세 로그
