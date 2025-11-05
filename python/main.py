"""
main.py

딜리봇 장애물 감지 시스템
- LiDAR + 초음파 센서 읽기
- 장애물 감지 (정량 지표 기반)
- 결과 출력 + CSV 데이터 로깅
"""

import time
import threading
import queue
import sys
import os
import csv
from datetime import datetime
import pygame, sys

pygame.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
js.init()
clock = pygame.time.Clock()

import config
from lidar_reader import LidarReader
from ultrasonic_reader import UltrasonicReader
from obstacle_detector import ObstacleDetector

# ----------------------------
# MCU UART 통신 모듈
# ----------------------------
try:
    import serial
except ImportError:
    serial = None


class McuUart:
    """
    간단한 라인 기반 UART 링크.
    - 송신: "OBS,<t_s>,<level>,<dir>,<front>,<left>,<right>\n"
      예: "OBS,12.345,2,front,0.53,0.70,0.65\n"
    - 수신: "PING" -> "PONG\n"으로 응답
           "REQ"  -> 최신 상태 프레임 즉시 1회 전송
    """

    def __init__(self,
                 port=None,
                 baud=None,
                 read_timeout=0.01,
                 rx_queue_max=128):
        self.port = port or getattr(config, "MCU_PORT", "/dev/serial0")
        self.baud = baud or getattr(config, "MCU_BAUD", 115200)
        self.read_timeout = read_timeout

        self.ser = None
        self.alive = False

        self._rx_thread = None
        self._rx_q = queue.Queue(maxsize=rx_queue_max)
        self._send_lock = threading.Lock()

        self._last_status_line = None  # 최신 상태 라인 캐시

    # ---- lifecycle ----
    def start(self):
        if serial is None:
            print("[MCU] pyserial 미설치: 'pip install pyserial' 필요", file=sys.stderr)
            return False
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=self.read_timeout
            )
            self.alive = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            print(f"[MCU] Open {self.port} @ {self.baud}")
            return True
        except Exception as e:
            print(f"[MCU] 포트 오픈 실패: {e}", file=sys.stderr)
            self.ser = None
            return False

    def stop(self):
        self.alive = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.2)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        print("[MCU] Closed")

    # ---- internal ----
    def _rx_loop(self):
        buf = b""
        while self.alive and self.ser:
            try:
                line = self.ser.readline()  # \n 기준
                if not line:
                    continue
                try:
                    text = line.decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue
                if not text:
                    continue
                # 큐가 가득 차면 가장 오래된 항목 버림
                if self._rx_q.full():
                    try:
                        self._rx_q.get_nowait()
                    except queue.Empty:
                        pass
                self._rx_q.put_nowait(text)
            except Exception:
                # read 에러는 무시하고 재시도
                time.sleep(0.01)

    def _send_line(self, line: str):
        if not self.ser:
            return
        data = (line if line.endswith("\n") else line + "\n").encode("utf-8", errors="ignore")
        with self._send_lock:
            try:
                self.ser.write(data)
            except Exception:
                pass  # 일시적인 write 에러 무시

    # ---- API ----
    def handle_incoming(self):
        """수신 큐의 모든 명령을 처리"""
        processed = 0
        while True:
            try:
                msg = self._rx_q.get_nowait()
            except queue.Empty:
                break
            processed += 1
            upper = msg.strip().upper()
            if upper == "PING":
                self._send_line("PONG")
            elif upper == "REQ":
                if self._last_status_line:
                    self._send_line(self._last_status_line)
        return processed

    def send_status(self, status: dict, t_since_start: float):
        """현재 상태를 라인 프레임으로 직렬화하여 전송"""
        dir_map = {
            "전방": "front", "앞": "front",
            "좌": "left", "왼쪽": "left",
            "우": "right", "오른쪽": "right",
            "none": "none", "": "none",
        }
        raw_dir = (status.get("danger_direction") or "none").strip()
        direction = dir_map.get(raw_dir, raw_dir.lower())
        if direction not in ("front", "left", "right", "none"):
            direction = "none"

        line = "OBS,{:.3f},{:d},{},{:.3f},{:.3f},{:.3f}".format(
            float(t_since_start),
            int(status.get("danger_level", 0)),
            direction,
            float(status.get("front_distance", -1.0)),
            float(status.get("left_distance", -1.0)),
            float(status.get("right_distance", -1.0)),
        )
        self._last_status_line = line
        self._send_line(line)


# ============================================================
# 장애물 감지 시스템
# ============================================================

class ObstacleDetectionSystem:
    def __init__(self):
        print("장애물 감지 시스템 초기화 중...")

        # 센서 초기화
        self.lidar = LidarReader()
        self.ultrasonic = UltrasonicReader()
        self.detector = ObstacleDetector(self.lidar, self.ultrasonic)

        self.running = False
        self.start_time = 0

        # 로그 관련
        self.log_file = None
        self.log_writer = None

        print("초기화 완료\n")

    def _init_logger(self):
        """CSV 로그 파일 초기화"""
        os.makedirs("logs", exist_ok=True)
        filename = datetime.now().strftime("logs/log_%Y%m%d_%H%M%S.csv")
        self.log_file = open(filename, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["time", "front", "left", "right", "danger_level", "direction"])
        print(f"[Logger] 데이터 로깅 시작 → {filename}")

    def _log_status(self, status, elapsed_time):
        """현재 상태를 CSV로 기록"""
        if not self.log_writer:
            return
        self.log_writer.writerow([
            round(elapsed_time, 2),
            round(status["front_distance"], 3),
            round(status["left_distance"], 3),
            round(status["right_distance"], 3),
            status["danger_level"],
            status["danger_direction"] or "none"
        ])
        self.log_file.flush()

    def start(self):
        """시스템 시작"""
        print("시스템 시작 중...\n")
        self.lidar.start()
        self.ultrasonic.start()
        time.sleep(2.0)

        self._init_logger()
        self.running = True
        self.start_time = time.time()

        print("시스템 시작 완료\n")
        return True

    def stop(self):
        """시스템 정지"""
        print("\n시스템 종료 중...")
        self.running = False
        self.lidar.stop()
        self.ultrasonic.stop()
        if self.log_file:
            self.log_file.close()
            print("[Logger] 로그 파일 저장 완료")
        print("종료 완료")

    def run(self):
        """메인 루프"""
        last_update_time = 0
        last_status_time = 0
        print("장애물 감지 중... (Ctrl+C로 종료)\n")

        try:
            while self.running:
                current_time = time.time()
                for e in pygame.event.get():
                    if e.type == pygame.JOYBUTTONDOWN:
                        print("Button", e.button, "DOWN")
                clock.tick(120)

                # 센서 업데이트 (20Hz)
                if current_time - last_update_time >= config.SENSOR_READ_INTERVAL:
                    last_update_time = current_time
                    self.detector.update()

                # 상태 출력 및 로그 기록 (1Hz)
                if current_time - last_status_time >= config.STATUS_PRINT_INTERVAL:
                    last_status_time = current_time
                    status = self.detector.get_status()
                    self.print_status(status)
                    self._log_status(status, current_time - self.start_time)

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n사용자 중단")

    def print_status(self, status):
        """상태 출력"""
        icons = {0: '✓', 1: '⚠️', 2: '🚨'}
        icon = icons[status['danger_level']]
        level_text = {0: '안전', 1: '경고', 2: '위험'}[status['danger_level']]

        print(f"[{time.time() - self.start_time:5.1f}s] "
              f"{icon} {level_text:4s} | "
              f"전방:{status['front_distance']:5.2f}m "
              f"좌:{status['left_distance']:5.2f}m "
              f"우:{status['right_distance']:5.2f}m")

        if status['danger_level'] == 2:
            print(f"       🚨 {status['danger_direction']} 방향 장애물 감지!")
        elif status['danger_level'] == 1:
            print(f"       ⚠️  {status['danger_direction']} 방향 접근 중")


def main():
    system = ObstacleDetectionSystem()
    if system.start():
        system.run()
    system.stop()


if __name__ == '__main__':
    main()
