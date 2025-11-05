#!/usr/bin/env python3
"""
ultrasonic_node.py

ROS2 초음파 센서 노드 (HY-SRF05 3개)
- /ultrasonic/front, /left, /right 발행
- CSV 로깅 (자동 타임스탬프)
- RViz2 MarkerArray 시각화
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import time
import os
import csv
from datetime import datetime
import random

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("[WARN] RPi.GPIO 없음 - 시뮬레이션 모드 사용")

# 설정 파일
try:
    from delivery_sensor.lib import config
except ImportError:
    from delivery_sensor.lib import config


class UltrasonicNode(Node):
    """초음파 센서 ROS2 노드 (로깅 + RViz2 시각화)"""

    def __init__(self):
        super().__init__('ultrasonic_node')

        # 기본 설정
        self.declare_parameter('update_rate', 20.0)
        rate = self.get_parameter('update_rate').value
        self.period = 1.0 / rate

        self.pins = {
            'front': (config.ULTRA_FRONT_TRIG, config.ULTRA_FRONT_ECHO),
            'left': (config.ULTRA_LEFT_TRIG, config.ULTRA_LEFT_ECHO),
            'right': (config.ULTRA_RIGHT_TRIG, config.ULTRA_RIGHT_ECHO),
        }

        # GPIO 설정
        if HAS_GPIO:
            GPIO.setmode(GPIO.BCM)
            for trig, echo in self.pins.values():
                GPIO.setup(trig, GPIO.OUT)
                GPIO.setup(echo, GPIO.IN)
                GPIO.output(trig, GPIO.LOW)
            self.get_logger().info("GPIO 초기화 완료")
        else:
            self.get_logger().warn("GPIO 없음 - 더미 센서 모드")

        # ROS 퍼블리셔
        self.front_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.left_pub = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.right_pub = self.create_publisher(Range, '/ultrasonic/right', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ultrasonic/markers', 10)

        # 로깅 경로 설정
        log_dir = os.path.expanduser('~/.ros/logs/ultrasonic')
        os.makedirs(log_dir, exist_ok=True)
        log_name = os.path.join(log_dir, f"ros2_ultra_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.log_file = open(log_name, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["time(s)", "front(m)", "left(m)", "right(m)"])
        self.start_time = time.time()
        self.get_logger().info(f"CSV 로깅 시작: {log_name}")

        # 주기 타이머
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.last_status_time = time.time()

    # -----------------------
    # 거리 측정
    # -----------------------
    def measure_distance(self, trig, echo):
        if not HAS_GPIO:
            return random.uniform(0.2, 3.5)  # 시뮬레이션용
        try:
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)

            timeout = time.time() + 0.03
            while GPIO.input(echo) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return 9.99
            while GPIO.input(echo) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return 9.99

            duration = pulse_end - pulse_start
            distance = (duration * 34300) / 2
            return max(0.02, min(distance / 100.0, 4.0))
        except Exception as e:
            self.get_logger().warn(f"센서 읽기 실패: {e}")
            return 9.99

    # -----------------------
    # 메시지 생성
    # -----------------------
    def create_range_msg(self, frame, distance):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance
        return msg

    # -----------------------
    # RViz 마커
    # -----------------------
    def make_marker(self, mid, x, y, dist):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "ultrasonic"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.scale.x = m.scale.y = m.scale.z = 0.15
        # 거리 기반 색상 (빨강~파랑)
        ratio = max(0.0, min(dist / 4.0, 1.0))
        m.color.r = 1.0 - ratio
        m.color.g = ratio
        m.color.b = 0.2
        m.color.a = 0.7
        m.lifetime = Duration(sec=1)
        return m

    def publish_markers(self, f, l, r):
        arr = MarkerArray()
        arr.markers.append(self.make_marker(0, f, 0.0, f))
        arr.markers.append(self.make_marker(1, 0.0, l, l))
        arr.markers.append(self.make_marker(2, 0.0, -r, r))
        self.marker_pub.publish(arr)

    # -----------------------
    # 주기적 콜백
    # -----------------------
    def timer_callback(self):
        front = self.measure_distance(*self.pins['front'])
        left = self.measure_distance(*self.pins['left'])
        right = self.measure_distance(*self.pins['right'])

        # 발행
        self.front_pub.publish(self.create_range_msg("ultrasonic_front_link", front))
        self.left_pub.publish(self.create_range_msg("ultrasonic_left_link", left))
        self.right_pub.publish(self.create_range_msg("ultrasonic_right_link", right))
        self.publish_markers(front, left, right)

        # 로깅
        elapsed = time.time() - self.start_time
        self.log_writer.writerow([round(elapsed, 2), f"{front:.3f}", f"{left:.3f}", f"{right:.3f}"])
        self.log_file.flush()

        # 1Hz 상태 출력
        if time.time() - self.last_status_time > 1.0:
            self.get_logger().info(
                f"Front={front:.2f}m  Left={left:.2f}m  Right={right:.2f}m"
            )
            self.last_status_time = time.time()

    # -----------------------
    # 종료 처리
    # -----------------------
    def destroy_node(self):
        if HAS_GPIO:
            GPIO.cleanup()
        if self.log_file:
            self.log_file.close()
            self.get_logger().info("CSV 로그 저장 완료")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
