#!/usr/bin/env python3
"""
ultrasonic_node.py

초음파 센서 ROS2 노드
HY-SRF05 센서 3개 (전방, 좌측, 우측) 읽기

발행 토픽:
- /ultrasonic/front (sensor_msgs/Range)
- /ultrasonic/left (sensor_msgs/Range)
- /ultrasonic/right (sensor_msgs/Range)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time

# 기존 코드 재사용
from sensor.lib import config

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("[WARN] RPi.GPIO 없음 - 더미 데이터 모드")


class UltrasonicNode(Node):
    """초음파 센서 노드"""
    
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        # 파라미터
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('front_trig', config.ULTRA_FRONT_TRIG)
        self.declare_parameter('front_echo', config.ULTRA_FRONT_ECHO)
        self.declare_parameter('left_trig', config.ULTRA_LEFT_TRIG)
        self.declare_parameter('left_echo', config.ULTRA_LEFT_ECHO)
        self.declare_parameter('right_trig', config.ULTRA_RIGHT_TRIG)
        self.declare_parameter('right_echo', config.ULTRA_RIGHT_ECHO)
        
        update_rate = self.get_parameter('update_rate').value
        
        self.pins = {
            'front': (
                self.get_parameter('front_trig').value,
                self.get_parameter('front_echo').value
            ),
            'left': (
                self.get_parameter('left_trig').value,
                self.get_parameter('left_echo').value
            ),
            'right': (
                self.get_parameter('right_trig').value,
                self.get_parameter('right_echo').value
            )
        }
        
        # GPIO 초기화
        if HAS_GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            for direction, (trig, echo) in self.pins.items():
                GPIO.setup(trig, GPIO.OUT)
                GPIO.setup(echo, GPIO.IN)
                GPIO.output(trig, GPIO.LOW)
            
            time.sleep(0.1)
            self.get_logger().info('GPIO 초기화 완료')
        else:
            self.get_logger().warn('GPIO 없음 - 더미 모드')
        
        # 발행자
        self.front_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.left_pub = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.right_pub = self.create_publisher(Range, '/ultrasonic/right', 10)
        
        # 타이머
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'초음파 센서 노드 시작 ({update_rate}Hz)')
    
    def measure_distance(self, trig, echo):
        """
        거리 측정 (cm)
        
        Returns:
            float: 거리 (cm), 실패 시 999.0
        """
        if not HAS_GPIO:
            # 더미 데이터
            import random
            return random.uniform(30.0, 200.0)
        
        try:
            # 트리거
            GPIO.output(trig, GPIO.HIGH)
            time.sleep(0.00001)  # 10us
            GPIO.output(trig, GPIO.LOW)
            
            # 에코 대기 (최대 30ms)
            timeout = time.time() + 0.03
            
            # LOW → HIGH
            pulse_start = time.time()
            while GPIO.input(echo) == GPIO.LOW:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return 999.0
            
            # HIGH → LOW
            pulse_end = time.time()
            while GPIO.input(echo) == GPIO.HIGH:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return 999.0
            
            # 거리 계산
            duration = pulse_end - pulse_start
            distance = (duration * 34300) / 2  # cm
            
            # 유효 범위 확인
            if 2.0 <= distance <= 400.0:
                return distance
            else:
                return 999.0
        
        except Exception:
            return 999.0
    
    def timer_callback(self):
        """주기적 측정 및 발행"""
        stamp = self.get_clock().now().to_msg()
        
        # 전방
        front_dist = self.measure_distance(*self.pins['front'])
        front_msg = self.create_range_msg(stamp, 'ultrasonic_front', front_dist)
        self.front_pub.publish(front_msg)
        
        # 좌측
        left_dist = self.measure_distance(*self.pins['left'])
        left_msg = self.create_range_msg(stamp, 'ultrasonic_left', left_dist)
        self.left_pub.publish(left_msg)
        
        # 우측
        right_dist = self.measure_distance(*self.pins['right'])
        right_msg = self.create_range_msg(stamp, 'ultrasonic_right', right_dist)
        self.right_pub.publish(right_msg)
    
    def create_range_msg(self, stamp, frame_id, distance_cm):
        """Range 메시지 생성"""
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # 15도 (라디안)
        msg.min_range = 0.02  # 2cm
        msg.max_range = 4.0   # 400cm
        msg.range = distance_cm / 100.0  # m 단위
        return msg
    
    def destroy_node(self):
        """노드 종료"""
        if HAS_GPIO:
            GPIO.cleanup()
            self.get_logger().info('GPIO 정리 완료')
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
