#!/usr/bin/env python3
"""
test_sensors.py

센서 개별 테스트
- LiDAR 단독 테스트
- 초음파 단독 테스트
- 장애물 감지 테스트
"""

import time
import sys

import config
from lidar_reader import LidarReader
from ultrasonic_reader import UltrasonicReader
from obstacle_detector import ObstacleDetector


def test_lidar():
    """LiDAR 센서 테스트"""
    print("\nLiDAR 센서 테스트")
    print("=" * 40)
    
    lidar = LidarReader()
    
    if not lidar.start():
        print("ERROR: LiDAR 시작 실패")
        return False
    
    try:
        print("\n전방 거리 측정 중... (Ctrl+C로 중단)\n")
        
        for i in range(20):
            distance = lidar.get_front_distance()
            
            print(f"[{i+1:2d}] 전방: {distance:6.2f}m", end='')
            
            if distance <= config.LIDAR_DANGER_DISTANCE:
                print("  🚨 위험!")
            elif distance <= config.LIDAR_WARNING_DISTANCE:
                print("  ⚠️  경고")
            else:
                print("  ✓ 안전")
            
            time.sleep(0.5)
        
        print("\n테스트 완료")
        return True
        
    except KeyboardInterrupt:
        print("\n중단")
        return False
    
    finally:
        lidar.stop()


def test_ultrasonic():
    """초음파 센서 테스트"""
    print("\n초음파 센서 테스트")
    print("=" * 40)
    
    ultrasonic = UltrasonicReader()
    
    if not ultrasonic.start():
        print("ERROR: 초음파 센서 시작 실패")
        return False
    
    try:
        print("\n3방향 거리 측정 중... (Ctrl+C로 중단)\n")
        
        for i in range(20):
            distances = ultrasonic.get_distances()
            
            print(f"[{i+1:2d}] "
                  f"전방:{distances['front']:5.2f}m  "
                  f"좌:{distances['left']:5.2f}m  "
                  f"우:{distances['right']:5.2f}m")
            
            if any(d <= config.ULTRASONIC_DANGER_DISTANCE for d in distances.values()):
                print("     🚨 위험 거리 감지!")
            
            time.sleep(0.5)
        
        print("\n테스트 완료")
        return True
        
    except KeyboardInterrupt:
        print("\n중단")
        return False
    
    finally:
        ultrasonic.stop()


def test_obstacle_detection():
    """장애물 감지 테스트"""
    print("\n장애물 감지 테스트")
    print("=" * 40)
    
    lidar = LidarReader()
    ultrasonic = UltrasonicReader()
    detector = ObstacleDetector(lidar, ultrasonic)
    
    # 센서 시작
    lidar.start()
    ultrasonic.start()
    
    time.sleep(2)
    
    try:
        print("\n감지 중... (Ctrl+C로 중단)\n")
        
        icons = {0: '✓', 1: '⚠️', 2: '🚨'}
        
        for i in range(40):
            detector.update()
            status = detector.get_status()
            
            icon = icons[status['danger_level']]
            
            print(f"[{i+1:2d}] {icon} "
                  f"전방:{status['front_distance']:5.2f}m "
                  f"좌:{status['left_distance']:5.2f}m "
                  f"우:{status['right_distance']:5.2f}m")
            
            if status['danger_level'] == 2:
                print(f"     🚨 위험! {status['danger_direction']} 방향")
            
            time.sleep(0.5)
        
        print("\n테스트 완료")
        return True
        
    except KeyboardInterrupt:
        print("\n중단")
        return False
    
    finally:
        lidar.stop()
        ultrasonic.stop()


def main():
    """메인 메뉴"""
    print("\n딜리봇 센서 테스트")
    print("1. LiDAR")
    print("2. 초음파")
    print("3. 장애물 감지")
    print("")
    
    choice = input("선택: ").strip()
    
    if choice == '1':
        test_lidar()
    elif choice == '2':
        test_ultrasonic()
    elif choice == '3':
        test_obstacle_detection()
    else:
        print("종료")


if __name__ == '__main__':
    main()
