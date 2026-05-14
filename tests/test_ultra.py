# tests/test_ultra.py
# HC-SR04 초음파 센서 단위 테스트
# 실행: python tests/test_ultra.py

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.ultra import get_distance, close
from time import sleep

if __name__ == "__main__":
    print("초음파 센서 테스트 시작 (Ctrl+C로 종료)")
    try:
        while True:
            dist = get_distance()
            print(f"거리: {dist} cm")
            sleep(0.2)
    except KeyboardInterrupt:
        print("\n테스트 종료")
    finally:
        close()