# tests/test_astra.py
# Orbbec Astra 깊이 카메라 단위 테스트
# 실행: python tests/test_astra.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.astra import AstraCamera
import time

if __name__ == "__main__":
    print("Astra 카메라 테스트 시작")
    cam = AstraCamera()

    try:
        print("깊이 프레임 10회 읽기")
        for i in range(10):
            depth = cam.get_depth_frame()
            center = cam.get_center_distance()
            print(f"[{i+1:2d}] shape={depth.shape} | 중앙 거리={center:.0f}mm | "
                  f"min={depth.min()} max={depth.max()}")
            time.sleep(0.2)
        print("\nAstra 정상 동작 확인")
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        cam.close()