# tests/test_camera.py
# 카메라 단위 테스트
# 실행: python tests/test_camera.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.camera import Camera
import numpy as np

if __name__ == "__main__":
    print("카메라 테스트 시작")
    cam = Camera(width=640, height=480)

    try:
        frame = cam.capture()
        print(f"프레임 shape : {frame.shape}")
        print(f"dtype        : {frame.dtype}")
        print(f"픽셀 범위    : min={frame.min()}, max={frame.max()}")

        # 10프레임 연속 캡처 속도 측정
        import time
        t0 = time.time()
        for _ in range(10):
            cam.capture()
        elapsed = time.time() - t0
        print(f"10프레임 소요 : {elapsed:.2f}s ({10/elapsed:.1f} fps)")

        print("\n카메라 정상 동작 확인")
    finally:
        cam.close()