# tests/test_motor.py
# DC 모터 단위 테스트
# 실행: python tests/test_motor.py
# 주의: 로봇을 바닥에서 들어올린 상태로 실행할 것

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.motor import MotorController
import time

if __name__ == "__main__":
    mc = MotorController()
    print("모터 테스트 시작 — 로봇을 들어올린 상태로 진행하세요")

    try:
        print("전진 50% — 2초")
        mc.forward(50)
        time.sleep(2)
        mc.stop()
        time.sleep(1)

        print("후진 50% — 2초")
        mc.backward(50)
        time.sleep(2)
        mc.stop()
        time.sleep(1)

        print("테스트 완료")
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        mc.close()