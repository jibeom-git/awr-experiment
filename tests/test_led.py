# tests/test_led.py
# WS2812 LED 단위 테스트
# 실행: python tests/test_led.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.led import LEDController
import time

if __name__ == "__main__":
    led = LEDController()
    print("LED 테스트 시작")

    try:
        print("초록 — 정상 가동 (3초)")
        led.set_running()
        time.sleep(3)

        print("노랑 점멸 — 추론 중 (3초)")
        led.set_thinking()
        time.sleep(3)

        print("빨강 점멸 — 오류 (3초)")
        led.set_error()
        time.sleep(3)

        print("소등")
        led.off()
        print("테스트 완료")
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        led.close()