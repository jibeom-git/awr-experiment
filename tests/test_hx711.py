# tests/test_hx711.py
# HX711 로드셀 단위 테스트
# 실행: python tests/test_hx711.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.hx711 import HX711
from time import sleep

if __name__ == "__main__":
    hx = HX711()
    print("HX711 테스트 시작")
    print("영점 설정 중... (로드셀에 아무것도 올리지 마세요)")
    sleep(1)
    hx.tare(samples=10)
    print("영점 완료. 이제 물체를 올려보세요. (Ctrl+C로 종료)")

    try:
        while True:
            raw = hx.get_raw()
            val = hx.get_grams()
            print(f"RAW: {raw:10d}  |  보정값: {val:10.1f}")
            sleep(0.3)
    except KeyboardInterrupt:
        print("\n테스트 종료")
    finally:
        hx.close()