# tests/calibrate_hx711.py
# HX711 로드셀 캘리브레이션 스크립트
# 사용법: python tests/calibrate_hx711.py
# 결과:  REF_UNIT 값을 sensors/hx711.py에 입력하면 됨

import sys
sys.path.insert(0, '/home/pi/insite')
from sensors.hx711 import HX711
import time

def main():
    print("=" * 45)
    print("   HX711 로드셀 캘리브레이션")
    print("=" * 45)

    hx = HX711()

    # ── Step 1: 영점 (아무것도 없는 상태) ──────────────
    print("\n[Step 1] 로드셀 위에 아무것도 올리지 마세요.")
    input("         준비되면 Enter ▶ ")
    print("         영점 측정 중... (20회 평균)")
    hx.tare(samples=20)
    empty_raw = sum(hx.read()['raw'] for _ in range(10)) / 10
    print(f"         영점 raw 값: {empty_raw:.0f}")

    # ── Step 2: 알고 있는 무게 올리기 ──────────────────
    print("\n[Step 2] 무게를 아는 물체를 로드셀 위에 올리세요.")
    weight_str = input("         물체의 무게(g)를 입력하세요: ")
    known_weight = float(weight_str)
    input("         물체를 올린 후 Enter ▶ ")

    print("         무게 측정 중... (20회 평균)")
    loaded_raw = sum(hx.read()['raw'] for _ in range(20)) / 20
    print(f"         하중 raw 값: {loaded_raw:.0f}")

    # ── REF_UNIT 계산 ───────────────────────────────────
    ref_unit = (loaded_raw - empty_raw) / known_weight
    print()
    print("=" * 45)
    print(f"  ✓ REF_UNIT = {ref_unit:.2f}")
    print("=" * 45)

    # ── Step 3: 검증 ────────────────────────────────────
    hx.setReferenceUnit(ref_unit)
    print("\n[Step 3] 검증 — 같은 물체로 측정값 확인")
    print("         (물체 그대로 두고 Enter)")
    input("         Enter ▶ ")
    for i in range(5):
        grams = hx.get_grams()
        print(f"         측정값 {i+1}: {grams:.1f}g  (목표: {known_weight}g)")
        time.sleep(0.5)

    # ── 결과 안내 ───────────────────────────────────────
    print()
    print("─" * 45)
    print("  sensors/hx711.py 에 아래 줄 추가하세요:")
    print()
    print(f"  OFFSET   = {empty_raw:.0f}")
    print(f"  REF_UNIT = {ref_unit:.2f}")
    print()
    print("  또는 dashboard.py 초기화 부분에:")
    print(f"  hx711.OFFSET_A   = {empty_raw:.0f}")
    print(f"  hx711.REF_UNIT_A = {ref_unit:.2f}")
    print("─" * 45)

    hx.close()

if __name__ == '__main__':
    main()
