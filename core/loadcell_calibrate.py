#!/usr/bin/env python3
"""
~/insite/core/loadcell_calibrate.py
HX711 로드셀 터미널 캘리브레이션 스크립트

실행:
    source ~/insite/.venv/bin/activate
    python3 ~/insite/core/loadcell_calibrate.py

절차:
    Step 1. 바구니만 올린 상태에서 영점(tare)
    Step 2. 알고 있는 무게의 추를 올리고 캘리브레이션
    Step 3. 테스트 측정으로 정확도 확인
    Step 4. 저장 — ~/insite/loadcell_cal.txt 에 기록

결과 파일: ~/insite/loadcell_cal.txt
    line 1: OFFSET_A (영점 raw 값)
    line 2: REF_UNIT_A (raw/g 변환 계수)
"""

import sys
import os
import time
from statistics import median, stdev

# core/ 에서 실행하므로 상위 디렉토리(~/insite)를 path에 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── HX711 초기화 ─────────────────────────────────────────────────────────────
try:
    from sensors.hx711 import HX711
    hx = HX711(dout=5, pd_sck=6, gain=128)
    if hx._mock:
        print("[ERROR] HX711이 mock 모드입니다. GPIO를 확인하세요.")
        sys.exit(1)
    print("[OK] HX711 초기화 완료")
except Exception as e:
    print(f"[ERROR] HX711 초기화 실패: {e}")
    sys.exit(1)

CAL_FILE = os.path.expanduser('~/insite/loadcell_cal.txt')

# ── 유틸 ─────────────────────────────────────────────────────────────────────
def divider(char='─', n=52):
    print(char * n)

def read_stable(n=20, interval=0.1, label='측정 중'):
    """n회 읽어서 이상값 제거 후 평균 반환"""
    print(f"  {label} ({n}샘플, {n*interval:.0f}초)...", end='', flush=True)
    vals = []
    for _ in range(n):
        vals.append(float(hx.getLong()))
        time.sleep(interval)
        print('.', end='', flush=True)
    print()
    med = median(vals)
    sd  = stdev(vals) if len(vals) > 1 else 0.0
    # 3σ 클리핑
    clean = [v for v in vals if abs(v - med) <= 3 * sd] if sd > 0 else vals
    avg   = sum(clean) / len(clean)
    print(f"  raw 평균={avg:.0f}  표준편차={sd:.0f}  유효샘플={len(clean)}/{n}")
    return avg, sd

def save_cal(offset, ref_unit):
    with open(CAL_FILE, 'w') as f:
        f.write(f"{offset}\n{ref_unit}\n")
    print(f"  저장 완료: {CAL_FILE}")

def load_cal():
    if not os.path.exists(CAL_FILE):
        return None, None
    try:
        with open(CAL_FILE) as f:
            lines = f.read().splitlines()
        return float(lines[0]), float(lines[1])
    except:
        return None, None

# ── 메인 ─────────────────────────────────────────────────────────────────────
def main():
    divider('═')
    print("  INSITE LoadCell 터미널 캘리브레이션")
    divider('═')

    # 기존 캘리브레이션 확인
    old_offset, old_ref = load_cal()
    if old_offset is not None and old_ref is not None:
        print(f"\n기존 캘리브레이션 발견:")
        print(f"  OFFSET={old_offset:.0f}  REF_UNIT={old_ref:.4f}")
        hx.OFFSET_A   = float(old_offset)
        hx.REF_UNIT_A = float(old_ref)

    while True:
        divider()
        print("메뉴:")
        print("  1. 현재 raw 값 실시간 모니터링")
        print("  2. 영점(tare) 설정")
        print("  3. 캘리브레이션 (추 무게 입력)")
        print("  4. 현재 설정으로 무게 테스트 측정")
        print("  5. 캘리브레이션 저장")
        print("  6. 종료")
        divider()

        try:
            choice = input("선택 (1-6): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n종료")
            break

        # ── 1. 실시간 모니터링 ──────────────────────────────────────────
        if choice == '1':
            print("\n실시간 raw 값 (Ctrl+C로 중단)")
            try:
                while True:
                    raw = hx.getLong()
                    grams = (raw - hx.OFFSET_A) / hx.REF_UNIT_A if hx.REF_UNIT_A != 1.0 else 0
                    print(f"\r  raw={raw:>10}  offset={hx.OFFSET_A:>10.0f}  diff={raw-hx.OFFSET_A:>10.0f}  {grams:>8.1f}g    ", end='', flush=True)
                    time.sleep(0.2)
            except KeyboardInterrupt:
                print("\n모니터링 종료")

        # ── 2. 영점 설정 ────────────────────────────────────────────────
        elif choice == '2':
            print("\n[Step 1] 영점(tare) 설정")
            print("  바구니(용기)만 로드셀에 올린 상태인지 확인하세요.")
            print("  물체가 완전히 정지한 뒤 Enter를 누르세요.")
            try:
                input("  준비되면 Enter: ")
            except (KeyboardInterrupt, EOFError):
                continue

            avg, sd = read_stable(n=50, interval=0.1, label='영점 측정')

            # 표준편차가 너무 크면 경고
            if sd > abs(avg) * 0.01 and sd > 1000:
                print(f"  [경고] 표준편차({sd:.0f})가 큽니다. 진동이 있거나 물체가 흔들리는 것 같습니다.")
                try:
                    yn = input("  그래도 적용하시겠습니까? (y/N): ").strip().lower()
                    if yn != 'y':
                        continue
                except (KeyboardInterrupt, EOFError):
                    continue

            hx.OFFSET_A = avg
            hx._buf.clear()
            print(f"\n  [완료] OFFSET_A = {hx.OFFSET_A:.0f}")
            print("  (아직 저장되지 않음 — 메뉴 5번으로 저장)")

        # ── 3. 캘리브레이션 ─────────────────────────────────────────────
        elif choice == '3':
            if hx.OFFSET_A == 0.0:
                print("\n  [오류] 먼저 영점(tare)을 설정하세요. (메뉴 2)")
                continue

            print("\n[Step 2] 캘리브레이션")
            print("  알고 있는 무게의 추를 로드셀(바구니)에 올리세요.")
            print("  예: 100g 추, 500g 추 등 정확히 알고 있는 무게")

            try:
                known_str = input("  올린 추의 무게(g)를 입력하세요: ").strip()
                known_g   = float(known_str)
                if known_g <= 0:
                    raise ValueError
            except (ValueError, KeyboardInterrupt, EOFError):
                print("  [오류] 양수 숫자를 입력하세요.")
                continue

            try:
                input(f"  {known_g}g 추를 올린 뒤 Enter: ")
            except (KeyboardInterrupt, EOFError):
                continue

            avg, sd = read_stable(n=30, interval=0.1, label=f'{known_g}g 측정')
            diff = avg - hx.OFFSET_A

            print(f"\n  raw 차이(diff): {diff:.0f}")

            if abs(diff) < 1000:
                print(f"  [경고] diff({diff:.0f})가 너무 작습니다. 추를 올렸는지, offset이 맞는지 확인하세요.")
                continue

            ref = diff / known_g
            print(f"  계산된 REF_UNIT = {diff:.0f} / {known_g} = {ref:.4f}")

            # 검증: 이 REF_UNIT으로 몇 g인지
            measured = diff / ref
            print(f"  검증: {diff:.0f} / {ref:.4f} = {measured:.1f}g (입력값: {known_g}g)")

            try:
                yn = input(f"\n  REF_UNIT={ref:.4f} 을 적용하시겠습니까? (y/N): ").strip().lower()
                if yn != 'y':
                    continue
            except (KeyboardInterrupt, EOFError):
                continue

            hx.REF_UNIT_A = ref
            hx._buf.clear()
            print(f"\n  [완료] REF_UNIT_A = {hx.REF_UNIT_A:.4f}")
            print("  (아직 저장되지 않음 — 메뉴 5번으로 저장)")

        # ── 4. 테스트 측정 ──────────────────────────────────────────────
        elif choice == '4':
            if hx.OFFSET_A == 0.0 or hx.REF_UNIT_A == 1.0:
                print("\n  [경고] 영점/캘리브레이션이 설정되지 않았습니다.")

            print(f"\n현재 설정: OFFSET={hx.OFFSET_A:.0f}  REF_UNIT={hx.REF_UNIT_A:.4f}")
            print("실시간 무게 측정 (Ctrl+C로 중단)")
            try:
                while True:
                    grams = hx.get_grams()
                    bar   = '█' * min(int(abs(grams) / 10), 30)
                    sign  = '+' if grams >= 0 else ''
                    print(f"\r  {sign}{grams:>8.1f} g  {bar}    ", end='', flush=True)
                    time.sleep(0.2)
            except KeyboardInterrupt:
                print("\n측정 종료")

        # ── 5. 저장 ─────────────────────────────────────────────────────
        elif choice == '5':
            if hx.OFFSET_A == 0.0:
                print("\n  [오류] 영점을 먼저 설정하세요.")
                continue
            print(f"\n저장할 값:")
            print(f"  OFFSET_A   = {hx.OFFSET_A:.0f}")
            print(f"  REF_UNIT_A = {hx.REF_UNIT_A:.4f}")
            try:
                yn = input(f"  {CAL_FILE} 에 저장하시겠습니까? (y/N): ").strip().lower()
                if yn != 'y':
                    continue
            except (KeyboardInterrupt, EOFError):
                continue
            save_cal(hx.OFFSET_A, hx.REF_UNIT_A)
            print("  서버 재시작 시 자동으로 적용됩니다.")

        # ── 6. 종료 ─────────────────────────────────────────────────────
        elif choice == '6':
            print("종료")
            break
        else:
            print("  1~6 중 하나를 선택하세요.")

    hx.close()

if __name__ == '__main__':
    main()