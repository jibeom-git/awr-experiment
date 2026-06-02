# tests/test_loadcell_lone.py
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import RPi.GPIO as GPIO # type: ignore
    from sensors.hx711 import HX711
except ImportError:
    print("[Error] 드라이버 종속성 모듈이 유실되었거나 가상환경 내부가 아닙니다.")
    sys.exit(1)

def main():
    hx = HX711(dout=5, pd_sck=6)
    
    print("="*60)
    print("      AGV LOADCELL SINGLE INDEPENDENT TESTER")
    print("="*60)
    
    hx.init_chassis_zero_calibration(times=20)
    
    print(" -> [MONITORING ACTIVE] 실시간 무게 데이터 스트림 스캔을 시작합니다.")
    print(" -> 관측을 안전하게 종료하려면 터미널에서 Ctrl + C 를 누르십시오.\n")
    print(f"{'Time Stamp':<15} | {'Raw ADC Digital':<20} | {'Converted Weight':<18}")
    print("-"*60)
    
    while True:
        try:
            raw_average = hx.read_average(times=3)
            current_weight = hx.get_weight(times=5)
            current_time = time.strftime('%H:%M:%S')
            print(f"{current_time:<15} | {raw_average:<20.1f} | {current_weight:<15.1f} g")
            time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[System] 로드셀 단독 계측 모니터링을 안전하게 종료합니다.")
            break
        except Exception as err:
            print(f"\n[Runtime Error] 데이터 버스 인터럽트 발생: {err}")
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    finally:
        GPIO.cleanup()
        print("[System] GPIO 물리 자원 반환 완료.")
