# tests/test_ultrasonic_lone.py
# 명시적으로 'ultra' 드라이버를 참조하여 단독 거동을 검증하는 프로그램

import os
import sys
import time

# 최상위 디렉토리 주소를 시스템 패스 버스에 강제 덤프
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import RPi.GPIO as GPIO # type: ignore
    # [수정] 기존 파일명인 ultra 패키지로부터 센서 클래스를 디스패치합니다.
    from sensors.ultra import UltrasonicSensor
except ImportError as e:
    print(f"[Import Error] 드라이버 참조 실패: {e}")
    sys.exit(1)

def main():
    # 하드웨어 핀 23(TRIG), 24(ECHO) 인터페이스 할당
    sonar = UltrasonicSensor(trigger_pin=23, echo_pin=24)
    
    print("="*60)
    print("      AGV ULTRASONIC ('ultra.py') REAL-TIME TESTER")
    print("="*60)
    print(" -> 실시간 거리 계측 연속 스캔을 개시합니다.")
    print(" -> 검증을 종료하려면 터미널에서 Ctrl + C 를 누르십시오.\n")
    print(f"{'Time Stamp':<15} | {'Measured Distance (cm)':<25} | {'Status Scan'}")
    print("-"*60)
    
    while True:
        try:
            # 5회 이동 평균 필터가 적용된 정밀 데이터 수집
            distance = sonar.read_average_distance(times=5)
            current_time = time.strftime('%H:%M:%S')
            
            if distance > 0:
                # 상위 AI 엔진의 감속 분기 임계 조건(35cm 및 15cm) 스위칭 매핑 검증
                status = "NORMAL_CRUISE" if distance > 35.0 else "AI_SLOWDOWN_ZONE"
                if distance <= 15.0:
                    status = "CRITICAL_STOP_WARN"
                
                print(f"{current_time:<15} | {distance:<25.2f} | [{status}]")
            else:
                print(f"{current_time:<15} | {'ERROR (OUT OF BOUND)':<25} | [MIS_SCAN_DROP]")
                
            time.sleep(0.2) # 하드웨어 부하 경감을 위한 샘플링 지연 마진
            
        except KeyboardInterrupt:
            print("\n[System] 초음파 계측 모니터링을 안전하게 종료합니다.")
            break
        except Exception as e:
            print(f"\n[Runtime Warning] 버스 데이터 인입 누수: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    finally:
        GPIO.cleanup()
        print("[System] GPIO 물리 자원 반환 완료.")
