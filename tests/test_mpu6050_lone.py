# tests/test_mpu6050_lone.py
# IMU 모든 항목(6가지) 출력 및 완벽 영점 조절 기본 탑재 검증 프로그램

import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from sensors.mpu6050 import MPU6050
except ImportError as e:
    print(f"[Import Error] 드라이버 패키지 스캔 실패: {e}")
    sys.exit(1)

def main():
    imu = MPU6050(bus_id=5, address=0x68)
    
    print("="*80)
    print("      AGV IMU MPU6050 6-DOF FULL SYSTEM INDEPENDENT TESTER")
    print("="*80)
    
    # 1. [요구사항] 주행 전 정적 평지 기하학적 캘리브레이션 트리거
    imu.init_chassis_pitch_calibration(times=20)
    
    # 2. 방위각(Yaw)과 가상 위치 변위(pos_x, pos_y)를 출발점 기준 0.0으로 완전 초기화
    imu.reset_yaw()
    print(" -> [YAW/POSITION RESET] 방위각 및 상대 변위 레지스터 0.0 프리셋 완료")
    
    print(" -> 실시간 차량 주행 데이터 스트림 6개 파라미터 스캔을 개시합니다.")
    print(" -> 검증을 종료하려면 터미널에서 Ctrl + C 를 누르십시오.\n")
    
    # 6가지 출력 가시성 확보를 위한 데이터프레임 헤더 출력
    header = f"{'Time Stamp':<10} | {'Roll(deg)':<10} | {'Pitch(deg)':<10} | {'Yaw(deg)':<10} | {'Yaw_Rate':<10} | {'Pos_X(cm)':<10} | {'Pos_Y(cm)':<10}"
    print(header)
    print("-"*105)
    
    while True:
        try:
            # imu.get_all() 버스로부터 6가지 공학 데이터 팩 파싱
            imu_data = imu.get_all()
            
            roll_val     = imu_data.get("roll", 0.0)
            pitch_val    = imu_data.get("pitch", 0.0)
            yaw_val      = imu_data.get("yaw", 0.0)
            yaw_rate_val = imu_data.get("yaw_rate", 0.0)
            pos_x_val    = imu_data.get("pos_x", 0.0)
            pos_y_val    = imu_data.get("pos_y", 0.0)
            
            current_time = time.strftime('%H:%M:%S')
            
            # 포맷 정렬 연속 출력 집행
            print(f"{current_time:<10} | {roll_val:<10.1f} | {pitch_val:<10.1f} | {yaw_val:<10.1f} | {yaw_rate_val:<10.1f} | {pos_x_val:<10.1f} | {pos_y_val:<10.1f}")
            time.sleep(0.3)
            
        except KeyboardInterrupt:
            print("\n[System] IMU 가속도 계측 관측 인터페이스를 안전하게 종료합니다.")
            break
        except Exception as e:
            print(f"\n[Runtime Warning] 데이터 버스 수집 누수 예외 대피: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()