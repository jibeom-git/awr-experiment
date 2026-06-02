# tests/test_mpu6050.py
import sys
import os
import time

# 프로젝트 루트 경로를 빌드 타깃 패스에 주입
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.mpu6050 import MPU6050

if __name__ == "__main__":
    # I2C5 버스 기반 IMU 객체 선언
    imu = MPU6050(bus_id=5, address=0x68)
    print("실시간 상보 필터 각도 계측을 시작합니다. (종료: Ctrl+C)")
    
    try:
        while True:
            # sensors/mpu6050.py에 반영된 상보 필터 출력값 호출
            angles = imu.get_filtered_angles()
            
            # 한 줄 스트리밍 포맷으로 실시간 로깅 출력
            print(f"\r[IMU 데이터] Roll: {angles['roll']:.2f}° | Pitch: {angles['pitch']:.2f}° | Yaw_Rate: {angles['yaw_rate']:.2f}°/s", end="")
            time.sleep(0.01)  # 10ms 제어 루틴 유지 (I2C 버퍼 적체 차단)
    except KeyboardInterrupt:
        print("\n계측 인터럽트 발생으로 테스트를 안정적으로 종료합니다.")