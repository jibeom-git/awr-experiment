# tests/run_calibration.py
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sensors.mpu6050 import MPU6050

if __name__ == "__main__":
    # IMU 인스턴스 초기화 (I2C5 버스 연결)
    imu = MPU6050(bus_id=5, address=0x68)
    
    # 1000개의 샘플을 채집하여 오프셋 계산 및 파일 저장
    imu.calibrate_sensors(samples=1000)