# tests/test_mpu6050.py
# MPU-6050 단위 테스트
# 실행: python tests/test_mpu6050.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.mpu6050 import MPU6050
from time import sleep

if __name__ == "__main__":
    imu = MPU6050()
    print("MPU-6050 테스트 시작 (Ctrl+C로 종료)")
    try:
        while True:
            accel = imu.get_accel()
            gyro  = imu.get_gyro()
            print(f"가속도 (g)  | X: {accel['x']:7.4f}  Y: {accel['y']:7.4f}  Z: {accel['z']:7.4f}")
            print(f"자이로 (°/s)| X: {gyro['x']:7.4f}  Y: {gyro['y']:7.4f}  Z: {gyro['z']:7.4f}")
            print("-" * 55)
            sleep(0.3)
    except KeyboardInterrupt:
        print("\n테스트 종료")
    finally:
        imu.close()