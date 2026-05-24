# python tests/calibration_gyro.py
# /home/pi/insite/tests/calibration_gyro.py

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.mpu6050 import MPU6050


def calibrate_gyro(imu, samples=300):

    print()
    print("===================================")
    print(" MPU6050 AUTO CALIBRATION")
    print(" DO NOT MOVE ROBOT")
    print("===================================")
    print()

    time.sleep(2)

    sx = sy = sz = 0.0

    for i in range(samples):

        gyro = imu.get_gyro_raw()

        sx += gyro['x']
        sy += gyro['y']
        sz += gyro['z']

        print(
            f"\r[{i+1}/{samples}] "
            f"x={gyro['x']:.3f} "
            f"y={gyro['y']:.3f} "
            f"z={gyro['z']:.3f}",
            end=''
        )

        time.sleep(0.005)

    offset = {
        'x': sx / samples,
        'y': sy / samples,
        'z': sz / samples
    }

    print()
    print()
    print("========== RESULT ==========")
    print(offset)
    print("============================")
    print()

    return offset


# 단독 실행 테스트용
if __name__ == "__main__":

    imu = MPU6050()

    offset = calibrate_gyro(imu)

    print(offset)