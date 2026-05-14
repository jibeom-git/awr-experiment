# sensors/mpu6050.py
# MPU-6050 자이로/가속도 센서 드라이버
# I2C 버스 5번 (GPIO12: SDA, GPIO13: SCL)
# 레지스터 직접 접근 방식 (smbus2 사용)

import smbus2
import time

# I2C 버스 5번, MPU-6050 기본 주소 0x68
BUS_NUM   = 5
ADDR      = 0x68

# 주요 레지스터 주소
PWR_MGMT_1   = 0x6B  # 전원 관리 (슬립 해제용)
ACCEL_XOUT_H = 0x3B  # 가속도 X 상위 바이트 시작
GYRO_XOUT_H  = 0x43  # 자이로 X 상위 바이트 시작

# 스케일 팩터 (기본 설정 기준)
ACCEL_SCALE = 16384.0  # ±2g 범위 → LSB/g
GYRO_SCALE  = 131.0    # ±250°/s 범위 → LSB/(°/s)

class MPU6050:
    def __init__(self):
        self.bus = smbus2.SMBus(BUS_NUM)
        # 슬립 모드 해제 (PWR_MGMT_1 레지스터에 0 기록)
        self.bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

    def _read_word_2c(self, reg):
        """16비트 2의 보수 값 읽기"""
        high = self.bus.read_byte_data(ADDR, reg)
        low  = self.bus.read_byte_data(ADDR, reg + 1)
        val  = (high << 8) + low
        return val - 65536 if val >= 32768 else val

    def get_accel(self) -> dict:
        """가속도 데이터 반환 (단위: g)"""
        return {
            'x': round(self._read_word_2c(ACCEL_XOUT_H)     / ACCEL_SCALE, 4),
            'y': round(self._read_word_2c(ACCEL_XOUT_H + 2) / ACCEL_SCALE, 4),
            'z': round(self._read_word_2c(ACCEL_XOUT_H + 4) / ACCEL_SCALE, 4),
        }

    def get_gyro(self) -> dict:
        """자이로 데이터 반환 (단위: °/s)"""
        return {
            'x': round(self._read_word_2c(GYRO_XOUT_H)     / GYRO_SCALE, 4),
            'y': round(self._read_word_2c(GYRO_XOUT_H + 2) / GYRO_SCALE, 4),
            'z': round(self._read_word_2c(GYRO_XOUT_H + 4) / GYRO_SCALE, 4),
        }

    def close(self):
        self.bus.close()