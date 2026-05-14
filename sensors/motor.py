# sensors/motor.py
# DC 모터 드라이버
# PCA9685 I2C (0x5f) 기반 — 공식 Move.py 구조 기반
# M1: 채널 14/15, M2: 채널 12/13, M3: 채널 10/11, M4: 채널 8/9

import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

# PCA9685 채널 핀 번호 (공식 문서 기준)
M1_IN1, M1_IN2 = 15, 14
M2_IN1, M2_IN2 = 12, 13
M3_IN1, M3_IN2 = 11, 10
M4_IN1, M4_IN2 =  8,  9

# 모터 방향 보정 (로봇 구조상 좌/우 모터 극성 반전)
M1_DIR =  1
M2_DIR = -1
M3_DIR =  1
M4_DIR = -1

class MotorController:
    def __init__(self):
        i2c = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(i2c, address=0x5f)
        self.pwm.frequency = 50

        self.m1 = motor.DCMotor(self.pwm.channels[M1_IN1], self.pwm.channels[M1_IN2])
        self.m2 = motor.DCMotor(self.pwm.channels[M2_IN1], self.pwm.channels[M2_IN2])
        self.m3 = motor.DCMotor(self.pwm.channels[M3_IN1], self.pwm.channels[M3_IN2])
        self.m4 = motor.DCMotor(self.pwm.channels[M4_IN1], self.pwm.channels[M4_IN2])

        for m in [self.m1, self.m2, self.m3, self.m4]:
            m.decay_mode = motor.SLOW_DECAY

    def _speed(self, speed_pct: int) -> float:
        """0~100 퍼센트를 0.0~1.0으로 변환"""
        return max(0.0, min(1.0, speed_pct / 100.0))

    def set_motor(self, channel: int, direction: int, speed_pct: int):
        """
        channel: 1~4
        direction: 1=정방향, -1=역방향
        speed_pct: 0~100
        """
        s = self._speed(speed_pct) * direction
        motors = {
            1: (self.m1, M1_DIR),
            2: (self.m2, M2_DIR),
            3: (self.m3, M3_DIR),
            4: (self.m4, M4_DIR),
        }
        m, dir_corr = motors[channel]
        m.throttle = s * dir_corr

    def forward(self, speed_pct: int = 50):
        self.set_motor(1,  1, speed_pct)
        self.set_motor(2,  1, speed_pct)
        self.set_motor(3,  1, speed_pct)
        self.set_motor(4,  1, speed_pct)

    def backward(self, speed_pct: int = 50):
        self.set_motor(1, -1, speed_pct)
        self.set_motor(2, -1, speed_pct)
        self.set_motor(3, -1, speed_pct)
        self.set_motor(4, -1, speed_pct)

    def stop(self):
        for m in [self.m1, self.m2, self.m3, self.m4]:
            m.throttle = 0

    def close(self):
        self.stop()
        self.pwm.deinit()