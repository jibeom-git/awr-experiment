# sensors/motor.py
# DC 모터 드라이버
# PCA9685 I2C (0x5f) 기반 — 공식 Move.py 구조 기반
# M1: 채널 14/15, M2: 채널 12/13, M3: 채널 10/11, M4: 채널 8/9
#
# [수정 내역]
# 1. rotate_left / rotate_right 메서드 추가
#    - 서버(insite_dashboard_server.py)의 key_down 핸들러가
#      'rotate-left', 'rotate-right' 명령 사용 → 미구현 시 AttributeError
#    - 공식 Move.py 로직 기준: 좌측 회전 = 우측 바퀴 정방향 + 좌측 바퀴 역방향
# 2. move() 메서드 추가
#    - 서버가 move.move(speed, direction, turn) 인터페이스 호출
#    - 공식 Move.py와 동일한 시그니처로 래핑
# 3. mock 모드 추가 (PCA9685 없는 환경에서 import 크래시 방지)
# 4. stop() 예외 처리 강화

import time

try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import motor as adafruit_motor
    _HW_AVAILABLE = True
except (ImportError, Exception):
    _HW_AVAILABLE = False
    print("[Motor] adafruit 라이브러리 없음 — mock 모드로 동작")

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

# 회전 시 안쪽 바퀴 속도 비율 (differential turn radius)
TURN_RADIUS = 0.3


class MotorController:
    def __init__(self):
        self._mock = not _HW_AVAILABLE
        self.m1 = self.m2 = self.m3 = self.m4 = None
        self.pwm = None

        if self._mock:
            print("[Motor] mock 모드")
            return

        try:
            i2c = busio.I2C(SCL, SDA)
            self.pwm = PCA9685(i2c, address=0x5f)
            self.pwm.frequency = 50

            self.m1 = adafruit_motor.DCMotor(
                self.pwm.channels[M1_IN1], self.pwm.channels[M1_IN2])
            self.m2 = adafruit_motor.DCMotor(
                self.pwm.channels[M2_IN1], self.pwm.channels[M2_IN2])
            self.m3 = adafruit_motor.DCMotor(
                self.pwm.channels[M3_IN1], self.pwm.channels[M3_IN2])
            self.m4 = adafruit_motor.DCMotor(
                self.pwm.channels[M4_IN1], self.pwm.channels[M4_IN2])

            for m in [self.m1, self.m2, self.m3, self.m4]:
                m.decay_mode = adafruit_motor.SLOW_DECAY

        except Exception as e:
            print(f"[Motor] 초기화 실패 — mock 모드: {e}")
            self._mock = True

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────
    def _speed(self, speed_pct: int) -> float:
        """0~100 퍼센트를 0.0~1.0으로 변환"""
        return max(0.0, min(1.0, speed_pct / 100.0))

    def _set_throttle(self, m, direction: int, speed: float):
        """모터 개별 throttle 설정. 예외 발생 시 로그만 출력"""
        if m is None:
            return
        try:
            m.throttle = speed * direction
        except Exception as e:
            print(f"[Motor] throttle 설정 오류: {e}")

    def set_motor(self, channel: int, direction: int, speed_pct: int):
        """
        channel: 1~4
        direction: 1=정방향, -1=역방향
        speed_pct: 0~100
        """
        if self._mock:
            return
        s = self._speed(speed_pct)
        mapping = {
            1: (self.m1, M1_DIR),
            2: (self.m2, M2_DIR),
            3: (self.m3, M3_DIR),
            4: (self.m4, M4_DIR),
        }
        if channel not in mapping:
            return
        m, dir_corr = mapping[channel]
        self._set_throttle(m, direction * dir_corr, s)

    # ── 이동 명령 메서드 ───────────────────────────────────────────────────
    def forward(self, speed_pct: int = 50):
        """전진"""
        if self._mock:
            print(f"[Motor mock] forward {speed_pct}%")
            return
        s = self._speed(speed_pct)
        self._set_throttle(self.m1, M1_DIR,  s)
        self._set_throttle(self.m2, M2_DIR,  s)
        self._set_throttle(self.m3, M3_DIR,  s)
        self._set_throttle(self.m4, M4_DIR,  s)

    def backward(self, speed_pct: int = 50):
        """후진"""
        if self._mock:
            print(f"[Motor mock] backward {speed_pct}%")
            return
        s = self._speed(speed_pct)
        self._set_throttle(self.m1, -M1_DIR, s)
        self._set_throttle(self.m2, -M2_DIR, s)
        self._set_throttle(self.m3, -M3_DIR, s)
        self._set_throttle(self.m4, -M4_DIR, s)

    def rotate_left(self, speed_pct: int = 50):
        """
        [추가] 제자리 좌회전
        공식 Move.py 기준:
          우측 바퀴(M1, M2) 정방향 + 좌측 바퀴(M3, M4) 역방향
        """
        if self._mock:
            print(f"[Motor mock] rotate_left {speed_pct}%")
            return
        s = self._speed(speed_pct)
        self._set_throttle(self.m1,  M1_DIR, s)   # 우측 전진
        self._set_throttle(self.m2,  M2_DIR, s)
        self._set_throttle(self.m3, -M3_DIR, s)   # 좌측 후진
        self._set_throttle(self.m4, -M4_DIR, s)

    def rotate_right(self, speed_pct: int = 50):
        """
        [추가] 제자리 우회전
        좌측 바퀴(M3, M4) 정방향 + 우측 바퀴(M1, M2) 역방향
        """
        if self._mock:
            print(f"[Motor mock] rotate_right {speed_pct}%")
            return
        s = self._speed(speed_pct)
        self._set_throttle(self.m1, -M1_DIR, s)   # 우측 후진
        self._set_throttle(self.m2, -M2_DIR, s)
        self._set_throttle(self.m3,  M3_DIR, s)   # 좌측 전진
        self._set_throttle(self.m4,  M4_DIR, s)

    def turn_left(self, speed_pct: int = 50):
        """전진 좌조향 (안쪽 바퀴 감속)"""
        if self._mock:
            return
        s = self._speed(speed_pct)
        self._set_throttle(self.m1, M1_DIR, s)
        self._set_throttle(self.m2, M2_DIR, s)
        self._set_throttle(self.m3, M3_DIR, s * TURN_RADIUS)
        self._set_throttle(self.m4, M4_DIR, s * TURN_RADIUS)

    def turn_right(self, speed_pct: int = 50):
        """전진 우조향 (안쪽 바퀴 감속)"""
        if self._mock:
            return
        s = self._speed(speed_pct)
        self._set_throttle(self.m1, M1_DIR, s * TURN_RADIUS)
        self._set_throttle(self.m2, M2_DIR, s * TURN_RADIUS)
        self._set_throttle(self.m3, M3_DIR, s)
        self._set_throttle(self.m4, M4_DIR, s)

    def stop(self):
        """
        전체 모터 완전 정지.
        throttle = 0  → SLOW_DECAY 모드에서 브레이크 상태 유지 (완전 차단 아님)
        throttle = None → PCA9685 PWM 출력 자체를 0으로 만들어 완전 차단
        두 방법 모두 적용해 이중으로 정지를 보장한다.
        """
        if self._mock:
            print("[Motor mock] stop")
            return
        for m in [self.m1, self.m2, self.m3, self.m4]:
            if m is not None:
                try:
                    m.throttle = 0     # 브레이크
                except Exception:
                    pass
                try:
                    m.throttle = None  # PWM 완전 차단
                except Exception:
                    pass
        # PCA9685 채널 레벨에서도 직접 0으로 강제
        if self.pwm is not None:
            try:
                for ch in [M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                           M3_IN1, M3_IN2, M4_IN1, M4_IN2]:
                    self.pwm.channels[ch].duty_cycle = 0
            except Exception:
                pass

    # ── 공식 Move.py 호환 인터페이스 ──────────────────────────────────────
    def move(self, speed: int, direction: int, turn: str, radius: float = TURN_RADIUS):
        """
        [추가] 공식 Move.py의 move() 와 동일한 시그니처.
        서버에서 move.move(speed, direction, turn) 형태로 호출할 수 있도록 래핑.

        speed     : 0~100
        direction :  1=전진, -1=후진
        turn      : "mid", "left", "right", "rotate-left", "rotate-right"
        """
        if speed == 0:
            self.stop()
            return

        if direction == 1:
            if turn == "rotate-left":
                self.rotate_left(speed)
            elif turn == "rotate-right":
                self.rotate_right(speed)
            elif turn == "left":
                self.turn_left(speed)
            elif turn == "right":
                self.turn_right(speed)
            else:  # "mid"
                self.forward(speed)
        else:  # direction == -1
            self.backward(speed)

    def motorStop(self):
        """Move.py 호환 정지 메서드 별칭"""
        self.stop()

    def setup(self):
        """Move.py 호환 setup() 별칭 (이미 __init__에서 초기화되므로 no-op)"""
        pass

    def close(self):
        self.stop()
        if self.pwm is not None:
            try:
                self.pwm.deinit()
            except Exception:
                pass