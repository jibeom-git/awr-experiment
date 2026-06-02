# sensors/ultra.py
# HC-SR04 초음파 센서 드라이버 — GPIO 직접 제어 방식

import time

try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    _GPIO_AVAILABLE = False
    print("[Ultrasonic] RPi.GPIO 없음 — mock 모드로 동작")


class UltrasonicSensor:
    def __init__(self, trigger_pin: int = 23, echo_pin: int = 24):
        """Trig=GPIO23, Echo=GPIO24 핀 배정에 맞춰 초기화"""
        self.TRIG  = trigger_pin
        self.ECHO  = echo_pin
        self._mock = not _GPIO_AVAILABLE

        if self._mock:
            print("[Ultrasonic] mock 모드")
            return

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.TRIG, GPIO.OUT)
            GPIO.setup(self.ECHO, GPIO.IN)
            GPIO.output(self.TRIG, False)
            time.sleep(0.1)
        except Exception as e:
            print(f"[Ultrasonic] GPIO 초기화 실패 — mock 모드: {e}")
            self._mock = True

    def read_distance_cm(self) -> float:
        """단일 음파 펄스로 거리를 계측. 유효 범위 2~400cm, 이외는 -1.0 반환."""
        if self._mock:
            import random
            return round(random.uniform(30, 300), 1)

        try:
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            t0 = time.time()
            pulse_start = t0
            while GPIO.input(self.ECHO) == 0:
                pulse_start = time.time()
                if pulse_start - t0 > 0.02:
                    return -1.0

            pulse_end = time.time()
            t1 = pulse_end
            while GPIO.input(self.ECHO) == 1:
                pulse_end = time.time()
                if pulse_end - t1 > 0.02:
                    return -1.0

            dist = ((pulse_end - pulse_start) * 34300) / 2.0
            return float(dist) if 2.0 <= dist <= 400.0 else -1.0
        except Exception as e:
            print(f"[Ultrasonic] 계측 오류: {e}")
            return -1.0

    def read_average_distance(self, times: int = 5) -> float:
        """이동 평균 필터로 미스 스캔 억제"""
        samples = [self.read_distance_cm() for _ in range(times) if True]
        valid   = [d for d in samples if d > 0]
        if not valid:
            return -1.0
        return sum(valid) / len(valid)

    def get_distance(self) -> float:
        """대시보드 인터페이스용 단일 거리값 반환 (read_distance_cm 별칭)"""
        return self.read_distance_cm()

    def close(self):
        """GPIO 자원 반환"""
        if not self._mock:
            try:
                GPIO.cleanup([self.TRIG, self.ECHO])
            except Exception:
                pass
