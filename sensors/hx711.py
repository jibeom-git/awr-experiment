# sensors/hx711.py
# 라즈베리파이 GPIO 대응 HX711 자가 진단 및 영점 캘리브레이션 기본 탑재 드라이버

import time

try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    _GPIO_AVAILABLE = False
    print("[HX711] RPi.GPIO 없음 — mock 모드로 동작")

from ai_core.config import SCALE_LOADCELL  # 실측 교정 상수 동적 바인딩


class HX711:
    def __init__(self, dout: int = 5, pd_sck: int = 6, gain: int = 128):
        """
        gain 파라미터는 하위 호환성을 위해 수용하나 현재 구현에서는 128 고정 사용.
        생성 즉시 물리 GPIO 포트를 개방한다.
        """
        self.DOUT   = dout
        self.PD_SCK = pd_sck
        self.offset = 0.0
        self.scale  = SCALE_LOADCELL  # 교정 가중치 팩터 기본 탑재
        self._mock  = not _GPIO_AVAILABLE

        if self._mock:
            print("[HX711] mock 모드")
            return

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.PD_SCK, GPIO.OUT)
            GPIO.setup(self.DOUT, GPIO.IN)
        except Exception as e:
            print(f"[HX711] GPIO 초기화 실패 — mock 모드: {e}")
            self._mock = True

    def is_ready(self) -> bool:
        """24비트 컨버터 칩셋의 변환 가용 신호를 스캔"""
        if self._mock:
            return True
        return GPIO.input(self.DOUT) == 0

    def read_raw(self) -> float:
        """하드웨어 직렬 버스로부터 24비트 디지털 원시 신호를 수집"""
        if self._mock:
            import random
            return random.uniform(-50, 50) * abs(self.scale) + self.offset

        while not self.is_ready():
            time.sleep(0.001)

        raw_count = 0
        for _ in range(24):
            GPIO.output(self.PD_SCK, True)
            raw_count = raw_count << 1
            GPIO.output(self.PD_SCK, False)
            if GPIO.input(self.DOUT):
                raw_count += 1

        # 25번째 클럭: 채널 설정 게인 128 리셋
        GPIO.output(self.PD_SCK, True)
        GPIO.output(self.PD_SCK, False)

        # 2의 보수 부호 처리
        if raw_count & 0x800000:
            raw_count -= 0x1000000

        return float(raw_count)

    def read_average(self, times: int = 5) -> float:
        """산술 평균 필터를 적용한 다중 샘플링"""
        total = sum(self.read_raw() for _ in range(times))
        return total / float(times)

    # ── 공개 캘리브레이션 인터페이스 ──────────────────────────────────────────

    def tare(self, samples: int = 20):
        """로드셀 영점 보정 — 현재 하중을 0g 기준으로 설정"""
        print(f"[HX711] TARE 실행 중 ({samples} 샘플)...")
        self.offset = self.read_average(samples)
        print(f"[HX711] TARE 완료 (Offset: {self.offset:.1f})")

    def calibrate(self, known_grams: float, samples: int = 10):
        """알려진 무게를 올린 상태에서 스케일 팩터를 재계산"""
        if known_grams <= 0:
            return
        raw_with_weight = self.read_average(samples)
        self.scale = (raw_with_weight - self.offset) / known_grams
        print(f"[HX711] CAL 완료 (Scale: {self.scale:.4f}, Known: {known_grams}g)")

    def init_chassis_zero_calibration(self, times: int = 20):
        """부팅 시 차체 공차 영점 교정 (tare 별칭)"""
        print("=" * 50)
        print("[HARDWARE INIT] AGV 중량 센서 자가 영점 교정 개시")
        self.tare(samples=times)
        print("=" * 50)

    def get_weight(self, times: int = 5) -> float:
        """실시간 중량값 반환 (영점 및 스케일 보정 적용, 단위: g)"""
        if self.scale == 0:
            return 0.0
        raw_val = self.read_average(times)
        return float((raw_val - self.offset) / self.scale)
