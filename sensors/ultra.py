# sensors/ultra.py
# HC-SR04 초음파 센서 드라이버
# GPIO23: Trig, GPIO24: Echo
# gpiozero 라이브러리 사용 (공식 문서 기준)
#
# [수정 내역]
# - 모듈 레벨 즉시 초기화 제거 → UltrasonicSensor 클래스로 캡슐화
#   (import 시점에 GPIO가 없으면 크래시 나던 문제 해결)
# - get_distance() None 가드 추가 (측정 실패 시 -1.0 반환)
# - close() 후 재초기화 지원
# - 서버 insite_dashboard_server.py의 ultra.get_distance() 호출 인터페이스 유지

from gpiozero import DistanceSensor

TRIG = 23
ECHO = 24
MAX_DISTANCE_M = 2  # 최대 측정 거리 2m


class UltrasonicSensor:
    def __init__(self, trig: int = TRIG, echo: int = ECHO):
        self._trig   = trig
        self._echo   = echo
        self._sensor = None
        self._init()

    def _init(self):
        """DistanceSensor 인스턴스 생성. 실패 시 None으로 유지하여 상위 레이어에서 처리"""
        try:
            self._sensor = DistanceSensor(
                echo=self._echo,
                trigger=self._trig,
                max_distance=MAX_DISTANCE_M
            )
        except Exception as e:
            print(f"[Ultra] 초기화 실패: {e}")
            self._sensor = None

    def get_distance(self) -> float:
        """
        거리 측정 후 cm 단위로 반환.
        센서 미응답 또는 초기화 실패 시 -1.0 반환.
        """
        if self._sensor is None:
            return -1.0
        try:
            d = self._sensor.distance
            if d is None:
                return -1.0
            return round(d * 100.0, 2)
        except Exception as e:
            print(f"[Ultra] 측정 오류: {e}")
            return -1.0

    def close(self):
        """GPIO 자원 해제"""
        if self._sensor is not None:
            try:
                self._sensor.close()
            except Exception:
                pass
            self._sensor = None


# ── 하위 호환: 모듈 레벨 함수 인터페이스 유지 ────────────────────────────────
# 기존 코드가 ultra.get_distance() 형태로 호출할 경우를 위한 래퍼
# 단, 이 방식은 Pi 환경에서만 정상 동작 (GPIO 없으면 _instance.get_distance() = -1.0)
_instance: UltrasonicSensor | None = None

def _get_instance() -> UltrasonicSensor:
    global _instance
    if _instance is None:
        _instance = UltrasonicSensor()
    return _instance

def get_distance() -> float:
    return _get_instance().get_distance()

def close():
    if _instance is not None:
        _instance.close()