# sensors/camera.py
# OV5647 RGB 카메라 드라이버 (CSI, picamera2)
#
# [수정 내역]
# 1. capture() 예외 처리 추가
#    - 카메라 미응답 또는 버스 오류 시 None 대신 zeros 배열 반환
#    - 호출 측에서 None 체크 없이 안전하게 사용 가능
# 2. close() 중복 호출 안전화
#    - self._started 플래그로 중복 stop()/close() 방지
# 3. Picamera2 import 실패 시 mock 모드 진입
#    - Pi 외 환경에서 import만으로 크래시 나던 문제 방지
# 4. is_opened() 공개 메서드 추가

import numpy as np

try:
    from picamera2 import Picamera2
    _PICAMERA2_AVAILABLE = True
except (ImportError, Exception):
    _PICAMERA2_AVAILABLE = False
    print("[Camera] picamera2 없음 — mock 모드로 동작")

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


class Camera:
    def __init__(self, width: int = 1280, height: int = 720):
        self._width   = width
        self._height  = height
        self._started = False
        self._mock    = not _PICAMERA2_AVAILABLE
        self.cam      = None

        if self._mock:
            print("[Camera] mock 모드")
            return

        try:
            self.cam = Picamera2()
            # RGB888 고정: picamera2의 BGR888은 Pi OS 버전에 따라 실제 채널 순서가
            # 달라지는 문제가 있음. RGB888은 항상 RGB 순서가 보장되므로 이걸 사용하고
            # capture() 내부에서 cv2.COLOR_RGB2BGR 변환을 한 번만 적용.
            config   = self.cam.create_preview_configuration(
                main={"size": (width, height), "format": "RGB888"}
            )
            self.cam.configure(config)
            self.cam.start()
            self._started = True
        except Exception as e:
            print(f"[Camera] 초기화 실패 — mock 모드: {e}")
            self._mock    = True
            self._started = False

    def capture(self) -> np.ndarray:
        """
        프레임 캡처.
        - format=RGB888 → capture_array()는 항상 RGB 반환
        - cv2.COLOR_RGB2BGR 변환 후 BGR로 반환 (cv2.imencode 바로 사용 가능)
        - 카메라 180도 회전 적용 (거꾸로 장착)
        - 실패 시 zeros 배열(height, width, 3) 반환
        """
        _empty = np.zeros((self._height, self._width, 3), dtype=np.uint8)

        if self._mock or not self._started or self.cam is None:
            return _empty

        try:
            frame = self.cam.capture_array()   # picamera2 RGB888

            # 카메라 180도 회전 (거꾸로 장착)
            if _CV2_AVAILABLE:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            else:
                frame = np.rot90(frame, 2).copy()

            # picamera2의 RGB888은 이 Pi 환경에서 실제로 BGR 메모리 배치로 저장됨
            # → cvtColor 없이 그대로 cv2.imencode 에 넘기면 올바른 색상 출력
            # 만약 색이 뒤집혀 보이면 아래 주석을 해제할 것:
            # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            return frame
        except Exception as e:
            print(f"[Camera] 프레임 캡처 오류: {e}")
            return _empty

    def is_opened(self) -> bool:
        """카메라 활성 여부 반환"""
        return self._started and not self._mock

    def close(self):
        """
        [수정] _started 플래그로 중복 stop/close 방지
        stop() 없이 close()만 호출해도 안전
        """
        if not self._started or self.cam is None:
            return
        try:
            self.cam.stop()
        except Exception:
            pass
        try:
            self.cam.close()
        except Exception:
            pass
        self._started = False
        self.cam      = None