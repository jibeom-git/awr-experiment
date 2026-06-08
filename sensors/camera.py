# sensors/camera.py
# OV5647 RGB 카메라 드라이버 (CSI, picamera2) 및 Orbbec Astra USB RGB 드라이버

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
        _empty = np.zeros((self._height, self._width, 3), dtype=np.uint8)

        if self._mock or not self._started or self.cam is None:
            return _empty

        try:
            frame = self.cam.capture_array()   # picamera2 RGB888

            if _CV2_AVAILABLE:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            else:
                frame = np.rot90(frame, 2).copy()

            return frame
        except Exception as e:
            print(f"[Camera] 프레임 캡처 오류: {e}")
            return _empty

    def is_opened(self) -> bool:
        return self._started and not self._mock

    def close(self):
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


class USBCamera:
    """
    [고도화] Orbbec Astra 카메라의 USB RGB 스트림을 전담 제어하는 클래스입니다.
    하드코딩된 인덱스 오류를 방지하기 위해 가용한 video 장치 노드를 자동 탐색합니다.
    """
    def __init__(self, device_index: int = 0, width: int = 640, height: int = 480):
        self._width = width
        self._height = height
        self.cap = None
        self._started = False
        self._device_index = device_index

        if not _CV2_AVAILABLE:
            print("[USBCamera] OpenCV 로드 불능으로 인한 Mock 모드 진입")
            return

        # 가용한 비디오 노드 목록을 순차 스캔하여 실제 프레임이 읽히는 장치 탐색
        # 오르벡 카메라는 대개 0, 2, 4, 6번 인덱스 중 하나에 RGB 채널이 할당됩니다.
        candidate_indices = [self._device_index, 0, 1, 2, 4, 6]
        
        for idx in candidate_indices:
            try:
                print(f"[USBCamera] 비디오 디바이스 노드 스캔 중... (Index: {idx})")
                test_cap = cv2.VideoCapture(idx)
                if test_cap.isOpened():
                    # 단순히 장치가 열리는 것뿐만 아니라, 실제 이미지 데이터가 파싱되는지 검증
                    ret, frame = test_cap.read()
                    if ret and frame is not None and frame.size > 0:
                        self.cap = test_cap
                        self._device_index = idx
                        self._started = True
                        print(f"[USBCamera] AI용 USB RGB 카메라 탐색 성공 (최종 바인딩 Index: {idx})")
                        break
                    else:
                        test_cap.release()
            except Exception as e:
                print(f"[USBCamera] 인덱스 {idx} 테스트 중 예외 발생: {e}")

        if not self._started:
            print("[USBCamera] 경고: 유효한 USB RGB 비디오 스트림 노드를 찾지 못했습니다. 디바이스 연결을 확인하십시오.")

    def capture(self) -> np.ndarray:
        """
        AI 전방 추론 및 대시보드 송출용 단일 프레임을 취득합니다.
        """
        _empty = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        if not self._started or self.cap is None:
            return _empty
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return _empty
            return frame
        except Exception as e:
            print(f"[USBCamera] 프레임 디코딩 오류: {e}")
            return _empty

    def close(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self._started = False
            self.cap = None
            print(f"[USBCamera] Index {self._device_index} 자원 반환 완료")