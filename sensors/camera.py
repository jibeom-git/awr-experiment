# sensors/camera.py
# Raspberry Pi 카메라 드라이버 (picamera2 기반)
# OV5647 CSI 카메라

from picamera2 import Picamera2
import numpy as np

class Camera:
    def __init__(self, width=640, height=480):
        self.cam = Picamera2()
        config = self.cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.cam.configure(config)
        self.cam.start()

    def capture(self) -> np.ndarray:
        """BGR numpy 배열로 프레임 반환 (OpenCV 호환)"""
        frame = self.cam.capture_array()
        return frame

    def close(self):
        self.cam.stop()
        self.cam.close()