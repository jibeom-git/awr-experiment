# sensors/camera.py
from picamera2 import Picamera2
import numpy as np
import cv2

class Camera:
    def __init__(self, width=640, height=480):
        self.cam = Picamera2()
        config = self.cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.cam.configure(config)
        self.cam.start()

    def capture(self) -> np.ndarray:
        frame = self.cam.capture_array()
        # 카메라가 거꾸로 장착되어 있어 180도 회전
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def close(self):
        self.cam.stop()
        self.cam.close()