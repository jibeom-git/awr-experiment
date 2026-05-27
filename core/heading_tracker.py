# core/heading_tracker.py
import time, threading

class HeadingTracker:
    def __init__(self, imu):
        self.imu     = imu
        self.yaw     = 0.0
        self._running = False

    def start(self):
        self._running = True
        self._last    = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            now = time.time()
            dt  = now - self._last
            self._last = now
            gz  = self.imu.get_gyro()['z']   # offset 보정된 값
            self.yaw += gz * dt               # 적분
            time.sleep(0.01)                  # 100Hz

    def get(self):
        return self.yaw

    def reset(self):
        self.yaw = 0.0

    def stop(self):
        self._running = False