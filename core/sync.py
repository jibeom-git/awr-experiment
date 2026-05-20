# core/sync.py
# 멀티센서 동기화 모듈
# 각 센서를 독립 스레드로 실행하고 공유 큐로 데이터 집계
# 타임스탬프 기준으로 동기화된 단일 프레임 생성

import threading
import time
import queue
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

@dataclass
class SensorFrame:
    """동기화된 센서 데이터 단위"""
    timestamp: float = field(default_factory=time.time)

    # RGB 카메라
    rgb: Optional[np.ndarray] = None

    # 깊이 카메라
    depth: Optional[np.ndarray] = None

    # MPU-6050
    accel: Optional[dict] = None   # x, y, z (g)
    gyro:  Optional[dict] = None   # x, y, z (°/s)

    # HX711
    weight: Optional[float] = None  # raw diff

    # HC-SR04
    distance: Optional[float] = None  # cm

class SensorSynchronizer:
    def __init__(self, use_rgb=True, use_depth=True,
                 use_imu=True, use_weight=True, use_ultra=True):

        self.use_rgb    = use_rgb
        self.use_depth  = use_depth
        self.use_imu    = use_imu
        self.use_weight = use_weight
        self.use_ultra  = use_ultra

        # 최신 센서값 저장 (스레드 안전)
        self._lock   = threading.Lock()
        self._latest = SensorFrame()

        # 출력 큐 (외부에서 소비)
        self.output_queue = queue.Queue(maxsize=10)

        self._running = False
        self._threads = []

        self._init_sensors()

    def _init_sensors(self):
        """센서 초기화"""
        import sys
        sys.path.insert(0, '/home/pi/insite')

        if self.use_rgb:
            from sensors.camera import Camera
            self.cam = Camera(width=640, height=480)

        if self.use_depth:
            from sensors.astra import AstraCamera
            self.astra = AstraCamera()

        if self.use_imu:
            from sensors.mpu6050 import MPU6050
            self.imu = MPU6050()

        if self.use_weight:
            from sensors.hx711 import HX711
            self.hx711 = HX711()
            self.hx711.tare(samples=5)

        if self.use_ultra:
            from sensors.ultra import get_distance
            self._get_distance = get_distance

        print("센서 초기화 완료")

    # ── 각 센서 수집 스레드 ────────────────────────────────

    def _rgb_loop(self):
        while self._running:
            try:
                frame = self.cam.capture()
                with self._lock:
                    self._latest.rgb = frame
            except Exception as e:
                print(f"[RGB] 오류: {e}")

    def _depth_loop(self):
        while self._running:
            try:
                depth = self.astra.get_depth_frame()
                with self._lock:
                    self._latest.depth = depth
            except Exception as e:
                print(f"[Depth] 오류: {e}")

    def _imu_loop(self):
        while self._running:
            try:
                accel = self.imu.get_accel()
                gyro  = self.imu.get_gyro()
                with self._lock:
                    self._latest.accel = accel
                    self._latest.gyro  = gyro
                time.sleep(0.01)  # 100Hz
            except Exception as e:
                print(f"[IMU] 오류: {e}")

    def _weight_loop(self):
        while self._running:
            try:
                data = self.hx711.read()
                with self._lock:
                    self._latest.weight = data['diff']
            except Exception as e:
                print(f"[Weight] 오류: {e}")

    def _ultra_loop(self):
        while self._running:
            try:
                dist = self._get_distance()
                with self._lock:
                    self._latest.distance = dist
                time.sleep(0.05)  # 20Hz
            except Exception as e:
                print(f"[Ultra] 오류: {e}")

    def _aggregator_loop(self, fps=10):
        """지정 fps로 현재 최신값을 스냅샷해서 output_queue에 추가"""
        interval = 1.0 / fps
        while self._running:
            t0 = time.time()
            with self._lock:
                frame = SensorFrame(
                    timestamp = time.time(),
                    rgb       = self._latest.rgb.copy() if self._latest.rgb is not None else None,
                    depth     = self._latest.depth.copy() if self._latest.depth is not None else None,
                    accel     = dict(self._latest.accel) if self._latest.accel else None,
                    gyro      = dict(self._latest.gyro) if self._latest.gyro else None,
                    weight    = self._latest.weight,
                    distance  = self._latest.distance,
                )

            # 큐가 가득 차면 오래된 것 버림
            if self.output_queue.full():
                try:
                    self.output_queue.get_nowait()
                except:
                    pass
            self.output_queue.put(frame)

            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── 시작/종료 ──────────────────────────────────────────

    def start(self, fps=10):
        self._running = True
        targets = []

        if self.use_rgb:    targets.append(self._rgb_loop)
        if self.use_depth:  targets.append(self._depth_loop)
        if self.use_imu:    targets.append(self._imu_loop)
        if self.use_weight: targets.append(self._weight_loop)
        if self.use_ultra:  targets.append(self._ultra_loop)

        for target in targets:
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

        agg = threading.Thread(
            target=self._aggregator_loop, args=(fps,), daemon=True
        )
        agg.start()
        self._threads.append(agg)

        print(f"동기화 시작 ({fps}fps)")

    def stop(self):
        self._running = False
        if self.use_rgb:    self.cam.close()
        if self.use_depth:  self.astra.close()
        if self.use_imu:    self.imu.close()
        if self.use_weight: self.hx711.close()
        print("동기화 종료")