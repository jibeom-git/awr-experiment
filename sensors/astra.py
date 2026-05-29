# ~/insite/sensors/astra.py
import os
import sys
import ctypes
import cv2
import numpy as np
from openni import openni2

class AdvancedPerceptionFilter:
    def __init__(self):
        """화질 보정 알고리즘과 사각지대 메모리가 결합된 고급 인지 필터"""
        self.last_valid_center = 1200.0
        self.last_valid_min = 1000.0
        self.is_blind_zone_triggered = False
        self.prev_depth_smooth = None  # 시계열 smoothing용 버퍼
        self.alpha = 0.7  # 현재 프레임 반영 가중치 ($0 \le \alpha \le 1$)

    def process_filter(self, raw_depth, ultrasonic_cm: float):
        # 1. 런타임 수치 화질 개선 파이프라인
        depth_f32 = np.asarray(raw_depth, dtype=np.float32)
        
        # [에지 보존 노이즈 필터링]: 경계선은 살리고 노면 고주파 노이즈만 제거 (시각적/수치적 화질 극대화)
        # 파라미터: d=5 (픽셀 주변 직경), sigmaColor=50 (깊이 차이 허용치), sigmaSpace=50 (좌표 거리 허용치)
        depth_denoised = cv2.bilateralFilter(depth_f32, d=5, sigmaColor=50, sigmaSpace=50)
        
        # [시계열 잔상 정렬 (EMA)]: 프레임 변동 노이즈(Jittering)를 시간축으로 억제
        if self.prev_depth_smooth is None:
            self.prev_depth_smooth = depth_denoised
        else:
            self.prev_depth_smooth = self.alpha * depth_denoised + (1.0 - self.alpha) * self.prev_depth_smooth
            
        # 2. 보정된 고화질 행렬 기반으로 주행 트랙 관심 영역(ROI) 슬라이싱 ($200 \times 200$)
        roi = self.prev_depth_smooth[240:440, 220:420]
        total_pixels = roi.size
        
        # 모폴로지 보완 연산
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        roi_filtered = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        
        zero_pixels = np.sum(roi_filtered == 0.0)
        depth_hole_ratio = float((zero_pixels / total_pixels) * 100.0)
        valid_pixels = roi_filtered[roi_filtered > 0.0]
        
        # 사각지대 가드 판정
        if depth_hole_ratio > 85.0:
            if ultrasonic_cm < 60.0 or self.is_blind_zone_triggered:
                self.is_blind_zone_triggered = True
                return float(ultrasonic_cm * 10.0), 0.0, float(max(100.0, (ultrasonic_cm * 10.0) - 20.0)), 80.0, depth_hole_ratio, True
                
        # 3. 고화질 정형 특징량 연산
        if valid_pixels.size > 0:
            self.is_blind_zone_triggered = False
            
            roi_center = roi_filtered[90:110, 90:110]
            valid_center = roi_center[roi_center > 0.0]
            depth_center = float(np.median(valid_center)) if valid_center.size > 0 else float(np.median(valid_pixels))
            
            upper_rows = roi_filtered[0:5, :]
            lower_rows = roi_filtered[-5:, :]
            valid_upper = upper_rows[upper_rows > 0.0]
            valid_lower = lower_rows[lower_rows > 0.0]
            mean_upper = float(np.mean(valid_upper)) if valid_upper.size > 0 else depth_center
            mean_lower = float(np.mean(valid_lower)) if valid_lower.size > 0 else depth_center
            
            depth_gradient = mean_upper - mean_lower
            depth_min = float(np.min(valid_pixels))
            depth_std = float(np.std(valid_pixels))
            
            self.last_valid_center = depth_center
            self.last_valid_min = depth_min
            
            return depth_center, depth_gradient, depth_min, depth_std, depth_hole_ratio, False
        else:
            return 0.0, 0.0, 0.0, 0.0, depth_hole_ratio, False


class AstraDepthSensor:
    def __init__(self):
        self.lib_path = "/usr/local/lib/libOpenNI2.so"
        self.depth_stream = None
        self.is_initialized = False
        
        if not os.path.exists(self.lib_path):
            raise FileNotFoundError(f"[Astra] 라이브러리 파일 부재: {self.lib_path}")
            
        self._initialize_device()
        self.filter = AdvancedPerceptionFilter()

    def _initialize_device(self):
        try:
            openni2.initialize(os.path.dirname(self.lib_path))
            self.device = openni2.Device.open_any()
            self.depth_stream = self.device.create_depth_stream()
            self.depth_stream.start()
            print("[Astra] 화질 보정용 단안 뎁스 하드웨어 세션 독점 기동")
            self.is_initialized = True
        except Exception as e:
            print(f"[Astra] 하드웨어 개방 실패: {e}")
            self.close()
            sys.exit(1)

    def get_depth_frame(self):
        if not self.is_initialized or self.depth_stream is None:
            return np.zeros((480, 640), dtype=np.uint16)
        try:
            frame = self.depth_stream.read_frame()
            frame_data = frame.get_buffer_as_uint16()
            address = ctypes.addressof(frame_data) + 8
            
            buffer = (ctypes.c_uint16 * (480 * 640)).from_address(address)
            return np.frombuffer(buffer, dtype=np.uint16).reshape(480, 640).copy()
        except Exception as e:
            print(f"[Astra] 프레임 파싱 드롭: {e}")
            return np.zeros((480, 640), dtype=np.uint16)

    def close(self):
        if self.depth_stream is not None:
            self.depth_stream.stop()
        openni2.unload()