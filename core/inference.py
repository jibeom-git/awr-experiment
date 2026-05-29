# ~/insite/core/inference.py
import os
import cv2
import joblib
import numpy as np

class AdvancedPerceptionFilter:
    def __init__(self):
        """
        하드웨어 광학 사각지대(Dead Zone) 및 적외선 흡수 요철을 
        소프트웨어 레벨에서 필터링하기 위한 시계열 메모리 버퍼 초기화
        """
        self.last_valid_center = 1200.0  # 초기 베이스라인 물리 거리 (mm)
        self.last_valid_min = 1000.0
        self.is_blind_zone_triggered = False

    def process_filter(self, raw_depth, ultrasonic_cm):
        """
        공간적 모폴로지와 이종 센서(초음파) 차원 융합을 처리하는 런타임 루틴
        """
        # 하단 노면 주행 트랙 관심 영역 슬라이싱 (세로 240~440, 가로 220~420)
        # 연산 오버헤드를 제어하기 위해 전체 640x480 행렬을 200x200 행렬로 축소
        roi = raw_depth[240:440, 220:420]
        total_pixels = roi.size
        
        # 1. 공간 보완: 5x5 사각형 커널 기반 Morphology Closing 연산 수행
        # 표면 검은색 테이프 등으로 발생하는 격자 유실(Hole) 노이즈를 주변 평균값으로 메움
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        roi_filtered = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        
        # 유실율(Hole Ratio) 정량화 계산
        zero_pixels = np.sum(roi_filtered == 0)
        depth_hole_ratio = (zero_pixels / total_pixels) * 100.0
        valid_pixels = roi_filtered[roi_filtered > 0]
        
        # 2. 시계열 및 초음파 데이터 가상 융합 판정 (Dead-Reckoning)
        # 결손율이 85%를 초과하여 카메라 눈멀림이 발생했을 때
        if depth_hole_ratio > 85.0:
            # 데드존이 없는 초음파 센서가 근거리(60cm 이내) 차단을 가리키는 경우 초근접 상태 확정
            if ultrasonic_cm < 60.0 or self.is_blind_zone_triggered:
                self.is_blind_zone_triggered = True
                
                # 수치가 0으로 튀어 추론이 마비되는 것을 방지하기 위해 가상 정형 데이터 역주입
                depth_center = float(ultrasonic_cm * 10.0)  # cm -> mm 단위 스케일 변환
                depth_min = float(max(100.0, (ultrasonic_cm * 10.0) - 20.0))
                depth_gradient = 0.0  # 수직 벽면 상태 추정
                depth_std = 80.0      # 단차 분산 강제 부여
                
                return depth_center, depth_gradient, depth_min, depth_std, depth_hole_ratio
                
        # 3. 정상 계측 범위 내부인 경우 4대 핵심 특징량 통계 수치 연산
        if len(valid_pixels) > 0:
            self.is_blind_zone_triggered = False
            
            # depth_center: ROI 중심 20x20 국소 영역의 중앙값(Median)
            roi_center = roi_filtered[90:110, 90:110]
            valid_center = roi_center[roi_center > 0]
            depth_center = np.median(valid_center) if len(valid_center) > 0 else np.median(valid_pixels)
            
            # depth_gradient: 상단 5행과 하단 5행의 유효 평균 깊이 차이 (트랙 경사도 판별 변수)
            upper_rows = roi_filtered[0:5, :]
            lower_rows = roi_filtered[-5:, :]
            valid_upper = upper_rows[upper_rows > 0]
            valid_lower = lower_rows[lower_rows > 0]
            mean_upper = np.mean(valid_upper) if len(valid_upper) > 0 else depth_center
            mean_lower = np.mean(valid_lower) if len(valid_lower) > 0 else depth_center
            depth_gradient = mean_upper - mean_lower
            
            # depth_min & depth_std 연산
            depth_min = np.min(valid_pixels)
            depth_std = np.std(valid_pixels)
            
            # 시계열 직전 프레임 메모리 버퍼 업데이트
            self.last_valid_center = depth_center
            self.last_valid_min = depth_min
            
            return float(depth_center), float(depth_gradient), float(depth_min), float(depth_std), depth_hole_ratio
        else:
            return 0.0, 0.0, 0.0, 0.0, depth_hole_ratio


class RobotInference:
    def __init__(self, model_dir="~/insite/models/"):
        """사전 컴파일된 가중치 및 정규화 스케일러 파일을 바인딩하는 추론 엔진"""
        base_path = os.path.expanduser(model_dir)
        
        # 파일 경로의 유효성 가드 확인 후 직직렬화 인스턴스 로드
        try:
            self.scaler = joblib.load(os.path.join(base_path, "scaler.pkl"))
            self.classifier = joblib.load(os.path.join(base_path, "obstacle_classifier.pkl"))
            self.selector = joblib.load(os.path.join(base_path, "path_selector.pkl"))
            self.models_loaded = True
        except FileNotFoundError:
            # 데이터 수집 전 단계에서는 하위 호환성을 위해 추론 바이패스 모드 설정
            print("[Inference] 사전 학습된 pkl 파일이 부재합니다. 바이패스 주행 모드로 기동합니다.")
            self.models_loaded = False
            
        # 고급 보정 필터 내장
        self.filter = AdvancedPerceptionFilter()

    def predict(self, sensor_frame, raw_depth_matrix) -> dict:
        """
        13차원 확장 피처 세그먼트를 정렬하여 XGBoost 분류 및 제어 신호를 출력
        """
        ultrasonic_dist = sensor_frame.distance 
        
        # 보정 필터를 통과한 무결성 깊이 4대 요약 특징량 및 유실율 확보
        dc, dg, dmin, dstd, dhole = self.filter.process_filter(raw_depth_matrix, ultrasonic_dist)
        
        # [규칙 기반 비상 제어 바이패스 가드]
        # 모델 추론 연산 전, 초근접 완전 유실 혹은 기계적 제동 한계거리(150mm) 내부 진입 시 즉각 정지 신호 반환
        if dhole > 95.0 or (0.0 < dmin < 150.0):
            return {
                "obstacle_label": 3, "path_label": 2, "passable": False, "speed": "stop", "hole_ratio": dhole
            }

        # 학습 모델이 아직 로드되지 않은 초기 데이터 수집 단계의 경우 기본 패싱 구조 반환
        if not self.models_loaded:
            return {
                "obstacle_label": 0, "path_label": 0, "passable": True, "speed": "fast", "hole_ratio": dhole,
                "features": [dc, dg, dmin, dstd, dhole] # 수집용 원시 특징 배열 전달
            }

        # XGBoost 입력 데이터 매트릭스 차원 정렬 ($13\text{차원}$ 변수 매핑)
        input_features = np.array([[
            sensor_frame.accel_x, sensor_frame.accel_y, sensor_frame.accel_z,
            sensor_frame.gyro_x,  sensor_frame.gyro_y,  sensor_frame.gyro_z,
            sensor_frame.weight,  ultrasonic_dist,
            dc, dg, dmin, dstd, dhole
        ]], dtype=np.float32)
        
        # StandardScaler 가상 정규화 변환 및 다중 클래스 고속 분류 추론 수행
        scaled_features = self.scaler.transform(input_features)
        obstacle_pred = int(self.classifier.predict(scaled_features)[0])
        path_pred = int(self.selector.predict(scaled_features)[0])
        
        passable_map = {0: True, 1: True, 2: True, 3: False, 4: True}
        speed_map = {0: "fast", 1: "fast", 2: "slow", 3: "stop", 4: "slow"}
        
        return {
            "obstacle_label": obstacle_pred, "path_label": path_pred,
            "passable": passable_map.get(obstacle_pred, False),
            "speed": speed_map.get(obstacle_pred, "stop"),
            "hole_ratio": dhole
        }