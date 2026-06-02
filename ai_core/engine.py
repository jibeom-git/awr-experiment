# ai_core/engine.py
# AGVAIEngine: FSM + XGBoost + Isolation Forest 통합 AI 의사결정 엔진
# 기존 app/dashboard.py 의 evaluate_state_and_calculate_output() 인터페이스 하위호환 유지

import os
import cv2
import time
import numpy as np

from . import config
from .client import GeminiClient            # 기존 app/dashboard.py 하위호환용
from .vlm_client import VLMClient           # 신규 Few-shot VLM 클라이언트
from .logger import BlackboxLogger
from .trainer import ModelTrainer, OBS_ENC, MODE_ENC


class AGVAIEngine:
    """
    AGV 자율주행 AI 엔진.
    - 신규 인터페이스: evaluate(sensor_data, mode, node_id)
    - 하위호환 인터페이스: evaluate_state_and_calculate_output(...)
    """

    def __init__(self, mode: str = "SAFE", sensors: dict = None):
        self.mode = mode.upper()

        # ── 경로/노드 상태 ──────────────────────────────────────────────────
        self.current_node_count    = 0
        self.active_route          = "UNKNOWN"
        self.route_availability    = {"A": True, "B": True, "C": True}
        self._route_initialized    = False
        self._heavy_reroute_logged = False

        # ── 이전 Z축 충격량 (Isolation Forest 피처용) ───────────────────────
        self._prev_impact_z = 0.0

        # ── 컴포넌트 초기화 ─────────────────────────────────────────────────
        self.logger  = BlackboxLogger()
        self.trainer = ModelTrainer()
        self.trainer.load_or_train()          # 모델 로드 또는 초기 합성 학습

        self.vlm    = VLMClient()             # 신규 Few-shot VLM
        self.gemini = GeminiClient()          # 하위호환용 구형 Gemini 클라이언트

        # ── 마지막 VLM 판단 캐시 (대시보드 표시용) ──────────────────────────
        self.last_vlm_result = {
            "obstacle_type": "none",
            "passable": True,
            "confidence": 0.0,
            "reason": "N/A",
        }
        # 이상 점수 캐시
        self.last_anomaly_score = 0.0

        # ── 하위호환: 구형 log_path (app/dashboard.py 가 참조) ──────────────
        self.log_path = self.logger.log_path

        # 장애물 DB 로드
        self._obstacle_db = self.logger.get_obstacle_db()

        # 디렉토리 준비
        os.makedirs(config.MODELS_DIR, exist_ok=True)
        os.makedirs(config.OBSTACLES_DIR, exist_ok=True)

    # =========================================================================
    # ① 신규 인터페이스: dashboard.py (루트) 전용
    # =========================================================================

    def evaluate(self, sensor_data: dict, mode: str, node_id: int) -> dict:
        """
        매 노드 정차 시 호출되는 AI 의사결정 진입점.

        Args:
            sensor_data: {pitch, weight_g, sonic, accel_z, accel_x, pitch_delta, ...}
            mode:        "FAST" | "SAFE"
            node_id:     현재 노드 번호

        Returns:
            {"route": str, "speed": int, "fsm_state": str, "reason": str,
             "anomaly_score": float, "gemini_confidence": float}
        """
        self.mode              = mode.upper()
        self.current_node_count = node_id

        pitch    = float(sensor_data.get("pitch",     0.0))
        weight   = float(sensor_data.get("weight_g",  0.0))
        sonic    = float(sensor_data.get("sonic",     400.0))
        accel_z  = float(sensor_data.get("accel_z",   9.81))
        mock_obs = str(sensor_data.get("mock_obs",   "None"))

        # ── 1. Isolation Forest 이상 점수 계산 ─────────────────────────────
        anomaly_score = self.trainer.anomaly_score(pitch, weight, sonic, accel_z)
        self.last_anomaly_score = anomaly_score

        # ── 2. VLM 호출 여부 결정 ───────────────────────────────────────────
        gemini_passable    = True
        gemini_confidence  = 0.5
        gemini_obs_type    = "none"
        gemini_reason      = "N/A"

        in_vlm_zone = (0 < sonic <= config.TH_SONIC_SLOWDOWN) or mock_obs != "None"
        use_vlm     = in_vlm_zone and anomaly_score > 0.6

        if use_vlm and mock_obs == "None":
            # 전방 이미지 촬영 → 저장 → VLM 호출
            img_path = self._capture_and_save_obstacle_img(sensor_data)
            self._obstacle_db = self.logger.get_obstacle_db()
            vlm_resp = self.vlm.analyze_obstacle(img_path, {
                "pitch":    pitch,
                "weight_g": weight,
                "sonic":    sonic,
            }, self._obstacle_db)

            gemini_passable   = bool(vlm_resp.get("passable", True))
            gemini_confidence = float(vlm_resp.get("confidence", 0.5))
            gemini_obs_type   = str(vlm_resp.get("obstacle_type", "unknown"))
            gemini_reason     = str(vlm_resp.get("reason", "N/A"))
            self.last_vlm_result = vlm_resp

        # ── 3. XGBoost로 route + speed 예측 ────────────────────────────────
        obs_enc  = OBS_ENC.get(gemini_obs_type, 0)
        mode_enc = MODE_ENC.get(self.mode, 0)
        features = [
            pitch, weight, sonic, obs_enc, mode_enc,
            int(gemini_passable), gemini_confidence, self._prev_impact_z
        ]
        xgb_route, xgb_speed = self.trainer.predict_route_speed(features)

        # XGBoost 결과가 없으면 FSM 규칙 기반으로 폴백
        if xgb_route is None:
            xgb_route, xgb_speed = self._rule_based_route_speed(
                pitch, weight, sonic, mode, gemini_passable,
                gemini_obs_type, mock_obs
            )

        route = xgb_route
        speed = xgb_speed

        # ── 4. 물리 안전 규칙 강제 적용 (XGBoost override) ─────────────────
        fsm_state, route, speed = self._apply_safety_override(
            route, speed, pitch, weight, sonic, mode, gemini_passable,
            gemini_obs_type, gemini_confidence, gemini_reason, mock_obs, node_id
        )

        # ── 5. CSV 로그 기록 ────────────────────────────────────────────────
        self.logger.log_event(
            node        = node_id,
            route       = route,
            obs_type    = gemini_obs_type,
            status      = fsm_state,
            speed       = speed,
            msg         = f"XGBoost+FSM: {fsm_state} anomaly={anomaly_score:.2f}",
            vlm_reason  = gemini_reason,
            impact_z    = self._prev_impact_z,
            gemini_confidence = gemini_confidence,
        )

        self.active_route = route
        return {
            "route":            route,
            "speed":            speed,
            "fsm_state":        fsm_state,
            "reason":           gemini_reason,
            "anomaly_score":    anomaly_score,
            "gemini_confidence": gemini_confidence,
        }

    def _apply_safety_override(
        self, route, speed, pitch, weight, sonic, mode,
        passable, obs_type, confidence, reason, mock_obs, node_id
    ):
        """물리 안전 규칙 강제 적용: XGBoost 결과를 하드 규칙으로 덮어씀"""

        # 비상 제동
        if 0 < sonic <= config.TH_SONIC_CRITICAL:
            return "CRITICAL_STOP", route, config.SPEED_STOP

        # 장애물 구간 처리
        in_obs_zone = (0 < sonic <= config.TH_SONIC_SLOWDOWN) or mock_obs != "None"
        if in_obs_zone:
            if mock_obs != "None":
                passable, obs_type, reason, speed = self._handle_mock_obs(
                    mock_obs, pitch, weight, mode
                )
            if not passable:
                route_key = route.replace("ROUTE_", "")
                if route_key in self.route_availability:
                    self.route_availability[route_key] = False
                alt = self._calculate_alt_route()
                if alt == config.DEADLOCK:
                    return "FATAL_DEADLOCK", config.DEADLOCK, config.SPEED_STOP
                route = alt
                return "PATH_BLOCKED", route, config.SPEED_SAFE_LOW
            return f"CAUTIOUS_{obs_type.upper()}", route, speed

        # FAST 모드 + 고중량 → ROUTE_A 차단
        if (mode == config.MODE_FAST and weight > config.TH_LOAD_HEAVY
                and route == config.ROUTE_A and not self._heavy_reroute_logged):
            route = config.ROUTE_B
            self._heavy_reroute_logged = True

        # 고중량 + 경사 속도 상한
        if abs(pitch) >= config.TH_PITCH_HILL and weight > config.TH_LOAD_HEAVY:
            return "HILL_HEAVY_GUARD", route, config.SPEED_HEAVY_HILL

        # A경로 고경사
        if route == config.ROUTE_A and abs(pitch) >= config.TH_PITCH_STEEP_HILL:
            spd = config.SPEED_HIGH_POWER if mode == config.MODE_FAST else config.SPEED_DEFAULT
            return "STEEP_HILL_CLIMB", route, spd

        # B경로 완경사
        if route == config.ROUTE_B and abs(pitch) >= config.TH_PITCH_HILL:
            return "MEDIUM_HILL_CLIMB", route, config.SPEED_MEDIUM_POWER

        # SAFE + 고중량 평지
        if mode == config.MODE_SAFE and weight > config.TH_LOAD_HEAVY:
            return "SAFE_HEAVY_GUARD", route, config.SPEED_SAFE_LOW

        return "NORMAL_CRUISE", route, speed

    def _handle_mock_obs(self, mock_obs, pitch, weight, mode):
        """가상 장애물 주입 처리 — 실제 VLM 호출 없이 규칙 적용"""
        if mock_obs == "Bump_5cm":
            return False, "bump_5cm", "Height exceeds vehicle ground clearance", config.SPEED_STOP
        if mock_obs == "Bump_3cm":
            spd = (config.SPEED_HEAVY_HILL
                   if (mode == config.MODE_SAFE and weight > config.TH_LOAD_HEAVY)
                   else config.SPEED_DEFAULT)
            return True, "bump_3cm", "Passable bump, cautious slowdown applied", spd
        if mock_obs == "Vinyl":
            if abs(pitch) >= config.TH_PITCH_HILL:
                return False, "vinyl_on_hill", "Vinyl on slope: slip risk", config.SPEED_STOP
            return True, "vinyl_flat", "Flat vinyl: crawl speed", config.SPEED_SAFE_LOW
        return True, "none", "N/A", config.SPEED_DEFAULT

    def _rule_based_route_speed(self, pitch, weight, sonic, mode, passable, obs_type, mock_obs):
        """XGBoost 없을 때 FSM 규칙으로 route/speed 결정"""
        if not self._route_initialized:
            self.active_route     = config.ROUTE_A if mode == config.MODE_FAST else config.ROUTE_C
            self._route_initialized = True
        route = self.active_route

        if 0 < sonic <= config.TH_SONIC_CRITICAL:
            return route, config.SPEED_STOP
        if abs(pitch) >= config.TH_PITCH_HILL and weight > config.TH_LOAD_HEAVY:
            return route, config.SPEED_HEAVY_HILL
        if route == config.ROUTE_A and abs(pitch) >= config.TH_PITCH_STEEP_HILL:
            return route, config.SPEED_HIGH_POWER if mode == config.MODE_FAST else config.SPEED_DEFAULT
        if route == config.ROUTE_B and abs(pitch) >= config.TH_PITCH_HILL:
            return route, config.SPEED_MEDIUM_POWER
        if mode == config.MODE_SAFE and weight > config.TH_LOAD_HEAVY:
            return route, config.SPEED_SAFE_LOW
        return route, config.SPEED_DEFAULT

    def _capture_and_save_obstacle_img(self, sensor_data: dict) -> str:
        """
        전방 카메라 이미지를 data/obstacles/obs_NNN.jpg 로 저장하고 경로 반환.
        카메라 접근은 sensor_data['frame'] 키 (dashboard.py가 삽입) 또는 빈 이미지 사용.
        """
        frame = sensor_data.get("frame", None)
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        idx      = len(self.logger.get_obstacle_db()) + 1
        filename = f"obs_{idx:03d}.jpg"
        path     = os.path.join(config.OBSTACLES_DIR, filename)
        try:
            cv2.imwrite(path, frame)
        except Exception as e:
            print(f"[Engine] 장애물 이미지 저장 실패: {e}")
            return ""
        return path

    def _calculate_alt_route(self) -> str:
        """사용 가능한 대안 경로 반환. 없으면 DEADLOCK"""
        order = (["A", "B", "C"] if self.mode == config.MODE_FAST else ["C", "B", "A"])
        for key in order:
            if self.route_availability.get(key, True):
                return f"ROUTE_{key}"
        return config.DEADLOCK

    # =========================================================================
    # ② 장애물 통과 결과 업데이트 + 자동 재학습
    # =========================================================================

    def update_obstacle_result(self, impact_z: float, actual_result: str):
        """
        장애물 통과 후 실제 결과를 obstacle_db에 업데이트.
        누적 데이터 20건 이상이면 trainer.retrain() 자동 호출.
        """
        self._prev_impact_z = impact_z
        self.logger.update_last_obstacle_result(actual_result, impact_z)
        self._obstacle_db = self.logger.get_obstacle_db()

        row_count = self.logger.get_csv_row_count()
        if row_count >= 20:
            import threading
            t = threading.Thread(
                target=self.trainer.retrain,
                args=(self.logger.log_path,),
                daemon=True, name="retrain"
            )
            t.start()

    # =========================================================================
    # ③ DEADLOCK 처리 (신규)
    # =========================================================================

    def handle_deadlock(self, frame=None) -> dict:
        """모든 경로 차단 시 비상 정지 + 증거 이미지 저장"""
        self.logger.log_event(
            node=self.current_node_count, route=config.DEADLOCK,
            obs_type="all_blocked", status="FATAL_DEADLOCK",
            speed=config.SPEED_STOP,
            msg="All routes closed. Operator intervention required.",
        )
        dump_path = os.path.join(config.LOG_ROOT, "deadlock_front_evidence.jpg")
        if frame is not None:
            try:
                cv2.imwrite(dump_path, frame)
            except Exception:
                pass
        print(f"[Engine] 데드락 — 증거 이미지: {dump_path}")
        return {
            "route": config.DEADLOCK, "speed": config.SPEED_STOP,
            "fsm_state": "WAITING_OPERATOR_COMMAND",
            "reason": "DEADLOCK: ALL ROUTES BLOCKED",
            "anomaly_score": 1.0, "gemini_confidence": 0.0,
        }

    # =========================================================================
    # ④ 하위호환 인터페이스 (app/dashboard.py 가 호출)
    # =========================================================================

    def _init_admin_logger(self):
        """하위호환: 기존 CSV 헤더 초기화 — logger.py가 처리하므로 no-op"""
        pass

    def report_event_to_admin(self, node, route, obs_type, status, speed, msg, vlm_reason="N/A"):
        """하위호환: 기존 CSV 로그 메서드"""
        self.logger.log_event(
            node=node, route=route, obs_type=obs_type,
            status=status, speed=speed, msg=msg, vlm_reason=vlm_reason,
        )

    def evaluate_state_and_calculate_output(
        self,
        sensor_snapshot: dict,
        ai_camera_frame: np.ndarray,
        mock_obs: str = "None",
    ) -> dict:
        """
        하위호환 인터페이스 (app/dashboard.py 에서 호출).
        내부적으로 신규 evaluate() 로직을 재사용한다.
        """
        pitch      = sensor_snapshot.get('pitch', 0.0)
        weight     = sensor_snapshot.get('weight_g', 0.0)
        ultrasonic = sensor_snapshot.get('distance_cm', 400.0)
        node_trig  = sensor_snapshot.get('node_trigger', False)

        if node_trig:
            self.current_node_count += 1

        # 초기 경로 배정 (첫 호출 시)
        if not self._route_initialized:
            self.active_route       = config.ROUTE_A if self.mode == "FAST" else config.ROUTE_C
            self._route_initialized = True

        # FAST 모드 고중량 ROUTE_A → ROUTE_B 강제 전환
        if (self.mode == "FAST"
                and weight > config.TH_LOAD_HEAVY
                and self.active_route == config.ROUTE_A
                and not self._heavy_reroute_logged):
            self.active_route        = config.ROUTE_B
            self._heavy_reroute_logged = True
            self.report_event_to_admin(
                self.current_node_count, self.active_route, "none",
                "PATH_BLOCKED", config.SPEED_DEFAULT,
                "FAST_HEAVY_LOAD: ROUTE_A 20DEG ROLLOVER RISK → ROUTE_B",
            )

        # SAFE 모드 고중량 ROUTE_A/B → ROUTE_C 강제 전환
        if (self.mode == "SAFE"
                and weight > config.TH_LOAD_HEAVY
                and self.active_route in (config.ROUTE_A, config.ROUTE_B)
                and not self._heavy_reroute_logged):
            self.active_route        = config.ROUTE_C
            self._heavy_reroute_logged = True
            self.report_event_to_admin(
                self.current_node_count, self.active_route, "none",
                "PATH_BLOCKED", config.SPEED_DEFAULT,
                "SAFE_HEAVY_LOAD: HIGH LOAD DETECTED → FLAT ROUTE_C",
            )

        # 비상 정지
        if 0.0 < ultrasonic <= config.TH_SONIC_CRITICAL:
            return {
                "speed_limit_pct": config.SPEED_STOP,
                "state": "CRITICAL_STOP",
                "route": self.active_route,
            }

        # VLM 구간 처리
        in_obs_zone = (0.0 < ultrasonic <= config.TH_SONIC_SLOWDOWN)
        if in_obs_zone or mock_obs != "None":
            if mock_obs == "Bump_5cm":
                vlm = {"obstacle_type": "bump_5cm", "passable": False,
                       "recommended_speed_limit": config.SPEED_STOP,
                       "reason": "Height exceeds vehicle ground clearance"}
            elif mock_obs == "Bump_3cm":
                bump_spd = (config.SPEED_HILL_HEAVY_CAP
                            if (self.mode == "SAFE" and weight > config.TH_LOAD_HEAVY)
                            else config.SPEED_DEFAULT)
                vlm = {"obstacle_type": "bump_3cm", "passable": True,
                       "recommended_speed_limit": bump_spd,
                       "reason": "Passable bump, cautious slowdown applied"}
            elif mock_obs == "Vinyl":
                if abs(pitch) >= config.TH_PITCH_MEDIUM_HILL:
                    vlm = {"obstacle_type": "vinyl_on_hill", "passable": False,
                           "recommended_speed_limit": config.SPEED_STOP,
                           "reason": "Vinyl on slope: slip risk, impassable"}
                else:
                    vlm = {"obstacle_type": "vinyl_flat", "passable": True,
                           "recommended_speed_limit": config.SPEED_SAFE_LOW,
                           "reason": "Flat vinyl: cautious crawl at minimum speed"}
            else:
                ctx = {'distance_cm': ultrasonic, 'weight_g': weight, 'pitch': pitch}
                vlm = self.gemini.analyze_obstacle(ai_camera_frame, ctx, self.mode)

            obs_type  = vlm.get('obstacle_type', 'unknown')
            passable  = vlm.get('passable', False)
            vlm_speed = vlm.get('recommended_speed_limit', config.SPEED_SAFE_LOW)
            reason    = vlm.get('reason', 'VLM_DECISION')

            if passable:
                self.report_event_to_admin(
                    self.current_node_count, self.active_route, obs_type,
                    "CAUTIOUS_SLOWDOWN", vlm_speed,
                    f"VLM: passable obstacle → proceed at {vlm_speed}%",
                    vlm_reason=reason,
                )
                return {
                    "speed_limit_pct": vlm_speed,
                    "state": f"CAUTIOUS_{obs_type.upper()}",
                    "route": self.active_route,
                }
            else:
                route_key = self.active_route.replace("ROUTE_", "")
                if route_key in self.route_availability:
                    self.route_availability[route_key] = False
                self.report_event_to_admin(
                    self.current_node_count, self.active_route, obs_type,
                    "PATH_BLOCKED", config.SPEED_STOP,
                    "VLM: obstacle impassable → rerouting", vlm_reason=reason,
                )
                alt = self._calculate_alternative_route()
                if alt == config.DEADLOCK:
                    return self._handle_system_deadlock_scenario(ai_camera_frame)
                self.active_route = alt
                return {
                    "speed_limit_pct": config.SPEED_SAFE_LOW,
                    "state": "REROUTING_RUN",
                    "route": self.active_route,
                }

        # 정상 주행 분기
        if abs(pitch) >= config.TH_PITCH_MEDIUM_HILL and weight > config.TH_LOAD_HEAVY:
            return {"speed_limit_pct": config.SPEED_HILL_HEAVY_CAP,
                    "state": "HILL_HEAVY_GUARD", "route": self.active_route}
        if self.active_route == config.ROUTE_A and abs(pitch) >= config.TH_PITCH_STEEP_HILL:
            spd = config.SPEED_HIGH_POWER if self.mode == "FAST" else config.SPEED_DEFAULT
            return {"speed_limit_pct": spd, "state": "STEEP_HILL_CLIMB_FAST",
                    "route": self.active_route}
        if self.active_route == config.ROUTE_B and abs(pitch) >= config.TH_PITCH_MEDIUM_HILL:
            return {"speed_limit_pct": config.SPEED_MEDIUM_POWER,
                    "state": "MEDIUM_HILL_CLIMB", "route": self.active_route}
        if self.mode == "SAFE" and weight > config.TH_LOAD_HEAVY:
            return {"speed_limit_pct": config.SPEED_SAFE_LOW,
                    "state": "SAFE_HEAVY_GUARD", "route": self.active_route}
        return {"speed_limit_pct": config.SPEED_DEFAULT,
                "state": "NORMAL_CRUISE", "route": self.active_route}

    def _calculate_alternative_route(self) -> str:
        """하위호환: 가용 경로 우선순위 탐색"""
        order = (["A", "B", "C"] if self.mode == "FAST" else ["C", "B", "A"])
        for key in order:
            if self.route_availability.get(key, True):
                return f"ROUTE_{key}"
        return config.DEADLOCK

    def _handle_system_deadlock_scenario(self, raw_frame: np.ndarray) -> dict:
        """하위호환: 데드락 비상 정지"""
        self.report_event_to_admin(
            self.current_node_count, self.active_route, "all_blocked",
            "FATAL_DEADLOCK", config.SPEED_STOP,
            "All routes closed. Operator intervention required.",
        )
        dump_path = os.path.join(config.LOG_ROOT, "deadlock_front_evidence.jpg")
        if raw_frame is not None and raw_frame.size > 0:
            cv2.imwrite(dump_path, raw_frame)
        print(f"[Engine] 데드락 — 증거 이미지: {dump_path}")
        return {
            "speed_limit_pct": config.SPEED_STOP,
            "state": "WAITING_OPERATOR_COMMAND",
            "route": config.DEADLOCK,
        }
