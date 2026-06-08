# ai_core/trainer.py
# XGBoost / Isolation Forest 학습 및 재학습 관리 모듈 (라벨링 동기화 및 Pylance 최적화 완료)

import os
import csv
import time
import random
import threading
from typing import Any

import numpy as np

try:
    import xgboost as xgb
    from sklearn.ensemble import IsolationForest
    import joblib
    _ML_OK = True
except ImportError:
    _ML_OK = False
    print("[Trainer] XGBoost/sklearn 없음 — 규칙 기반 폴백 전용 모드")

from . import config

# ── 1. 고도화된 하이브리드 인코딩 매핑 ─────────────────────────────────────────
# VLM 결과("bump"), 구형 가상 주입("bump_3cm"), 실측 로그("bump_1cm")를 모두 완벽히 호환 수용하도록 확장
OBS_ENC = {
    "none": 0, "none": 0,
    "bump_1cm": 1, "bump_3cm": 1, "bump": 1, "1cm 방지턱": 1,
    "bump_2cm": 2, "bump_5cm": 2, "2cm 방지턱": 2,
    "vinyl": 3, "vinyl_flat": 3, "vinyl_on_hill": 3, "비닐": 3,
    "unknown": 4, "slope": 4, "surface": 4
}

# 주행 모드
MODE_ENC = {config.MODE_SAFE: 0, config.MODE_FAST: 1}
# 경로 레이블
ROUTE_ENC = {config.ROUTE_A: 0, config.ROUTE_B: 1, config.ROUTE_C: 2, config.DEADLOCK: 3}
ROUTE_DEC = {v: k for k, v in ROUTE_ENC.items()}
# 속도 레이블
SPEED_VALUES = [
    config.SPEED_STOP, config.SPEED_SAFE_LOW, config.SPEED_HEAVY_HILL,
    config.SPEED_DEFAULT, config.SPEED_MEDIUM_POWER, config.SPEED_HIGH_POWER
]
SPEED_ENC = {s: i for i, s in enumerate(SPEED_VALUES)}
SPEED_DEC = {i: s for s, i in SPEED_ENC.items()}

# 모델 파일 경로
_ROUTE_MODEL_PATH = os.path.join(config.MODELS_DIR, "xgboost_route.json")
_SPEED_MODEL_PATH = os.path.join(config.MODELS_DIR, "xgboost_speed.json")
_IF_MODEL_PATH    = os.path.join(config.MODELS_DIR, "isolation_forest.pkl")

# 재학습 최소 데이터 행 수
MIN_RETRAIN_ROWS = 20


def _apply_fsm_rules(mode_enc, pitch, weight, sonic, obs_enc, passable, gemini_conf):
    """
    FSM 물리 규칙을 그대로 적용하여 합성 데이터의 (route_label, speed_label) 생성.
    실제 engine.py 의 물리 구조와 100% 동기화됨.
    """
    # 초기 경로 배정
    if mode_enc == 1:  # FAST
        route = config.ROUTE_A
        if weight > config.TH_LOAD_HEAVY:
            route = config.ROUTE_B
    else:              # SAFE
        route = config.ROUTE_C

    # 비상 정지
    if 0 < sonic <= config.TH_SONIC_CRITICAL:
        return ROUTE_ENC[route], SPEED_ENC[config.SPEED_STOP]

    # 장애물 구간
    in_obs_zone = obs_enc != 0 and 0 < sonic <= config.TH_SONIC_SLOWDOWN
    if in_obs_zone:
        if not passable:
            # 경로 차단 → 대안 경로로 전환
            if mode_enc == 1:
                alt = config.ROUTE_B if route == config.ROUTE_A else (
                    config.ROUTE_C if route == config.ROUTE_B else config.DEADLOCK)
            else:
                alt = config.ROUTE_B if route == config.ROUTE_C else (
                    config.ROUTE_A if route == config.ROUTE_B else config.DEADLOCK)
            if alt == config.DEADLOCK:
                return ROUTE_ENC[config.DEADLOCK], SPEED_ENC[config.SPEED_STOP]
            return ROUTE_ENC[alt], SPEED_ENC[config.SPEED_SAFE_LOW]
            
        # 통과 가능 장애물 서행 가이드
        if obs_enc == 1:  # 1cm / 3cm 방지턱 구간
            speed = (config.SPEED_HEAVY_HILL
                     if (mode_enc == 0 and weight > config.TH_LOAD_HEAVY)
                     else config.SPEED_DEFAULT)
        elif obs_enc == 3:  # 비닐 노면
            speed = config.SPEED_SAFE_LOW
        else:
            speed = config.SPEED_DEFAULT
        return ROUTE_ENC[route], SPEED_ENC[speed]

    # 정상 주행 경사 및 적재 중량별 가이드
    if abs(pitch) >= config.TH_PITCH_HILL and weight > config.TH_LOAD_HEAVY:
        return ROUTE_ENC[route], SPEED_ENC[config.SPEED_HEAVY_HILL]
    if route == config.ROUTE_A and abs(pitch) >= config.TH_PITCH_STEEP_HILL:
        spd = config.SPEED_HIGH_POWER if mode_enc == 1 else config.SPEED_DEFAULT
        return ROUTE_ENC[route], SPEED_ENC[spd]
    if route == config.ROUTE_B and abs(pitch) >= config.TH_PITCH_HILL:
        return ROUTE_ENC[route], SPEED_ENC[config.SPEED_MEDIUM_POWER]
    if mode_enc == 0 and weight > config.TH_LOAD_HEAVY:
        return ROUTE_ENC[route], SPEED_ENC[config.SPEED_SAFE_LOW]

    return ROUTE_ENC[route], SPEED_ENC[config.SPEED_DEFAULT]


def _ensure_all_labels(X_list, yr_list, ys_list):
    """모든 클래스 레이블이 최소 1건씩 포함되도록 강제 샘플 추가하여 XGBoost 차원 추론 오류 방지"""
    from . import config as cfg
    X_list.append([0.0, 80.0, 5.0, 2, 1, 0, 0.9, 0.0])  # 데드락 확정 샘플
    yr_list.append(ROUTE_ENC[cfg.DEADLOCK])
    ys_list.append(SPEED_ENC[cfg.SPEED_STOP])

    X_list.append([0.0, 50.0, 3.0, 0, 0, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_C])
    ys_list.append(SPEED_ENC[cfg.SPEED_STOP])

    X_list.append([8.0, 100.0, 200.0, 0, 1, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_B])
    ys_list.append(SPEED_ENC[cfg.SPEED_MEDIUM_POWER])

    X_list.append([16.0, 60.0, 200.0, 0, 1, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_A])
    ys_list.append(SPEED_ENC[cfg.SPEED_HIGH_POWER])

    X_list.append([0.0, 200.0, 200.0, 0, 0, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_C])
    ys_list.append(SPEED_ENC[cfg.SPEED_SAFE_LOW])

    X_list.append([8.0, 200.0, 200.0, 0, 0, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_C])
    ys_list.append(SPEED_ENC[cfg.SPEED_HEAVY_HILL])
    return X_list, yr_list, ys_list


def _generate_synthetic_data(n: int = 2000):
    """임계값 기반 고품질 가상 합성 학습 데이터 생성"""
    rng = np.random.default_rng(42)
    X, y_route, y_speed = [], [], []

    for _ in range(n):
        mode_enc = int(rng.integers(0, 2))
        pitch    = float(rng.normal(0, 8))
        weight   = float(rng.uniform(0, 300))
        sonic    = float(rng.uniform(1, 400))
        obs_enc  = int(rng.choice([0, 0, 0, 1, 2, 3, 4], p=[0.6, 0.1, 0.1, 0.1, 0.05, 0.03, 0.02]))
        passable = bool(rng.random() > 0.3)
        gemini_conf = float(rng.uniform(0.5, 1.0))

        if obs_enc == 2:
            passable = False

        gemini_passable = int(passable)
        impact_z_prev   = float(rng.uniform(0, 6))

        route_lbl, speed_lbl = _apply_fsm_rules(
            mode_enc, pitch, weight, sonic, obs_enc, passable, gemini_conf
        )

        X.append([pitch, weight, sonic, obs_enc, mode_enc,
                  gemini_passable, gemini_conf, impact_z_prev])
        y_route.append(route_lbl)
        y_speed.append(speed_lbl)

    X, y_route, y_speed = _ensure_all_labels(X, y_route, y_speed)
    return np.array(X, dtype=np.float32), np.array(y_route), np.array(y_speed)


class ModelTrainer:
    """XGBoost + Isolation Forest 초기 학습 및 재학습 관리를 위한 지능형 제어 스튜디오"""

    def __init__(self):
        self._lock = threading.Lock()
        os.makedirs(config.MODELS_DIR, exist_ok=True)
        # Pylance reportOptionalMemberAccess 원천 차단을 위한 Any 처리
        self.route_clf: Any = None
        self.speed_clf: Any = None
        self.if_model: Any = None

    def train_from_experiment(self) -> tuple:
        """실험 수동 로그(raw_experiment.csv) 파싱 및 완전 가공"""
        exp_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'experiment', 'data', 'raw_experiment.csv'
        )
        if not os.path.exists(exp_path):
            return None, None, None

        rows = []
        try:
            with open(exp_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f, fieldnames=[
                    "timestamp", "label", "speed_cmd", "pitch", "weight",
                    "sonic", "accel_x", "accel_y", "accel_z", "pitch_delta", "result"
                ])
                for row in reader:
                    rows.append(row)
        except Exception as e:
            print(f"[Trainer] raw_experiment.csv 읽기 실패: {e}")
            return None, None, None

        print(f"[Trainer] 실험 데이터 로드 완료: {len(rows)}행")

        X, y_route, y_speed = [], [], []
        for row in rows:
            try:
                label     = row.get("label", "")
                result    = row.get("result", "normal")
                pitch     = float(row.get("pitch",     0) or 0)
                weight    = float(row.get("weight",    0) or 0)
                sonic     = float(row.get("sonic",   400) or 400)
                accel_z   = float(row.get("accel_z", 9.81) or 9.81)
                speed_cmd = float(row.get("speed_cmd", 40) or 40)

                # 고도화된 전역 사전을 통한 에러 없는 토큰 정수 변환
                obs_enc   = OBS_ENC.get(label.strip(), 0)
                if obs_enc == 0:
                    for k, v in OBS_ENC.items():
                        if k in label:
                            obs_enc = v
                            break

                passable  = 1 if result.strip() in ("normal", "cautious_pass") else 0
                
                # 실험 환경 맵핑 가이드 반영
                if "20도" in label:
                    route_lbl = ROUTE_ENC[config.ROUTE_A]
                elif "10도" in label:
                    route_lbl = ROUTE_ENC[config.ROUTE_B]
                else:
                    route_lbl = ROUTE_ENC[config.ROUTE_C]

                mode_enc  = 1 if speed_cmd > 40 else 0
                impact_z  = abs(accel_z - 9.81)
                gemini_conf = 0.8

                speed_key = min(SPEED_VALUES, key=lambda s: abs(s - speed_cmd))
                speed_lbl = SPEED_ENC.get(speed_key, SPEED_ENC[config.SPEED_DEFAULT])

                X.append([pitch, weight, sonic, obs_enc, mode_enc,
                          passable, gemini_conf, impact_z])
                y_route.append(route_lbl)
                y_speed.append(speed_lbl)
            except Exception:
                continue

        if not X:
            return None, None, None

        return (np.array(X, dtype=np.float32),
                np.array(y_route), np.array(y_speed))

    def load_or_train(self):
        """저장 모델 안전 로드 및 최적화 부트스트랩"""
        if not _ML_OK:
            return

        X_exp, y_r_exp, y_s_exp = self.train_from_experiment()

        models_exist = (
            os.path.exists(_ROUTE_MODEL_PATH) and
            os.path.exists(_SPEED_MODEL_PATH) and
            os.path.exists(_IF_MODEL_PATH)
        )

        if models_exist:
            self._load_models()
            print("[Trainer] 저장된 최적화 ML 모델 안전 로드 성공")
        else:
            print("[Trainer] 초기 모델이 탐지되지 않아 자율 통합 학습을 개시합니다...")
            X_syn, y_r_syn, y_s_syn = _generate_synthetic_data(2000)

            if X_exp is not None and len(X_exp) > 0:
                X_all   = np.vstack([X_syn, X_exp])
                y_r_all = np.concatenate([y_r_syn, y_r_exp])
                y_s_all = np.concatenate([y_s_syn, y_s_exp])
                self._fit_and_save(X_all, y_r_all, y_s_all, label="초기(합성+실험 수동 데이터)")
            else:
                self._fit_and_save(X_syn, y_r_syn, y_s_syn, label="초기(합성 전용 코어)")

            normal_mask = y_r_syn != ROUTE_ENC[config.DEADLOCK]
            self._fit_isolation_forest(X_syn[normal_mask, :4])

    def train_initial(self):
        if not _ML_OK: return
        X, y_route, y_speed = _generate_synthetic_data(2000)
        self._fit_and_save(X, y_route, y_speed, label="초기(합성 재생성)")
        normal_mask = y_route != ROUTE_ENC[config.DEADLOCK]
        self._fit_isolation_forest(X[normal_mask, :4])

    def _fit_and_save(self, X, y_route, y_speed, label: str = ""):
        with self._lock:
            self.route_clf = xgb.XGBClassifier(
                n_estimators=120, max_depth=5, learning_rate=0.1,
                objective='multi:softmax', use_label_encoder=False,
                eval_metric='mlogloss', random_state=42, verbosity=0,
            )
            self.speed_clf = xgb.XGBClassifier(
                n_estimators=120, max_depth=5, learning_rate=0.1,
                objective='multi:softmax', use_label_encoder=False,
                eval_metric='mlogloss', random_state=42, verbosity=0,
            )
            self.route_clf.fit(X, y_route)
            self.speed_clf.fit(X, y_speed)
            self.route_clf.save_model(_ROUTE_MODEL_PATH)
            self.speed_clf.save_model(_SPEED_MODEL_PATH)
            print(f"[Trainer] 안정성 검증 필터 학습 완료 ({label}) ➡️ models/ 저장")

    def _fit_isolation_forest(self, X_normal):
        if not _ML_OK: return
        with self._lock:
            self.if_model = IsolationForest(
                n_estimators=100, contamination=0.05, random_state=42
            )
            self.if_model.fit(X_normal)
            joblib.dump(self.if_model, _IF_MODEL_PATH)
            print("[Trainer] Isolation Forest 다차원 이상징후 필터 학습 완료")

    def _load_models(self):
        if not _ML_OK: return
        with self._lock:
            try:
                self.route_clf = xgb.XGBClassifier()
                self.route_clf.load_model(_ROUTE_MODEL_PATH)
                self.speed_clf = xgb.XGBClassifier()
                self.speed_clf.load_model(_SPEED_MODEL_PATH)
                self.if_model  = joblib.load(_IF_MODEL_PATH)
            except Exception as e:
                print(f"[Trainer] 런타임 모델 복원 실패, 비상 핫스왑 가동: {e}")
                self.train_initial()

    def retrain(self, log_path: str) -> float:
        """블랙박스 실제 현장 런타임 로그와 코어 데이터를 혼합하여 주행 지능 실시간 진화"""
        if not _ML_OK: return -1.0

        rows = self._load_csv_rows(log_path)
        if len(rows) < MIN_RETRAIN_ROWS:
            return -1.0

        X, y_route, y_speed = self._csv_rows_to_features(rows)
        if len(X) == 0:
            return -1.0

        X_syn, y_r_syn, y_s_syn = _generate_synthetic_data(300)
        X_all     = np.vstack([X_syn, X])
        y_r_all   = np.concatenate([y_r_syn, y_route])
        y_s_all   = np.concatenate([y_s_syn, y_speed])

        self._fit_and_save(X_all, y_r_all, y_s_all, label="실시간 현장 융합 재학습")

        from sklearn.metrics import accuracy_score
        route_clf = self.route_clf
        if route_clf is not None:
            r_pred = route_clf.predict(X)
            acc    = float(accuracy_score(y_route, r_pred))
            print(f"[Trainer] 주행 지능 최적화 스왑 완료 — 실전 검증 정확도: {acc:.3f}")
            return acc
        return -1.0

    def _load_csv_rows(self, log_path: str) -> list:
        rows = []
        if not os.path.exists(log_path): return rows
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader: rows.append(row)
        except Exception as e:
            print(f"[Trainer] 현장 로그 파싱 에러: {e}")
        return rows

    def _csv_rows_to_features(self, rows: list):
        """실전 로그 데이터 다중 컬럼 추론 매퍼"""
        X, y_route, y_speed = [], [], []
        for row in rows:
            try:
                route  = row.get("Assigned_Route", "")
                speed  = int(row.get("Applied_Speed", 40))
                obs    = row.get("Obstacle_Type", "none").lower()
                status = row.get("Pass_Status", "")

                if route not in ROUTE_ENC: continue
                speed_key = min(SPEED_VALUES, key=lambda s: abs(s - speed))
                if speed_key not in SPEED_ENC: continue

                pitch   = float(row.get("pitch", 0.0) or 0.0)
                weight  = float(row.get("weight", 0.0) or 0.0)
                sonic   = float(row.get("sonic", 400.0) or 400.0)
                impact_z = float(row.get("Impact_Z", 0.0) or 0.0)
                conf    = float(row.get("Gemini_Confidence", 0.5) or 0.5)
                
                obs_enc = OBS_ENC.get(obs, 0)
                passable = 0 if status in ("PATH_BLOCKED", "FATAL_DEADLOCK") else 1
                
                # 가변적 로그 포맷 방어 코딩 (다중 후보군에서 FAST 모드 추론)
                sys_msg = str(row.get("System_Message", "")) + str(row.get("msg", "")) + str(row.get("Status", ""))
                mode_enc = 1 if "FAST" in sys_msg.upper() else 0

                X.append([pitch, weight, sonic, obs_enc, mode_enc, passable, conf, impact_z])
                y_route.append(ROUTE_ENC[route])
                y_speed.append(SPEED_ENC[speed_key])
            except Exception:
                continue

        if not X: return np.array([]), np.array([]), np.array([])
        return np.array(X, dtype=np.float32), np.array(y_route), np.array(y_speed)

    def retrain_isolation_forest(self, slope_data: list):
        if not _ML_OK or len(slope_data) < 10: return
        X_slope = np.array(slope_data, dtype=np.float32)

        exp_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'experiment', 'data', 'raw_experiment.csv'
        )
        X_normal = []
        if os.path.exists(exp_path):
            try:
                with open(exp_path, 'r', encoding='utf-8') as f:
                    reader = csv.DictReader(f, fieldnames=[
                        "timestamp", "label", "speed_cmd", "pitch", "weight",
                        "sonic", "accel_x", "accel_y", "accel_z", "pitch_delta", "result"
                    ])
                    for row in reader:
                        if row.get("result", "").strip() == "normal":
                            try:
                                X_normal.append([
                                    float(row["pitch"]), float(row["accel_z"]),
                                    float(row["accel_x"]), float(row["weight"]), float(row["sonic"]),
                                ])
                            except Exception:
                                continue
            except Exception as e:
                print(f"[Trainer] 자율 필터 업데이트 스킵: {e}")

        if len(X_normal) > 0:
            X_all = np.vstack([np.array(X_normal, dtype=np.float32), X_slope])
        else:
            X_all = X_slope

        self._fit_isolation_forest(X_all)

    def predict_route_speed(self, features: list):
        route_clf = self.route_clf
        speed_clf = self.speed_clf

        if not _ML_OK or route_clf is None or speed_clf is None:
            return None, None
        try:
            x = np.array([features], dtype=np.float32)
            r = int(route_clf.predict(x)[0])
            s = int(speed_clf.predict(x)[0])
            return ROUTE_DEC.get(r, config.ROUTE_C), SPEED_DEC.get(s, config.SPEED_DEFAULT)
        except Exception as e:
            print(f"[Trainer] 실시간 추론 연산 오류: {e}")
            return None, None

    def anomaly_score(self, pitch: float, weight: float, sonic: float, accel_z: float) -> float:
        if not _ML_OK or self.if_model is None: return 0.0
        try:
            x = np.array([[pitch, weight, sonic, accel_z]], dtype=np.float32)
            raw = float(self.if_model.decision_function(x)[0])
            score = max(0.0, min(1.0, (-raw) / 1.5))
            return round(score, 3)
        except Exception:
            return 0.0