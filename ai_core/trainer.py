# ai_core/trainer.py
# XGBoost / Isolation Forest 학습 및 재학습 관리 모듈

import os
import csv
import time
import random
import threading

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

# ── 인코딩 매핑 ─────────────────────────────────────────────────────────────
# 장애물 타입
OBS_ENC = {"none": 0, "bump_3cm": 1, "bump_5cm": 2, "vinyl": 3, "unknown": 4}
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
    실제 engine.py 의 로직과 100% 일치해야 한다.
    """
    # 초기 경로 배정
    if mode_enc == 1:  # FAST
        route = config.ROUTE_A
        if weight > config.TH_LOAD_HEAVY:
            route = config.ROUTE_B
    else:              # SAFE
        route = config.ROUTE_C
        if weight > config.TH_LOAD_HEAVY:
            route = config.ROUTE_C  # SAFE + heavy → C 유지

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
        # 통과 가능 장애물 속도 결정
        if obs_enc == 1:  # bump_3cm
            speed = (config.SPEED_HEAVY_HILL
                     if (mode_enc == 0 and weight > config.TH_LOAD_HEAVY)
                     else config.SPEED_DEFAULT)
        elif obs_enc == 3:  # vinyl
            speed = config.SPEED_SAFE_LOW
        else:
            speed = config.SPEED_DEFAULT
        return ROUTE_ENC[route], SPEED_ENC[speed]

    # 정상 주행 속도 결정
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
    """
    모든 클래스 레이블이 최소 1건씩 포함되도록 강제 샘플 추가.
    XGBoost num_class 추론 오류 방지.
    """
    from . import config as cfg
    # DEADLOCK 샘플 (ROUTE_ENC[DEADLOCK]=3, sonic<=10 → SPEED_STOP=0)
    X_list.append([0.0, 80.0, 5.0, 2, 1, 0, 0.9, 0.0])   # bump_5cm, FAST, not passable → deadlock
    yr_list.append(ROUTE_ENC[cfg.DEADLOCK])
    ys_list.append(SPEED_ENC[cfg.SPEED_STOP])

    # SPEED_STOP 샘플 (sonic critical)
    X_list.append([0.0, 50.0, 3.0, 0, 0, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_C])
    ys_list.append(SPEED_ENC[cfg.SPEED_STOP])

    # SPEED_MEDIUM_POWER 샘플 (ROUTE_B + pitch>=7)
    X_list.append([8.0, 100.0, 200.0, 0, 1, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_B])
    ys_list.append(SPEED_ENC[cfg.SPEED_MEDIUM_POWER])

    # SPEED_HIGH_POWER 샘플 (ROUTE_A + pitch>=15 + FAST)
    X_list.append([16.0, 60.0, 200.0, 0, 1, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_A])
    ys_list.append(SPEED_ENC[cfg.SPEED_HIGH_POWER])

    # SPEED_SAFE_LOW 샘플 (SAFE + heavy)
    X_list.append([0.0, 200.0, 200.0, 0, 0, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_C])
    ys_list.append(SPEED_ENC[cfg.SPEED_SAFE_LOW])

    # SPEED_HEAVY_HILL 샘플 (pitch>=7 + weight>150)
    X_list.append([8.0, 200.0, 200.0, 0, 0, 1, 0.5, 0.0])
    yr_list.append(ROUTE_ENC[cfg.ROUTE_C])
    ys_list.append(SPEED_ENC[cfg.SPEED_HEAVY_HILL])

    return X_list, yr_list, ys_list


def _generate_synthetic_data(n: int = 2000):
    """
    config.py 물리 임계값을 기반으로 합성 학습 데이터 2000건 생성.
    피처: [pitch, weight, sonic, obs_enc, mode_enc, gemini_passable, gemini_conf, impact_z_prev]
    레이블: [route_label, speed_label]
    """
    rng = np.random.default_rng(42)
    X, y_route, y_speed = [], [], []

    for _ in range(n):
        mode_enc = int(rng.integers(0, 2))
        pitch    = float(rng.normal(0, 8))           # 정규분포 경사각
        weight   = float(rng.uniform(0, 300))         # 적재 중량 g
        sonic    = float(rng.uniform(1, 400))          # 초음파 거리 cm
        obs_enc  = int(rng.choice([0, 0, 0, 1, 2, 3, 4], p=[0.6, 0.1, 0.1, 0.1, 0.05, 0.03, 0.02]))
        passable = bool(rng.random() > 0.3)           # 70% 통과 가능
        gemini_conf = float(rng.uniform(0.5, 1.0))

        # bump_5cm는 항상 통과 불가
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

    # 모든 레이블이 최소 1건 포함되도록 보정
    X, y_route, y_speed = _ensure_all_labels(X, y_route, y_speed)
    return np.array(X, dtype=np.float32), np.array(y_route), np.array(y_speed)


class ModelTrainer:
    """XGBoost + Isolation Forest 초기 학습 및 재학습 관리"""

    def __init__(self):
        self._lock = threading.Lock()
        os.makedirs(config.MODELS_DIR, exist_ok=True)
        # 모델 인스턴스 (None이면 미학습 상태)
        self.route_clf = None
        self.speed_clf = None
        self.if_model  = None

    def load_or_train(self):
        """
        저장된 모델이 존재하면 로드, 없으면 합성 데이터로 초기 학습 후 저장.
        ML 라이브러리가 없으면 None 상태로 반환 (규칙 기반 폴백 사용).
        """
        if not _ML_OK:
            return

        models_exist = (
            os.path.exists(_ROUTE_MODEL_PATH) and
            os.path.exists(_SPEED_MODEL_PATH) and
            os.path.exists(_IF_MODEL_PATH)
        )

        if models_exist:
            self._load_models()
            print("[Trainer] 저장된 모델 로드 완료")
        else:
            print("[Trainer] 초기 모델 없음 — 합성 데이터로 초기 학습 시작...")
            self.train_initial()

    def train_initial(self):
        """합성 데이터 2000건으로 초기 모델 학습"""
        if not _ML_OK:
            return

        X, y_route, y_speed = _generate_synthetic_data(2000)
        self._fit_and_save(X, y_route, y_speed, label="초기(합성)")

        # Isolation Forest: 정상 주행 샘플만 사용 (DEADLOCK 제외)
        normal_mask = y_route != ROUTE_ENC[config.DEADLOCK]
        X_if = X[normal_mask, :4]  # pitch, weight, sonic, accel_z 대용 (impact_z_prev)
        self._fit_isolation_forest(X_if)

    def _fit_and_save(self, X, y_route, y_speed, label: str = ""):
        """XGBoost 학습 및 저장"""
        with self._lock:
            n_route = len(ROUTE_ENC)
            n_speed = len(SPEED_ENC)

            # num_class는 XGBClassifier 가 학습 데이터에서 자동 추론
            self.route_clf = xgb.XGBClassifier(
                n_estimators=120, max_depth=5, learning_rate=0.1,
                objective='multi:softmax',
                use_label_encoder=False, eval_metric='mlogloss',
                random_state=42, verbosity=0,
            )
            self.speed_clf = xgb.XGBClassifier(
                n_estimators=120, max_depth=5, learning_rate=0.1,
                objective='multi:softmax',
                use_label_encoder=False, eval_metric='mlogloss',
                random_state=42, verbosity=0,
            )
            self.route_clf.fit(X, y_route)
            self.speed_clf.fit(X, y_speed)
            self.route_clf.save_model(_ROUTE_MODEL_PATH)
            self.speed_clf.save_model(_SPEED_MODEL_PATH)
            print(f"[Trainer] XGBoost 학습 완료 ({label}) — 저장: models/")

    def _fit_isolation_forest(self, X_normal):
        """Isolation Forest 학습 및 저장"""
        if not _ML_OK:
            return
        with self._lock:
            self.if_model = IsolationForest(
                n_estimators=100, contamination=0.05, random_state=42
            )
            self.if_model.fit(X_normal)
            joblib.dump(self.if_model, _IF_MODEL_PATH)
            print("[Trainer] Isolation Forest 학습 완료 — 저장: models/")

    def _load_models(self):
        """저장된 모델 파일 로드"""
        if not _ML_OK:
            return
        with self._lock:
            try:
                self.route_clf = xgb.XGBClassifier()
                self.route_clf.load_model(_ROUTE_MODEL_PATH)
                self.speed_clf = xgb.XGBClassifier()
                self.speed_clf.load_model(_SPEED_MODEL_PATH)
                self.if_model  = joblib.load(_IF_MODEL_PATH)
            except Exception as e:
                print(f"[Trainer] 모델 로드 실패 — 재학습: {e}")
                self.train_initial()

    def retrain(self, log_path: str) -> float:
        """
        real_agv_history.csv 에 누적된 실제 데이터로 XGBoost 재학습.
        데이터가 MIN_RETRAIN_ROWS 건 미만이면 재학습 없이 -1.0 반환.
        성공 시 학습 정확도 반환.
        """
        if not _ML_OK:
            print("[Trainer] ML 라이브러리 없음 — 재학습 불가")
            return -1.0

        rows = self._load_csv_rows(log_path)
        if len(rows) < MIN_RETRAIN_ROWS:
            print(f"[Trainer] 데이터 부족 ({len(rows)} < {MIN_RETRAIN_ROWS}) — 재학습 스킵")
            return -1.0

        X, y_route, y_speed = self._csv_rows_to_features(rows)
        if len(X) == 0:
            return -1.0

        # 합성 데이터와 실제 데이터를 3:7 혼합하여 일반화 성능 향상
        X_syn, y_r_syn, y_s_syn = _generate_synthetic_data(300)
        X_all     = np.vstack([X_syn, X])
        y_r_all   = np.concatenate([y_r_syn, y_route])
        y_s_all   = np.concatenate([y_s_syn, y_speed])

        self._fit_and_save(X_all, y_r_all, y_s_all, label="재학습(실제+합성)")

        # 실제 데이터 정확도 평가
        from sklearn.metrics import accuracy_score
        r_pred = self.route_clf.predict(X)
        acc    = float(accuracy_score(y_route, r_pred))
        print(f"[Trainer] 재학습 완료 — Route 정확도: {acc:.3f}")
        return acc

    def _load_csv_rows(self, log_path: str) -> list:
        """CSV에서 유효한 데이터 행 로드"""
        rows = []
        if not os.path.exists(log_path):
            return rows
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    rows.append(row)
        except Exception as e:
            print(f"[Trainer] CSV 읽기 오류: {e}")
        return rows

    def _csv_rows_to_features(self, rows: list):
        """CSV 행들을 XGBoost 학습 피처로 변환"""
        X, y_route, y_speed = [], [], []
        for row in rows:
            try:
                route  = row.get("Assigned_Route", "")
                speed  = int(row.get("Applied_Speed", 40))
                obs    = row.get("Obstacle_Type", "none").lower()
                status = row.get("Pass_Status", "")

                if route not in ROUTE_ENC:
                    continue
                speed_key = min(SPEED_VALUES, key=lambda s: abs(s - speed))
                if speed_key not in SPEED_ENC:
                    continue

                # 피처 파싱 (CSV에 센서값이 없으면 기본값 사용)
                pitch   = float(row.get("pitch", 0.0) or 0.0)
                weight  = float(row.get("weight", 0.0) or 0.0)
                sonic   = float(row.get("sonic", 400.0) or 400.0)
                impact_z = float(row.get("Impact_Z", 0.0) or 0.0)
                conf    = float(row.get("Gemini_Confidence", 0.5) or 0.5)
                obs_enc = OBS_ENC.get(obs, 0)
                passable = 0 if status in ("PATH_BLOCKED", "FATAL_DEADLOCK") else 1
                mode_enc = 1 if "FAST" in row.get("System_Message", "") else 0

                X.append([pitch, weight, sonic, obs_enc, mode_enc, passable, conf, impact_z])
                y_route.append(ROUTE_ENC[route])
                y_speed.append(SPEED_ENC[speed_key])
            except Exception:
                continue

        if not X:
            return np.array([]), np.array([]), np.array([])
        return np.array(X, dtype=np.float32), np.array(y_route), np.array(y_speed)

    def predict_route_speed(self, features: list):
        """
        XGBoost로 route + speed 예측.
        features: [pitch, weight, sonic, obs_enc, mode_enc, gemini_passable, gemini_conf, impact_z_prev]
        반환: (route_str, speed_int) | (None, None) if not ready
        """
        if not _ML_OK or self.route_clf is None or self.speed_clf is None:
            return None, None
        try:
            x = np.array([features], dtype=np.float32)
            r = int(self.route_clf.predict(x)[0])
            s = int(self.speed_clf.predict(x)[0])
            return ROUTE_DEC.get(r, config.ROUTE_C), SPEED_DEC.get(s, config.SPEED_DEFAULT)
        except Exception as e:
            print(f"[Trainer] 예측 오류: {e}")
            return None, None

    def anomaly_score(self, pitch: float, weight: float, sonic: float, accel_z: float) -> float:
        """
        Isolation Forest로 이상 점수 계산.
        반환: 0.0~1.0 (높을수록 이상, > 0.6이면 VLM 호출 권장)
        """
        if not _ML_OK or self.if_model is None:
            return 0.0
        try:
            x = np.array([[pitch, weight, sonic, accel_z]], dtype=np.float32)
            # decision_function: 음수일수록 이상. 정상 범위 [-0.5, 0] → 0.0~0.5로 매핑
            raw = float(self.if_model.decision_function(x)[0])
            # 정규화: raw -1.0 → score 1.0, raw 0.5 → score 0.0
            score = max(0.0, min(1.0, (-raw) / 1.5))
            return round(score, 3)
        except Exception:
            return 0.0
