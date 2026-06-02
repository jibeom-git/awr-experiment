# ai_core/config.py
# 스마트 팩토리 AGV 전역 상수 및 임계값 레지스터
# experiment/data/thresholds.json 파일이 존재하면 자동으로 로드하여 덮어씀

import os
import json

# ── 1. 구동계 출력 파라미터 (0 ~ 100 정수, Move.py 기준) ─────────────────────
SPEED_STOP           = 0    # 비상 정지 및 노드 정차
SPEED_SAFE_LOW       = 15   # 평지 비닐 노면 임계 저속 서행
SPEED_HEAVY_HILL     = 30   # 고중량 경사 감속 상한 (SPEED_HILL_HEAVY_CAP 별칭)
SPEED_HILL_HEAVY_CAP = 30   # 하위 호환 유지
SPEED_DEFAULT        = 40   # 평지 기본 크루징
SPEED_MEDIUM_POWER   = 45   # B 경로 10도 언덕 정속
SPEED_HIGH_POWER     = 55   # A 경로 20도 고경사 돌파

# ── 2. 주행 모드 토큰 ────────────────────────────────────────────────────────
MODE_FAST = "FAST"
MODE_SAFE = "SAFE"

# ── 3. 경로 상태 토큰 ────────────────────────────────────────────────────────
ROUTE_A   = "ROUTE_A"    # 20도 언덕, 최단거리
ROUTE_B   = "ROUTE_B"    # 10도 언덕, 중간거리
ROUTE_C   = "ROUTE_C"    # 언덕 없음, 최장거리
DEADLOCK  = "DEADLOCK"   # 모든 경로 차단

# ── 4. 센서 물리 임계값 (experiment 실측 후 thresholds.json으로 교체 예정) ───
SCALE_LOADCELL       = -292.007127   # 로드셀 스케일 팩터
TH_LOAD_HEAVY        = 150.0         # 화물 고중량 기준선 (g)
TH_SONIC_CRITICAL    = 10.0          # 충돌 방지 비상 제동 거리 (cm)
TH_SONIC_SLOWDOWN    = 50.0          # VLM 호출 구간 시작 거리 (cm)
TH_PITCH_HILL        = 7.0           # 경사 감지 각도 (도)
TH_PITCH_MEDIUM_HILL = 7.0           # 하위 호환 별칭
TH_PITCH_STEEP_HILL  = 15.0          # A경로 고경사 판정선 (도)
TH_PITCH_DELTA_FAIL  = 3.0           # 등판 실패 pitch_delta 임계값
TH_IMPACT_Z          = 4.0           # 방지턱 충격 Z축 순간 변화량 임계값
TH_SLIP_ACCEL        = 0.05          # 슬립 감지 가속도 임계값 (m/s²)

# ── 5. VLM Few-shot 설정 ─────────────────────────────────────────────────────
MAX_FEWSHOT_EXAMPLES = 5    # 프롬프트에 포함할 과거 사례 최대 수

# ── 6. 블랙박스 로그 경로 설정 ───────────────────────────────────────────────
LOG_ROOT         = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
DRIVING_LOG_NAME = "real_agv_history.csv"
OBSTACLE_DB_FILE = os.path.join(LOG_ROOT, "obstacle_db.json")
OBSTACLES_DIR    = os.path.join(LOG_ROOT, "obstacles")
MODELS_DIR       = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'models')

# ── 7. 토폴로지 노드 식별자 ───────────────────────────────────────────────────
NODE_START       = 0
NODE_ROUTE_SPLIT = 1

# ── 8. AI 추론 모델명 ─────────────────────────────────────────────────────────
GEMINI_MODEL_NAME = "gemini-2.5-flash"

# ── 9. thresholds.json 자동 로드 (존재 시 위 기본값을 덮어씀) ────────────────
_THRESHOLDS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    'experiment', 'data', 'thresholds.json'
)

def _load_thresholds():
    """thresholds.json 이 존재하면 해당 값으로 전역 상수를 동적 갱신"""
    if not os.path.exists(_THRESHOLDS_FILE):
        return
    try:
        with open(_THRESHOLDS_FILE, 'r', encoding='utf-8') as f:
            overrides = json.load(f)
        # 현재 모듈의 전역 네임스페이스에 직접 반영
        g = globals()
        for key, val in overrides.items():
            if key in g and isinstance(val, (int, float)):
                g[key] = val
        print(f"[CONFIG] thresholds.json 로드 완료: {list(overrides.keys())}")
    except Exception as e:
        print(f"[CONFIG] thresholds.json 로드 실패 (기본값 유지): {e}")

_load_thresholds()
