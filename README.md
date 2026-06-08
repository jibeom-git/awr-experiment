# Insite AGV — 스마트 팩토리 자율주행 AGV

## 1. 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **프로젝트명** | Insite AGV |
| **목적** | 스마트 팩토리 환경에서 화물을 운반하는 자율주행 AGV |
| **하드웨어** | Raspberry Pi 4 + Adeept AWR V3.0 4WD |
| **AI 구성** | Isolation Forest + Gemini 2.5 Flash VLM + XGBoost |
| **AI 파트** | 한지범 — 센서 융합, IsoForest/XGBoost 경로·속도 판단, VLM 연동, FSM, 데이터 수집/학습 |
| **라인트래킹 파트** | 팀원 — `app/cv_dashboard.py` 기반 라인 추종 및 분기점 경로 제어 |

---

## 2. 시스템 아키텍처

INSITE AGV는 **두 개의 독립된 Flask 프로세스**가 HTTP API로 통신하며 동작한다.

```
┌──────────────────────────────┐        ┌──────────────────────────────┐
│  dashboard.py (포트 5000)     │        │  app/cv_dashboard.py          │
│  AI 학습 및 데이터 수집 대시보드 │        │  (포트 5001, 팀원 담당)        │
│  - 센서 수집 / 경사로 학습 UI   │        │  - CSI 카메라 라인트래킹       │
│  - IsoForest / XGBoost 학습   │   HTTP │  - 분기점 경로 토큰 처리       │
│  - 모델 재학습 트리거          │ ─────▶ │  - 모터 직접 제어              │
└──────────────────────────────┘ /config │                              │
                                  /start  │  /state (GET) ◀─ 상태 조회    │
                                  /stop   └──────────────────────────────┘
                                  /reset
                                  /reroute
┌──────────────────────────────┐
│  app/auto_dashboard.py        │
│  (포트 5003)                  │
│  최종 자율주행 실행 대시보드     │   ─── 동일한 HTTP API로 cv_dashboard(5001)와 통신,
│  - FAST/SAFE 모드 선택         │       AI 판단 결과(route/speed)를 라인트래킹에 전달
│  - 경로 A→B→C 자율 판단/우회    │
│  - 무게 기반 속도 보정          │
└──────────────────────────────┘
```

- **`dashboard.py` (5000)**: AI 의사결정 엔진(`ai_core`)을 구동하며 센서 로그 수집, 경사로 학습 시나리오 진행, IsolationForest/XGBoost 모델 학습·재학습을 담당하는 **연구·개발용** 대시보드.
- **`app/auto_dashboard.py` (5003)**: 사용자가 FAST/SAFE 모드만 선택하면, 로봇이 ROUTE_A → B → C 순서로 장애물·경사·무게를 종합 판단해 자율주행하는 **최종 실행용** 대시보드.
- **`app/cv_dashboard.py` (5001)**: OV5647 CSI 카메라로 라인을 추종하고 분기점에서 `/config`, `/start`, `/stop`, `/reset`, `/reroute` API를 통해 AI 판단 결과(경로·속도)를 전달받아 모터를 직접 제어한다. **라인트래킹 파트(팀원) 담당, 수정 금지.**

---

## 3. 폴더 구조

```
insite/
├── ai_core/                          # AI 의사결정 엔진 (AI 파트 핵심 모듈)
│   ├── engine.py                     # AGVAIEngine: FSM + XGBoost + IsolationForest 통합 엔진
│   ├── trainer.py                    # ModelTrainer: XGBoost/IsoForest 학습·재학습·추론
│   ├── vlm_client.py                 # VLMClient: Gemini 2.5 Flash 장애물 측정(Few-shot)
│   ├── client.py                     # GeminiClient: google-generativeai 기반 구형 VLM 클라이언트(비활성)
│   ├── commander.py                  # CommanderVLM: 자연어 명령 → 주행모드/경로 판단
│   ├── signal_detector.py            # SignalDetectorVLM: OpenCV HSV + Gemini 신호등 색상 인식
│   ├── logger.py                     # BlackboxLogger: CSV 블랙박스 로그 + 장애물 DB 관리
│   ├── config.py                     # 전역 상수 (속도, 임계값, 경로 토큰, thresholds.json 자동 로드)
│   ├── migrate_data.py               # raw_experiment.csv 정제·이관 스크립트
│   └── __init__.py
│
├── sensors/                          # 하드웨어 드라이버 (공용)
│   ├── camera.py                     # Camera(picamera2 CSI) / USBCamera(OpenCV, 자동 인덱스 탐색)
│   ├── Picamera.py                   # picamera2 CSI 전용 드라이버
│   ├── motor.py                      # PCA9685 I2C(0x5f) MotorController
│   ├── mpu6050.py                    # MPU-6050 IMU (소프트 I2C5, GPIO12/13, 주소 0x68)
│   ├── hx711.py                      # HX711 로드셀 드라이버 + 영점(TARE) 캘리브레이션
│   ├── ultra.py                      # HC-SR04 초음파 거리 센서
│   ├── tracker.py                    # 3채널 IR 라인트래커 (GPIO17/27/22)
│   └── led.py                        # WS2812 RGB LED 상태 표시 (SPI0, GPIO10)
│
├── core/                             # 공용 유틸리티
│   ├── data_collector.py             # 비동기 멀티스레드 센서 데이터 수집기
│   ├── loadcell_calibrate.py         # HX711 터미널 캘리브레이션 스크립트
│   ├── waypoint_graph.json           # 웨이포인트/경로 그래프 정의 (현재 비어 있음)
│   └── __init__.py
│
├── app/                              # Flask 대시보드 앱 모음
│   ├── auto_dashboard.py             # ★ 최종 자율주행 실행 대시보드 (포트 5003)
│   ├── auto_dashboard_v2_manual.py   # 자율주행 + 키보드 수동 조종 결합 실험판
│   ├── auto_dashboard_v3.py          # 자율주행 대시보드 — 듀얼 카메라/캘리브레이션 통합판
│   ├── insite_final.py               # 자율주행 + 라인트래킹 + 신호등 통합 최종판 (포트 5003)
│   ├── cv_dashboard.py               # ★ 라인트래킹 대시보드 (포트 5001, 팀원 담당 — 수정 금지)
│   ├── cv_dashboard_bypass.py        # cv_dashboard 우회 테스트용 사본 (수정 금지)
│   ├── dashboard_collect.py          # 장애물 이미지/데이터 수집 전용 대시보드 (포트 5002)
│   ├── Dashboard_model_by.py         # ObstacleEngine 연동 실험용 대시보드 변형
│   ├── dashboard.py                  # 웨이포인트 기록 + 수동 제어 대시보드 (구버전, 포트 5001)
│   └── templates/
│       └── index.html                # app용 대시보드 UI 템플릿
│
├── experiment/                       # 학습 데이터 수집 및 분석 도구
│   ├── manual_drive.py               # 수동 조종 + 실시간 센서 로깅 서버 (포트 5001)
│   ├── label_tool.py                 # CSV 구간 레이블링 웹 UI (포트 5002, Chart.js)
│   ├── analyze.py                    # raw_experiment.csv 분석 → thresholds.json/리포트 생성
│   ├── visualize.py                  # experiment_v2.csv 분석 → HTML 리포트(report.html) 생성
│   ├── migrate_legacy.py             # raw_experiment.csv(레거시) → experiment_v2.csv 변환
│   ├── label_tool 등 templates/
│   │   ├── drive.html                # 수동 조종 UI 템플릿
│   │   └── label.html                # 레이블링 인터페이스 템플릿
│   ├── obstacle_photos/              # 장애물 사진 수집 결과 (다수의 obstacle_*.jpg)
│   ├── vlm_labels.csv                # VLM 레이블링 결과 CSV
│   └── data/
│       ├── experiment_v2.csv         # 경사로 학습 데이터 (session/phase/route 구조화 포맷)
│       ├── raw_experiment.csv        # 수동 실험 센서 로그 (레거시 포맷)
│       ├── raw_experiment_backup.csv # raw_experiment.csv 백업본
│       ├── thresholds.json           # analyze.py로 추출된 센서 임계값
│       └── analysis_report.html      # 분석 결과 리포트
│
├── models/                           # 학습된 ML 모델 파일
│   ├── xgboost_route.json            # 경로 선택 XGBoost 모델
│   ├── xgboost_speed.json            # 속도 제어 XGBoost 모델
│   └── isolation_forest.pkl          # 이상 감지 IsolationForest 모델
│
├── data/                             # 런타임 로그 및 장애물 DB
│   ├── real_agv_history.csv          # AI 판단 블랙박스 로그 (Logger 출력)
│   ├── real_agv_history_backup.csv   # 헤더 변경 시 자동 백업본
│   ├── obstacle_db.json              # 장애물 이미지 + VLM 판단 + 실제 결과 누적 DB
│   ├── obstacle_data.json            # 장애물 데이터 보조 파일
│   ├── deadlock_front_evidence.jpg   # FATAL_DEADLOCK 발생 시 저장되는 증거 이미지
│   ├── images/                       # 장애물 수집 이미지 (obstacle_001.jpg 등)
│   └── obstacles/                    # 런타임 장애물 이미지 저장 디렉토리 (obs_NNN.jpg)
│
├── templates/
│   └── index.html                    # dashboard.py(루트) 대시보드 UI 템플릿
│
├── tests/                            # 개별 센서·통합 테스트 스크립트
│   ├── test_mpu6050_lone.py          # IMU 단독 테스트
│   ├── test_loadcell_lone.py         # 로드셀 단독 테스트
│   ├── test_ultrasonic_lone.py       # 초음파 단독 테스트
│   ├── motor_test.py                 # 모터 구동계 테스트
│   ├── line_tracking.py              # 라인트래킹 + 모터 통합 테스트
│   ├── cv_follow.py                  # OpenCV 라인 추종 테스트
│   ├── route_follow.py               # 전체 경로 주행 테스트
│   ├── gyro_duration.py              # 자이로 지속 측정 테스트
│   ├── run_calibration.py            # 센서 캘리브레이션 실행
│   └── vlm_labeling.py               # VLM 레이블링 테스트
│
├── dashboard.py                      # ★ AI 학습/데이터 수집 대시보드 진입점 (포트 5000)
├── run_real_agv.py                   # FSM 엔진 + 하드웨어 통합 실행 런처 (수동 실험용)
├── gyro_offset.txt                   # MPU-6050 캘리브레이션 오프셋 캐시
├── loadcell_cal.txt                  # HX711 캘리브레이션 계수 캐시
└── cfg[green_min_area]               # (임시 생성 파일 — 분기점 색상 임계값 실험 잔여물)
```

---

## 4. 파일별 역할 설명

### 메인 실행 파일

| 파일 | 역할 |
|------|------|
| `dashboard.py` | AI 학습 및 데이터 수집 대시보드 진입점(포트 5000). 센서 안전 초기화, AI 엔진(`AGVAIEngine`) 구동, 경사로 학습 소켓 이벤트(`slope_start`/`slope_pause`/`slope_set_obstacle`/`slope_stop` 등) 처리 |
| `app/auto_dashboard.py` | 최종 자율주행 실행 대시보드(포트 5003). 사용자가 FAST/SAFE 모드만 선택하면 ROUTE_A→B→C 순서로 자율 판단·우회하며 주행 |
| `app/cv_dashboard.py` | 라인트래킹 대시보드(포트 5001). CSI 카메라 기반 라인 추종 및 분기점 경로 토큰 처리 (팀원 담당, **수정 금지**) |
| `run_real_agv.py` | `AGVAIEngine` + 모든 하드웨어 드라이버를 결합해 단독 실행하는 통합 런처 (가상 장애물 키 입력 실험용) |

### AI 코어 (`ai_core/`)

| 파일 | 역할 |
|------|------|
| `engine.py` | `AGVAIEngine` — IsolationForest 이상 감지 → VLM 호출 판단 → XGBoost 경로/속도 예측 → 물리 안전 규칙(override) → CSV 로깅까지의 전체 FSM 파이프라인을 통합. `evaluate()`(신규) / `evaluate_state_and_calculate_output()`(구버전 하위호환) 두 인터페이스 제공 |
| `trainer.py` | `ModelTrainer` — XGBoost(route/speed) 및 IsolationForest 모델의 초기 학습(합성 데이터 2,000건 + 실험 데이터), 자동 재학습(`retrain`), 실시간 추론(`predict_route_speed`, `anomaly_score`) 담당 |
| `vlm_client.py` | `VLMClient` — Gemini 2.5 Flash에 장애물 이미지를 전송해 높이/경사/표면 등 물리 수치만 측정(판단은 하지 않음). `analyze_obstacle()`이 결과를 엔진이 기대하는 `passable`/`recommended_speed_limit` 형식으로 변환 |
| `client.py` | `GeminiClient` — `google-generativeai` 패키지 기반 구형 VLM 클라이언트. 모듈 충돌로 `engine.py`에서 비활성화된 하위호환용 |
| `commander.py` | `CommanderVLM` — 사용자의 자연어 명령(예: "물건 떨어뜨리면 안돼")을 Gemini로 분석해 주행 모드(safe/fast)와 권장 경로(A/B/C)를 결정 |
| `signal_detector.py` | `SignalDetectorVLM` — OpenCV HSV 픽셀 카운팅으로 신호등 색상을 1차 추정(`detect_via_cv`)하고, 불확실 시 Gemini로 보강하는 하이브리드 신호등 인식기 |
| `logger.py` | `BlackboxLogger` — 모든 FSM 이벤트를 `data/real_agv_history.csv`에 누적 기록하고, 장애물 이미지/판단/결과를 `data/obstacle_db.json`에 저장·관리 |
| `config.py` | 속도 프로파일, 모드/경로 토큰, 센서 물리 임계값 등 전역 상수 정의. `experiment/data/thresholds.json`이 존재하면 해당 값으로 자동 덮어씀 |
| `migrate_data.py` | `raw_experiment.csv`의 손상/오타 데이터를 정제하고 표준 헤더로 재작성하는 이관 스크립트 |

### 센서 드라이버 (`sensors/`)

| 파일 | 역할 |
|------|------|
| `camera.py` | `Camera`(picamera2 CSI) / `USBCamera`(OpenCV USB, 재부팅마다 바뀌는 `/dev/video*` 인덱스 자동 탐색) |
| `Picamera.py` | picamera2 기반 CSI 카메라 전용 드라이버 |
| `motor.py` | PCA9685 I2C(주소 `0x5f`) 기반 `MotorController` — PWM 모터 제어 |
| `mpu6050.py` | MPU-6050 IMU 드라이버 — 소프트웨어 I2C5(GPIO12/13), pitch/roll/yaw 및 가속도 계산, 캘리브레이션 |
| `hx711.py` | HX711 로드셀 드라이버 — GPIO 직접 제어, TARE(영점) 캘리브레이션 내장 |
| `ultra.py` | HC-SR04 초음파 거리 센서 드라이버 — GPIO 트리거/에코 직접 제어 |
| `tracker.py` | 3채널 IR `LineTracker` — 좌/중/우 라인 감지 보조 센서 |
| `led.py` | WS2812 RGB LED 기반 상태 표시(SPI0/GPIO10), GPIO 충돌 회피를 위해 전방 LED는 GPIO25/27/22 사용 |

### 실험 데이터 수집 (`experiment/`)

| 파일 | 역할 |
|------|------|
| `manual_drive.py` | 수동 조종 + 실시간 센서 로깅 서버(포트 5001). 키보드로 직접 주행하며 `raw_experiment.csv`에 데이터 적재 |
| `label_tool.py` | 수집된 CSV를 구간 단위로 드래그 선택해 `result` 라벨을 일괄 수정하는 Chart.js 기반 웹 UI(포트 5002) |
| `analyze.py` | `raw_experiment.csv`를 분석해 센서 임계값을 자동 추출 → `thresholds.json` 및 `analysis_report.html` 생성 |
| `visualize.py` | `experiment_v2.csv`(경사로 학습 데이터)를 분석해 `report.html` 시각화 리포트 생성 |
| `migrate_legacy.py` | 헤더 없는 레거시 `raw_experiment.csv`를 `session_id`/`phase`/`route` 구조를 갖는 `experiment_v2.csv` 포맷으로 변환 |

### 학습 모델 (`models/`)

| 파일 | 역할 |
|------|------|
| `xgboost_route.json` | XGBoost 경로 선택 분류기 모델 (`ROUTE_A`/`B`/`C`/`DEADLOCK`) |
| `xgboost_speed.json` | XGBoost 속도 제어 분류기 모델 (6단계 속도 레이블) |
| `isolation_forest.pkl` | 정상 주행 패턴 학습용 IsolationForest 이상 감지 모델 |

### 설정 파일

| 파일 | 역할 |
|------|------|
| `ai_core/config.py` | 속도/모드/경로/센서 임계값 등 전역 상수 레지스트리. `experiment/data/thresholds.json` 자동 로드 |
| `experiment/data/thresholds.json` | `analyze.py`가 실측 데이터로부터 추출한 센서 임계값 — 로드 시 `config.py` 기본값을 덮어씀 |
| `gyro_offset.txt` / `loadcell_cal.txt` | MPU-6050/HX711 캘리브레이션 결과 캐시 파일 |
| `.gitignore` | `.venv/`, `__pycache__/`, `Log/`, `*.pyc`, `.env` 등 버전관리 제외 목록 |

---

## 5. AI 모델 설명

### 5-1. Isolation Forest

| 항목 | 내용 |
|------|------|
| **역할** | 정상 주행 패턴 대비 센서값의 이상 정도를 점수화하여, 일정 점수 초과 시 Gemini VLM 호출을 트리거 |
| **입력** | `pitch`, `weight`, `sonic`, `accel_z` (4차원 센서 벡터) |
| **출력** | `anomaly_score` (0.0~1.0). `decision_function` 결과를 정규화한 값으로, `0.6` 초과 + 장애물 구간 진입 시 VLM 호출(`use_vlm`) |
| **학습 데이터** | 합성 데이터(정상 주행 패턴, `_generate_synthetic_data`) + 경사로 학습 시 수집된 정상/슬립위험(`normal`/`slip_risk`) 실측 데이터로 `retrain_isolation_forest()`를 통해 재학습 |

### 5-2. XGBoost (경로/속도 분류기 2종)

| 항목 | 내용 |
|------|------|
| **역할** | 센서값 + Gemini VLM 판단 결과를 종합하여 최적 주행 경로(`route_clf`)와 속도(`speed_clf`)를 동시에 예측. XGBoost 결과가 없으면 FSM 규칙 기반 폴백(`_rule_based_route_speed`)으로 대체되고, 최종적으로는 물리 안전 규칙(`_apply_safety_override`)이 우선 적용됨 |
| **입력 피처 8개** | 아래 표 참고 |
| **출력** | 경로 (`ROUTE_A`/`ROUTE_B`/`ROUTE_C`/`DEADLOCK`) + 속도 (6단계: `STOP`/`SAFE_LOW`/`HEAVY_HILL`/`DEFAULT`/`MEDIUM_POWER`/`HIGH_POWER`) |
| **학습 데이터** | 합성 데이터(`_generate_synthetic_data`, FSM 물리 규칙 기반 2,000건) + 실험 수동 로그(`raw_experiment.csv`) + 실시간 블랙박스 로그(`real_agv_history.csv`, 20행 누적 시 자동 재학습) |

**입력 피처 8개 (`features` 배열 순서):**

| # | 피처명 | 설명 |
|---|--------|------|
| 1 | `pitch` | IMU pitch 각도 — 경사 감지 (도) |
| 2 | `weight` | 로드셀 적재 중량 (g) |
| 3 | `sonic` | 초음파 전방 거리 (cm) |
| 4 | `obs_enc` | 장애물 종류 인코딩 (`OBS_ENC`: 0=없음, 1=방지턱(1cm/3cm), 2=방지턱(2cm/5cm), 3=비닐, 4=기타/unknown) |
| 5 | `mode_enc` | 주행 모드 인코딩 (`MODE_ENC`: 0=SAFE, 1=FAST) |
| 6 | `gemini_passable` | Gemini 통과 가능 판단 (0 또는 1) |
| 7 | `gemini_conf` | Gemini 신뢰도 (0.0~1.0) |
| 8 | `impact_z_prev` | 직전 노드 통과 시 측정된 Z축 충격량 |

### 5-3. Gemini 2.5 Flash VLM

| 항목 | 내용 |
|------|------|
| **역할** | 전방 USB 웹캠 이미지를 분석해 장애물의 물리 수치(높이, 경사각, 표면 종류, 신뢰도)를 측정. 통과 가능 여부 자체는 판단하지 않으며, `analyze_obstacle()`이 측정값을 기준으로 `passable`/`recommended_speed_limit`을 산출해 XGBoost 입력으로 전달 |
| **호출 조건** | `0 < sonic ≤ TH_SONIC_SLOWDOWN(20cm)` 또는 가상 장애물 주입 상태이면서, **동시에** IsolationForest `anomaly_score > 0.6`인 경우 (`in_vlm_zone and use_vlm`) |
| **입력** | 전방 USB 웹캠 JPEG 이미지 + Few-shot 프롬프트(차체 높이 13cm/지상고 2cm 명시, `obstacle_db.json` 과거 사례 자동 첨부) |
| **출력** | `obstacle_type`, `height_cm`(0.0~2.0), `slope_deg`, `surface_type`, `confidence`, `description` → `passable`(높이<3cm·경사<25°·비닐 아님), `recommended_speed_limit`(0/20/30/35/50%) 으로 가공 |
| **관련 모델명** | `gemini-2.5-flash` (`config.GEMINI_MODEL_NAME`) |

---

## 6. 경로 결정 시나리오

> 표의 값은 `_apply_safety_override` / `_handle_mock_obs` 등 FSM 물리 규칙이 최종적으로 적용된 결과 기준이다.
> **우회 순서: A → B → C (항상 이 순서로 차단된 경로의 다음 경로로 우회)**

### FAST 모드 — 무게 없음

| 경로 | 장애물 없음 | 장애물 1cm | 장애물 2cm |
|------|-----------|-----------|-----------|
| A | 속도 70%로 통과 | B 우회 | B 우회 |
| B | 기본 속도로 통과 | 속도 70%로 통과 | C 우회 |
| C | 기본 속도로 통과 | 속도 60%로 통과 | STOP |

### FAST 모드 — 무게 140g

| 경로 | 장애물 없음 | 장애물 1cm | 장애물 2cm |
|------|-----------|-----------|-----------|
| A | B 우회 | B 우회 | B 우회 |
| B | 속도 80%로 통과 | C 우회 | C 우회 |
| C | 기본 속도로 통과 | 속도 70%로 통과 | STOP |

### SAFE 모드 — 무게 없음

| 경로 | 장애물 없음 | 장애물 1cm | 장애물 2cm |
|------|-----------|-----------|-----------|
| B | 기본 속도로 통과 | 속도 60%로 통과 | C 우회 |
| C | 기본 속도로 통과 | 속도 60%로 통과 | STOP |

### SAFE 모드 — 무게 140g

| 경로 | 장애물 없음 | 장애물 1cm | 장애물 2cm |
|------|-----------|-----------|-----------|
| B | 속도 60%로 통과 | C 우회 | C 우회 |
| C | 기본 속도로 통과 | 속도 70%로 통과 | STOP |

### 경사로 속도 프로파일

| 구간 | 적용 속도 |
|------|----------|
| 오르막 | 기본 속도의 70% |
| 내리막 | 기본 속도의 30% |
| 다음 노드 감지 시 | 기본 속도로 복귀 |

---

## 7. 실험 데이터 수집 시나리오 (경사로 학습)

`dashboard.py` (포트 5000)의 **경사로 학습 UI**를 사용한 데이터 수집 절차는 다음과 같다 (`on_slope_start`/`on_slope_pause`/`on_slope_set_obstacle`/`on_slope_bump_fail`/`on_slope_stop` 소켓 이벤트로 구현):

```
① 기록 시작 (slope_start)
   - 경로(A/B/C), 오르막/내리막 속도 입력 후 시작
   - cv_dashboard에 /config(base_speed) → /start 전송, 데이터 기록 개시 (phase="up")

② 오르막 주행
   - 실시간 센서값(pitch, weight, sonic, accel 등)을 phase="up"으로 기록

③ 일시 중지 (slope_pause)
   - 오르막 주행 종료 시 사용자가 일시 중지
   - cv_dashboard에 /stop 전송, phase="up_done"으로 전환

④ 결과 선택: 성공 / 슬립위험 / 실패
   ├─ 실패 (hill_fail)
   │     → 오르막 데이터에 즉시 result="hill_fail" 적용
   │     → session_id="{route}_up_{counter}" 로 즉시 저장 후 세션 종료 (내리막 진행 없음)
   │
   └─ 성공(normal) / 슬립위험(slip_risk)
         → 오르막 데이터에 해당 result 소급 적용
         → 클라이언트에 장애물 선택 팝업(show_obstacle_modal) 표시

⑤ 장애물 선택 (slope_set_obstacle): 없음 / 1cm / 2cm
   - 선택 즉시 내리막 자동 시작 (별도 재개 동작 불필요)
   - cv_dashboard에 /config(down_speed) → /start 전송, phase="down"으로 데이터 기록 개시
   - 선택한 장애물 정보는 내리막 구간 데이터에만 기록됨

⑥ 기록 종료
   ├─ 내리막 중 방지턱 통과 실패 (slope_bump_fail)
   │     → 내리막 데이터에 result="bump_fail" 일괄 적용 후 즉시 저장·세션 종료
   │
   └─ 정상 종료 (slope_stop) → 내리막 결과 선택
         → 사용자가 선택한 결과 라벨(normal/slip_risk 등)을 recording 상태인 행에 일괄 적용
         → session_id="{route}_{counter}" 로 CSV 저장
         → result가 normal/slip_risk인 데이터로 IsolationForest 자동 재학습(retrain_isolation_forest)

저장 위치: experiment/data/experiment_v2.csv
```

---

## 8. 실행 방법

```bash
# venv 활성화 (필수)
cd ~/insite
source .venv/bin/activate

# AI 학습 대시보드 (데이터 수집용, 포트 5000)
python dashboard.py

# 최종 자율주행 대시보드 (포트 5003)
python app/auto_dashboard.py

# 라인트래킹 (팀원, 포트 5001)
python app/cv_dashboard.py

# 데이터 분석 리포트 생성
python experiment/visualize.py

# 레거시 데이터 마이그레이션
python experiment/migrate_legacy.py
```

---

## 9. 하드웨어 구성표

| 부품 | 모델 | 역할 | 연결/핀 정보 |
|------|------|------|-------------|
| 메인 컴퓨터 | Raspberry Pi 4 | 전체 제어 | — |
| 차체 | Adeept AWR V3.0 4WD | 4륜 구동 플랫폼 | — |
| 모터 드라이버 | PCA9685 | PWM 모터 제어 | I2C, 주소 `0x5f` (`sensors/motor.py`) |
| IMU | MPU-6050 | 자세(pitch/roll/yaw)·충격 감지 | 소프트웨어 I2C5 — GPIO12(SDA) / GPIO13(SCL), 주소 `0x68` (`sensors/mpu6050.py`) |
| 로드셀 | HX711 | 화물 무게 측정 (TARE 필수) | GPIO5(DOUT) / GPIO6(PD_SCK) (`sensors/hx711.py`) |
| 초음파 센서 | HC-SR04 | 전방 장애물 거리 측정 | GPIO23(Trig) / GPIO24(Echo) (`sensors/ultra.py`) |
| IR 라인트래커 | 3채널 | 라인 감지 보조 | S1=GPIO17 / S2=GPIO27 / S3=GPIO22 (`sensors/tracker.py`) |
| RGB LED | WS2812 | 상태 표시 | SPI0, GPIO10 (전방 LED는 충돌 회피를 위해 GPIO25/27/22 사용, `sensors/led.py`) |
| 라인트래킹 카메라 | OV5647 (CSI) | 하단 라인 추종 (라인트래킹 파트) | CSI 포트, picamera2 (`sensors/Picamera.py`, `Camera`) |
| 전방 카메라 | USB 웹캠 | 장애물 인식·VLM 입력 (AI 파트) | USB, OpenCV `USBCamera` — 재부팅마다 `/dev/video*` 인덱스 자동 탐색 (`sensors/camera.py`) |

---

## 10. GitHub 브랜치 관리

| 브랜치 | 용도 |
|--------|------|
| `master` | 안정 버전 |
| `dev` | AI 파트(한지범) 작업 브랜치 |

**현재 작업 저장:**
```bash
cd ~/insite
git add .
git commit -m "작업 내용"
git push origin dev
```

**이전 커밋으로 불러오기 (복구):**
```bash
# 커밋 목록 확인
git log --oneline -10

# 작업 중인 변경사항을 모두 버리고 특정 커밋으로 복구
git checkout 커밋해시 -- .
```

> `git checkout 커밋해시 -- .` 실행 시 해당 커밋 이후의 변경사항이 모두 사라진다. 복구 전 반드시 현재 상태를 커밋하거나 `git stash`로 보관할 것.

---

## 11. 주의사항

- **`app/cv_dashboard.py` 절대 수정 금지** — 라인트래킹 파트(팀원) 담당 파일이다.
- **`app/cv_dashboard_bypass.py` 절대 수정 금지** — cv_dashboard 우회 테스트용 파일이다.
- **`base_speed=45` 기본값 유지** — 경사로/방지턱 구간에서만 일시적으로 변경하고 통과 후 즉시 기본값으로 복귀시킬 것.
- **USB 웹캠 인덱스 변경 주의** — 재부팅마다 `/dev/video0`, `/dev/video2` 등 인덱스가 바뀐다. `sensors/camera.py`의 자동 탐색 로직을 우선 신뢰하되, 실패 시 수동 확인이 필요하다.
- **로드셀 TARE(영점) 필수** — 측정 전 반드시 `hx711.tare()`로 영점을 잡아야 정확한 무게값을 얻을 수 있다.
- **포트 충돌 주의** — `5000`(dashboard.py) / `5001`(cv_dashboard.py, manual_drive.py) / `5003`(auto_dashboard.py) 이 동시에 충돌하지 않도록 실행 전 다른 프로세스 종료 여부를 확인할 것.
- **`GEMINI_API_KEY` 환경변수 설정 필수** — VLM(`vlm_client.py`), 신호등 인식(`signal_detector.py`), 명령 파서(`commander.py`) 모두 이 키가 없으면 `EnvironmentError`로 초기화 실패한다.
