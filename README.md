# Insite AGV — 스마트 팩토리 자율주행 AGV

## 1. 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **프로젝트명** | Insite AGV |
| **목적** | 스마트 팩토리 환경에서 화물을 자율주행으로 운반하는 AGV |
| **플랫폼** | Raspberry Pi 4 (4GB) + Adeept AWR V3.0 4WD |
| **AI 모델** | Isolation Forest + Gemini 2.5 Flash VLM + XGBoost |
| **AI 파트** | 한지범 — VLM 연동, IsoForest/XGBoost 경로·속도 판단, 신호등 인식, FSM |
| **라인트래킹 파트** | 팀원 — OV5647 CSI 카메라 기반 라인 추종, 경로 분기 제어 |
| **라즈베리파이 IP** | `192.168.0.50` |

---

## 2. 시스템 아키텍처

### 두 프로세스 구조

| 프로세스 | 파일 | 포트 | 담당 |
|---------|------|------|------|
| **AI 제어 대시보드** | `dashboard.py` | `5000` | AI 파트 (한지범) |
| **라인트래킹 대시보드** | `app/cv_dashboard.py` | `5001` | 라인트래킹 파트 (팀원) |

두 프로세스는 서로 독립적으로 실행되며, **HTTP API를 통해 통신**한다.

```
dashboard.py (포트 5000)
    │
    ├─ AI 판단 결과 → HTTP POST → cv_dashboard.py (포트 5001)
    │       /config  : 경로(route)와 기본 속도(base_speed) 전송
    │       /start   : 라인트래킹 주행 시작
    │       /stop    : 주행 정지
    │       /reset   : 상태 초기화
    │
    └─ cv_dashboard 상태 조회 → HTTP GET → /state
```

- **AI 대시보드(5000)**: 센서 데이터 수집, IsoForest 이상 감지, Gemini VLM 호출, XGBoost 경로/속도 결정, FSM 상태 관리
- **라인트래킹 대시보드(5001)**: CSI 카메라로 라인 추종, 분기점에서 경로 토큰 처리, 모터 직접 제어

---

## 3. 폴더 구조

```
insite/
├── ai_core/                      # AI 의사결정 엔진 (AI 파트 담당)
│   ├── engine.py                 # FSM + XGBoost + IsoForest 통합 메인 엔진
│   ├── vlm_client.py             # Gemini 2.5 Flash Few-shot 프롬프트 VLM 클라이언트
│   ├── client.py                 # Gemini API 기본 클라이언트
│   ├── config.py                 # 전역 상수 (속도 프로파일, 경로 토큰, 임계값)
│   ├── trainer.py                # XGBoost/IsoForest 학습 및 자동 재학습
│   ├── signal_detector.py        # 신호등 감지 (OpenCV HSV + Gemini fallback)
│   ├── logger.py                 # 블랙박스 CSV 로거 + 장애물 이미지 DB 관리
│   └── __init__.py
│
├── sensors/                      # 하드웨어 드라이버 (공용)
│   ├── camera.py                 # OV5647 CSI + USB 웹캠 카메라 드라이버
│   ├── Picamera.py               # picamera2 CSI 전용 드라이버
│   ├── motor.py                  # PCA9685 I2C(0x5f) 모터 제어
│   ├── mpu6050.py                # MPU-6050 IMU (software I2C5, GPIO12/13)
│   ├── hx711.py                  # HX711 로드셀 (GPIO5/6) + 캘리브레이션
│   ├── ultra.py                  # HC-SR04 초음파 (GPIO23/24)
│   ├── tracker.py                # 3채널 IR 라인트래커 (GPIO17/27/22)
│   └── led.py                    # WS2812 RGB LED (GPIO10, SPI0)
│
├── core/                         # 공용 유틸리티
│   ├── data_collector.py         # 비동기 멀티스레드 센서 데이터 수집기
│   ├── loadcell_calibrate.py     # HX711 캘리브레이션 스크립트
│   ├── waypoint_graph.json       # 웨이포인트/경로 정의 (A, B, C)
│   └── __init__.py
│
├── app/                          # Flask 대시보드 앱
│   ├── cv_dashboard.py           # ★ 라인트래킹 + 경로 분기 대시보드 (팀원 담당, 수정 금지)
│   ├── cv_dashboard_bypass.py    # cv_dashboard 우회 테스트용
│   ├── dashboard.py              # 웨이포인트 기록 + 수동 제어 (구버전)
│   ├── dashboard_collect.py      # 데이터 수집 변형 (팀원 담당, 수정 금지)
│   ├── static/                   # 정적 파일 (CSS, JS)
│   └── templates/
│       └── index.html            # 대시보드 UI 템플릿
│
├── experiment/                   # 학습 데이터 수집 및 분석 도구
│   ├── manual_drive.py           # 수동 조종 + 실시간 센서 로깅 (포트 5001)
│   ├── label_tool.py             # CSV 레이블링 웹 UI (포트 5002)
│   ├── analyze.py                # 임계값 추출 분석 스크립트
│   ├── templates/
│   │   ├── drive.html            # 수동 조종 UI 템플릿
│   │   └── label.html            # 레이블링 인터페이스 템플릿
│   └── data/
│       ├── raw_experiment.csv    # 수동 실험 센서 로그 (21,450행)
│       ├── thresholds.json       # analyze.py로 추출된 센서 임계값
│       └── analysis_report.html  # 분석 결과 리포트
│
├── tests/                        # 개별 센서 및 통합 테스트
│   ├── test_mpu6050_lone.py      # IMU 단독 테스트
│   ├── test_loadcell_lone.py     # 로드셀 단독 테스트
│   ├── test_ultrasonic_lone.py   # 초음파 단독 테스트
│   ├── motor_test.py             # 모터/구동계 테스트
│   ├── line_tracking.py          # 라인트래킹 + 모터 통합 테스트
│   ├── cv_follow.py              # OpenCV 라인 추종 테스트
│   ├── route_follow.py           # 전체 경로 주행 테스트
│   ├── gyro_duration.py          # 자이로 지속 측정 테스트
│   └── run_calibration.py        # 센서 캘리브레이션 실행
│
├── models/                       # 학습된 ML 모델
│   ├── xgboost_route.json        # 경로 선택 XGBoost 모델
│   ├── xgboost_speed.json        # 속도 제어 XGBoost 모델
│   └── isolation_forest.pkl      # 이상 감지 IsolationForest 모델
│
├── data/                         # 런타임 로그 및 장애물 DB
│   ├── real_agv_history.csv      # 주행 블랙박스 로그
│   ├── real_agv_history_backup.csv
│   ├── obstacle_db.json          # 장애물 이미지 + 판단 결과 누적 DB
│   ├── obstacle_data.json        # 장애물 데이터 보조 파일
│   ├── deadlock_front_evidence.jpg  # DEADLOCK 발생 시 촬영 증거 이미지
│   ├── images/                   # 장애물 이미지 저장
│   └── obstacles/                # 런타임 장애물 이미지 저장 디렉토리
│
├── templates/
│   └── index.html                # 루트 대시보드 UI 템플릿
│
├── dashboard.py                  # AI 의사결정 대시보드 진입점 (포트 5000)
├── run_real_agv.py               # 전체 시스템 통합 실행
├── gyro_offset.txt               # MPU-6050 캘리브레이션 오프셋 캐시
├── loadcell_cal.txt              # HX711 캘리브레이션 계수 캐시
├── Log/                          # 시스템 로그 디렉토리
└── .venv/                        # Python 가상환경
```

---

## 4. 하드웨어 구성

| 부품 | 모델 | 역할 | 연결 |
|------|------|------|------|
| 메인 컴퓨터 | Raspberry Pi 4 (4GB) | 전체 제어 | — |
| 차체 | Adeept AWR V3.0 4WD | 4륜 구동 플랫폼 | — |
| 확장보드 | Adeept Robot HAT V3.3 | GPIO/전원 분배 | — |
| 모터 드라이버 | PCA9685 + DRV8833 | PWM 모터 제어 | I2C1, 주소 `0x5f` |
| 라인트래킹 카메라 | OV5647 CSI | 하단 라인 감지 (라인트래킹 파트) | CSI 포트 |
| 전방 카메라 | USB 웹캠 | 장애물 인식 (AI 파트) | USB (재부팅마다 인덱스 변경, 자동 탐색) |
| IMU | MPU-6050 | 자세/충격 감지 | software I2C5, GPIO12(SDA)/GPIO13(SCL) |
| 로드셀 | HX711 (5 kg) | 화물 무게 측정 | GPIO5(DAT) / GPIO6(SCK) |
| 초음파 센서 | HC-SR04 | 전방 거리 측정 | GPIO23(Trig) / GPIO24(Echo) |
| IR 라인트래커 | 3채널 | 라인 감지 보조 | GPIO17(S1) / GPIO27(S2) / GPIO22(S3) |
| RGB LED | WS2812 | 상태 표시 | SPI, GPIO10 |
| Buzzer | — | 경보음 | GPIO18 |

### 카메라 구성 요약

| 카메라 | 연결 방식 | 담당 파트 | 용도 |
|--------|---------|---------|------|
| OV5647 | CSI (picamera2) | 라인트래킹 파트 | 라인 추종, 경로 분기 |
| USB 웹캠 | USB (OpenCV) | AI 파트 | 신호등 감지, 장애물 인식, Gemini VLM 입력 |

> USB 웹캠은 재부팅마다 `/dev/video0`, `/dev/video2` 등 인덱스가 변경된다. `sensors/camera.py`에 자동 탐색 로직이 구현되어 있다.

---

## 5. AI 모델 구조

### 5-1. Isolation Forest

| 항목 | 내용 |
|------|------|
| **역할** | 정상 주행 패턴 대비 이상 감지 → Gemini VLM 호출 트리거 |
| **입력** | 초음파 거리, IMU pitch/accel, 로드셀 무게, 속도 명령 등 센서 벡터 |
| **출력** | 이상 점수 (anomaly_score) — 임계값 초과 시 VLM 호출 |
| **파일** | `ai_core/trainer.py`, `models/isolation_forest.pkl` |

### 5-2. Gemini 2.5 Flash VLM

| 항목 | 내용 |
|------|------|
| **역할** | 전방 카메라 이미지를 보고 장애물 통과 가능 여부 판단 |
| **호출 조건** | IsolationForest 이상 감지 또는 초음파 거리 임계값 이하 |
| **입력** | USB 웹캠 이미지 + Few-shot 예시 (`obstacle_db.json`에서 자동 첨부) |
| **출력** | 통과 가능 여부(`passable`), 장애물 종류(`obs_type`), 신뢰도(`confidence`), 이유(`reason`) |
| **파일** | `ai_core/vlm_client.py`, `ai_core/signal_detector.py` |

### 5-3. XGBoost

| 항목 | 내용 |
|------|------|
| **역할** | 센서값 + Gemini 판단 결과를 종합하여 최적 경로와 속도 결정 |
| **파일** | `ai_core/trainer.py`, `models/xgboost_route.json`, `models/xgboost_speed.json` |

**입력 피처 8개:**

| # | 피처명 | 설명 |
|---|--------|------|
| 1 | `pitch` | IMU pitch 각도 (경사 감지) |
| 2 | `weight` | 로드셀 적재 중량 (g) |
| 3 | `sonic` | 초음파 전방 거리 (cm) |
| 4 | `obs_enc` | 장애물 종류 인코딩 (0=없음, 1=bump_3cm, 2=bump_2cm, 3=vinyl, 4=기타) |
| 5 | `mode_enc` | 주행 모드 인코딩 (0=NORMAL, 1=FAST) |
| 6 | `gemini_passable` | Gemini 통과 가능 판단 (0/1) |
| 7 | `gemini_conf` | Gemini 신뢰도 (0.0~1.0) |
| 8 | `impact_z_prev` | 이전 노드에서의 Z축 충격량 |

**출력:** 경로 선택 (ROUTE_A / ROUTE_B / ROUTE_C / DEADLOCK) + 속도 명령 (0~80%)

### 5-4. FSM (유한 상태 기계)

| FSM 상태 | 조건 |
|---------|------|
| `NORMAL_CRUISE` | 장애물 없음, 정상 평지 주행 |
| `STEEP_HILL_CLIMB` / `STEEP_HILL_CLIMB_FAST` | ROUTE_A, 경사 20° 이상 — 고출력 돌파 |
| `MEDIUM_HILL_CLIMB` | ROUTE_B, 경사 10° 이상 — 중속 정속 |
| `HILL_HEAVY_GUARD` | 경사 + 고중량 — 감속 상한 적용 |
| `SAFE_HEAVY_GUARD` | 고중량 적재, 평지 — 저속 서행 |
| `CAUTIOUS_{OBS_TYPE}` | 장애물 감지, 통과 가능 판단 — 서행 통과 |
| `PATH_BLOCKED` | 현재 경로 통과 불가 — 우회 경로 탐색 |
| `REROUTING_RUN` | 대안 경로로 재설정 후 주행 재개 |
| `CRITICAL_STOP` | 초음파 임계 거리 이하 — 비상 정지 |
| `FATAL_DEADLOCK` | 모든 경로 차단 — 운영자 개입 대기 |
| `WAITING_OPERATOR_COMMAND` | DEADLOCK 상태, 관리자 명령 대기 |

---

## 6. 최종 동작 시나리오

```
① 출발 전
   - AI 대시보드(포트 5000) 실행
   - XGBoost/IsolationForest 모델 로드
   - 라인트래킹 대시보드(포트 5001)에 경로/속도 사전 전송
   - 웹 UI에서 [Start Drive] 버튼 클릭

② 신호등 감지
   - USB 웹캠으로 신호등 연속 촬영
   - OpenCV HSV로 빨강/노랑/초록 1차 판단
   - 불확실 시 Gemini 2.5 Flash fallback 호출
   - 빨강/노랑 → 정차 대기 / 초록 → cv_dashboard /start 전송하여 주행 시작

③ 정상 주행
   - cv_dashboard가 CSI 카메라로 라인 추종
   - AI 대시보드가 IMU, 초음파, 로드셀 지속 수집
   - IsolationForest가 이상 여부 실시간 감시
   - FSM 상태: NORMAL_CRUISE / STEEP_HILL_CLIMB / MEDIUM_HILL_CLIMB

④ 장애물 처리
   - IsolationForest 이상 감지 또는 초음파 임계값 이하
   - USB 웹캠으로 장애물 촬영
   - obstacle_db.json에서 유사 사례 자동 첨부하여 Gemini 호출
   - 통과 가능 → CAUTIOUS 상태로 서행 통과
   - 통과 불가 → PATH_BLOCKED → 우회 경로 탐색

⑤ 우회 경로 선택
   - XGBoost가 현재 센서값으로 대안 경로 결정 (ROUTE_A/B/C 중 가능한 것)
   - cv_dashboard에 /stop → /reset → /config(새 경로) → /start 순서로 전송
   - FSM 상태: REROUTING_RUN

⑥ 통과 후 학습
   - MPU-6050으로 IMU 충격량 측정 → 통과 결과 검증
   - 결과를 obstacle_db.json에 저장
   - 누적 20건 이상 → XGBoost 자동 재학습
   - 다음 Gemini 호출 시 이 사례가 Few-shot으로 포함

⑦ DEADLOCK
   - 모든 경로(A/B/C) 통과 불가 판단 시
   - deadlock_front_evidence.jpg 촬영 저장
   - FSM 상태: FATAL_DEADLOCK → WAITING_OPERATOR_COMMAND
   - 한국어 DEADLOCK 경보 출력 및 웹 UI 알림
   - 운영자가 웹 UI 채팅으로 해제 명령 전송 → 상태 리셋 후 재시도
```

---

## 7. 실행 방법

```bash
# venv 활성화 (필수)
source ~/insite/.venv/bin/activate
cd ~/insite

# AI 대시보드 (포트 5000) — AI 파트
python dashboard.py
# 접속: http://192.168.0.50:5000

# 라인트래킹 대시보드 (포트 5001) — 팀원 파트
python app/cv_dashboard.py
# 접속: http://192.168.0.50:5001

# 실험 데이터 수집 (수동 조종 + 센서 로깅)
python experiment/manual_drive.py
# 접속: http://192.168.0.50:5001

# CSV 레이블링 도구
python experiment/label_tool.py
# 접속: http://192.168.0.50:5002

# 데이터 분석 및 임계값 추출
python experiment/analyze.py
```
python app/auto_dashboard.py

python app/auto_dashboard_v3.py

---

## 8. 실험 데이터 현황

### raw_experiment.csv — 21,450행 수집 완료

| result | 행 수 | 설명 |
|--------|-------|------|
| `normal` | 12,107 | 정상 평지 주행 |
| `hill_fail` | 6,170 | 언덕 등판 실패 (속도 부족) |
| `cautious_pass` | 1,804 | 서행 통과 성공 |
| `impact` | 1,291 | 충격 발생 |
| `slip` | 78 | 슬립 감지 |

### thresholds.json — 추출된 임계값

| 임계값 | 값 | 의미 |
|--------|-----|------|
| `TH_PITCH_DELTA_FAIL` | 1.0° | 언덕 실패 판단 pitch 변화량 |
| `TH_SLIP_ACCEL` | 4.1175 | 슬립 판단 가속도 비율 |
| `TH_IMPACT_Z` | 10.81 m/s² | 충격 판단 Z축 가속도 |
| `cautious_pass_speed_avg` | 50.79% | 서행 통과 평균 속도 |

---

## 9. 주의사항

- **`app/cv_dashboard.py` 절대 수정 금지** — 라인트래킹 파트 담당 파일이다.
- **`app/dashboard_collect.py` 절대 수정 금지** — 팀원 담당 파일이다.
- **키보드 제어**: `ArrowUp` / `ArrowDown` / `ArrowLeft` / `ArrowRight` / `Space` 만 사용. `W/A/S/D` 금지.
- **USB 웹캠**: 재부팅마다 `/dev/video` 인덱스가 변경된다. `sensors/camera.py`에 자동 탐색이 구현되어 있으나, 탐색 실패 시 수동으로 인덱스 확인 필요.
- **venv 활성화 필수**: 모든 실행 전 `source ~/insite/.venv/bin/activate` 실행.
- **포트 충돌**: `experiment/manual_drive.py`와 `app/cv_dashboard.py`는 모두 5001 포트를 사용하므로 동시 실행 불가.
- **GPIO 충돌**: 두 대시보드를 동시에 실행하면 모터/센서 GPIO 충돌이 발생한다. 실험 전 다른 프로세스 종료 확인.
- **라즈베리파이 안전 종료**: 실험 후 반드시 `sudo shutdown -h now` 실행 후 초록 LED 꺼진 뒤 전원 분리.

---

## 10. 네트워크 접속 정보

| 장소 | SSID | 비밀번호 |
|------|------|---------|
| 연구실 | `326_AP1` | `43024302a!` |
| 실험실 | `스마트팩토리608` | `smart608` |
| 실험실 5GHz | `스마트팩토리608_5G` | `smart608` |
| 이동 중 | `JB` (한지범 핫스팟) | `11112222` |

---

## 11. 개발 환경

```bash
# SSH 접속
ssh pi@192.168.0.50
# 또는
ssh pi@pi.local

# Claude Code 실행
source ~/insite/.venv/bin/activate
cd ~/insite
claude --dangerously-skip-permissions

# Gemini API 키 설정
echo 'export GEMINI_API_KEY="발급받은_키_입력"' >> ~/.bashrc
source ~/.bashrc
```

```
# VSCode Remote-SSH 설정 (~/.ssh/config)
Host awr-pi
    HostName pi.local
    User pi
    Port 22
```

---

## 12. 데이터 초기화

```bash
# 주행 블랙박스 로그 초기화
> data/real_agv_history.csv

# 장애물 판단 DB 초기화
echo '[]' > data/obstacle_db.json

# 장애물 이미지 전체 삭제
rm -f data/obstacles/*.jpg data/images/*.jpg
```
---

## 13. GitHub 브랜치 관리

### 브랜치 구조

| 브랜치 | 용도 |
|---|---|
| `master` | 안정 버전 (팀원과 공유) |
| `dev` | AI 파트 작업 브랜치 (한지범) |

---

### 현재 작업 저장 (업로드)

```bash
cd ~/insite
git add .
git commit -m "2026.06.08_3"
git push origin dev
```

---

### 이전 저장 상태로 복구 (다운로드)

**커밋 목록 확인:**
```bash
git log --oneline -10
```

**특정 커밋으로 완전 복구:**
```bash
# 작업 중인 변경사항 전부 버리고 특정 커밋으로 복구
git checkout 커밋해시 -- .

# 예시
git checkout 1b698c1 -- .
```

**주의:** `git checkout 커밋해시 -- .` 실행 시 커밋 이후 변경사항이 전부 사라집니다. 복구 전 반드시 현재 상태를 커밋하거나 stash에 저장하세요.

---

### 작업 전 안전 저장 후 복구하는 올바른 순서

```bash
# 1. 현재 상태 먼저 저장
git add .
git commit -m "작업 전 백업"

# 2. 이전 상태로 복구
git checkout 1b698c1 -- .

# 3. 복구 후 다시 최신으로 돌아오려면
git checkout dev -- .
```

---

### Claude Code 작업 시 필수 규칙

Claude Code가 작업 중 커밋을 안 하면 나중에 복구가 불가능합니다. 반드시 프롬프트 맨 앞에 아래 문장을 추가하세요.

```
각 수정 단계 완료 시마다 반드시
git add . && git commit -m "작업내용" 실행해라.
dashboard.py 를 직접 실행하지 마라.
python -m py_compile 로 문법 검증만 해라.
```