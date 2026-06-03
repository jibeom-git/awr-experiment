# Insite — VLM 기반 자율주행 AGV

## 1. 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **프로젝트명** | Insite |
| **목표** | VLM(Vision Language Model)과 멀티센서 융합 기반 자율주행 AGV |
| **플랫폼** | Raspberry Pi 4 + Adeept AWR V3.0 4WD |
| **AI 모델** | Google Gemini 2.5 Flash (장애물 판단) + XGBoost (경로/속도 제어) |
| **AI 파트** | 한지범 — Gemini VLM 연동, IsoForest/XGBoost 경로 판단, 신호등 인식 |
| **라인트래킹 파트** | 팀원 — OV5647 카메라 기반 라인 추종, 경로 분기 제어 |

---

## 2. 하드웨어 구성표

| 부품 | 모델 | 역할 | 비고 |
|------|------|------|------|
| 메인 컴퓨터 | Raspberry Pi 4 (4GB) | 전체 제어 | IP 고정: `192.168.0.50` |
| 차체 | Adeept AWR V3.0 4WD | 4륜 구동 플랫폼 | — |
| 확장보드 | Adeept Robot HAT V3.3 | GPIO/전원 분배 | — |
| 모터 드라이버 | PCA9685 + DRV8833 | PWM 모터 제어 | I2C1, 주소 `0x5f` |
| 라인트래킹 카메라 | OV5647 CSI | 하단 라인 감지 | picamera2 사용 |
| 전방 카메라 | Orbbec Astra USB | 장애물 인식 | RGB 모드 전용 |
| IMU | MPU-6050 | 자세/충격 감지 | software I2C bus5, GPIO12/13 |
| 로드셀 | HX711 (5 kg) | 화물 무게 측정 | GPIO5(DAT) / GPIO6(SCK) |
| 초음파 센서 | HC-SR04 | 전방 거리 측정 | GPIO23(Trig) / GPIO24(Echo) |
| RGB LED | WS2812 | 상태 표시 | SPI, GPIO10 |

---

## 3. 배선 정보

### 라즈베리파이 → Adeept HAT 배선 (I2C1 및 제어 핀)

| 핀 번호 | GPIO | 기능 | 연결 대상 |
|---------|------|------|-----------|
| Pin 3 | GPIO2 (SDA1) | I2C1 Data | HAT I2C 제어 |
| Pin 5 | GPIO3 (SCL1) | I2C1 Clock | HAT I2C 제어 |
| Pin 6 | GND | 공통 그라운드 | — |
| Pin 11 | GPIO17 | 라인트래킹 S1 | IR 센서 채널 1 |
| Pin 12 | GPIO18 | Buzzer 제어 | 부저 |
| Pin 13 | GPIO27 | 라인트래킹 S2 | IR 센서 채널 2 |
| Pin 15 | GPIO22 | 라인트래킹 S3 | IR 센서 채널 3 |
| Pin 16 | GPIO23 | 초음파 Trig | HC-SR04 |
| Pin 18 | GPIO24 | 초음파 Echo | HC-SR04 |
| Pin 19 | GPIO10 (SPI_MOSI) | WS2812 Data | RGB LED |
| Pin 20 | GND | 추가 그라운드 | — |
| Pin 23 | GPIO11 (SPI_SCLK) | SPI Clock | — |
| Pin 24 | GPIO8 (SPI_CE0) | SPI Chip Select 0 | — |
| Pin 26 | GPIO7 (SPI_CE1) | SPI Chip Select 1 | — |

### MPU-6050 배선 (I2C5 분리)

| MPU-6050 핀 | 라즈베리파이 핀 | 설명 |
|-------------|----------------|------|
| VCC | Pin 1 (3.3V) | 전원 |
| GND | Pin 9 (GND) | 그라운드 |
| SDA | Pin 32 (GPIO12) | software I2C5 Data |
| SCL | Pin 33 (GPIO13) | software I2C5 Clock |

### HX711 로드셀 배선

| HX711 핀 | 라즈베리파이 핀 | 설명 |
|----------|----------------|------|
| VDD | Pin 1 (3.3V) | 로직 전원 |
| VCC | Pin 2 (5V) | 센서 전원 |
| GND | Pin 14 (GND) | 그라운드 |
| DAT | Pin 29 (GPIO5) | 데이터 출력 |
| SCK | Pin 31 (GPIO6) | 클럭 입력 |

---

## 4. 트랙 및 경로 구성

| 경로 | 언덕 경사 | 거리 | 최소 속도 | 특징 |
|------|----------|------|----------|------|
| **ROUTE_A** | 20° | 최단 | 55% 이상 | 고속 필요, 충격 위험 |
| **ROUTE_B** | 10° | 중간 | 45% | 균형형 |
| **ROUTE_C** | 없음 | 최장 | — | 가장 안전, 저속 가능 |

- 각 경로 분기점에 **검은색 사각형 노드** 배치 — AGV가 정차 후 AI 판단 수행
- **신호등 규칙:** 빨강/노랑 → 정차 대기, 초록 → 출발

---

## 5. 네트워크 접속 정보

> **주의:** 라즈베리파이와 본인 PC가 **같은 WiFi**에 연결되어 있어야 접속 가능합니다.

| 장소 | SSID | 비밀번호 |
|------|------|---------|
| 연구실 | `326_AP1` | `43024302a!` |
| 실험실 | `스마트팩토리608` | `smart608` |
| 실험실 5GHz | `스마트팩토리608_5G` | `smart608` |
| 이동 중 | `JB` (한지범 핫스팟) | `11112222` |

---

## 6. 윈도우 PC 초기 환경 설정 (최초 1회)

### 6-1. VSCode 설치

1. `https://code.visualstudio.com` 접속
2. **Download for Windows** 클릭 → 설치
3. 설치 중 **"Add to PATH"** 반드시 체크

### 6-2. Remote-SSH 확장 설치

1. VSCode 실행 → 왼쪽 Extensions 아이콘 클릭
2. `Remote - SSH` 검색 → **Microsoft** 제공 항목 설치

### 6-3. SSH 설정 파일 등록

1. `Ctrl+Shift+P` → `Remote-SSH: Open SSH Configuration File` → Enter
2. `C:\Users\사용자명\.ssh\config` 선택
3. 아래 내용 붙여넣기 후 저장 (`Ctrl+S`)

```
Host awr-pi
    HostName pi.local
    User pi
    Port 22
```

### 6-4. Git 설치

1. `https://git-scm.com/download/win` 접속
2. **64-bit Git for Windows Setup** 다운로드 → 기본값으로 설치

### 6-5. GitHub Collaborator 초대

1. GitHub 계정 생성 (없는 경우)
2. **한지범에게 본인 GitHub 계정명 전달** → Collaborator 초대 수락

---

## 7. 라즈베리파이 접속 방법

### 방법 A — VSCode Remote-SSH (권장)

1. VSCode 실행
2. `Ctrl+Shift+P` → `Remote-SSH: Connect to Host` → `awr-pi` 선택
3. 새 창이 열리면 접속 완료
4. **Terminal → New Terminal** 로 Pi 터미널 사용

### 방법 B — PowerShell 직접 접속

```powershell
ssh pi@pi.local
```

> `pi.local` 이 안 되면 IP로 시도: `ssh pi@192.168.0.50`

### 접속 후 매번 실행 (필수)

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
```

성공하면 프롬프트가 아래처럼 바뀝니다:

```
(.venv) pi@pi:~/insite $
```

---

## 8. 디렉토리 구조

```
insite/
├── ai_core/                    # AI 의사결정 엔진 (AI 파트 담당)
│   ├── engine.py               # FSM + XGBoost + IsoForest 통합 메인 엔진
│   ├── client.py               # Gemini 2.5 Flash API 클라이언트
│   ├── vlm_client.py           # Few-shot 프롬프트 관리 VLM 클라이언트
│   ├── config.py               # 전역 상수 (속도 프로파일, 경로 토큰, 임계값)
│   ├── trainer.py              # XGBoost/IsoForest 학습 및 자동 재학습
│   ├── signal_detector.py      # 신호등 감지 (HSV + Gemini fallback)
│   ├── logger.py               # 블랙박스 CSV 로거 + 장애물 이미지 DB 관리
│   └── __init__.py
│
├── sensors/                    # 하드웨어 드라이버 (공용)
│   ├── camera.py               # OV5647 CSI + Orbbec Astra USB 카메라
│   ├── motor.py                # PCA9685 I2C(0x5f) 모터 제어
│   ├── mpu6050.py              # MPU-6050 IMU (I2C5, GPIO12/13)
│   ├── hx711.py                # HX711 로드셀 (GPIO5/6) + 캘리브레이션
│   ├── ultra.py                # HC-SR04 초음파 (GPIO23/24)
│   ├── tracker.py              # 3채널 IR 라인트래커 (GPIO17/27/22)
│   └── led.py                  # WS2812 RGB LED (GPIO10, SPI0)
│
├── core/                       # 공용 유틸리티
│   ├── data_collector.py       # 비동기 센서 데이터 수집기
│   ├── loadcell_calibrate.py   # HX711 캘리브레이션 스크립트
│   └── waypoint_graph.json     # 웨이포인트/경로 정의 (A, B, C)
│
├── app/                        # Flask 대시보드 앱
│   ├── cv_dashboard.py         # 라인트래킹 + 경로 분기 대시보드 (팀원 담당)
│   ├── dashboard.py            # 웨이포인트 기록 + 수동 제어
│   ├── dashboard_collect.py    # 데이터 수집 변형 (팀원 담당, 수정 금지)
│   └── templates/
│       └── index.html          # 대시보드 UI 템플릿
│
├── experiment/                 # 학습 데이터 수집 및 분석 도구
│   ├── manual_drive.py         # 수동 조종 + 실시간 센서 로깅 (포트 5001)
│   ├── label_tool.py           # CSV 레이블링 웹 UI (포트 5002)
│   ├── analyze.py              # 임계값 추출 분석 스크립트
│   ├── templates/
│   │   ├── drive.html          # 수동 조종 UI 템플릿
│   │   └── label.html          # 레이블링 인터페이스 템플릿
│   └── data/
│       └── raw_experiment.csv  # 수동 실험 센서 로그
│
├── tests/                      # 개별 센서 및 통합 테스트
│   ├── test_mpu6050_lone.py    # IMU 단독 테스트
│   ├── test_loadcell_lone.py   # 로드셀 단독 테스트
│   ├── test_ultrasonic_lone.py # 초음파 단독 테스트
│   ├── motor_test.py           # 모터/구동계 테스트
│   ├── line_tracking.py        # 라인트래킹 + 모터 통합 테스트
│   ├── cv_follow.py            # OpenCV 라인 추종 테스트
│   ├── route_follow.py         # 전체 경로 주행 테스트
│   └── run_calibration.py      # 센서 캘리브레이션 실행
│
├── models/                     # 학습된 ML 모델
│   ├── xgboost_route.json      # 경로 선택 모델
│   ├── xgboost_speed.json      # 속도 제어 모델
│   └── isolation_forest.pkl    # 이상 감지 모델
│
├── data/                       # 런타임 로그 및 장애물 DB
│   ├── real_agv_history.csv    # 주행 블랙박스 로그
│   ├── obstacle_db.json        # 장애물 이미지 + 판단 결과 누적 DB
│   └── obstacles/              # 런타임 장애물 이미지 저장 디렉토리
│
├── templates/
│   └── index.html              # 루트 대시보드 UI 템플릿
│
├── dashboard.py                # AI 의사결정 대시보드 진입점 (포트 5000)
├── run_real_agv.py             # 전체 시스템 통합 실행
├── gyro_offset.txt             # MPU-6050 캘리브레이션 오프셋 캐시
├── loadcell_cal.txt            # HX711 캘리브레이션 계수 캐시
└── .venv/                      # Python 가상환경
```

---

## 9. 실행 파일 및 명령어

### 라인트래킹 + 경로 주행 대시보드 — 팀원 담당

- **파일:** `app/cv_dashboard.py`
- **접속:** `http://192.168.0.50:5001`

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
python app/cv_dashboard.py
```

---

### AI 의사결정 대시보드 — AI 파트 담당

- **파일:** `dashboard.py` (루트)
- **접속:** `http://192.168.0.50:5000`

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
python dashboard.py
```

---

### 수동 조종 + 실험 데이터 수집

- **파일:** `experiment/manual_drive.py`
- **접속:** `http://192.168.0.50:5001`

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
python experiment/manual_drive.py
```

---

### 레이블링 도구

- **파일:** `experiment/label_tool.py`
- **접속:** `http://192.168.0.50:5002`

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
python experiment/label_tool.py
```

---

### 임계값 분석

- **파일:** `experiment/analyze.py`

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
python experiment/analyze.py
```

---

## 10. Gemini API 키 설정

```bash
echo 'export GEMINI_API_KEY="발급받은_키_입력"' >> ~/.bashrc
source ~/.bashrc
```

확인:

```bash
echo $GEMINI_API_KEY
```

---

## 11. AI 파트 동작 원리

### 자기개선 사이클

```
노드 정차
   │
   ▼
Isolation Forest → 이상 감지?
   │ Yes
   ▼
Orbbec Astra로 사진 촬영
   │
   ▼
Gemini 2.5 Flash 호출 (Few-shot 프롬프트 포함)
   │  ← obstacle_db.json 에서 유사 사례 자동 첨부
   ▼
경로 토큰 + 속도 결정 (XGBoost 보정)
   │
   ▼
AGV 통과 실행
   │
   ▼
MPU-6050 으로 IMU 충격량 측정
   │
   ▼
결과를 obstacle_db.json 에 저장
   │
   ▼
누적 20건 이상 → XGBoost 자동 재학습
   │
   ▼
다음 Gemini 호출 시 이 사례가 Few-shot 으로 포함
```

### 각 모듈 역할

| 모듈 | 파일 | 역할 |
|------|------|------|
| **Isolation Forest** | `ai_core/trainer.py` | 정상 주행 대비 이상 패턴 감지 → VLM 호출 트리거 |
| **XGBoost** | `ai_core/trainer.py` | 센서값 + Gemini 판단 기반 경로/속도 최종 결정 |
| **Gemini 2.5 Flash** | `ai_core/vlm_client.py` | Few-shot 프롬프트로 장애물 통과 가능 여부 판단 |
| **신호등 감지** | `ai_core/signal_detector.py` | OpenCV HSV 1차 판단, 불확실 시 Gemini fallback |
| **블랙박스 로거** | `ai_core/logger.py` | 주행 이력 CSV 기록 + 장애물 이미지 DB 누적 |

---

## 12. 데이터 파일 설명

| 파일/디렉토리 | 형식 | 내용 |
|--------------|------|------|
| `data/real_agv_history.csv` | CSV | 주행 블랙박스: 타임스탬프, 노드, 경로, 장애물, 속도, Gemini 판단 이유 |
| `data/obstacle_db.json` | JSON | 장애물 이미지 경로 + 판단 결과 + 통과 여부 누적 DB |
| `data/obstacles/` | 디렉토리 | 런타임 중 촬영된 장애물 이미지 저장 |
| `experiment/data/raw_experiment.csv` | CSV | 수동 조종 실험 데이터: 센서값, 상태, 어노테이션 |
| `experiment/data/thresholds.json` | JSON | `analyze.py` 실행으로 추출된 센서 임계값 |
| `models/xgboost_route.json` | Booster JSON | 경로 선택 XGBoost 모델 |
| `models/xgboost_speed.json` | Booster JSON | 속도 제어 XGBoost 모델 |
| `models/isolation_forest.pkl` | Joblib | 이상 감지 IsolationForest 모델 |

---

## 13. 데이터 초기화 방법

```bash
# 주행 블랙박스 로그 초기화
> data/real_agv_history.csv

# 장애물 판단 DB 초기화
echo '[]' > data/obstacle_db.json

# 장애물 이미지 전체 삭제
rm -f data/obstacles/*.jpg
```

---

## 14. Claude Code 사용법

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
claude --dangerously-skip-permissions
```

> **주의:** `app/cv_dashboard.py`, `app/dashboard_collect.py` 는 팀원 담당 파일입니다.
> Claude Code 사용 시에도 해당 파일 수정을 요청하지 마세요.

---

## 15. 코드 수정 및 GitHub 업로드

```bash
cd ~/insite

# 변경된 파일 스테이징
git add 파일명
# 또는 전체 추가
git add .

# 커밋 메시지 작성
git commit -m "수정 내용 간단히 설명"

# GitHub 업로드
git push origin master
```

---

## 16. 주의사항

> **팀원 파일 수정 금지**
> `app/cv_dashboard.py`, `app/dashboard_collect.py` 는 라인트래킹 파트 담당 파일입니다.
> AI 파트 작업 중 이 파일들을 수정하지 마세요.

- **포트 충돌:** AI 대시보드(5000)와 라인트래킹 대시보드(5001)는 동시 실행 가능하지만, **5001 포트를 사용하는 두 서버를 동시에 실행하면 충돌**합니다.
- **GPIO 충돌:** 두 프로세스가 같은 GPIO 핀을 동시에 제어하면 오류가 발생합니다. 실험 전 다른 대시보드가 실행 중인지 확인하세요.
- **라즈베리파이 안전 종료:** 실험 후 반드시 아래 명령어로 종료 후 전원을 뽑으세요.

```bash
sudo shutdown -h now
```

초록 LED가 꺼진 후 전원을 분리합니다.

---

## 17. 자주 발생하는 문제 해결

| 증상 | 원인 | 해결 방법 |
|------|------|-----------|
| `pi.local` 접속 안 됨 | mDNS 미지원 또는 네트워크 문제 | `ssh pi@192.168.0.50` 으로 IP 직접 접속 |
| `(.venv)` 안 보임 | 가상환경 미활성화 | `source ~/insite/.venv/bin/activate` 실행 |
| 대시보드 브라우저 접속 안 됨 | 서버 미실행 또는 다른 네트워크 | Pi에서 `python dashboard.py` 실행 확인, 같은 WiFi 연결 확인 |
| 모터가 움직이지 않을 때 | I2C 연결 불량 또는 전원 문제 | `i2cdetect -y 1` 로 `0x5f` 감지 여부 확인 |
| 센서값이 고정될 때 | 센서 초기화 실패 또는 배선 불량 | 해당 센서 단독 테스트 스크립트 실행 |
| Gemini API 오류 발생 시 | API 키 미설정 또는 할당량 초과 | `echo $GEMINI_API_KEY` 로 키 확인, Google AI Studio에서 할당량 확인 |
| 카메라 영상이 안 나올 때 | picamera2 미설치 또는 CSI 연결 불량 | `rpicam-hello` 로 카메라 동작 확인 |
| I2C 장치가 안 잡힐 때 | 배선 불량 또는 I2C 미활성화 | `sudo raspi-config` → Interface Options → I2C 활성화 확인 |
