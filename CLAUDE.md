# CLAUDE.md — Insite 자율주행 로봇 프로젝트

> 이 파일을 먼저 읽고 작업할 것. 전체 재작성 금지. Surgical Changes only.

---

## 1. 프로젝트 목표

라인 트래킹 기반 자율주행 로봇.

**발표에서 반드시 보여야 할 것 (우선순위 순):**
1. 라인을 따라 자율주행 (트래킹 센서 기반)
2. 검은 원 마커에서 웨이포인트 인식 및 정지
3. 장애물 감지 시 정지 (초음파 + Gemini Vision)
4. 경로 A/B/C 분기 선택 (교차점에서 어느 방향으로 갈지)

**포기한 것:**
- XGBoost 경로 선택 (학습 데이터 없음, 구현 복잡, 불필요)
- 좌표 기반 내비게이션 (MPU-6050으로 불가)
- 시간 기반 웨이포인트 재생 (라인 트래킹으로 대체)

---

## 2. 하드웨어

| 센서 | 핀/주소 | 라이브러리 |
|---|---|---|
| 라인트래킹 (3개) | GPIO17(우), GPIO27(중), GPIO22(좌) | gpiozero InputDevice |
| DC모터 PCA9685 | I2C1 0x5f | adafruit |
| MPU-6050 IMU | Software I2C bus5, GPIO12/13, 0x68 | smbus2 |
| HC-SR04 초음파 | TRIG=23, ECHO=24 | gpiozero DistanceSensor |
| OV5647 카메라 | CSI | picamera2 |
| HX711 로드셀 | DAT=5, CLK=6 | RPi.GPIO |
| ADS7830 배터리 | I2C1 0x48 | smbus |

**트래킹 센서 값:**
- 검은 라인 위 = 1, 흰 바닥 = 0
- 웨이포인트 마커(검은 원) = left=1, mid=1, right=1 (3개 동시)
- 직진 정상 = left=0, mid=1, right=0
- 좌 이탈 = left=1, mid=0, right=0 → 우로 조향
- 우 이탈 = left=0, mid=0, right=1 → 좌로 조향

---

## 3. 파일 구조

```
~/insite/
├── app/
│   ├── dashboard.py              # Flask 서버 (포트 5001) — 수정 최소화
│   └── templates/index.html     # 브라우저 UI
├── core/
│   ├── waypoint_navigator.py    # 자율주행 로직 — 여기만 수정
│   ├── line_tracker.py          # 라인 트래킹 드라이버 (새로 생성)
│   ├── waypoint_graph.json      # 웨이포인트/경로 데이터
│   ├── gemini_client.py         # Gemini Flash API (이미 있음)
│   └── loadcell_calibrate.py
├── sensors/
│   ├── camera.py    # capture() → BGR ndarray, is_opened()
│   ├── mpu6050.py   # get_all() → {roll,pitch,yaw,yaw_rate}
│   ├── hx711.py     # get_weight() → float(g)
│   ├── motor.py     # move(speed,dir,turn), rotate_left/right(), stop()
│   ├── led.py       # set_running/error/off()
│   └── ultra.py     # get_distance() → float(cm)
├── gyro_offset.txt
└── loadcell_cal.txt
```

---

## 4. 핵심 설계: 라인 트래킹 + 웨이포인트

### 트랙 구성
```
흰 바닥 위에 검은 테이프 라인
교차점(분기)에 검은 원(지름 15cm 이상) = 웨이포인트 마커
```

### 주행 모드
```
MODE 1: LINE_FOLLOW (라인 추종)
  - mid=1이면 직진
  - left=1, mid=0이면 우조향
  - right=1, mid=0이면 좌조향
  - all=1이면 → WAYPOINT_DETECTED

MODE 2: WAYPOINT_DETECTED (웨이포인트 감지)
  - 정지
  - 센서값 + IMU yaw 기록
  - 다음 경로 방향 결정
  - 300ms 후 재출발

MODE 3: OBSTACLE_STOP (장애물 정지)
  - 초음파 < 20cm && pitch < 5° → 정지
  - Gemini Vision으로 실제 장애물인지 확인 (선택)
  - 해제 후 재출발
```

### 경로 분기 설계
```
교차점(웨이포인트)에서:
  straight = 직진 계속
  left     = 좌회전 후 라인 재탐색
  right    = 우회전 후 라인 재탐색

waypoint_graph.json에 각 웨이포인트의 다음 방향 저장:
{
  "waypoints": {
    "1": {"direction": "straight", "comment": "출발"},
    "2": {"direction": "left",     "comment": "좌회전 분기"},
    "3": {"direction": "right",    "comment": "경사로 진입"}
  },
  "paths": {
    "A": [1, 2, 3],
    "B": [1, 3]
  }
}
```

---

## 5. 구현 순서 (Think Before Coding)

### Step 1: line_tracker.py 생성
```python
# ~/insite/core/line_tracker.py
from gpiozero import InputDevice

class LineTracker:
    PIN_LEFT   = 22  # GPIO22 = S3
    PIN_MIDDLE = 27  # GPIO27 = S2
    PIN_RIGHT  = 17  # GPIO17 = S1

    def read(self) -> dict:
        # 반환: {'left':int, 'mid':int, 'right':int, 'all_black':bool}

    def is_waypoint(self) -> bool:
        # 3개 동시 = 1 → 웨이포인트 마커
        # 단, 일시적 노이즈 제거: 연속 3회 확인

    def get_error(self) -> int:
        # PID 제어용 오차: -1(좌이탈), 0(중앙), +1(우이탈)
```

### Step 2: waypoint_navigator.py 수정 (Surgical)
```
기존 시간기반 _drive_to_wp → 라인 트래킹 루프로 교체
기존 _rotate_by_rel → 분기점에서 방향 전환으로 교체
나머지 (장애물 감지, 언덕 감지) → 유지
```

### Step 3: dashboard.py 수정 (최소)
```
LineTracker 센서값 dashboard에 표시 추가
웨이포인트 저장 방식: 대시보드 버튼 제거, 검은 원 자동 감지로 변경
```

---

## 6. 수정 금지 목록 (Simplicity First)

- `sensors/` 파일들 메서드 시그니처 변경 금지
- dashboard.py 전체 구조 변경 금지
- 작동하는 카메라/IMU/초음파 로직 건드리지 말 것
- 한 번에 하나의 버그/기능만 수정

---

## 7. 현재 상태

### 작동함
- 브라우저 대시보드, 카메라 스트리밍, 방향키 모터 제어
- IMU roll/pitch/yaw (교정 완료, ±0.06°/10s)
- 초음파, 배터리 표시
- 웨이포인트 저장/경로 저장 UI

### 작동 안 함 (이 세션에서 수정)
- **라인 트래킹 드라이버 없음** → `core/line_tracker.py` 생성 필요
- **라인 추종 주행 로직 없음** → `waypoint_navigator.py` 수정 필요
- **기존 웨이포인트 재생 안 됨** → 라인 트래킹으로 대체하므로 구버전 방식 제거

---

## 8. 실행 / 테스트

```bash
# 환경 활성화
source ~/insite/.venv/bin/activate

# 트래킹 센서 단독 테스트 (새로 만든 후)
python3 ~/insite/core/line_tracker.py

# 서버 실행
python3 ~/insite/app/dashboard.py
# 접속: http://192.168.0.50:5001

# 기존 웨이포인트 삭제 (새 방식으로 재수집)
rm ~/insite/core/waypoint_graph.json
```

---

## 9. Gemini 장애물 감지 연동 (나중에)

```python
# 초음파 20cm 이하 + pitch < 5° → 장애물 의심
# → 카메라 프레임 캡처 → Gemini 호출
# gemini-1.5-flash, GEMINI_API_KEY 환경변수 필요

result = gemini.analyze_obstacle(frame, {}, {'distance_cm': ultra})
if result['passable']:
    continue  # 경사로 등 통과 가능
else:
    stop()    # 실제 장애물
```