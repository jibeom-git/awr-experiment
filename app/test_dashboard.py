#!/usr/bin/env python3
# app/test_dashboard.py
# Insite AGV 실험용 테스트 대시보드 (포트 5004)
#
# 기능:
# 1. 신호등 생략 → 즉시 출발
# 2. 출발 전 조건 설정 (모드/무게/경로/경사속도)
# 3. 장애물 버튼 주입 (VLM 없이 직접 클릭)
# 4. 경사 자동 감지 → 속도 자동 조절
# 5. 모든 데이터 시계열 CSV 저장

import sys, os, time, threading, copy, csv, collections
from datetime import datetime
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

from sensors.motor import MotorController

app = Flask(__name__)

# ══════════════════════════════════════════════════════
# 하드웨어 초기화
# ══════════════════════════════════════════════════════

try:
    from sensors.camera import Camera
    cam = Camera(width=320, height=240)
    CAM_OK = True
    print("[OK] Pi 카메라")
except Exception as e:
    cam = None; CAM_OK = False
    print(f"[SKIP] Pi 카메라: {e}")

latest_webcam = None
latest_picam  = None

try:
    from sensors.mpu6050 import MPU6050
    imu = MPU6050(bus_id=5, address=0x68)
    time.sleep(0.5)
    _baseline = imu.get_accel()
    IMU_OK = True
    print(f"[OK] IMU baseline x={_baseline['x']:.3f}")
except Exception as e:
    imu = None; _baseline = {'x': 0.0}; IMU_OK = False
    print(f"[SKIP] IMU: {e}")

try:
    from gpiozero import DistanceSensor
    import signal as _sig
    def _to(s,f): raise TimeoutError()
    _sig.signal(_sig.SIGALRM, _to); _sig.alarm(3)
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
    _sig.alarm(0); ULTRA_OK = True
    print("[OK] 초음파")
except Exception as e:
    ultra = None; ULTRA_OK = False
    print(f"[SKIP] 초음파: {e}")

try:
    from sensors.hx711 import HX711
    hx711 = HX711(dout=5, pd_sck=6)
    time.sleep(1); hx711.tare(samples=10)
    HX711_OK = True
    print("[OK] 로드셀")
except Exception as e:
    hx711 = None; HX711_OK = False
    print(f"[SKIP] 로드셀: {e}")

motor = MotorController()

# ══════════════════════════════════════════════════════
# 라우트 정의 (auto_dashboard_v3 동일)
# ══════════════════════════════════════════════════════
ROUTES = {
    "A":    {1: "straight", 2: "stop"},
    "B":    {1: "left",  2: "right", 3: "right", 4: "stop"},
    "C":    {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight", 6: "stop"},
    "A->B": {1: "left",  2: "right", 3: "right", 4: "stop"},
    "A->C": {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight", 6: "stop"},
    "B->C": {1: "left",  2: "right", 3: "right", 4: "straight", 5: "stop"},
}

# 우회 순서: A→B→C
DETOUR_NEXT = {"A": "B", "B": "C"}

# ══════════════════════════════════════════════════════
# 경사 감지 임계값
# ══════════════════════════════════════════════════════
TH_UPHILL   =  7.0   # pitch > +7도 → 오르막
TH_DOWNHILL = -7.0   # pitch < -7도 → 내리막
TH_FLAT     =  2.0   # |pitch| < 2도 → 평지

# ══════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════
lock = threading.Lock()

config = {
    "base_speed":     50,
    "turn_speed":     30,
    "spin_speed":     25,
    "thresh":         43,
    "roi_top":        0.7,
    "dead_zone":      50,
    "kp":             0.8,
    "ki":             0.002,
    "forward_time":   0.5,
    "slope_up_speed": 70,    # 오르막 속도
    "slope_down_speed": 30,  # 내리막 속도
    "green_min_area": 120,
    "running":        False,
    "route":          "B",
    "user_mode":      "safe",
    "sim_weight":     "none",   # none / heavy(140g)
    "obstacle_pending": False,  # 장애물 판단 대기 중
}

state = {
    "action":          "정지",
    "error":           0,
    "green_area":      0,
    "junction_count":  0,
    "route":           "B",
    "distance":        -1,
    "weight_g":        0.0,
    "pitch":           0.0,
    "fps":             0,
    "lost":            False,
    "terrain":         "flat",      # flat / uphill / downhill
    "cruise_speed":    50,
    "ai_status":       "대기중",
    "user_mode":       "safe",
    "obstacle_pending": False,      # 장애물 판단 팝업 표시 여부
}

# 판단 로그 (최대 100건)
_log = collections.deque(maxlen=100)

def add_log(msg: str, level: str = "info"):
    _log.appendleft({
        "time":  datetime.now().strftime("%H:%M:%S"),
        "msg":   msg,
        "level": level
    })
    print(f"[{level.upper()}] {msg}")

# ══════════════════════════════════════════════════════
# 실험 데이터 저장
# ══════════════════════════════════════════════════════
_session_id    = None
_session_rows  = []
_session_lock  = threading.Lock()

SAVE_DIR = "/home/pi/insite/experiment/data/test_sessions"
os.makedirs(SAVE_DIR, exist_ok=True)

FIELDNAMES = [
    "timestamp", "session_id", "mode", "weight_g", "route",
    "junction_count", "action", "obstacle", "ai_decision",
    "speed", "error_px", "green_area", "pitch", "distance_cm",
    "terrain", "result"
]

def start_session():
    global _session_id, _session_rows
    _session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    with _session_lock:
        _session_rows = []
    add_log(f"세션 시작: {_session_id}", "ok")

def record_row(extra: dict = {}):
    """현재 상태를 한 행으로 기록 (10Hz)"""
    if _session_id is None:
        return
    with lock:
        s = dict(state)
    row = {
        "timestamp":     datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "session_id":    _session_id,
        "mode":          s.get("user_mode", "safe"),
        "weight_g":      s.get("weight_g", 0.0),
        "route":         s.get("route", "--"),
        "junction_count": s.get("junction_count", 0),
        "action":        s.get("action", "--"),
        "obstacle":      extra.get("obstacle", "none"),
        "ai_decision":   extra.get("ai_decision", "--"),
        "speed":         s.get("cruise_speed", 0),
        "error_px":      s.get("error", 0),
        "green_area":    s.get("green_area", 0),
        "pitch":         s.get("pitch", 0.0),
        "distance_cm":   s.get("distance", -1),
        "terrain":       s.get("terrain", "flat"),
        "result":        extra.get("result", "recording"),
    }
    with _session_lock:
        _session_rows.append(row)

def save_session(result_label: str = "완료"):
    """세션 CSV 저장"""
    if not _session_rows:
        add_log("저장할 데이터 없음", "warn")
        return
    # result 소급 적용
    for r in _session_rows:
        if r["result"] == "recording":
            r["result"] = result_label
    path = os.path.join(SAVE_DIR, f"session_{_session_id}.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(_session_rows)
    add_log(f"저장 완료: {len(_session_rows)}행 → {path}", "ok")

# ══════════════════════════════════════════════════════
# 분기점 카운트
# ══════════════════════════════════════════════════════
g_jc   = 0
g_lock = threading.Lock()

def get_jc():
    with g_lock: return g_jc

def inc_jc():
    global g_jc
    with g_lock:
        g_jc += 1
        return g_jc

def reset_jc():
    global g_jc
    with g_lock: g_jc = 0

# ══════════════════════════════════════════════════════
# 모터 헬퍼
# ══════════════════════════════════════════════════════
def go_forward(speed):
    motor.set_motor(1,1,speed); motor.set_motor(2,1,speed)
    motor.set_motor(3,1,speed); motor.set_motor(4,1,speed)

def go_backward(speed):
    motor.set_motor(1,-1,speed); motor.set_motor(2,-1,speed)
    motor.set_motor(3,-1,speed); motor.set_motor(4,-1,speed)

def turn_left(ls, rs):
    motor.set_motor(4,1,ls); motor.set_motor(3,1,ls)
    motor.set_motor(2,1,rs); motor.set_motor(1,1,rs)

def turn_right(ls, rs):
    motor.set_motor(4,1,rs); motor.set_motor(3,1,rs)
    motor.set_motor(2,1,ls); motor.set_motor(1,1,ls)

def spin_left(speed):
    motor.set_motor(4,-1,speed); motor.set_motor(3,-1,speed)
    motor.set_motor(2,1,speed);  motor.set_motor(1,1,speed)

def spin_right(speed):
    motor.set_motor(4,1,speed);  motor.set_motor(3,1,speed)
    motor.set_motor(2,-1,speed); motor.set_motor(1,-1,speed)

# ══════════════════════════════════════════════════════
# 센서 헬퍼
# ══════════════════════════════════════════════════════
def get_weight_g() -> float:
    sim = config.get("sim_weight", "none")
    if sim == "heavy":
        return 140.0
    if hx711 is None:
        return 0.0
    try:
        return max(0.0, float(hx711.get_weight(times=3)))
    except:
        return 0.0

def get_pitch() -> float:
    if imu is None:
        return 0.0
    try:
        data = imu.get_all()
        return round(float(data.get("pitch", 0.0)), 1)
    except:
        return 0.0

# ══════════════════════════════════════════════════════
# 라인 + 분기점 감지
# ══════════════════════════════════════════════════════
def detect_line_and_green(frame, cfg):
    if frame is None or len(frame.shape) < 3:
        return None, 0, False, frame

    h, w  = frame.shape[:2]
    roi_y = int(h * cfg["roi_top"])
    roi   = frame[roi_y:h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, cfg["thresh"], 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    M  = cv2.moments(binary)
    cx = None
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"]) + roi_y
    else:
        cy = roi_y

    roi_hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower      = np.array([30, 80, 50])
    upper      = np.array([85, 255, 255])
    green_mask = cv2.inRange(roi_hsv, lower, upper)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_area = int(np.sum(green_mask > 0))
    is_junction = cfg["green_min_area"] < green_area < 5000

    # 디버그 오버레이
    debug = frame.copy()
    cv2.line(debug, (0, roi_y), (w, roi_y), (255,180,0), 1)
    dz = int(cfg["dead_zone"] * 2)
    # 빨간 영역 표시
    cv2.rectangle(debug,
                  (w//2 - dz, roi_y),
                  (w//2 + dz, h),
                  (0, 0, 180), 1)
    if cx is not None:
        cv2.circle(debug, (cx, cy), 7, (0,255,80), -1)
        err = cx - w//2
        cv2.putText(debug, f"err={err:+d}", (5, roi_y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,80), 1)
    if is_junction:
        cv2.putText(debug, f"NODE {green_area}px", (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    return cx, green_area, is_junction, debug

# ══════════════════════════════════════════════════════
# 분기점 행동 실행
# ══════════════════════════════════════════════════════
def execute_junction(action_at: str, cfg: dict):
    go_forward(45)
    time.sleep(cfg["forward_time"])
    motor.motorStop()
    time.sleep(0.1)

    if action_at == "stop":
        go_forward(45)
        time.sleep(0.2)
        motor.motorStop()
        config["running"] = False
        add_log("도착 — 주행 완료", "ok")
        save_session("완료")
        return

    if action_at == "straight":
        go_forward(45)
        time.sleep(0.4)
        return

    if action_at in ("left", "right"):
        spin_fn = spin_left if action_at == "left" else spin_right
        spin_fn(35)
        time.sleep(0.55)
        # 라인 재탐색 (dead_zone 기준)
        fine_end = time.time() + 3.0
        while time.time() < fine_end:
            if cam is None: break
            frm = cam.capture()
            frm = cv2.cvtColor(frm, cv2.COLOR_RGB2BGR)
            cx_s, _, _, _ = detect_line_and_green(frm, cfg)
            dz = int(cfg["dead_zone"] * 2 * 0.8)
            if cx_s is not None and abs(cx_s - 160) < dz:
                motor.motorStop()
                break
            spin_fn(34)
            time.sleep(0.02)
        motor.motorStop()

# ══════════════════════════════════════════════════════
# 우회 실행
# ══════════════════════════════════════════════════════
def do_reroute(new_route: str):
    add_log(f"우회 시작 → {new_route}", "warn")
    motor.motorStop()
    time.sleep(0.3)

    # 초록 노드 찾을 때까지 후진
    timeout = time.time() + 10.0
    while time.time() < timeout:
        if cam is None: break
        frm = cam.capture()
        frm = cv2.cvtColor(frm, cv2.COLOR_RGB2BGR)
        h, w   = frm.shape[:2]
        roi_y  = int(config["roi_top"] * h)
        roi    = frm[roi_y:h, :]
        hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask   = cv2.inRange(hsv, np.array([30,80,50]), np.array([85,255,255]))
        if int(np.sum(mask > 0)) > config["green_min_area"]:
            motor.motorStop()
            break
        go_backward(config["base_speed"])
        time.sleep(0.03)

    motor.motorStop()
    time.sleep(0.5)
    reset_jc()
    config["route"]   = new_route
    config["running"] = True
    with lock:
        state["junction_count"] = 0
        state["route"]          = new_route
        state["action"]         = f"우회→{new_route}"
    add_log(f"우회 완료 — {new_route} 재출발", "ok")

# ══════════════════════════════════════════════════════
# 장애물 규칙표 적용 (VLM 없이 버튼 입력)
# ══════════════════════════════════════════════════════
def apply_obstacle_rule(bump: str):
    """
    bump: none / bump_1cm / bump_2cm
    대시보드 버튼 클릭으로 호출됨 (VLM 대체)
    """
    mode   = config.get("user_mode", "safe").lower()
    weight = get_weight_g()
    heavy  = weight >= 140.0
    route  = config["route"].split("->")[-1] if "->" in config["route"] else config["route"]

    action = "pass"
    speed  = config["base_speed"]

    if mode == "fast":
        if not heavy:
            if route == "A":
                if bump == "none":      action, speed = "pass", 70
                else:                   action = "detour"
            elif route == "B":
                if bump == "none":      action = "pass"
                elif bump == "bump_1cm": action, speed = "pass", 70
                elif bump == "bump_2cm": action = "detour"
            elif route == "C":
                if bump == "none":      action = "pass"
                elif bump == "bump_1cm": action, speed = "pass", 60
                elif bump == "bump_2cm": action = "stop"
        else:
            if route == "A":            action = "detour"
            elif route == "B":
                if bump == "none":      action, speed = "pass", 80
                else:                   action = "detour"
            elif route == "C":
                if bump == "none":      action = "pass"
                elif bump == "bump_1cm": action, speed = "pass", 70
                elif bump == "bump_2cm": action = "stop"
    else:  # safe
        if not heavy:
            if route == "B":
                if bump == "none":      action = "pass"
                elif bump == "bump_1cm": action, speed = "pass", 60
                elif bump == "bump_2cm": action = "detour"
            elif route == "C":
                if bump == "none":      action = "pass"
                elif bump == "bump_1cm": action, speed = "pass", 60
                elif bump == "bump_2cm": action = "stop"
        else:
            if route == "B":
                if bump == "none":      action, speed = "pass", 60
                else:                   action = "detour"
            elif route == "C":
                if bump == "none":      action = "pass"
                elif bump == "bump_1cm": action, speed = "pass", 70
                elif bump == "bump_2cm": action = "stop"

    add_log(f"장애물 판단: {bump} → {action} ({speed}%)", "ok" if action=="pass" else "warn")
    record_row({"obstacle": bump, "ai_decision": action})

    with lock:
        state["obstacle_pending"] = False
        state["ai_status"]        = f"{action.upper()} ({bump})"
    config["obstacle_pending"] = False

    if action == "detour":
        next_r = DETOUR_NEXT.get(route)
        if next_r:
            new_route_str = f"{route}->{next_r}"
            threading.Thread(
                target=do_reroute, args=(new_route_str,),
                daemon=True, name="reroute"
            ).start()
        else:
            add_log("우회 불가 — 최종 정지", "err")
            motor.motorStop()
            config["running"] = False
            save_session("경로차단")

    elif action == "stop":
        motor.motorStop()
        config["running"] = False
        add_log("최종 정지 — 운영자 개입 필요", "err")
        save_session("장애물정지")

    else:  # pass
        # 일시적 속도 변경 후 복귀
        original = config["base_speed"]
        config["base_speed"] = speed
        config["running"]    = True
        def _restore(orig=original):
            time.sleep(3.0)
            config["base_speed"] = orig
        threading.Thread(target=_restore, daemon=True).start()

# ══════════════════════════════════════════════════════
# 경사 감지 FSM
# ══════════════════════════════════════════════════════
# 상태: flat → uphill → flat_top → downhill → flat
_terrain_state   = "flat"
_flat_start_time = 0.0
_FLAT_HOLD_SEC   = 2.0   # 평지가 2초 이상 유지돼야 정상 도착으로 판단

def update_terrain(pitch: float) -> str:
    """
    pitch 값으로 지형 상태 업데이트.
    반환: flat / uphill / downhill
    """
    global _terrain_state, _flat_start_time

    if pitch > TH_UPHILL:
        if _terrain_state != "uphill":
            add_log(f"오르막 진입 (pitch={pitch}°) → 속도 {config['slope_up_speed']}%", "info")
        _terrain_state   = "uphill"
        _flat_start_time = 0.0

    elif pitch < TH_DOWNHILL:
        if _terrain_state != "downhill":
            add_log(f"내리막 진입 (pitch={pitch}°) → 속도 {config['slope_down_speed']}%", "info")
        _terrain_state   = "downhill"
        _flat_start_time = 0.0

    elif abs(pitch) < TH_FLAT:
        if _terrain_state in ("uphill", "downhill"):
            if _flat_start_time == 0.0:
                _flat_start_time = time.time()
            elif time.time() - _flat_start_time > _FLAT_HOLD_SEC:
                add_log(f"평지 복귀 (pitch={pitch}°) → 기본 속도 복귀", "info")
                _terrain_state   = "flat"
                _flat_start_time = 0.0
        else:
            _terrain_state = "flat"

    return _terrain_state

def get_cruise_speed(terrain: str, jc: int) -> int:
    """지형 + 분기점 기반 속도 결정"""
    # 노드 근처에서는 무조건 기본 속도 (노드 인식 정확도 유지)
    if jc >= 1:
        return config["base_speed"]
    if terrain == "uphill":
        return config["slope_up_speed"]
    if terrain == "downhill":
        return config["slope_down_speed"]
    return config["base_speed"]

# ══════════════════════════════════════════════════════
# 백그라운드 센서 스레드
# ══════════════════════════════════════════════════════
def ultra_loop():
    while True:
        if ultra is not None:
            try:
                d = ultra.distance
                with lock:
                    state["distance"] = round(d*100, 1) if d else -1
            except: pass
        time.sleep(0.1)

def weight_loop():
    while True:
        w = get_weight_g()
        with lock: state["weight_g"] = round(w, 1)
        time.sleep(0.5)

def webcam_loop():
    global latest_webcam
    idx = None
    for i in [0,1,2,3]:
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret: idx = i; cap.release(); break
            cap.release()
    if idx is None:
        print("[SKIP] 웹캠 탐색 실패"); return
    _w = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    _w.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    _w.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print(f"[OK] 웹캠 index={idx}")
    while True:
        if _w.isOpened():
            ret, frm = _w.read()
            if ret:
                _, jpg = cv2.imencode('.jpg', frm, [cv2.IMWRITE_JPEG_QUALITY, 70])
                with lock: latest_webcam = jpg.tobytes()
        time.sleep(0.033)

threading.Thread(target=ultra_loop,  daemon=True, name="ultra").start()
threading.Thread(target=weight_loop, daemon=True, name="weight").start()
threading.Thread(target=webcam_loop, daemon=True, name="webcam").start()

# ══════════════════════════════════════════════════════
# 주행 루프
# ══════════════════════════════════════════════════════
def drive_loop():
    global latest_picam

    CENTER_X       = 160
    lost_dir       = 1
    fps_t          = time.time()
    fps_count      = 0
    integral       = 0.0
    prev_time      = time.time()
    green_seen     = False
    jc_cooldown    = 0.0
    stopped        = False
    record_timer   = 0.0

    while True:
        cfg = copy.deepcopy(config)

        # Pi 카메라
        frame = None
        if cam is not None:
            try:
                frame = cam.capture()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except: frame = None

        if frame is not None:
            cx, green_area, is_junc, debug = detect_line_and_green(frame, cfg)
            _, jpg = cv2.imencode('.jpg', debug, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with lock: latest_picam = jpg.tobytes()
        else:
            cx, green_area, is_junc = None, 0, False

        current_route = cfg["route"]

        fps_count += 1
        if time.time() - fps_t >= 1.0:
            with lock: state["fps"] = fps_count
            fps_count = 0; fps_t = time.time()

        # 장애물 판단 대기 중 → 정지 유지
        if cfg.get("obstacle_pending"):
            motor.motorStop()
            with lock:
                state["action"] = "장애물판단대기"
                state["obstacle_pending"] = True
            time.sleep(0.05)
            continue

        # 정지 상태
        if not cfg["running"]:
            if not stopped:
                motor.motorStop(); stopped = True
            integral = 0.0
            with lock:
                state["action"]         = "정지"
                state["green_area"]     = green_area
                state["junction_count"] = get_jc()
            time.sleep(0.05)
            continue
        else:
            stopped = False

        now = time.time()
        dt  = now - prev_time; prev_time = now
        if dt > 0.1: dt = 0.033

        # 경사 감지 → 속도 결정
        pitch   = get_pitch()
        terrain = update_terrain(pitch)
        jc      = get_jc()
        cruise  = get_cruise_speed(terrain, jc)

        with lock:
            state["pitch"]        = pitch
            state["terrain"]      = terrain
            state["cruise_speed"] = cruise

        # 10Hz 데이터 기록
        if now - record_timer >= 0.1:
            record_timer = now
            record_row()

        # 분기점 처리
        if is_junc and not green_seen and now > jc_cooldown:
            green_seen  = True
            jc_cooldown = now + 2.5
            jc = inc_jc()
            action_at = ROUTES[current_route].get(jc, "straight")
            add_log(f"노드 #{jc} — {current_route} — {action_at}", "info")
            with lock:
                state["junction_count"] = jc
                state["action"]         = f"노드#{jc}_{action_at}"
            execute_junction(action_at, cfg)
            continue

        if not is_junc:
            green_seen = False

        # 초음파 20cm 이하 → 장애물 판단 대기 (버튼 입력 대기)
        with lock: dist_now = state["distance"]
        if cfg["running"] and 0 < dist_now < 20 and not cfg.get("obstacle_pending"):
            config["obstacle_pending"] = True
            config["running"]          = False
            motor.motorStop()
            with lock:
                state["obstacle_pending"] = True
                state["ai_status"]        = "장애물 감지 — 종류 선택하세요"
            add_log(f"장애물 감지 (거리={dist_now}cm) — 판단 대기", "warn")
            time.sleep(0.03)
            continue

        # 라인 추종 PI 제어
        if cx is None:
            integral = 0.0
            spin_right(35) if lost_dir > 0 else spin_left(35)
            with lock: state["action"] = "라인유실_탐색"
        else:
            error    = cx - CENTER_X
            _dz      = int(cfg["dead_zone"] * 2)
            # 빨간 영역 80% 이내면 직진 유지
            if abs(error) < _dz * 0.8:
                error = 0
            integral   = max(min(integral + error * dt, 150), -150)
            spd_scaler = max(0.4, 1.0 - (cruise - 45) / 100.0)
            correction = int(error * cfg["kp"] * spd_scaler + integral * cfg["ki"])
            max_corr   = max(15, cruise - 20)
            correction = max(min(correction, max_corr), -max_corr)
            lost_dir   = 1 if error > 0 else (-1 if error < 0 else lost_dir)

            if abs(error) < _dz:
                go_forward(cruise)
                action_str = f"직진({terrain})"
            elif error > 0:
                rws = max(cruise - abs(correction), 20)
                turn_right(cruise, rws)
                action_str = f"우조정({abs(error)}px)"
            else:
                lws = max(cruise - abs(correction), 20)
                turn_left(lws, cruise)
                action_str = f"좌조정({abs(error)}px)"

            with lock:
                state["error"]      = error
                state["action"]     = action_str
                state["lost"]       = False
                state["route"]      = current_route
                state["green_area"] = green_area

        time.sleep(0.03)

threading.Thread(target=drive_loop, daemon=True, name="drive").start()

# ══════════════════════════════════════════════════════
# Flask 라우트
# ══════════════════════════════════════════════════════
def gen_stream(fn):
    while True:
        with lock: frm = fn()
        if frm:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frm + b'\r\n'
        time.sleep(0.033)

@app.route('/picam_feed')
def picam_feed():
    return Response(gen_stream(lambda: latest_picam),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/webcam_feed')
def webcam_feed():
    return Response(gen_stream(lambda: latest_webcam),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/state')
def get_state():
    with lock: return jsonify(state)

@app.route('/logs')
def get_logs():
    return jsonify(list(_log))

@app.route('/sessions')
def get_sessions():
    files = sorted(os.listdir(SAVE_DIR), reverse=True)[:10]
    return jsonify(files)

@app.route('/start', methods=['POST'])
def start():
    """즉시 출발 (신호등 생략)"""
    reset_jc()
    mode   = config.get("user_mode", "safe")
    weight = get_weight_g()
    heavy  = weight >= 140.0
    route  = config.get("route", "B")  # 설정된 경로 사용

    config["running"]          = True
    config["obstacle_pending"] = False

    with lock:
        state["route"]           = route
        state["junction_count"]  = 0
        state["ai_status"]       = "주행 중"
        state["action"]          = "출발"
        state["obstacle_pending"] = False

    start_session()
    add_log(f"즉시 출발 — 모드:{mode.upper()} 무게:{weight:.0f}g 경로:{route}", "ok")
    return jsonify({"status": "running", "route": route})

@app.route('/stop', methods=['POST'])
def stop():
    config["running"]          = False
    config["obstacle_pending"] = False
    motor.motorStop()
    with lock:
        state["action"]          = "정지"
        state["obstacle_pending"] = False
    add_log("정지 명령", "warn")
    save_session("중단")
    return jsonify({"status": "stopped"})

@app.route('/set_condition', methods=['POST'])
def set_condition():
    """출발 전 조건 설정"""
    data = request.get_json(force=True)
    if "mode" in data:
        m = data["mode"].lower()
        config["user_mode"] = m
        with lock: state["user_mode"] = m
        # 모드에 따라 기본 경로 설정
        config["route"] = "A" if m == "fast" else "B"
        with lock: state["route"] = config["route"]
    if "weight" in data:
        config["sim_weight"] = data["weight"]  # none / heavy
    if "route" in data:
        config["route"] = data["route"]
        with lock: state["route"] = data["route"]
    if "slope_up_speed" in data:
        config["slope_up_speed"] = int(data["slope_up_speed"])
    if "slope_down_speed" in data:
        config["slope_down_speed"] = int(data["slope_down_speed"])
    add_log(
        f"조건 설정 — 모드:{config['user_mode'].upper()} "
        f"무게:{config['sim_weight']} 경로:{config['route']} "
        f"오르막:{config['slope_up_speed']}% 내리막:{config['slope_down_speed']}%",
        "info"
    )
    return jsonify({"status": "ok", "config": {
        "mode": config["user_mode"],
        "weight": config["sim_weight"],
        "route": config["route"],
        "slope_up": config["slope_up_speed"],
        "slope_down": config["slope_down_speed"],
    }})

@app.route('/obstacle', methods=['POST'])
def obstacle():
    """장애물 버튼 클릭 → 규칙표 적용"""
    data = request.get_json(force=True)
    bump = data.get("type", "none")  # none / bump_1cm / bump_2cm
    threading.Thread(
        target=apply_obstacle_rule, args=(bump,),
        daemon=True, name="obstacle"
    ).start()
    return jsonify({"status": "ok", "bump": bump})

@app.route('/tare', methods=['POST'])
def tare():
    if hx711 is not None:
        try:
            hx711.tare(samples=30)
            add_log("로드셀 TARE 완료", "ok")
            return jsonify({"status": "ok"})
        except Exception as e:
            return jsonify({"status": "error", "msg": str(e)})
    return jsonify({"status": "skip"})

@app.route('/save_session', methods=['POST'])
def api_save_session():
    data   = request.get_json(force=True)
    result = data.get("result", "수동저장")
    save_session(result)
    return jsonify({"status": "ok"})

@app.route('/config', methods=['GET', 'POST'])
def handle_config():
    if request.method == 'POST':
        data = request.get_json(force=True)
        for k, v in data.items():
            if k in config: config[k] = v
        return jsonify({"status": "ok"})
    return jsonify(config)

@app.route('/')
def index():
    return render_template_string(HTML)

# ══════════════════════════════════════════════════════
# HTML
# ══════════════════════════════════════════════════════
HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>INSITE AGV · TEST LAB</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;600;700&family=Barlow+Condensed:wght@400;600;700&display=swap');
:root {
  --bg0:#07090d;--bg1:#0c0f16;--bg2:#111520;--bg3:#161c2a;--bg4:#1c2438;
  --border:#1e2840;--border2:#27334e;
  --cyan:#00d4ff;--cyan-d:rgba(0,212,255,.12);--cyan-g:rgba(0,212,255,.3);
  --green:#00e676;--yellow:#ffcc02;--orange:#ff9100;--red:#ff3d3d;
  --text0:#e4ecf8;--text1:#8090b0;--text2:#384a6a;
  --mono:'JetBrains Mono',monospace;--label:'Barlow Condensed',sans-serif;
}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;background:var(--bg0);color:var(--text0);
  font-family:var(--mono);font-size:12px;overflow:hidden}

header{height:42px;background:var(--bg1);border-bottom:1px solid var(--border);
  display:flex;align-items:center;justify-content:space-between;padding:0 14px;
  flex-shrink:0;position:relative}
header::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:linear-gradient(90deg,transparent,var(--cyan),transparent)}
.logo{font-family:var(--label);font-size:16px;font-weight:700;
  letter-spacing:.2em;color:var(--cyan)}
.logo em{font-style:normal;font-size:10px;font-weight:400;
  color:var(--yellow);letter-spacing:.1em;margin-left:8px}
.hdr-r{display:flex;align-items:center;gap:12px}
.dot{width:6px;height:6px;border-radius:50%;background:var(--red)}
.dot.live{background:var(--green);box-shadow:0 0 6px var(--green)}
.clock{font-size:10px;color:var(--text2)}

.layout{display:grid;grid-template-columns:1fr 1fr 290px;
  height:calc(100vh - 42px);overflow:hidden}
.col{display:flex;flex-direction:column;border-right:1px solid var(--border);overflow:hidden}
.col:last-child{border-right:none}

.sec{font-family:var(--label);font-size:8px;font-weight:600;
  letter-spacing:.2em;color:var(--text2);text-transform:uppercase;
  padding:5px 10px 4px;border-bottom:1px solid var(--border);flex-shrink:0}

.cam-box{flex:1;background:#020508;position:relative;overflow:hidden;min-height:0}
.cam-box img{width:100%;height:100%;object-fit:contain;display:block}
.cam-lbl{position:absolute;top:7px;left:8px;font-size:8px;
  color:var(--yellow);letter-spacing:.1em;font-family:var(--label);opacity:.7}

/* 상태 바 */
.sbar{display:grid;grid-template-columns:repeat(4,1fr);
  border-bottom:1px solid var(--border);flex-shrink:0}
.sv{padding:7px 9px;border-right:1px solid var(--border)}
.sv:last-child{border-right:none}
.sv .k{font-family:var(--label);font-size:7px;color:var(--text2);
  letter-spacing:.12em;text-transform:uppercase;margin-bottom:3px}
.sv .v{font-family:var(--label);font-size:16px;font-weight:700;color:var(--text0);line-height:1}
.sv .u{font-size:9px;color:var(--text1);margin-left:2px}
.vc{color:var(--cyan)}.vg{color:var(--green)}.vy{color:var(--yellow)}.vr{color:var(--red)}

/* 컨트롤 */
.ctrl{background:var(--bg1);padding:8px 10px;flex-shrink:0;
  border-top:1px solid var(--border)}
.crow{display:flex;align-items:center;gap:6px;flex-wrap:wrap;margin-bottom:5px}
.crow:last-child{margin-bottom:0}
.lbl{font-family:var(--label);font-size:8px;color:var(--text2);
  letter-spacing:.12em;white-space:nowrap}

.btn{font-family:var(--label);font-size:11px;font-weight:600;
  letter-spacing:.06em;padding:5px 10px;border-radius:2px;cursor:pointer;
  border:1px solid var(--border2);background:var(--bg3);color:var(--text0);
  transition:all .15s;white-space:nowrap}
.btn:hover{border-color:var(--cyan);color:var(--cyan)}
.btn.g{background:rgba(0,230,118,.08);border-color:var(--green);color:var(--green)}
.btn.r{border-color:rgba(255,61,61,.4);color:var(--red)}
.btn.r:hover{background:rgba(255,61,61,.1)}
.btn.y{border-color:rgba(255,204,2,.4);color:var(--yellow)}
.btn.o{border-color:rgba(255,145,0,.4);color:var(--orange)}
.btn.c{background:var(--cyan-d);border-color:var(--cyan);color:var(--cyan)}
.btn.sel{box-shadow:0 0 8px var(--cyan-g)}

/* 오른쪽 패널 */
.rpanel{display:flex;flex-direction:column;overflow:hidden}

/* 경로 + 상태 */
.route-box{padding:10px 12px;border-bottom:1px solid var(--border);flex-shrink:0;
  text-align:center}
.route-big{font-family:var(--label);font-size:40px;font-weight:700;
  color:var(--cyan);line-height:1;transition:all .3s}
.route-big.changed{color:var(--orange);text-shadow:0 0 14px rgba(255,145,0,.5)}
.route-sub{font-size:9px;color:var(--text2);letter-spacing:.12em;
  font-family:var(--label);margin-top:3px}

/* 조건 설정 */
.cond-box{padding:8px 10px;border-bottom:1px solid var(--border);flex-shrink:0}
.cond-row{display:flex;gap:4px;margin-bottom:5px;align-items:center}
.cond-row:last-child{margin-bottom:0}

/* 장애물 팝업 */
.obs-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.8);
  z-index:100;align-items:center;justify-content:center}
.obs-overlay.show{display:flex}
.obs-modal{background:var(--bg2);border:1px solid var(--border2);
  border-radius:4px;padding:20px;min-width:260px;text-align:center}
.obs-title{font-family:var(--label);font-size:14px;font-weight:700;
  color:var(--yellow);letter-spacing:.1em;margin-bottom:6px}
.obs-sub{font-size:10px;color:var(--text1);margin-bottom:14px}
.obs-btns{display:flex;gap:8px;justify-content:center}

/* 판단 로그 */
.log-box{flex:1;overflow-y:auto;overflow-x:hidden}
.log-box::-webkit-scrollbar{width:3px}
.log-box::-webkit-scrollbar-thumb{background:var(--border2)}
.le{display:flex;gap:7px;padding:4px 10px;
  border-bottom:1px solid var(--border);align-items:flex-start}
.le:hover{background:var(--bg2)}
.lt{font-size:8px;color:var(--text2);white-space:nowrap;
  font-family:var(--label);padding-top:1px;min-width:45px}
.lm{font-size:10px;color:var(--text1);line-height:1.4;word-break:break-all}
.le.ok .lm{color:var(--green)}.le.warn .lm{color:var(--yellow)}
.le.err .lm{color:var(--red)}.le.info .lm{color:var(--text0)}

/* 지형 배지 */
.terrain-badge{padding:3px 8px;border-radius:2px;font-family:var(--label);
  font-size:10px;font-weight:700;letter-spacing:.08em}
.terrain-badge.flat{border:1px solid var(--border2);color:var(--text1)}
.terrain-badge.uphill{border:1px solid var(--orange);color:var(--orange);
  background:rgba(255,145,0,.08)}
.terrain-badge.downhill{border:1px solid var(--cyan);color:var(--cyan);
  background:var(--cyan-d)}

/* TOAST */
#toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);
  padding:6px 16px;border-radius:2px;font-size:11px;letter-spacing:.06em;
  border:1px solid var(--border2);background:var(--bg3);color:var(--text0);
  z-index:9999;pointer-events:none;opacity:0;transition:opacity .2s;
  font-family:var(--label);font-weight:600;white-space:nowrap}
#toast.show{opacity:1}
#toast.ok{border-color:var(--green);color:var(--green)}
#toast.err{border-color:var(--red);color:var(--red)}
#toast.warn{border-color:var(--orange);color:var(--orange)}
#toast.info{border-color:var(--cyan);color:var(--cyan)}

@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
</style>
</head>
<body>

<header>
  <div class="logo">INSITE TEST LAB<em>실험용 대시보드 · 포트 5004</em></div>
  <div class="hdr-r">
    <div style="display:flex;align-items:center;gap:5px;font-size:9px;color:var(--text2);font-family:var(--label)">
      <div class="dot" id="conn-dot"></div>
      <span id="conn-lbl">OFFLINE</span>
    </div>
    <div class="clock" id="clock"></div>
  </div>
</header>

<div class="layout">

  <!-- LEFT: Pi 카메라 -->
  <div class="col">
    <div class="sec">Pi Camera · Line Tracking</div>
    <div class="cam-box">
      <img src="/picam_feed" alt="">
      <div class="cam-lbl">LINE FOLLOW</div>
    </div>
    <div class="sbar">
      <div class="sv"><div class="k">ERROR</div>
        <div class="v" id="s-err">--<span class="u">px</span></div></div>
      <div class="sv"><div class="k">GREEN</div>
        <div class="v" id="s-green">--<span class="u">px</span></div></div>
      <div class="sv"><div class="k">노드</div>
        <div class="v vg" id="s-junc">0</div></div>
      <div class="sv"><div class="k">ACTION</div>
        <div class="v vc" id="s-action" style="font-size:10px">--</div></div>
    </div>
    <div class="ctrl">
      <div class="crow">
        <span class="lbl">THRESH</span>
        <input type="range" min="20" max="200" value="43" style="width:100px;accent-color:var(--cyan)"
          oninput="this.nextElementSibling.textContent=this.value;sendCfg('thresh',+this.value)">
        <span style="font-size:10px;color:var(--cyan);min-width:28px">43</span>
        <span class="lbl" style="margin-left:6px">DEAD_ZONE</span>
        <input type="range" min="10" max="100" value="50" style="width:80px;accent-color:var(--cyan)"
          oninput="this.nextElementSibling.textContent=this.value;sendCfg('dead_zone',+this.value)">
        <span style="font-size:10px;color:var(--cyan);min-width:24px">50</span>
      </div>
      <div class="crow">
        <span class="lbl">KP</span>
        <input type="range" min="1" max="200" value="80" style="width:80px;accent-color:var(--cyan)"
          oninput="this.nextElementSibling.textContent=(this.value/100).toFixed(2);sendCfg('kp',this.value/100)">
        <span style="font-size:10px;color:var(--cyan);min-width:28px">0.80</span>
        <span class="lbl" style="margin-left:6px">KI</span>
        <input type="range" min="0" max="20" value="2" style="width:80px;accent-color:var(--cyan)"
          oninput="this.nextElementSibling.textContent=(this.value/1000).toFixed(3);sendCfg('ki',this.value/1000)">
        <span style="font-size:10px;color:var(--cyan);min-width:36px">0.002</span>
      </div>
    </div>
  </div>

  <!-- MID: 웹캠 -->
  <div class="col">
    <div class="sec">USB Webcam · Obstacle View</div>
    <div class="cam-box">
      <img src="/webcam_feed" alt="">
      <div class="cam-lbl">OBSTACLE CAM</div>
    </div>
    <div class="sbar">
      <div class="sv"><div class="k">PITCH</div>
        <div class="v" id="s-pitch">--<span class="u">°</span></div></div>
      <div class="sv"><div class="k">TERRAIN</div>
        <div class="terrain-badge flat" id="s-terrain">FLAT</div></div>
      <div class="sv"><div class="k">DIST</div>
        <div class="v" id="s-dist">--<span class="u">cm</span></div></div>
      <div class="sv"><div class="k">SPEED</div>
        <div class="v vc" id="s-speed">0<span class="u">%</span></div></div>
    </div>
    <div class="ctrl">
      <div class="crow">
        <span class="lbl">오르막 속도</span>
        <input type="range" min="40" max="100" value="70" style="width:100px;accent-color:var(--orange)"
          oninput="this.nextElementSibling.textContent=this.value+'%';setCond('slope_up_speed',+this.value)">
        <span style="font-size:10px;color:var(--orange);min-width:32px">70%</span>
        <span class="lbl" style="margin-left:6px">내리막 속도</span>
        <input type="range" min="10" max="60" value="30" style="width:80px;accent-color:var(--cyan)"
          oninput="this.nextElementSibling.textContent=this.value+'%';setCond('slope_down_speed',+this.value)">
        <span style="font-size:10px;color:var(--cyan);min-width:32px">30%</span>
      </div>
      <div class="crow">
        <span class="lbl">무게</span>
        <span id="s-weight" style="font-family:var(--label);font-size:13px;color:var(--text0)">-- g</span>
        <button class="btn" onclick="fetch('/tare',{method:'POST'})">TARE</button>
        <span class="lbl" style="margin-left:6px">시뮬 무게</span>
        <button class="btn" id="btn-w-none" onclick="setCond('weight','none');hlBtn('btn-w-none','btn-w-heavy')">없음</button>
        <button class="btn y" id="btn-w-heavy" onclick="setCond('weight','heavy');hlBtn('btn-w-heavy','btn-w-none')">140g</button>
      </div>
    </div>
  </div>

  <!-- RIGHT: 제어 패널 -->
  <div class="rpanel">

    <!-- 경로 표시 -->
    <div class="route-box">
      <div class="route-big" id="route-big">B</div>
      <div class="route-sub" id="route-sub">SAFE MODE · 기본 속도 50%</div>
    </div>

    <!-- 조건 설정 + 출발 -->
    <div class="cond-box">
      <div class="cond-row">
        <span class="lbl">모드</span>
        <button class="btn c sel" id="btn-safe" onclick="setMode('safe')">🛡 SAFE</button>
        <button class="btn o" id="btn-fast" onclick="setMode('fast')">⚡ FAST</button>
      </div>
      <div class="cond-row">
        <span class="lbl">경로</span>
        <button class="btn" id="btn-ra" onclick="setRoute('A')">A</button>
        <button class="btn c sel" id="btn-rb" onclick="setRoute('B')">B</button>
        <button class="btn" id="btn-rc" onclick="setRoute('C')">C</button>
      </div>
      <div class="cond-row">
        <button class="btn g" style="flex:1;padding:8px" onclick="startDrive()">▶ 즉시 출발 (신호등 생략)</button>
        <button class="btn r" onclick="stopDrive()">■ 정지</button>
      </div>
      <div class="cond-row">
        <button class="btn" onclick="saveSession('수동저장')" style="font-size:10px">💾 세션 저장</button>
        <div id="s-ai" style="font-size:10px;color:var(--yellow);flex:1;text-align:right">대기중</div>
      </div>
    </div>

    <!-- 판단 로그 -->
    <div class="sec">판단 로그</div>
    <div class="log-box" id="log-box"></div>

  </div>
</div>

<!-- 장애물 판단 팝업 -->
<div class="obs-overlay" id="obs-overlay">
  <div class="obs-modal">
    <div class="obs-title">⚠ 장애물 감지</div>
    <div class="obs-sub">초음파 20cm 이하 — 장애물 종류를 선택하세요</div>
    <div class="obs-btns">
      <button class="btn g" onclick="sendObstacle('none')">장애물 없음</button>
      <button class="btn y" onclick="sendObstacle('bump_1cm')">1cm 방지턱</button>
      <button class="btn r" onclick="sendObstacle('bump_2cm')">2cm 방지턱</button>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
'use strict';

let prevRoute = 'B';
let curMode   = 'safe';
let curRoute  = 'B';

// 폴링
function poll() {
  fetch('/state').then(r=>r.json()).then(d=>{
    document.getElementById('conn-dot').classList.add('live');
    document.getElementById('conn-lbl').textContent = 'ONLINE';

    // 센서
    const dist = d.distance ?? -1;
    const distEl = document.getElementById('s-dist');
    distEl.innerHTML = (dist>=0?dist.toFixed(1):'--')+'<span class="u">cm</span>';
    distEl.style.color = dist>=0&&dist<15?'var(--red)':dist>=0&&dist<30?'var(--yellow)':'var(--green)';

    const wt = d.weight_g ?? 0;
    const wtEl = document.getElementById('s-weight');
    wtEl.textContent = wt.toFixed(1)+' g';
    wtEl.style.color = wt>=140?'var(--yellow)':'var(--text0)';

    const pt = d.pitch ?? 0;
    const ptEl = document.getElementById('s-pitch');
    ptEl.innerHTML = pt+'<span class="u">°</span>';
    ptEl.style.color = Math.abs(pt)>=7?'var(--orange)':'var(--text0)';

    // 지형 배지
    const terrain = d.terrain || 'flat';
    const tEl = document.getElementById('s-terrain');
    tEl.textContent = terrain === 'uphill' ? '오르막' : terrain === 'downhill' ? '내리막' : '평지';
    tEl.className = 'terrain-badge ' + terrain;

    document.getElementById('s-err').innerHTML   = (d.error??0)+'<span class="u">px</span>';
    document.getElementById('s-green').innerHTML = (d.green_area??0)+'<span class="u">px</span>';
    document.getElementById('s-junc').textContent = d.junction_count ?? 0;
    document.getElementById('s-action').textContent = d.action || '--';

    const spd = d.cruise_speed ?? 0;
    document.getElementById('s-speed').innerHTML = spd+'<span class="u">%</span>';

    // 경로
    const nr = d.route || 'B';
    const routeBig = document.getElementById('route-big');
    if (nr !== prevRoute) {
      routeBig.classList.add('changed');
      setTimeout(()=>routeBig.classList.remove('changed'), 2000);
      prevRoute = nr;
      showToast('경로 변경: '+nr, 'warn');
    }
    routeBig.textContent = nr;
    document.getElementById('route-sub').textContent =
      (d.user_mode||'safe').toUpperCase()+' MODE · 속도 '+spd+'%  지형: '+(tEl.textContent);

    // AI 상태
    document.getElementById('s-ai').textContent = d.ai_status || '대기중';

    // 장애물 팝업
    if (d.obstacle_pending) {
      document.getElementById('obs-overlay').classList.add('show');
    } else {
      document.getElementById('obs-overlay').classList.remove('show');
    }

  }).catch(()=>{
    document.getElementById('conn-dot').classList.remove('live');
    document.getElementById('conn-lbl').textContent = 'OFFLINE';
  });
}

function pollLogs() {
  fetch('/logs').then(r=>r.json()).then(logs=>{
    const box = document.getElementById('log-box');
    box.innerHTML = logs.map(e=>
      `<div class="le ${e.level}">
        <span class="lt">${e.time}</span>
        <span class="lm">${e.msg}</span>
      </div>`
    ).join('');
  }).catch(()=>{});
}

setInterval(poll,     300);
setInterval(pollLogs, 500);
poll(); pollLogs();

// 모드 설정
function setMode(mode) {
  curMode = mode;
  fetch('/set_condition',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode})});
  document.getElementById('btn-safe').className = 'btn'+(mode==='safe'?' c sel':'');
  document.getElementById('btn-fast').className = 'btn o'+(mode==='fast'?' sel':'');
  showToast('모드: '+mode.toUpperCase(), 'info');
}

// 경로 설정
function setRoute(r) {
  curRoute = r;
  fetch('/set_condition',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({route:r})});
  ['A','B','C'].forEach(x=>{
    document.getElementById('btn-r'+x.toLowerCase()).className = 'btn'+(x===r?' c sel':'');
  });
}

// 조건 설정
function setCond(key, val) {
  fetch('/set_condition',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({[key]:val})});
}

// 하이라이트 토글
function hlBtn(on, off) {
  document.getElementById(on).classList.add('sel');
  document.getElementById(off).classList.remove('sel');
}

// 출발/정지
function startDrive() {
  fetch('/start',{method:'POST'}).then(r=>r.json()).then(d=>{
    showToast('출발 — 경로:'+d.route, 'ok');
  });
}
function stopDrive() {
  fetch('/stop',{method:'POST'});
  showToast('정지', 'warn');
}

// 장애물 판단
function sendObstacle(type) {
  document.getElementById('obs-overlay').classList.remove('show');
  fetch('/obstacle',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({type})}).then(r=>r.json()).then(d=>{
    showToast('장애물 판단: '+type, 'info');
  });
}

// 세션 저장
function saveSession(result) {
  fetch('/save_session',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({result})});
  showToast('세션 저장됨', 'ok');
}

// Config 전송
function sendCfg(k,v) {
  fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({[k]:v})});
}

// 시계
setInterval(()=>{
  document.getElementById('clock').textContent =
    new Date().toLocaleTimeString('ko-KR',{hour12:false});
},1000);

// Toast
let _tt=null;
function showToast(msg,type='info') {
  const el=document.getElementById('toast');
  el.textContent=msg; el.className='show '+type;
  if(_tt)clearTimeout(_tt);
  _tt=setTimeout(()=>el.classList.remove('show'),2400);
}

// 키보드 수동 조작
const KEY_MAP = {
  'ArrowUp':'forward','ArrowDown':'backward',
  'ArrowLeft':'left','ArrowRight':'right',' ':'stop'
};
let _ak=null;
document.addEventListener('keydown',e=>{
  if(document.activeElement.tagName==='INPUT'||
     document.activeElement.tagName==='TEXTAREA') return;
  const d=KEY_MAP[e.key]; if(!d)return; e.preventDefault();
  if(_ak===e.key)return; _ak=e.key;
  fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({manual_dir:d})});
});
document.addEventListener('keyup',e=>{
  if(!KEY_MAP[e.key])return; e.preventDefault();
  _ak=null;
  fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({manual_dir:'stop'})});
});
</script>
</body>
</html>'''

if __name__ == '__main__':
    print("=" * 50)
    print("  INSITE AGV TEST DASHBOARD")
    print("  URL: http://192.168.0.50:5004")
    print("  저장: ~/insite/experiment/data/test_sessions/")
    print("=" * 50)
    app.run(host='0.0.0.0', port=5004, threaded=True)