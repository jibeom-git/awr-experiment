#!/usr/bin/env python3
# app/insite_final.py
# Insite AGV 최종 실행 대시보드 (포트 5003)
# 라인트래킹 + AI 판단 + 신호등 + 경로 결정 통합

import sys, os, time, threading, copy, warnings, json, collections
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

from sensors.motor import MotorController

app = Flask(__name__)

# ══════════════════════════════════════════════════════
# 하드웨어 초기화
# ══════════════════════════════════════════════════════

# Pi 카메라 (라인트래킹용)
try:
    from sensors.camera import Camera
    cam = Camera(width=320, height=240)
    CAM_AVAILABLE = True
    print("[OK] Pi 카메라")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] Pi 카메라: {e}")

# USB 웹캠 (장애물/신호등 인식용)
latest_webcam = None
try:
    WEBCAM_AVAILABLE = True
    print("[OK] USB 웹캠 스레드 예약")
except Exception as e:
    WEBCAM_AVAILABLE = False
    print(f"[SKIP] 웹캠: {e}")

# IMU
try:
    from sensors.mpu6050 import MPU6050
    imu = MPU6050(bus_id=5, address=0x68)
    time.sleep(0.5)
    _baseline = imu.get_accel()
    IMU_AVAILABLE = True
    print(f"[OK] IMU")
except Exception as e:
    imu = None
    _baseline = {'x': 0.0}
    IMU_AVAILABLE = False
    print(f"[SKIP] IMU: {e}")

# 초음파
try:
    from gpiozero import DistanceSensor
    import signal as _signal
    def _timeout(s, f): raise TimeoutError()
    _signal.signal(_signal.SIGALRM, _timeout)
    _signal.alarm(3)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
    _signal.alarm(0)
    ULTRA_AVAILABLE = True
    print("[OK] 초음파")
except Exception as e:
    ultra = None
    ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파: {e}")

# 로드셀
try:
    from sensors.hx711 import HX711
    hx711 = HX711(dout=5, pd_sck=6)
    time.sleep(1)
    hx711.tare(samples=10)
    HX711_AVAILABLE = True
    print("[OK] 로드셀")
except Exception as e:
    hx711 = None
    HX711_AVAILABLE = False
    print(f"[SKIP] 로드셀: {e}")

# 모터
motor = MotorController()

# AI 엔진
try:
    from ai_core.engine import AGVAIEngine
    _ai_engine = AGVAIEngine(mode="SAFE")
    AI_AVAILABLE = True
    print("[OK] AI 엔진")
except Exception as e:
    _ai_engine = None
    AI_AVAILABLE = False
    print(f"[SKIP] AI 엔진: {e}")

# 신호등 감지
try:
    from ai_core.signal_detector import detect_traffic_light
    SIGNAL_AVAILABLE = True
    print("[OK] 신호등 감지")
except Exception as e:
    SIGNAL_AVAILABLE = False
    print(f"[SKIP] 신호등: {e}")

# ══════════════════════════════════════════════════════
# 경로 결정 규칙 (Rule-based)
# ══════════════════════════════════════════════════════
# FAST: 150g 이상→A, 150g 미만→A
#   방지턱 2cm→B우회, 1cm(150g이상)→B우회, 1cm(150g미만)→통과
# SAFE: 150g 이상→B, 150g 미만→B
#   방지턱 2cm→C우회, 1cm(150g이상)→C우회, 1cm(150g미만)→통과

def decide_initial_route(mode: str, weight_g: float) -> str:
    """출발 시 무게+모드로 초기 경로 결정"""
    if mode == "fast":
        return "A"  # FAST는 항상 A 출발
    else:
        return "B"  # SAFE는 항상 B 출발

def decide_reroute(current_route: str, mode: str,
                   weight_g: float, obstacle: str) -> str | None:
    """
    장애물 감지 시 우회 경로 결정.
    반환: 우회 경로 문자열 또는 None(통과 가능)
    """
    heavy = weight_g >= 150.0

    if mode == "fast":
        if obstacle == "bump_2cm":
            return "A→B"  # 항상 B 우회
        elif obstacle == "bump_1cm":
            return "A→B" if heavy else None  # 150g 이상만 우회
    else:  # safe
        if current_route == "B":
            if obstacle == "bump_2cm":
                return "B→C"  # 항상 C 우회
            elif obstacle == "bump_1cm":
                return "B→C" if heavy else None
    return None

# ══════════════════════════════════════════════════════
# 라우트 정의 (분기점 행동)
# ══════════════════════════════════════════════════════
ROUTES = {
    "A":   {1: "straight", 2: "stop"},
    "B":   {1: "left",  2: "right", 3: "right", 4: "stop"},
    "C":   {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight", 6: "stop"},
    "A→B": {1: "left", 2: "right", 3: "right", 4: "stop"},
    "A→C": {1: "left", 2: "straight", 3: "right", 4: "right", 5: "straight", 6: "stop"},
    "B→C": {1: "left", 2: "right", 3: "right", 4: "straight", 5: "stop"},
}

DETOUR_MAP = {
    ("A", "B"): "A→B",
    ("A", "C"): "A→C",
    ("B", "C"): "B→C",
}

# ══════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════
lock = threading.Lock()

config = {
    "base_speed":     45,
    "turn_speed":     30,
    "spin_speed":     25,
    "thresh":         43,
    "roi_top":        0.7,
    "dead_zone":      15,
    "kp":             0.8,
    "ki":             0.002,
    "forward_time":   0.5,
    "slope_speed":    60,
    "green_h_min":    30,
    "green_h_max":    85,
    "green_s_min":    80,
    "green_v_min":    50,
    "green_min_area": 200,
    "running":        False,
    "route":          "A",
    "user_mode":      "safe",
}

state = {
    "action":         "정지",
    "error":          0,
    "green_area":     0,
    "junction_count": 0,
    "route":          "A",
    "distance":       -1,
    "weight_g":       0.0,
    "pitch":          0.0,
    "fps":            0,
    "lost":           False,
    "signal":         "UNKNOWN",
    "waiting_signal": False,
    # AI 판단
    "ai_status":      "대기중",
    "gemini_type":    "--",
    "gemini_conf":    "--",
    "xgb_label":      "--",
    "cmd_result":     "--",
    "user_mode":      "safe",
}

# 판단 로그 (최대 50건)
_decision_log = collections.deque(maxlen=50)

def add_log(msg: str, level: str = "info"):
    """판단 로그 추가"""
    entry = {
        "time": time.strftime("%H:%M:%S"),
        "msg":  msg,
        "level": level  # info / warn / ok / err
    }
    _decision_log.appendleft(entry)

# ══════════════════════════════════════════════════════
# 분기점 카운트
# ══════════════════════════════════════════════════════
g_junction_count = 0
g_junction_lock  = threading.Lock()

def get_junction_count():
    with g_junction_lock: return g_junction_count

def increment_junction():
    global g_junction_count
    with g_junction_lock:
        g_junction_count += 1
        return g_junction_count

def reset_junction():
    global g_junction_count
    with g_junction_lock:
        g_junction_count = 0

# ══════════════════════════════════════════════════════
# 모터 헬퍼
# ══════════════════════════════════════════════════════
def go_forward(speed):
    motor.set_motor(1,1,speed); motor.set_motor(2,1,speed)
    motor.set_motor(3,1,speed); motor.set_motor(4,1,speed)

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
    if hx711 is None: return 0.0
    try:
        w = hx711.get_weight(times=3)
        return max(0.0, float(w))
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
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) + roi_y
    else:
        cy = roi_y

    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower = np.array([cfg["green_h_min"], cfg["green_s_min"], cfg["green_v_min"]])
    upper = np.array([cfg["green_h_max"], 255, 255])
    green_mask = cv2.inRange(roi_hsv, lower, upper)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_area  = int(np.sum(green_mask > 0))
    is_junction = green_area > cfg["green_min_area"]

    debug = frame.copy()
    cv2.line(debug, (0, roi_y), (w, roi_y), (255,180,0), 1)
    if cx is not None:
        cv2.circle(debug, (cx, cy), 6, (0,255,80), -1)
        err = cx - w//2
        cv2.putText(debug, f"err={err:+d}", (5, roi_y-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,80), 1)
    if is_junction:
        cv2.putText(debug, "NODE", (5,18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    return cx, green_area, is_junction, debug

# ══════════════════════════════════════════════════════
# 분기점 행동
# ══════════════════════════════════════════════════════
def execute_junction(action, cfg):
    go_forward(cfg["base_speed"])
    time.sleep(cfg["forward_time"])
    motor.motorStop()
    time.sleep(0.1)

    if action == "stop":
        go_forward(cfg["base_speed"])
        time.sleep(0.2)
        motor.motorStop()
        config['running'] = False
        add_log("도착 — 주행 완료", "ok")
        return

    if action == "straight":
        go_forward(cfg["base_speed"])
        time.sleep(0.4)
        return

    if action in ("left", "right"):
        spin_fn = spin_left if action == "left" else spin_right
        spin_fn(35)
        time.sleep(1.5)
        fine_timeout = time.time() + 2.0
        while time.time() < fine_timeout:
            if cam is None: break
            frame = cam.capture()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w  = frame.shape[:2]
            roi_y = int(h * cfg["roi_top"])
            roi   = frame[roi_y:h, :]
            gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, cfg["thresh"], 255, cv2.THRESH_BINARY_INV)
            M = cv2.moments(binary)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                if abs(cx - w//2) < 20:
                    motor.motorStop()
                    return
            spin_fn(30)
            time.sleep(0.03)
        motor.motorStop()

# ══════════════════════════════════════════════════════
# 우회 처리
# ══════════════════════════════════════════════════════
def do_reroute(new_route: str):
    add_log(f"경로 우회: {new_route}", "warn")
    motor.motorStop()
    time.sleep(0.3)

    timeout = time.time() + 10.0
    while time.time() < timeout:
        if cam is None: break
        frame = cam.capture()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        h, w  = frame.shape[:2]
        roi_y = int(config["roi_top"] * h)
        roi   = frame[roi_y:h, :]
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([config["green_h_min"], config["green_s_min"], config["green_v_min"]])
        upper = np.array([config["green_h_max"], 255, 255])
        mask  = cv2.inRange(roi_hsv, lower, upper)
        if int(np.sum(mask > 0)) > config["green_min_area"]:
            motor.motorStop()
            break
        motor.set_motor(1,-1,config["base_speed"]); motor.set_motor(2,-1,config["base_speed"])
        motor.set_motor(3,-1,config["base_speed"]); motor.set_motor(4,-1,config["base_speed"])
        time.sleep(0.03)

    motor.motorStop()
    time.sleep(0.5)
    reset_junction()
    config['route']   = new_route
    config['running'] = True
    with lock:
        state["junction_count"] = 0
        state["route"]          = new_route
        state["action"]         = f"우회→{new_route}"

# ══════════════════════════════════════════════════════
# XGBoost 속도 조절 (경사 감지)
# ══════════════════════════════════════════════════════
# 수집한 experiment_v2.csv 데이터 기반
# pitch > TH_PITCH_HILL → 오르막 감지 → XGBoost 최적 속도 예측
# pitch < -TH_PITCH_HILL → 내리막 감지 → XGBoost 안전 속도 예측

TH_PITCH_HILL = 7.0   # 경사 감지 임계값 (도)

def get_xgb_speed(pitch: float, weight_g: float, route: str) -> int | None:
    """
    XGBoost 모델로 현재 경사+무게에 맞는 최적 속도 예측.
    반환: 속도(%) 또는 None(모델 없음)
    """
    if _ai_engine is None or not AI_AVAILABLE:
        return None
    try:
        trainer = _ai_engine.trainer
        if trainer is None or not hasattr(trainer, 'xgb_speed'):
            return None

        import numpy as np
        # 입력 피처: pitch, weight_g, sonic_cm, obs_enc,
        #            mode_enc, gemini_passable, gemini_conf, impact_z_prev
        route_enc = {"ROUTE_A": 0, "ROUTE_B": 1, "ROUTE_C": 2}.get(
            f"ROUTE_{route}", 1)
        mode_enc  = 1 if config.get("user_mode") == "fast" else 0

        X = np.array([[
            pitch,      # pitch
            weight_g,   # weight_g
            200.0,      # sonic_cm (장애물 없음)
            0,          # obs_enc (장애물 없음)
            mode_enc,   # mode_enc
            1,          # gemini_passable (통과 가능)
            0.9,        # gemini_conf
            0.0,        # impact_z_prev
        ]], dtype=np.float32)

        speed = trainer.xgb_speed.predict(X)[0]
        return max(15, min(80, int(speed)))
    except Exception as e:
        print(f"[XGB] 속도 예측 오류: {e}")
        return None

# ══════════════════════════════════════════════════════
# AI 판단 (장애물 → Gemini VLM + 경로 결정)
# ══════════════════════════════════════════════════════
def run_ai_decision(cur_route: str, frame_snap):
    if not AI_AVAILABLE:
        config["running"] = True
        return

    try:
        with lock:
            state["ai_status"] = "분석중..."
        add_log("장애물 감지 — AI 판단 시작", "warn")

        engine = _ai_engine
        engine.mode = config.get("user_mode", "safe").upper()

        pitch    = state.get("pitch", 0.0)
        weight_g = state.get("weight_g", 0.0)
        dist     = state.get("distance", -1)

        sensor_snapshot = {
            "pitch":         pitch,
            "weight_g":      weight_g,
            "distance_cm":   dist,
            "node_trigger":  False,
            "current_route": cur_route,
            "frame":         frame_snap,
        }

        result   = engine.evaluate_state_and_calculate_output(sensor_snapshot, frame_snap)
        ai_state = result.get("state", "--")
        ai_route = result.get("route", "--")
        ai_speed = result.get("speed_limit_pct", config["base_speed"])

        vlm          = engine.last_vlm_result if hasattr(engine, 'last_vlm_result') else {}
        gemini_type  = vlm.get("obstacle_type", "--")
        gemini_conf  = vlm.get("confidence",    "--")
        gemini_pass  = vlm.get("passable",       True)

        # 규칙 기반 경로 결정
        obstacle_key = None
        if gemini_type and "2" in str(gemini_type):
            obstacle_key = "bump_2cm"
        elif gemini_type and "1" in str(gemini_type):
            obstacle_key = "bump_1cm"

        reroute_str = None
        if obstacle_key:
            reroute_str = decide_reroute(
                cur_route,
                config.get("user_mode", "safe"),
                weight_g,
                obstacle_key
            )

        xgb_label = "pass"
        if not gemini_pass or reroute_str:
            xgb_label = "detour"
        elif "CAUTIOUS" in ai_state:
            xgb_label = "cautious"

        with lock:
            state["ai_status"]   = ai_state
            state["gemini_type"] = str(gemini_type)
            state["gemini_conf"] = f"{float(gemini_conf)*100:.0f}%" if gemini_conf != "--" else "--"
            state["xgb_label"]   = xgb_label

        add_log(
            f"Gemini: {gemini_type} | 통과={'가능' if gemini_pass else '불가'} "
            f"| XGB: {xgb_label}",
            "ok" if gemini_pass else "err"
        )

        if reroute_str:
            add_log(f"경로 변경 결정: {cur_route} → {reroute_str}", "warn")
            threading.Thread(
                target=do_reroute, args=(reroute_str,),
                daemon=True, name="reroute"
            ).start()
        elif ai_state in ("PATH_BLOCKED", "REROUTING_RUN"):
            route_key   = ai_route.replace("ROUTE_", "")
            reroute_key = (cur_route, route_key)
            new_route   = DETOUR_MAP.get(reroute_key)
            if new_route:
                add_log(f"AI 우회: {cur_route} → {new_route}", "warn")
                threading.Thread(
                    target=do_reroute, args=(new_route,),
                    daemon=True, name="reroute"
                ).start()
            else:
                config["running"] = True
        elif ai_state == "CRITICAL_STOP":
            motor.motorStop()
            config["running"] = False
            add_log("비상정지", "err")
        else:
            # XGBoost 속도 조절
            xgb_spd = get_xgb_speed(pitch, weight_g, cur_route)
            if xgb_spd and xgb_spd < config["base_speed"]:
                config["base_speed"] = xgb_spd
                add_log(f"XGBoost 속도 조절: {xgb_spd}%", "info")
            config["running"] = True

    except Exception as e:
        import traceback; traceback.print_exc()
        config["running"] = True
        add_log(f"AI 오류: {str(e)[:40]}", "err")
        with lock:
            state["ai_status"] = f"오류: {str(e)[:30]}"

# ══════════════════════════════════════════════════════
# 신호등 감지 루프
# ══════════════════════════════════════════════════════
check_signal_active = threading.Event()

def signal_check_loop():
    """
    초록불 3회 연속 확인 후 출발.
    빨강/노랑 → 정지 유지.
    """
    if not SIGNAL_AVAILABLE:
        add_log("신호등 모듈 없음 → 즉시 출발", "warn")
        config["running"] = True
        return

    add_log("신호등 대기 시작", "info")
    green_count = 0

    while check_signal_active.is_set():
        with lock:
            img_bytes = latest_webcam

        if img_bytes is not None:
            arr   = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            sig   = detect_traffic_light(frame)
        else:
            sig = "UNKNOWN"

        with lock:
            state["signal"] = sig

        if sig == "GO":
            green_count += 1
            add_log(f"신호등 GO ({green_count}/3)", "ok")
            if green_count >= 3:
                add_log("초록불 확정 — 출발!", "ok")
                with lock:
                    state["waiting_signal"] = False
                    state["action"] = "직진 주행"
                config["running"] = True
                check_signal_active.clear()
                return
        else:
            if green_count > 0:
                add_log(f"신호등 {sig} — 카운트 초기화", "warn")
            green_count = 0
            motor.motorStop()
            with lock:
                state["action"] = f"신호대기({sig})"

        time.sleep(0.15)

# ══════════════════════════════════════════════════════
# 백그라운드 스레드
# ══════════════════════════════════════════════════════
def ultra_loop():
    while True:
        if ultra is not None:
            try:
                d = ultra.distance
                with lock:
                    state["distance"] = round(d*100,1) if d else -1
            except:
                pass
        time.sleep(0.1)

def weight_loop():
    while True:
        w = get_weight_g()
        with lock:
            state["weight_g"] = round(w, 1)
        time.sleep(0.5)

def imu_loop():
    while True:
        if imu is not None:
            try:
                data = imu.get_all()
                with lock:
                    state["pitch"] = round(float(data.get('pitch', 0.0)), 1)
            except:
                pass
        time.sleep(0.05)

def webcam_loop():
    global latest_webcam
    # USB 웹캠 자동 탐색 (재부팅마다 인덱스 변경)
    idx = None
    for i in [0, 1, 2, 3]:
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                idx = i
                cap.release()
                break
            cap.release()

    if idx is None:
        print("[SKIP] USB 웹캠 탐색 실패")
        return

    _wcam = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    _wcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    _wcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print(f"[OK] USB 웹캠 index={idx}")

    while True:
        if _wcam.isOpened():
            ret, frame = _wcam.read()
            if ret:
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                with lock:
                    latest_webcam = jpeg.tobytes()
        time.sleep(0.033)

threading.Thread(target=ultra_loop,  daemon=True, name="ultra").start()
threading.Thread(target=weight_loop, daemon=True, name="weight").start()
threading.Thread(target=imu_loop,    daemon=True, name="imu").start()
threading.Thread(target=webcam_loop, daemon=True, name="webcam").start()

# ══════════════════════════════════════════════════════
# 주행 루프
# ══════════════════════════════════════════════════════
latest_picam = None

def drive_loop():
    global latest_picam

    CENTER_X          = 160
    lost_dir          = 1
    fps_t             = time.time()
    fps_count         = 0
    integral          = 0.0
    prev_time         = time.time()
    green_seen        = False
    junction_cooldown = 0.0
    stopped           = False
    prev_pitch        = 0.0

    while True:
        cfg = copy.deepcopy(config)

        frame = None
        if cam is not None:
            try:
                frame = cam.capture()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except:
                frame = None

        if frame is not None:
            cx, green_area, is_junction, debug = detect_line_and_green(frame, cfg)
            _, jpeg = cv2.imencode('.jpg', debug, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with lock:
                latest_picam = jpeg.tobytes()
        else:
            cx, green_area, is_junction = None, 0, False

        current_route = cfg["route"]

        fps_count += 1
        if time.time() - fps_t >= 1.0:
            with lock: state["fps"] = fps_count
            fps_count = 0; fps_t = time.time()

        if not cfg["running"]:
            if not stopped:
                motor.motorStop(); stopped = True
            integral = 0.0
            with lock:
                state["action"]         = "정지"
                state["green_area"]     = green_area
                state["junction_count"] = get_junction_count()
            time.sleep(0.05)
            continue
        else:
            stopped = False

        now = time.time()
        dt  = now - prev_time; prev_time = now
        if dt > 0.1: dt = 0.033

        # 분기점 처리
        if is_junction and not green_seen and now > junction_cooldown:
            green_seen      = True
            junction_cooldown = now + 3.0
            jc = increment_junction()
            integral = 0.0
            action_at = ROUTES[current_route].get(jc, "straight")
            add_log(f"분기점 #{jc} — {current_route} — {action_at}", "info")
            with lock:
                state["junction_count"] = jc
                state["action"] = f"분기점#{jc} {action_at}"
            execute_junction(action_at, cfg)
            continue

        if not is_junction:
            green_seen = False

        # 경사 감지 → XGBoost 속도 자동 조절
        with lock:
            cur_pitch  = state.get("pitch", 0.0)
            cur_weight = state.get("weight_g", 0.0)

        pitch_change = cur_pitch - prev_pitch
        prev_pitch   = cur_pitch

        if abs(cur_pitch) > TH_PITCH_HILL and abs(pitch_change) > 1.0:
            # 경사 진입 감지
            xgb_spd = get_xgb_speed(cur_pitch, cur_weight, current_route)
            if xgb_spd and xgb_spd != config["base_speed"]:
                config["base_speed"] = xgb_spd
                direction = "오르막" if cur_pitch > 0 else "내리막"
                add_log(
                    f"{direction} 감지 (pitch={cur_pitch}°) → XGBoost 속도: {xgb_spd}%",
                    "info"
                )

        # 초음파 < 20cm → AI 판단
        with lock:
            dist_now = state["distance"]

        if (cfg["running"] and 0 < dist_now < 20 and
                not getattr(drive_loop, '_ai_running', False)):
            drive_loop._ai_running = True
            config["running"] = False
            motor.motorStop()
            time.sleep(0.5)
            frame_snap = frame.copy() if frame is not None else np.zeros((240,320,3), np.uint8)

            def _run_ai(r=current_route, f=frame_snap):
                try:
                    time.sleep(0.5)
                    run_ai_decision(r, f)
                finally:
                    drive_loop._ai_running = False

            threading.Thread(target=_run_ai, daemon=True, name="ai").start()
            time.sleep(0.03)
            continue

        # 라인 추종 PI 제어
        on_slope = abs(cur_pitch) > TH_PITCH_HILL
        action   = "직진"

        if on_slope:
            go_forward(cfg["slope_speed"])
            action = f"경사({'오르막' if cur_pitch>0 else '내리막'})"
        elif cx is None:
            integral = 0.0
            action   = f"탐색({'우' if lost_dir>0 else '좌'})"
            spin_right(cfg["spin_speed"]) if lost_dir>0 else spin_left(cfg["spin_speed"])
        else:
            error    = cx - CENTER_X
            integral = max(min(integral + error*dt, 200), -200)
            correction = int(error*cfg["kp"] + integral*cfg["ki"])
            correction = max(min(correction, cfg["base_speed"]-10), -(cfg["base_speed"]-10))
            lost_dir = 1 if error>0 else (-1 if error<0 else lost_dir)

            if abs(error) < cfg["dead_zone"]:
                go_forward(cfg["base_speed"]); action = "직진"
            elif correction > 0:
                rs = max(cfg["base_speed"]-abs(correction), cfg["turn_speed"])
                turn_right(cfg["base_speed"], rs); action = f"우회전"
            else:
                ls = max(cfg["base_speed"]-abs(correction), cfg["turn_speed"])
                turn_left(ls, cfg["base_speed"]); action = f"좌회전"

        with lock:
            state["error"]          = cx - CENTER_X if cx else 0
            state["action"]         = action
            state["lost"]           = cx is None
            state["route"]          = current_route
            state["green_area"]     = green_area
            state["junction_count"] = get_junction_count()

        time.sleep(0.03)

threading.Thread(target=drive_loop, daemon=True, name="drive").start()

# ══════════════════════════════════════════════════════
# Flask 라우트
# ══════════════════════════════════════════════════════
def gen_stream(get_fn):
    while True:
        with lock: frame = get_fn()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
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
    return jsonify(list(_decision_log))

@app.route('/start', methods=['POST'])
def start():
    reset_junction()
    mode    = config.get('user_mode', 'safe')
    weight  = state.get('weight_g', 0.0)
    route   = decide_initial_route(mode, weight)
    config['route']   = route
    config['running'] = False

    if _ai_engine is not None:
        _ai_engine.mode = mode.upper()
        _ai_engine.route_availability = {"A": True, "B": True, "C": True}

    with lock:
        state["route"]          = route
        state["junction_count"] = 0
        state["ai_status"]      = "신호등 대기 중"
        state["action"]         = "신호대기"
        state["waiting_signal"] = True
        state["signal"]         = "UNKNOWN"

    add_log(f"출발 준비 — 모드:{mode.upper()} 무게:{weight:.0f}g → 경로:{route}", "ok")

    check_signal_active.set()
    threading.Thread(
        target=signal_check_loop, daemon=True, name="signal"
    ).start()

    return jsonify({'status': 'waiting_signal', 'route': route})

@app.route('/stop', methods=['POST'])
def stop():
    config['running'] = False
    check_signal_active.clear()
    motor.motorStop()
    with lock:
        state["action"]         = "정지"
        state["waiting_signal"] = False
    add_log("운영자 정지", "warn")
    return jsonify({'status': 'stopped'})

@app.route('/command', methods=['POST'])
def command():
    data      = request.get_json(force=True)
    user_text = data.get('text', '').strip()
    if not user_text:
        return jsonify({'status': 'error', 'msg': '텍스트 없음'})

    def _parse():
        try:
            from ai_core.commander import CommanderVLM
            cmd    = CommanderVLM()
            result = cmd.parse(user_text)
            mode   = result.get('mode', 'safe')
            route  = result.get('route', 'B')
            reason = result.get('reason', '')
            if isinstance(route, str):
                route = route.replace("ROUTE_","").strip().upper()
                if route not in ("A","B","C"):
                    route = "C"
            config['user_mode'] = mode
            if not config['running']:
                config['route'] = route
                with lock:
                    state['route'] = route
            with lock:
                state['cmd_result'] = f"[{mode.upper()}] {reason}"
                state['user_mode']  = mode
            if _ai_engine is not None:
                _ai_engine.mode = mode.upper()
            add_log(f"자연어 명령 → 모드:{mode.upper()} 이유:{reason}", "info")
        except Exception as e:
            with lock:
                state['cmd_result'] = f"오류: {str(e)[:40]}"
            add_log(f"명령 파싱 오류: {e}", "err")

    threading.Thread(target=_parse, daemon=True).start()
    return jsonify({'status': 'ok', 'msg': '판단 중...'})

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get('mode', 'safe').lower()
    if mode in ('fast', 'safe'):
        config['user_mode'] = mode
        with lock:
            state['user_mode'] = mode
        add_log(f"모드 변경: {mode.upper()}", "info")
    return jsonify({'status': 'ok', 'mode': config['user_mode']})

@app.route('/tare', methods=['POST'])
def tare():
    if hx711 is not None:
        try:
            hx711.tare(samples=30)
            add_log("로드셀 TARE 완료", "ok")
            return jsonify({'status': 'ok'})
        except Exception as e:
            return jsonify({'status': 'error', 'msg': str(e)})
    return jsonify({'status': 'skip'})

@app.route('/config', methods=['GET', 'POST'])
def handle_config():
    if request.method == 'POST':
        data = request.get_json(force=True)
        for k, v in data.items():
            if k in config: config[k] = v
        return jsonify({'status': 'ok'})
    return jsonify(config)

@app.route('/')
def index():
    return render_template_string(HTML)

# ══════════════════════════════════════════════════════
# HTML 템플릿
# ══════════════════════════════════════════════════════
HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>INSITE AGV · FINAL</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;600;700&family=Barlow+Condensed:wght@400;600;700&display=swap');
:root {
  --bg0:#07090d; --bg1:#0c0f16; --bg2:#111520; --bg3:#161c2a; --bg4:#1c2438;
  --border:#1e2840; --border2:#27334e;
  --cyan:#00d4ff; --cyan-d:rgba(0,212,255,.12); --cyan-g:rgba(0,212,255,.3);
  --green:#00e676; --yellow:#ffcc02; --orange:#ff9100; --red:#ff3d3d;
  --text0:#e4ecf8; --text1:#8090b0; --text2:#384a6a;
  --mono:'JetBrains Mono',monospace; --label:'Barlow Condensed',sans-serif;
}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;background:var(--bg0);color:var(--text0);
  font-family:var(--mono);font-size:13px;overflow:hidden}

/* HEADER */
header{height:44px;background:var(--bg1);border-bottom:1px solid var(--border);
  display:flex;align-items:center;justify-content:space-between;padding:0 16px;
  flex-shrink:0;position:relative}
header::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:linear-gradient(90deg,transparent,var(--cyan),transparent)}
.logo{font-family:var(--label);font-size:17px;font-weight:700;
  letter-spacing:.2em;color:var(--cyan);text-shadow:0 0 16px var(--cyan-g)}
.logo em{font-style:normal;font-size:10px;font-weight:400;
  color:var(--text1);letter-spacing:.1em;margin-left:10px}
.hdr-r{display:flex;align-items:center;gap:14px}
.conn{display:flex;align-items:center;gap:5px;font-size:9px;
  color:var(--text2);letter-spacing:.12em;font-family:var(--label)}
.dot{width:6px;height:6px;border-radius:50%;background:var(--red);
  box-shadow:0 0 5px var(--red);transition:all .3s}
.dot.live{background:var(--green);box-shadow:0 0 8px var(--green)}
.clock{font-size:10px;color:var(--text2);min-width:56px;text-align:right}

/* LAYOUT */
.layout{display:grid;grid-template-columns:1fr 1fr 300px;
  height:calc(100vh - 44px);overflow:hidden;gap:0}

/* LEFT/MID: 카메라 + 하단 */
.col{display:flex;flex-direction:column;border-right:1px solid var(--border);overflow:hidden}
.col:last-child{border-right:none}

.sec{font-family:var(--label);font-size:8px;font-weight:600;
  letter-spacing:.2em;color:var(--text2);text-transform:uppercase;
  padding:6px 12px 5px;border-bottom:1px solid var(--border);flex-shrink:0}

/* 카메라 */
.cam-box{flex:1;background:#020508;position:relative;overflow:hidden;min-height:0}
.cam-box img{width:100%;height:100%;object-fit:contain;display:block}
.cam-label{position:absolute;top:8px;left:10px;font-size:8px;
  color:var(--yellow);letter-spacing:.12em;font-family:var(--label);opacity:.7}
.signal-badge{position:absolute;top:8px;right:10px;
  padding:3px 8px;border-radius:2px;font-family:var(--label);
  font-size:10px;font-weight:700;letter-spacing:.1em}
.signal-badge.go{background:rgba(0,230,118,.2);border:1px solid var(--green);color:var(--green)}
.signal-badge.stop{background:rgba(255,61,61,.2);border:1px solid var(--red);color:var(--red)}
.signal-badge.unknown{background:rgba(255,204,2,.1);border:1px solid var(--yellow);color:var(--yellow)}

/* 하단 스트립 */
.strip{background:var(--bg1);padding:10px 12px;flex-shrink:0;
  border-top:1px solid var(--border)}
.strip-row{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:6px}
.strip-row:last-child{margin-bottom:0}
.lbl{font-family:var(--label);font-size:8px;color:var(--text2);
  letter-spacing:.15em;white-space:nowrap}

/* 버튼 */
.btn{font-family:var(--label);font-size:11px;font-weight:600;
  letter-spacing:.07em;padding:5px 11px;border-radius:2px;cursor:pointer;
  border:1px solid var(--border2);background:var(--bg3);color:var(--text0);
  transition:all .15s;white-space:nowrap}
.btn:hover{border-color:var(--cyan);color:var(--cyan)}
.btn.cyan{background:var(--cyan-d);border-color:var(--cyan);color:var(--cyan)}
.btn.green{background:rgba(0,230,118,.08);border-color:var(--green);color:var(--green)}
.btn.red{border-color:rgba(255,61,61,.4);color:var(--red)}
.btn.red:hover{background:rgba(255,61,61,.12)}
.btn.yellow{border-color:rgba(255,204,2,.4);color:var(--yellow)}
.btn.orange{border-color:rgba(255,145,0,.4);color:var(--orange)}
.btn.mode-sel-fast{background:var(--cyan-d);border-color:var(--cyan);color:var(--cyan);box-shadow:0 0 8px var(--cyan-g)}
.btn.mode-sel-safe{background:rgba(0,230,118,.08);border-color:var(--green);color:var(--green)}
.btn.start-run{animation:pulse-g 1.2s infinite}
@keyframes pulse-g{0%,100%{box-shadow:0 0 4px rgba(0,230,118,.3)}50%{box-shadow:0 0 14px rgba(0,230,118,.7)}}

/* 상태 바 */
.stat-bar{display:grid;grid-template-columns:repeat(4,1fr);
  border-bottom:1px solid var(--border);flex-shrink:0}
.s{padding:8px 10px;border-right:1px solid var(--border)}
.s:last-child{border-right:none}
.s .k{font-family:var(--label);font-size:7px;color:var(--text2);
  letter-spacing:.15em;text-transform:uppercase;margin-bottom:4px}
.s .v{font-family:var(--label);font-size:18px;font-weight:700;
  color:var(--text0);line-height:1}
.s .u{font-size:9px;color:var(--text1);font-weight:400;margin-left:2px}
.v.c{color:var(--cyan)} .v.g{color:var(--green)} .v.y{color:var(--yellow)} .v.r{color:var(--red)}

/* RIGHT PANEL */
.rpanel{display:flex;flex-direction:column;overflow:hidden}

/* 경로 표시 */
.route-box{padding:12px;border-bottom:1px solid var(--border);flex-shrink:0}
.route-main{font-family:var(--label);font-size:42px;font-weight:700;
  color:var(--cyan);line-height:1;text-align:center;transition:all .3s}
.route-main.changed{color:var(--orange);text-shadow:0 0 16px rgba(255,145,0,.5)}
.route-sub{font-family:var(--label);font-size:10px;color:var(--text2);
  text-align:center;letter-spacing:.15em;margin-top:4px}

/* 제어 버튼 */
.ctrl-box{padding:10px 12px;border-bottom:1px solid var(--border);flex-shrink:0}
.mode-row{display:flex;gap:6px;margin-bottom:8px}
.mode-btn{flex:1;padding:7px;font-family:var(--label);font-size:12px;
  font-weight:700;letter-spacing:.08em;border-radius:2px;cursor:pointer;
  border:1px solid var(--border2);background:var(--bg3);color:var(--text1);transition:all .2s}
.mode-btn:hover{border-color:var(--cyan);color:var(--cyan)}
.drive-row{display:flex;gap:6px}
.drive-btn{flex:1;padding:8px;font-family:var(--label);font-size:12px;
  font-weight:700;letter-spacing:.08em;border-radius:2px;cursor:pointer;
  border:1px solid var(--border2);background:var(--bg3);color:var(--text1);transition:all .2s}

/* AI 상태 */
.ai-box{padding:10px 12px;border-bottom:1px solid var(--border);flex-shrink:0}
.ai-status{padding:6px 10px;border-radius:2px;font-family:var(--label);
  font-size:11px;font-weight:700;letter-spacing:.08em;text-align:center;
  border:1px solid var(--border2);color:var(--text1);transition:all .3s;margin-bottom:8px}
.ai-status.ok{border-color:var(--green);color:var(--green);background:rgba(0,230,118,.07)}
.ai-status.warn{border-color:var(--yellow);color:var(--yellow);background:rgba(255,204,2,.07)}
.ai-status.err{border-color:var(--red);color:var(--red);background:rgba(255,61,61,.07)}
.ai-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.ai-item{background:rgba(0,0,0,.3);border:1px solid var(--border);
  border-radius:2px;padding:7px 9px}
.ai-item .k{font-size:7px;color:var(--text2);letter-spacing:.1em;
  font-family:var(--label);margin-bottom:3px}
.ai-item .v{font-size:11px;font-weight:600;color:var(--cyan)}
.v.pass{color:var(--green)} .v.cautious{color:var(--yellow)} .v.detour{color:var(--red)}

/* 채팅 */
.chat-box{padding:10px 12px;border-bottom:1px solid var(--border);flex-shrink:0}
.chat-row{display:flex;gap:6px}
.chat-in{flex:1;font-family:var(--mono);font-size:11px;
  background:var(--bg0);border:1px solid var(--border2);
  color:var(--text0);padding:6px 9px;border-radius:2px;outline:none}
.chat-in:focus{border-color:var(--cyan)}
.chat-in::placeholder{color:var(--text2)}
.chat-send{padding:6px 12px;font-family:var(--label);font-size:11px;
  font-weight:700;border-radius:2px;cursor:pointer;
  border:1px solid var(--cyan);background:var(--cyan-d);color:var(--cyan);
  letter-spacing:.06em;transition:all .15s}
.chat-send:hover{background:rgba(0,212,255,.2)}
.chat-result{font-size:10px;color:var(--text1);margin-top:6px;min-height:14px}

/* 판단 로그 */
.log-box{flex:1;overflow-y:auto;overflow-x:hidden}
.log-box::-webkit-scrollbar{width:3px}
.log-box::-webkit-scrollbar-thumb{background:var(--border2)}
.log-entry{display:flex;gap:8px;padding:5px 12px;
  border-bottom:1px solid var(--border);align-items:flex-start}
.log-entry:hover{background:var(--bg2)}
.log-t{font-size:8px;color:var(--text2);white-space:nowrap;font-family:var(--label);
  padding-top:1px;min-width:48px}
.log-m{font-size:10px;color:var(--text1);line-height:1.4;word-break:break-all}
.log-entry.ok .log-m{color:var(--green)}
.log-entry.warn .log-m{color:var(--yellow)}
.log-entry.err .log-m{color:var(--red)}
.log-entry.info .log-m{color:var(--text0)}

/* 신호등 */
.signal-row{display:flex;align-items:center;gap:8px;
  padding:6px 12px;border-bottom:1px solid var(--border);flex-shrink:0}
.signal-dot{width:10px;height:10px;border-radius:50%;
  background:var(--text2);transition:all .3s}
.signal-dot.go{background:var(--green);box-shadow:0 0 8px var(--green)}
.signal-dot.stop{background:var(--red);box-shadow:0 0 8px var(--red);animation:blink .5s infinite}
.signal-dot.unknown{background:var(--yellow)}
.signal-txt{font-family:var(--label);font-size:10px;color:var(--text1)}

@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}

/* TOAST */
#toast{position:fixed;bottom:18px;left:50%;transform:translateX(-50%);
  padding:7px 18px;border-radius:2px;font-size:11px;letter-spacing:.06em;
  border:1px solid var(--border2);background:var(--bg3);color:var(--text0);
  z-index:9999;pointer-events:none;opacity:0;transition:opacity .2s;
  font-family:var(--label);font-weight:600;white-space:nowrap}
#toast.show{opacity:1}
#toast.ok{border-color:var(--green);color:var(--green)}
#toast.err{border-color:var(--red);color:var(--red)}
#toast.warn{border-color:var(--orange);color:var(--orange)}
#toast.info{border-color:var(--cyan);color:var(--cyan)}
</style>
</head>
<body>

<header>
  <div class="logo">INSITE<em>AGV FINAL CONTROL</em></div>
  <div class="hdr-r">
    <div class="signal-row" style="border:none;padding:0">
      <div class="signal-dot" id="sig-dot"></div>
      <span class="signal-txt" id="sig-txt">신호등: --</span>
    </div>
    <div class="conn"><div class="dot" id="conn-dot"></div><span id="conn-lbl">OFFLINE</span></div>
    <div class="clock" id="clock"></div>
  </div>
</header>

<div class="layout">

  <!-- ── LEFT: Pi 카메라 (라인트래킹) ── -->
  <div class="col">
    <div class="sec">Pi Camera · Line Tracking</div>
    <div class="cam-box">
      <img src="/picam_feed" alt="Pi Cam">
      <div class="cam-label">LINE FOLLOW CAM</div>
    </div>
    <div class="stat-bar">
      <div class="s"><div class="k">ERROR</div><div class="v" id="s-err">--<span class="u">px</span></div></div>
      <div class="s"><div class="k">GREEN</div><div class="v" id="s-green">--<span class="u">px</span></div></div>
      <div class="s"><div class="k">분기점</div><div class="v g" id="s-junc">0</div></div>
      <div class="s"><div class="k">ACTION</div><div class="v c" id="s-action" style="font-size:12px">--</div></div>
    </div>
    <div class="strip">
      <div class="strip-row">
        <span class="lbl">WEIGHT</span>
        <span id="s-weight" style="font-family:var(--label);font-size:14px;color:var(--text0)">-- g</span>
        <button class="btn" onclick="fetch('/tare',{method:'POST'})">TARE</button>
        <span class="lbl" style="margin-left:8px">PITCH</span>
        <span id="s-pitch" style="font-family:var(--label);font-size:14px;color:var(--cyan)">--°</span>
        <span class="lbl" style="margin-left:8px">DIST</span>
        <span id="s-dist" style="font-family:var(--label);font-size:14px;color:var(--green)">-- cm</span>
      </div>
      <div class="strip-row">
        <span class="lbl">CONFIG</span>
        <label style="font-size:9px;color:var(--text2)">THRESH <span id="v-thresh">43</span></label>
        <input type="range" min="20" max="200" value="43" style="width:80px;accent-color:var(--cyan)"
          oninput="document.getElementById('v-thresh').textContent=this.value;sendCfg('thresh',+this.value)">
        <label style="font-size:9px;color:var(--text2)">SPEED <span id="v-speed">45</span></label>
        <input type="range" min="10" max="80" value="45" style="width:80px;accent-color:var(--cyan)"
          oninput="document.getElementById('v-speed').textContent=this.value;sendCfg('base_speed',+this.value)">
      </div>
    </div>
  </div>

  <!-- ── MID: USB 웹캠 (장애물/신호등) ── -->
  <div class="col">
    <div class="sec">USB Webcam · Obstacle + Signal</div>
    <div class="cam-box">
      <img src="/webcam_feed" alt="Webcam">
      <div class="cam-label">OBSTACLE / SIGNAL CAM</div>
      <div class="signal-badge unknown" id="sig-badge">신호 대기</div>
    </div>
    <div class="stat-bar">
      <div class="s"><div class="k">FSM STATE</div>
        <div class="v c" id="s-fsm" style="font-size:10px">IDLE</div></div>
      <div class="s"><div class="k">ROUTE</div>
        <div class="v c" id="s-route">--</div></div>
      <div class="s"><div class="k">SPEED</div>
        <div class="v" id="s-speed">0<span class="u">%</span></div></div>
      <div class="s"><div class="k">MODE</div>
        <div class="v g" id="s-mode">SAFE</div></div>
    </div>
    <div class="strip">
      <div class="strip-row">
        <span class="lbl">MOCK OBS</span>
        <button class="btn yellow" onclick="sendCfg('mock_obs','bump_1cm')">1cm 방지턱</button>
        <button class="btn orange" onclick="sendCfg('mock_obs','bump_2cm')">2cm 방지턱</button>
        <button class="btn" onclick="sendCfg('mock_obs','none')">리셋</button>
      </div>
      <div class="strip-row">
        <span class="lbl">FPS</span>
        <span id="s-fps" style="font-family:var(--label);font-size:14px;color:var(--text1)">--</span>
      </div>
    </div>
  </div>

  <!-- ── RIGHT: 제어 패널 ── -->
  <div class="rpanel">

    <!-- 경로 표시 -->
    <div class="route-box">
      <div class="route-main" id="route-big">A</div>
      <div class="route-sub" id="route-sub">CURRENT ROUTE</div>
    </div>

    <!-- 모드 + 시작/정지 -->
    <div class="ctrl-box">
      <div class="mode-row">
        <button class="mode-btn" id="btn-fast" onclick="setMode('fast')">⚡ FAST</button>
        <button class="mode-btn mode-sel-safe" id="btn-safe" onclick="setMode('safe')">🛡 SAFE</button>
      </div>
      <div class="drive-row">
        <button class="drive-btn" id="btn-start"
          style="border-color:rgba(0,230,118,.5);color:var(--green)"
          onclick="startDrive()">▶ 자율주행 시작</button>
        <button class="drive-btn"
          style="border-color:rgba(255,61,61,.4);color:var(--red)"
          onclick="stopDrive()">■ 정지</button>
      </div>
    </div>

    <!-- AI 판단 -->
    <div class="ai-box">
      <div class="ai-status" id="ai-status">대기중</div>
      <div class="ai-grid">
        <div class="ai-item"><div class="k">Gemini 장애물</div><div class="v" id="g-type">--</div></div>
        <div class="ai-item"><div class="k">신뢰도</div><div class="v" id="g-conf">--</div></div>
        <div class="ai-item"><div class="k">XGBoost</div><div class="v" id="xgb-lbl">--</div></div>
        <div class="ai-item"><div class="k">무게 판정</div><div class="v" id="weight-judge">--</div></div>
      </div>
    </div>

    <!-- 자연어 채팅 -->
    <div class="chat-box">
      <div class="sec" style="padding:0 0 6px;border:none">자연어 명령</div>
      <div class="chat-row">
        <input class="chat-in" id="cmd-in"
          placeholder="예: 빠르게 가줘 / 물건 조심해서"
          onkeydown="if(event.key==='Enter')sendCmd()">
        <button class="chat-send" onclick="sendCmd()">전송</button>
      </div>
      <div class="chat-result" id="cmd-result">--</div>
    </div>

    <!-- 판단 로그 -->
    <div class="sec">판단 로그</div>
    <div class="log-box" id="log-box"></div>

  </div>
</div>

<div id="toast"></div>

<script>
'use strict';

// ── 폴링 기반 (SocketIO 없이 HTTP 폴링) ────────────────────────────────────
let prevRoute = 'A';

function poll() {
  fetch('/state').then(r=>r.json()).then(d=>{
    // 연결 표시
    document.getElementById('conn-dot').classList.add('live');
    document.getElementById('conn-lbl').textContent = 'ONLINE';

    // 센서
    const dist = d.distance ?? -1;
    const distEl = document.getElementById('s-dist');
    distEl.textContent = dist >= 0 ? dist.toFixed(1)+' cm' : '-- cm';
    distEl.style.color = dist>=0&&dist<15 ? 'var(--red)' : dist>=0&&dist<40 ? 'var(--yellow)' : 'var(--green)';

    const wt = d.weight_g ?? 0;
    const wtEl = document.getElementById('s-weight');
    wtEl.textContent = wt.toFixed(1)+' g';
    wtEl.style.color = wt>300?'var(--red)':wt>150?'var(--yellow)':'var(--text0)';

    const pt = d.pitch ?? 0;
    document.getElementById('s-pitch').textContent = pt+'°';

    document.getElementById('s-err').innerHTML   = (d.error??0)+'<span class="u">px</span>';
    document.getElementById('s-green').innerHTML = (d.green_area??0)+'<span class="u">px</span>';
    document.getElementById('s-junc').textContent = d.junction_count ?? 0;
    document.getElementById('s-action').textContent = d.action || '--';
    document.getElementById('s-fps').textContent = (d.fps||'--')+' fps';

    // FSM / 경로
    document.getElementById('s-fsm').textContent  = d.ai_status || 'IDLE';
    document.getElementById('s-route').textContent = d.route || '--';
    document.getElementById('s-mode').textContent  = (d.user_mode||'safe').toUpperCase();

    // 경로 큰 표시
    const routeBig = document.getElementById('route-big');
    const nr = d.route || 'A';
    if (nr !== prevRoute) {
      routeBig.classList.add('changed');
      setTimeout(()=>routeBig.classList.remove('changed'), 2000);
      prevRoute = nr;
      showToast('경로 변경: '+nr, 'warn');
    }
    routeBig.textContent = nr;

    // 신호등
    const sig = d.signal || 'UNKNOWN';
    updateSignal(sig);

    // AI
    const aiEl = document.getElementById('ai-status');
    const aiS  = d.ai_status || '대기중';
    aiEl.textContent = aiS;
    aiEl.className   = 'ai-status' +
      (aiS.includes('분석')||aiS.includes('대기') ? ' warn' :
       aiS.includes('NORMAL')||aiS.includes('완료') ? ' ok' :
       aiS.includes('BLOCK')||aiS.includes('오류') ? ' err' : '');

    document.getElementById('g-type').textContent = d.gemini_type || '--';
    document.getElementById('g-conf').textContent = d.gemini_conf || '--';

    const xEl = document.getElementById('xgb-lbl');
    const xl  = d.xgb_label || '--';
    xEl.textContent = xl;
    xEl.className   = 'v'+(xl==='pass'?' pass':xl==='cautious'?' cautious':xl==='detour'?' detour':'');

    // 무게 판정
    const wj = wt >= 150 ? '과적 (150g+)' : '정상';
    document.getElementById('weight-judge').textContent = wj;
    document.getElementById('weight-judge').style.color = wt>=150?'var(--yellow)':'var(--green)';

    // 명령 결과
    const cr = d.cmd_result;
    if (cr && cr !== '--') {
      const crEl = document.getElementById('cmd-result');
      crEl.textContent = cr;
      crEl.style.color = cr.includes('FAST') ? 'var(--orange)'
                       : cr.includes('SAFE') ? 'var(--cyan)' : 'var(--text1)';
    }

    // 모드 버튼
    const mode = (d.user_mode||'safe').toLowerCase();
    document.getElementById('btn-fast').className = 'mode-btn'+(mode==='fast'?' mode-sel-fast':'');
    document.getElementById('btn-safe').className = 'mode-btn'+(mode==='safe'?' mode-sel-safe':'');

    // 시작 버튼 펄스
    document.getElementById('btn-start').className = 'drive-btn'+(d.running?' start-run':'');

  }).catch(()=>{
    document.getElementById('conn-dot').classList.remove('live');
    document.getElementById('conn-lbl').textContent = 'OFFLINE';
  });
}

function pollLogs() {
  fetch('/logs').then(r=>r.json()).then(logs=>{
    const box = document.getElementById('log-box');
    box.innerHTML = logs.map(e =>
      `<div class="log-entry ${e.level}">
        <span class="log-t">${e.time}</span>
        <span class="log-m">${e.msg}</span>
      </div>`
    ).join('');
  }).catch(()=>{});
}

setInterval(poll, 300);
setInterval(pollLogs, 500);
poll(); pollLogs();

// 신호등 표시
function updateSignal(sig) {
  const dot   = document.getElementById('sig-dot');
  const txt   = document.getElementById('sig-txt');
  const badge = document.getElementById('sig-badge');
  dot.className = 'signal-dot';
  if (sig === 'GO') {
    dot.classList.add('go');
    txt.textContent  = '신호등: GO';
    badge.textContent = 'GO';
    badge.className   = 'signal-badge go';
  } else if (sig === 'STOP') {
    dot.classList.add('stop');
    txt.textContent  = '신호등: STOP';
    badge.textContent = 'STOP';
    badge.className   = 'signal-badge stop';
  } else {
    dot.classList.add('unknown');
    txt.textContent  = '신호등: 감지 중';
    badge.textContent = '신호 대기';
    badge.className   = 'signal-badge unknown';
  }
}

// 제어
function setMode(mode) {
  fetch('/mode',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode})});
}
function startDrive() {
  fetch('/start',{method:'POST'}).then(r=>r.json()).then(d=>{
    showToast('출발 준비 — 신호등 대기 중', 'info');
  });
}
function stopDrive() {
  fetch('/stop',{method:'POST'});
  showToast('정지 명령 전송', 'warn');
}
function sendCmd() {
  const text = document.getElementById('cmd-in').value.trim();
  if (!text) return;
  document.getElementById('cmd-result').textContent = 'Gemini 판단 중...';
  fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({text})});
}
function sendCfg(k,v) {
  fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({[k]:v})});
}

// 시계
function updateClock() {
  document.getElementById('clock').textContent =
    new Date().toLocaleTimeString('ko-KR',{hour12:false});
}
setInterval(updateClock,1000); updateClock();

// Toast
let _tt = null;
function showToast(msg, type='info') {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className   = 'show '+type;
  if (_tt) clearTimeout(_tt);
  _tt = setTimeout(()=>el.classList.remove('show'), 2400);
}
</script>
</body>
</html>'''

if __name__ == '__main__':
    print("=" * 55)
    print("  INSITE AGV FINAL DASHBOARD")
    print("  URL: http://192.168.0.50:5003")
    print("=" * 55)
    app.run(host='0.0.0.0', port=5003, threaded=True)