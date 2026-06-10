# app/auto_dashboard_v5_AI.py
# 자율 주행 대시보드 - 양방향 카메라 동시 송출 버그 완치 및 캘리브레이션 통합본
#
# 실행: python app/auto_dashboard_v5_AI.py
# 접속: http://192.168.0.50:5004

import sys, os, time, threading, copy, warnings
from typing import Any
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

from sensors.camera import Camera
from sensors.motor  import MotorController

app = Flask(__name__)

# ── Pi 카메라 ──────────────────────────────────────────
try:
    cam = Camera(width=320, height=240)
    CAM_AVAILABLE = True
    print("[OK] Pi 카메라")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] Pi 카메라: {e}")

# ── USB 웹캠 ───────────────────────────────────────────
webcam = None
WEBCAM_AVAILABLE = False
for idx in [2, 0, 1, 4]:
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if cap.isOpened():
        webcam = cap
        webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        WEBCAM_AVAILABLE = True
        print(f"[OK] USB 웹캠 (인덱스: {idx})")
        break
    cap.release()
if not WEBCAM_AVAILABLE:
    print("[SKIP] USB 웹캠 없음")


# ── IMU ────────────────────────────────────────────────
try:
    from sensors.mpu6050 import MPU6050
    imu = MPU6050()
    time.sleep(0.5)
    _baseline = imu.get_accel()
    IMU_AVAILABLE = True
    print(f"[OK] IMU 기준값: x={_baseline['x']:.3f}")
except Exception as e:
    imu = None
    _baseline = {'x': 0.0}
    IMU_AVAILABLE = False
    print(f"[SKIP] IMU: {e}")

# ── 초음파 ────────────────────────────────────────────
try:
    from gpiozero import DistanceSensor
    import signal
    def _timeout(s, f): raise TimeoutError()
    signal.signal(signal.SIGALRM, _timeout)
    signal.alarm(3)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
    signal.alarm(0)
    ULTRA_AVAILABLE = True
    print("[OK] 초음파")
except Exception as e:
    ultra = None
    ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파: {e}")

# ── 로드셀 HX711 ───────────────────────────────────────
try:
    from sensors.hx711 import HX711
    hx711 = HX711()
    import time as _t; _t.sleep(1)
    hx711.tare(samples=10)
    HX711_AVAILABLE = True
    print("[OK] 로드셀 HX711")
except Exception as e:
    hx711 = None
    HX711_AVAILABLE = False
    print(f"[SKIP] 로드셀: {e}")

def get_weight_g() -> float:
    """현재 무게 반환 (g). 센서 없으면 0.0"""
    if hx711 is None:
        return 0.0
    try:
        w = hx711.get_weight(times=3)
        return max(0.0, w)
    except:
        return 0.0

def get_speed_for_weight(base_speed: int, weight_g: float) -> int:
    """
    무게 구간별 통과 속도 제한
    0 ~ 150g   → base_speed 유지
    150 ~ 300g → base_speed * 0.7
    300g 초과  → base_speed * 0.5
    """
    if weight_g < 150:
        return base_speed
    elif weight_g < 300:
        return max(20, int(base_speed * 0.7))
    else:
        return max(20, int(base_speed * 0.5))

# ── 모터 ──────────────────────────────────────────────
motor = MotorController()

# ── AI 엔진 (지연 로드) ───────────────────────────────
# AI_AVAILABLE = True
# print("[OK] AI 엔진 (지연 로드)")
try:
    from ai_core.engine import AGVAIEngine
    _ai_engine = AGVAIEngine()
    AI_AVAILABLE = True
    print("[OK]  AI 사용 (엔진 미로드)")
except Exception as e:
    _ai_engine = None
    AI_AVAILABLE = False
    print(f"[SKIP] AI 엔진: {e}")

# ══════════════════════════════════════════════════════
# 라우트 정의
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
    "spin_speed":     30,
    "thresh":         43,
    "roi_top":        0.7,
    "dead_zone":      10,
    "kp":             3.0,
    "ki":             0.002,
    "forward_time":   0.5,
    "slope_speed":    60,
    "green_h_min":    30,
    "green_h_max":    85,
    "green_s_min":    80,
    "green_v_min":    50,
    "green_min_area": 200,
    "running":        False,
    "route":          "A",       # 항상 A로 시작
    "user_mode":      "safe",    # fast / safe
}

state = {
    "action":         "정지",
    "error":          0,
    "green_area":     0,
    "junction_count": 0,
    "route":          "A",
    "distance":       -1,
    "fps":            0,
    "lost":           False,
    # AI 판단
    "ai_status":      "대기중",
    "gemini_type":    "--",
    "gemini_height":  "--",
    "gemini_conf":    "--",
    "xgb_label":      "--",
    "ai_action":      "--",
    "cmd_result":     "--"
}

latest_picam  = None
latest_webcam = None

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
    motor.set_motor(1, 1, speed)
    motor.set_motor(2, 1, speed)
    motor.set_motor(3, 1, speed)
    motor.set_motor(4, 1, speed)

def turn_left(left_speed, right_speed):
    motor.set_motor(4, 1, left_speed)
    motor.set_motor(3, 1, left_speed)
    motor.set_motor(2, 1, right_speed)
    motor.set_motor(1, 1, right_speed)

def turn_right(left_speed, right_speed):
    motor.set_motor(4, 1, left_speed)
    motor.set_motor(3, 1, left_speed)
    motor.set_motor(2, 1, right_speed)
    motor.set_motor(1, 1, right_speed)

def spin_left(speed):
    motor.set_motor(4, -1, speed)
    motor.set_motor(3, -1, speed)
    motor.set_motor(2,  1, speed)
    motor.set_motor(1,  1, speed)

def spin_right(speed):
    motor.set_motor(4,  1, speed)
    motor.set_motor(3,  1, speed)
    motor.set_motor(2, -1, speed)
    motor.set_motor(1, -1, speed)

# ══════════════════════════════════════════════════════
# 라인 + 초록 감지
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
    cv2.line(debug, (0, roi_y), (w, roi_y), (255,180,0), 2)
    cv2.line(debug, (w//2, roi_y), (w//2, h), (0,80,255), 1)
    dz = cfg["dead_zone"]
    cv2.line(debug, (w//2-dz, roi_y), (w//2-dz, h), (80,80,255), 1)
    cv2.line(debug, (w//2+dz, roi_y), (w//2+dz, h), (80,80,255), 1)

    if cx is not None:
        cv2.circle(debug, (cx, cy), 8, (0,255,80), -1)
        err = cx - w//2
        cv2.putText(debug, f"err={err:+d}", (5, roi_y-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,80), 1)

    if is_junction:
        cnts, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, cnts, -1, (0,255,0), 2)
        cv2.putText(debug, f"GREEN! {green_area}px", (5,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

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
        return

    if action == "straight":
        go_forward(cfg["base_speed"])
        time.sleep(0.4)
        return

    if action in ("left", "right"):
        spin_fn = spin_left if action == "left" else spin_right
        spin_fn(30)
        time.sleep(1.2)
        fine_timeout = time.time() + 2.0
        while time.time() < fine_timeout:
            if cam is not None:
                frame = cam.capture()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                break
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
    print(f"[Reroute] → {new_route}")
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
    print(f"[Reroute] {new_route} 재출발")

# ══════════════════════════════════════════════════════
# AI 판단
# ══════════════════════════════════════════════════════
def run_ai_decision(cur_route: str, frame_snap):
    if not AI_AVAILABLE:
        config["running"] = True
        return

    try:
        with lock:
            state["ai_status"] = "분석중..."

        engine = _ai_engine
        engine.mode = config.get("user_mode", "safe").upper()
        # 현재 경로 강제 설정 (engine 내부 초기화 방지)
        route_key = cur_route.split("→")[-1] if "→" in cur_route else cur_route
        engine.active_route = f"ROUTE_{route_key}"
        engine._route_initialized = True
        engine.route_availability = {"A": True, "B": True, "C": True}

        pitch = 0.0
        if imu is not None:
            accel = imu.get_accel()
            pitch = float(accel.get("x", 0.0))

        with lock:
            dist = state["distance"]

        sensor_snapshot = {
            "pitch":         pitch,
            "weight_g":      get_weight_g(),
            "distance_cm":   dist,
            "node_trigger":  False,
            "current_route": cur_route,
            "frame":         frame_snap,
        }

        result = engine.evaluate_state_and_calculate_output(sensor_snapshot, frame_snap)

        ai_state  = result.get("state", "--")
        ai_route  = result.get("route", "--")
        ai_speed  = result.get("speed_limit_pct", config["base_speed"])

        # Gemini 결과 추출
        vlm = engine.last_vlm_result if hasattr(engine, 'last_vlm_result') else {}
        gemini_type   = vlm.get("obstacle_type", "--")
        gemini_height = vlm.get("height_cm", "--")
        gemini_conf   = vlm.get("confidence", "--")

        # XGBoost 레이블 추출 (state에서 파싱)
        xgb_label = "pass"
        if "BLOCK" in ai_state or "REROUTE" in ai_state:
            xgb_label = "detour"
        elif "CAUTIOUS" in ai_state:
            xgb_label = "cautious"

        with lock:
            state["ai_status"]    = ai_state
            state["gemini_type"]  = str(gemini_type)
            state["gemini_height"]= str(gemini_height)
            state["gemini_conf"]  = f"{float(gemini_conf)*100:.0f}%" if gemini_conf != "--" else "--"
            state["xgb_label"]    = xgb_label
            state["ai_action"]    = ai_state

        print(f"[AI] {ai_state} | route={ai_route} | speed={ai_speed}")

        if ai_state in ("PATH_BLOCKED", "REROUTING_RUN"):
            route_key = ai_route.replace("ROUTE_", "")
            reroute_key = (cur_route, route_key)
            new_route_str = DETOUR_MAP.get(reroute_key)
            if new_route_str:
                threading.Thread(target=do_reroute, args=(new_route_str,),
                                 daemon=True, name="reroute").start()
            else:
                config["running"] = True

        elif ai_state == "CRITICAL_STOP":
            motor.motorStop()
            config["running"] = False
            with lock:
                state["action"] = "비상정지"

        else:
            original_speed = config["base_speed"]
            if ai_speed and int(ai_speed) < original_speed:
                config["base_speed"] = max(10, int(ai_speed))
                config["running"] = True

                def restore_speed(orig=original_speed):
                    time.sleep(3.0)
                    config["base_speed"] = orig
                    print(f"[AI] 속도 복귀: {orig}%")

                threading.Thread(target=restore_speed, daemon=True).start()
            else:
                config["running"] = True

    except Exception as e:
        import traceback
        print(f"[AI 오류] {e}")
        traceback.print_exc()
        config["running"] = True
        with lock:
            state["ai_status"] = f"오류: {str(e)[:30]}"
    finally:
        drive_loop._ai_running = False
        drive_loop._ai_cooldown = time.time() + 5.0

# def run_ai_decision(cur_route: str, frame_snap):
#     """규칙 기반 장애물 판단"""
#     try:
#         with lock:
#             state["ai_status"] = "분석중..."

#         # # Gemini로 장애물 높이 측정
#         # height_cm = 1.0
#         # obs_type  = "1cm"
#         # with lock:
#         #     state["gemini_type"]   = obs_type
#         #     state["gemini_height"] = str(height_cm)
#         #     state["gemini_conf"]   = "80%"
#         try:
#             import cv2 as _cv2
#             from ai_core.vlm_client import VLMClient
#             vlm = VLMClient()
#             _, jpeg = _cv2.imencode('.jpg', frame_snap, [_cv2.IMWRITE_JPEG_QUALITY, 85])
#             result = vlm.analyze(jpeg.tobytes())
#             height_cm = float(result.get("height_cm", 0.0))
#             obs_type  = result.get("obstacle_type", "none")
#             with lock:
#                 state["gemini_type"]   = obs_type
#                 state["gemini_height"] = str(height_cm)
#                 state["gemini_conf"]   = f"{float(result.get('confidence',0))*100:.0f}%"
#         except Exception as e:
#             print(f"[VLM] 오류: {e}")

#         mode   = config.get("user_mode", "safe").lower()
#         weight = get_weight_g()
#         route  = cur_route.split("→")[-1] if "→" in cur_route else cur_route
#         heavy  = weight >= 130.0

#         # 장애물 분류
#         if obs_type == "none" or height_cm == 0.0:
#             bump = "none"
#         elif height_cm <= 1.2:
#             bump = "1cm"
#         else:
#             bump = "2cm"

#         print(f"[Rule] mode={mode} route={route} weight={weight:.0f}g heavy={heavy} bump={bump}")

#         # ── 규칙표 적용 ──────────────────────────────────────
#         action = "pass"
#         speed  = config["base_speed"]

#         if mode == "fast":
#             if not heavy:
#                 if route == "A":
#                     if bump == "none":   action, speed = "pass",   60
#                     elif bump == "1cm":  action = "detour"
#                     elif bump == "2cm":  action = "detour"
#                 elif route == "B":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action, speed = "pass",   70
#                     elif bump == "2cm":  action = "detour"
#                 elif route == "C":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action, speed = "pass",   70
#                     elif bump == "2cm":  action = "stop"
#             else:  # heavy
#                 if route == "A":
#                     action = "detour"
#                 elif route == "B":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action = "detour"
#                     elif bump == "2cm":  action = "detour"
#                 elif route == "C":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action, speed = "pass",   70
#                     elif bump == "2cm":  action = "stop"

#         else:  # safe
#             if not heavy:
#                 if route == "B":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action, speed = "pass",   70
#                     elif bump == "2cm":  action = "detour"
#                 elif route == "C":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action, speed = "pass",   70
#                     elif bump == "2cm":  action = "stop"
#             else:  # heavy
#                 if route == "B":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action = "detour"
#                     elif bump == "2cm":  action = "detour"
#                 elif route == "C":
#                     if bump == "none":   action, speed = "pass",   config["base_speed"]
#                     elif bump == "1cm":  action, speed = "pass",   70
#                     elif bump == "2cm":  action = "stop"

#         print(f"[Rule] action={action} speed={speed}")

#         with lock:
#             state["xgb_label"] = "pass" if action == "pass" else \
#                                   "detour" if action == "detour" else "cautious"
#             state["ai_status"] = "CAUTIOUS_BUMP"    if action == "pass" else \
#                                   "REROUTING_RUN"    if action == "detour" else \
#                                   "CAUTIOUS_SLOWDOWN"

#         # ── 행동 실행 ─────────────────────────────────────────
#         DETOUR_MAP_RULE = {"A": "B", "B": "C"}

#         if action == "detour":
#             next_r = DETOUR_MAP_RULE.get(route)
#             if next_r:
#                 new_route_str = f"{route}→{next_r}"
#                 threading.Thread(target=do_reroute, args=(new_route_str,),
#                                  daemon=True, name="reroute").start()
#             else:
#                 config["running"] = True

#         elif action == "stop":
#             motor.motorStop()
#             config["running"] = False
#             with lock:
#                 state["action"] = "최종정지"

#         else:  # pass
#             original_speed = config["base_speed"]
#             config["base_speed"] = speed
#             config["running"] = True

#             def restore_speed(orig=original_speed):
#                 time.sleep(3.0)
#                 config["base_speed"] = orig
#                 print(f"[Rule] 속도 복귀: {orig}%")

#             threading.Thread(target=restore_speed, daemon=True).start()

#     except Exception as e:
#         import traceback
#         print(f"[Rule 오류] {e}")
#         traceback.print_exc()
#         config["running"] = True
#     finally:
#         drive_loop._ai_running = False
#         drive_loop._ai_cooldown = time.time() + 5.0

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

threading.Thread(target=weight_loop, daemon=True, name="weight").start()

def webcam_loop():
    global latest_webcam
    if webcam is None or not WEBCAM_AVAILABLE:
        print("[SKIP] 웹캠 없음")
        return
    while True:
        ret, frame = webcam.read()
        if ret:
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_webcam = jpeg.tobytes()
        else:
            time.sleep(0.1)
            continue
        time.sleep(0.033)

threading.Thread(target=ultra_loop,  daemon=True, name="ultra").start()
threading.Thread(target=webcam_loop, daemon=True, name="webcam").start()

# ══════════════════════════════════════════════════════
# 주행 루프
# ══════════════════════════════════════════════════════
# ── [추가] OpenCV 픽셀 감포초 + Gemini 3색 신호등 하이브리드 엔진 ──────────────────
def check_traffic_signal_before_start():
    """OpenCV 픽셀 필터링으로 토큰을 95% 절약하는 고효율 3색 신호등 관제 스레드"""
    if not hasattr(check_traffic_signal_before_start, '_active'):
        check_traffic_signal_before_start._active = True

    print("[Signal Check] 하이브리드 토큰 절약형 3색 신호등 시퀀스를 개시합니다.")
    
    try:
        from ai_core.signal_detector import SignalDetectorVLM
        detector = SignalDetectorVLM()
    except Exception as e:
        print(f"[Signal Check] 신호등 모듈 없음 → 즉시 출발: {e}")
        config["running"] = True
        return

    with lock:
        state["ai_status"] = "🚦 CV 픽셀 모니터링 중..."
        state["action"] = "신호등 대기"

    while True:
        # 사용자가 대시보드에서 [정지] 버튼 클릭 시 신호등 스캔 루프 즉각 탈출 브레이크
        if not getattr(check_traffic_signal_before_start, '_active', False):
            print("[Signal Check] 운영자 명령으로 신호등 시퀀스가 중단되었습니다.")
            return

        with lock:
            img_bytes = latest_webcam

        if img_bytes is not None:
            # ── [STEP 1] 비용이 안 드는 로컬 OpenCV 픽셀 필터링 선제 가동 ──
            import numpy as np
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            _, img_encoded = cv2.imencode('.jpg', frame)
            signal = detector.detect_via_cv(img_encoded.tobytes())
            if signal == "green_suspect":
                color = "green"
            elif signal == "yellow_suspect":
                color = "yellow"
            elif signal == "red_suspect":
                color = "red"
            else:
                color = "unknown"
            conf  = 1.0
            
            print(f"[Signal Check VLM] 교차 검증 결과 ➡️ {color.upper()} ({conf*100:.0f}%)")
            
            if color in ("red", "yellow"):
                # 빨간불 혹은 노란불인 경우: 절대 출발하지 않고 화면 표시만 갱신하며 루프 유지
                with lock:
                    state["ai_status"] = f"신호등: {color.upper()} (대기중)"
                    state["action"] = f"신호정차({color.upper()})"
                motor.motorStop() # 이중 하드웨어 브레이크 고정
                time.sleep(1.5) # 빨간불일 때는 다음 VLM 호출까지 자중하며 1.5초 대기
                
            elif color == "green":
                # 오직 초록불인 경우에만 자율주행 락을 해제하고 시퀀스 종료
                print("[Signal Check] 🟢 완벽한 초록불 확인! AGV 주행을 승인합니다.")
                with lock:
                    state["action"] = "직진 주행"
                    state["ai_status"] = "NORMAL_CRUISE"
                config["running"] = True # 주행 루프 엑셀레이터 ON!
                return
                
            else:
                with lock:
                    state["ai_status"] = "🚦 신호 불분명 (대기)"
                    
        time.sleep(0.1)

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

    while True:
        cfg = copy.deepcopy(config)

        # Pi 카메라 캡처
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

        # 정지 상태
        if not cfg["running"]:
            if not stopped:
                motor.motorStop(); stopped = True
            integral = 0.0
            with lock:
                state["action"] = "정지"
                state["green_area"] = green_area
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
            green_seen = True
            junction_cooldown = now + 3.0
            jc = increment_junction()
            integral = 0.0
            action_at = ROUTES[current_route].get(jc, "straight")
            print(f"[Junction] #{jc} | {current_route} | {action_at}")
            with lock:
                state["junction_count"] = jc
                state["action"] = f"분기점#{jc} {action_at}"
            execute_junction(action_at, cfg)
            continue

        if not is_junction:
            green_seen = False

        on_slope = False
        if imu is not None:
            accel = imu.get_accel()
            on_slope = abs(accel['x'] - _baseline['x']) > 0.3

        # 초음파 < 20cm → AI 판단
        with lock:
            dist_now = state["distance"]

        if (cfg["running"] and 0 < dist_now < 22 and on_slope and
                not getattr(drive_loop, '_ai_running', False) and
                time.time() > getattr(drive_loop, '_ai_cooldown', 0)):
            drive_loop._ai_running = True
            config["running"] = False
            motor.motorStop()
            time.sleep(0.5)

            #웹캠캡쳐
            # frame_snap = frame.copy() if frame is not None else np.zeros((240,320,3), np.uint8)
            #picamera 캡쳐
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
        correction = 0
        action = "직진"

        # on_slope = False
        # if imu is not None:
        #     accel = imu.get_accel()
        #     on_slope = abs(accel['x'] - _baseline['x']) > 0.15

        if on_slope:
            go_forward(cfg["slope_speed"])
            action = "경사직진"
        elif cx is None:
            integral = 0.0
            action = f"탐색({'우' if lost_dir>0 else '좌'})"
            spin_right(cfg["spin_speed"]) if lost_dir>0 else spin_left(cfg["spin_speed"])
        else:
            error = cx - CENTER_X
            integral = max(min(integral + error*dt, 200), -200)
            correction = int(error*cfg["kp"] + integral*cfg["ki"])
            correction = max(min(correction, cfg["base_speed"]-10), -(cfg["base_speed"]-10))
            lost_dir = 1 if error>0 else (-1 if error<0 else lost_dir)

            if abs(error) < cfg["dead_zone"]:
                go_forward(cfg["base_speed"])
                action = "직진"
            elif correction > 0:
                if abs(error) > 80:
                    spin_right(cfg["spin_speed"])
                    action = f"급우회전({abs(error)})"
                else:
                    right_speed = max(cfg["base_speed"] - abs(correction), cfg["turn_speed"])
                    turn_right(cfg["base_speed"], right_speed)
                    action = f"우회전({abs(correction)})"
            else:
                if abs(error) > 80:
                    spin_left(cfg["spin_speed"])
                    action = f"급좌회전({abs(error)})"
                else:
                    left_speed = max(cfg["base_speed"] - abs(correction), cfg["turn_speed"])
                    turn_left(left_speed, cfg["base_speed"])
                    action = f"좌회전({abs(correction)})"

            # if abs(error) < cfg["dead_zone"]:
            #     go_forward(cfg["base_speed"]); action = "직진"
            # elif correction > 0:
            #     rs = max(cfg["base_speed"]-abs(correction), cfg["turn_speed"])
            #     turn_right(cfg["base_speed"], rs); action = f"우회전({abs(correction)})"
            # else:
            #     ls = max(cfg["base_speed"]-abs(correction), cfg["turn_speed"])
            #     turn_left(ls, cfg["base_speed"]); action = f"좌회전({abs(correction)})"

        with lock:
            state["error"]  = cx - CENTER_X if cx else 0
            state["action"] = action
            state["lost"]   = cx is None
            state["route"]  = current_route
            state["green_area"] = green_area
            state["junction_count"] = get_junction_count()

        time.sleep(0.03)

threading.Thread(target=drive_loop, daemon=True, name="drive").start()

# ══════════════════════════════════════════════════════
# Flask
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

@app.route('/start', methods=['POST'])
def start():
    reset_junction()
    
    # 모드에 따라 경로 자동 설정
    mode  = config.get('user_mode', 'safe')
    route = "A" if mode == "fast" else "B"
    config['route'] = route

    # 엔진 초기 경로 설정
    if _ai_engine is not None:
        _ai_engine.mode = mode.upper()
        _ai_engine.active_route = f"ROUTE_{route}"
        _ai_engine._route_initialized = True
        _ai_engine._heavy_reroute_logged = False
        _ai_engine.route_availability = {"A": True, "B": True, "C": True}

    config['running'] = False

    check_traffic_signal_before_start._active = True
    threading.Thread(target=check_traffic_signal_before_start, daemon=True, name="signal_check").start()

    with lock:
        state["route"]          = route
        state["junction_count"] = 0
        state["ai_status"]      = "신호등 스캔 중..."
        state["action"]         = "신호대기정차"
        state["gemini_type"]    = "--"
        state["gemini_height"]  = "--"
        state["gemini_conf"]    = "--"
        state["xgb_label"]      = "--"
    return jsonify({'status': 'running'})

@app.route('/manual_control', methods=['POST'])
def manual_control():
    data = request.get_json(force=True)
    direction = data.get('direction', 'stop')
    speed = config.get('base_speed', 45)
    if direction == "forward":
        go_forward(speed)
    elif direction == "backward":
        motor.set_motor(1,-1,speed); motor.set_motor(2,-1,speed)
        motor.set_motor(3,-1,speed); motor.set_motor(4,-1,speed)
    elif direction == "left":
        spin_left(35)
    elif direction == "right":
        spin_right(35)
    else:
        motor.motorStop()
    return jsonify({'status': 'ok', 'direction': direction})

@app.route('/stop', methods=['POST'])
def stop():
    config['running'] = False
    # 신호등 감지 루프 가동 중이었다면 즉시 인터럽트 차단 플래그 OFF
    check_traffic_signal_before_start._active = False
    motor.motorStop()
    return jsonify({'status': 'stopped'})

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json(force=True)
    user_text = data.get('text', '').strip()
    if not user_text:
        return jsonify({'status': 'error', 'msg': '텍스트 없음'})

    def _parse():
                try:
                    from ai_core.commander import CommanderVLM
                    cmd = CommanderVLM()
                    result = cmd.parse(user_text)
                    mode  = result.get('mode', 'safe')
                    route = result.get('route', 'B')
                    reason= result.get('reason', '')
                    
                    # ── [방어적 코드 주입] 제미나이의 "ROUTE_C", "c" 등의 오판 완전 세척 ──
                    if isinstance(route, str):
                        route = route.replace("ROUTE_", "").strip().upper()
                        if route not in ("A", "B", "C"):
                            route = "C"  # 이상치 유입 시 가장 안전한 C경로로 강제 안전망 폴백
                    
                    config['user_mode'] = mode
                    if not config['running']:
                        config['route'] = route
                        with lock:
                            state['route'] = route
                    with lock:
                        state['cmd_result'] = f"[{mode.upper()}] {reason}"
                    # 모드 버튼 업데이트를 위해 state에 저장
                    state['user_mode'] = mode
                    print(f"[Commander] mode={mode} route={route}")
                    # 엔진 초기 경로도 같이 설정
                    if _ai_engine is not None:
                        _ai_engine.mode = mode.upper()
                        _ai_engine.active_route = f"ROUTE_{route}"
                        _ai_engine._route_initialized = True
                        _ai_engine._heavy_reroute_logged = False
                        _ai_engine.route_availability = {"A": True, "B": True, "C": True}
                except Exception as e:
                    import traceback
                    print(f"[AI 오류] {e}")
                    traceback.print_exc()
                    with lock:
                        state['cmd_result'] = f"오류: {str(e)[:40]}"
                        
    threading.Thread(target=_parse, daemon=True).start()
    return jsonify({'status': 'ok', 'msg': '판단 중...'})

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get('mode', 'safe').lower()
    if mode in ('fast', 'safe'):
        config['user_mode'] = mode
    return jsonify({'status': 'ok', 'mode': config['user_mode']})

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

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AWR Auto Drive</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&family=Syne:wght@700;800&display=swap');
  :root {
    --bg:#080c14; --panel:#0d1220; --border:#182038;
    --cyan:#00e5ff; --green:#00ff9d; --yellow:#ffd600;
    --red:#ff3d57; --orange:#ff9500; --dim:#2a3a5a;
    --text:#cdd6f4; --sub:#5a6a8a;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--text);font-family:'JetBrains Mono',monospace;min-height:100vh;}
  body::before{
    content:'';position:fixed;inset:0;pointer-events:none;z-index:0;
    background-image:linear-gradient(rgba(0,229,255,.02) 1px,transparent 1px),
                     linear-gradient(90deg,rgba(0,229,255,.02) 1px,transparent 1px);
    background-size:40px 40px;
  }

  /* 헤더 */
  header{
    position:relative;z-index:10;
    background:var(--panel);border-bottom:1px solid var(--border);
    padding:14px 24px;display:flex;align-items:center;justify-content:space-between;
  }
  .logo{font-family:'Syne',sans-serif;font-size:1.1rem;font-weight:800;
        color:var(--cyan);letter-spacing:3px;}
  .header-right{display:flex;align-items:center;gap:20px;}
  .fps-badge{font-size:.7rem;color:var(--dim);}

  /* 모드 토글 */
  .mode-toggle{display:flex;gap:4px;background:rgba(255,255,255,.04);
               border:1px solid var(--border);border-radius:8px;padding:4px;}
  .mode-btn{padding:7px 18px;border:none;border-radius:6px;
            font-family:'JetBrains Mono',monospace;font-size:.75rem;font-weight:700;
            cursor:pointer;transition:all .2s;background:transparent;
            color:var(--sub);text-transform:uppercase;letter-spacing:1px;}
  .mode-btn.active-safe{background:var(--cyan);color:#000;box-shadow:0 0 20px rgba(0,229,255,.3);}
  .mode-btn.active-fast{background:var(--orange);color:#000;box-shadow:0 0 20px rgba(255,149,0,.3);}

  /* 레이아웃 */
  .main{position:relative;z-index:1;display:grid;
        grid-template-columns:1fr 1fr 340px;gap:12px;padding:12px;}

  /* 카드 */
  .card{background:var(--panel);border:1px solid var(--border);border-radius:12px;padding:16px;
        position:relative;overflow:hidden;}
  .card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
    background:linear-gradient(90deg,transparent,var(--cyan),transparent);opacity:.3;}
  .card-title{font-size:.65rem;color:var(--sub);letter-spacing:2px;
              text-transform:uppercase;margin-bottom:12px;}

  /* 카메라 */
  .cam-wrap{position:relative;border-radius:8px;overflow:hidden;
            background:#000;border:1px solid var(--border);}
  .cam-wrap img{width:100%;display:block;}
  .cam-overlay{
    position:absolute;bottom:8px;left:8px;right:8px;
    display:flex;justify-content:space-between;align-items:flex-end;
  }
  .dist-tag{
    background:rgba(0,0,0,.8);border:1px solid var(--border);
    border-radius:6px;padding:4px 10px;font-size:.8rem;
  }
  .dist-val{font-weight:700;color:var(--green);}
  .dist-val.warn{color:var(--yellow);}
  .dist-val.danger{color:var(--red);animation:blink .3s step-end infinite;}
  @keyframes blink{50%{opacity:.2}}

  /* 오른쪽 패널 */
  .side{display:flex;flex-direction:column;gap:12px;}

  /* 루트 표시 */
  .route-display{text-align:center;padding:16px;}
  .route-label{font-size:.65rem;color:var(--sub);letter-spacing:2px;margin-bottom:8px;}
  .route-val{font-family:'Syne',sans-serif;font-size:3.5rem;font-weight:800;
             color:var(--cyan);line-height:1;transition:all .3s;}
  .route-val.changed{color:var(--orange);text-shadow:0 0 20px rgba(255,149,0,.5);}

  /* 시작/정지 버튼 */
  .ctrl-row{display:flex;gap:8px;}
  .btn-start{flex:1;padding:14px;border:none;border-radius:8px;cursor:pointer;
             background:linear-gradient(135deg,#006644,var(--green));
             color:#000;font-family:inherit;font-size:.9rem;font-weight:700;
             letter-spacing:1px;transition:all .2s;}
  .btn-start:hover{filter:brightness(1.1);}
  .btn-stop{flex:1;padding:14px;border:none;border-radius:8px;cursor:pointer;
            background:linear-gradient(135deg,#660011,var(--red));
            color:#fff;font-family:inherit;font-size:.9rem;font-weight:700;
            letter-spacing:1px;transition:all .2s;}
  .btn-stop:hover{filter:brightness(1.1);}

  /* AI 패널 */
  .ai-section{display:flex;flex-direction:column;gap:8px;}
  .ai-status-bar{
    padding:8px 12px;border-radius:6px;font-size:.75rem;font-weight:700;
    text-align:center;letter-spacing:1px;
    background:rgba(0,229,255,.08);border:1px solid rgba(0,229,255,.2);
    color:var(--cyan);transition:all .3s;
  }
  .ai-status-bar.analyzing{background:rgba(255,214,0,.08);border-color:rgba(255,214,0,.3);color:var(--yellow);}
  .ai-status-bar.detour{background:rgba(255,61,87,.08);border-color:rgba(255,61,87,.3);color:var(--red);}
  .ai-status-bar.pass{background:rgba(0,255,157,.08);border-color:rgba(0,255,157,.3);color:var(--green);}

  .ai-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;}
  .ai-item{background:rgba(0,0,0,.3);border:1px solid var(--border);
           border-radius:8px;padding:10px;}
  .ai-item .lbl{font-size:.6rem;color:var(--sub);letter-spacing:.5px;margin-bottom:4px;}
  .ai-item .val{font-size:.9rem;font-weight:600;color:var(--cyan);}
  .ai-item .val.pass{color:var(--green);}
  .ai-item .val.cautious{color:var(--yellow);}
  .ai-item .val.detour{color:var(--red);}

  /* 상태 그리드 */
  .status-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px;}
  .stat{background:rgba(0,0,0,.25);border:1px solid var(--border);border-radius:8px;padding:10px;}
  .stat .lbl{font-size:.65rem;color:var(--sub);margin-bottom:4px;}
  .stat .val{font-size:1rem;font-weight:700;color:var(--green);}
  .stat .val.warn{color:var(--yellow);}
  .stat .val.danger{color:var(--red);}

  /* 슬라이더 (숨길 수 있는 고급 설정) */
  details{margin-top:12px;}
  summary{font-size:.7rem;color:var(--sub);cursor:pointer;padding:8px 0;
          letter-spacing:1px;text-transform:uppercase;}
  .sliders{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px;}
  .sl-item label{font-size:.65rem;color:var(--sub);display:block;margin-bottom:3px;}
  .sl-item input{width:100%;accent-color:var(--cyan);}
  .sl-item span{font-size:.75rem;color:var(--cyan);}
</style>
</head>
<body>
<header>
  <div class="logo">AWR AUTO</div>
  <div class="header-right">
    <div class="mode-toggle">
      <button class="mode-btn active-safe" id="btn-safe" onclick="setMode('safe')">🛡 SAFE</button>
      <button class="mode-btn" id="btn-fast" onclick="setMode('fast')">⚡ FAST</button>
    </div>
    <span class="fps-badge" id="fps">-- fps</span>
  </div>
</header>

<div class="main">
  <!-- Pi 카메라 (라인추종용) -->
  <div class="card">
    <div class="card-title">Pi Camera · Line Follow</div>
    <div class="cam-wrap">
      <img src="/picam_feed" alt="Pi Cam">
      <div class="cam-overlay">
        <div class="dist-tag">초음파 <span class="dist-val" id="dist-tag">--</span></div>
        <div style="font-size:.7rem;color:var(--sub)" id="action-tag">--</div>
      </div>
    </div>
    <div class="status-grid">
      <div class="stat"><div class="lbl">ERROR (px)</div><div class="val" id="err-val">--</div></div>
      <div class="stat"><div class="lbl">GREEN AREA</div><div class="val" id="green-val">--</div></div>
      <div class="stat"><div class="lbl">분기점</div><div class="val" id="junc-val">0</div></div>
      <div class="stat"><div class="lbl">현재 루트</div><div class="val" id="route-stat">A</div></div>
    </div>
    <div class="stat" style="grid-column:1/-1;">
        <div class="lbl">적재 무게</div>
        <div class="val" id="weight-val">-- g</div>
    </div>
  </div>

  <!-- USB 웹캠 (장애물 확인용) -->
  <div class="card">
    <div class="card-title">USB Webcam · Obstacle View</div>
    <div class="cam-wrap">
      <img src="/webcam_feed" alt="Webcam">
    </div>
  </div>

  <!-- 오른쪽 패널 -->
  <div class="side">

    <!-- 루트 표시 -->
    <div class="card">
      <div class="route-display">
        <div class="route-label">CURRENT ROUTE</div>
        <div class="route-val" id="route-big">A</div>
      </div>
      <div class="ctrl-row">
        <button class="btn-start" onclick="startDrive()">▶ 자율주행 시작</button>
        <button class="btn-stop"  onclick="stopDrive()">■ 정지</button>
      </div>
    </div>

    <!-- 초음파 -->
    <div class="card" style="text-align:center;padding:16px;">
      <div class="card-title">Ultrasonic Distance</div>
      <div style="font-size:2.6rem;font-weight:700;color:var(--green);
                  line-height:1;transition:color .2s;" id="ultra-big">--</div>
      <div style="font-size:.7rem;color:var(--sub);margin-top:4px;">cm</div>
    </div>

    <!-- 명령 입력 -->
    <div class="card">
      <div class="card-title">자연어 명령</div>
      <div style="display:flex;gap:8px;margin-bottom:8px;">
        <input type="text" id="cmd-input"
          placeholder="예: 물건 떨어뜨리면 안돼 / 빨리 가줘"
          style="flex:1;background:rgba(0,0,0,.3);border:1px solid var(--border);
                 border-radius:6px;padding:8px 12px;color:var(--text);
                 font-family:inherit;font-size:.8rem;outline:none;"
          onkeydown="if(event.key==='Enter')sendCommand()">
        <button onclick="sendCommand()"
          style="padding:8px 16px;border:none;border-radius:6px;
                 background:var(--cyan);color:#000;cursor:pointer;
                 font-family:inherit;font-size:.8rem;font-weight:700;">
          전송
        </button>
      </div>
      <div style="font-size:.75rem;min-height:20px;" id="cmd-result">--</div>
    </div>

    <!-- AI 판단 -->
    <div class="card">
      <div class="card-title">AI 판단</div>
      <div class="ai-section">
        <div class="ai-status-bar" id="ai-status">대기중</div>
        <div class="ai-grid">
          <div class="ai-item">
            <div class="lbl">Gemini 장애물</div>
            <div class="val" id="g-type">--</div>
          </div>
          <div class="ai-item">
            <div class="lbl">추정 높이</div>
            <div class="val" id="g-height">-- cm</div>
          </div>
          <div class="ai-item">
            <div class="lbl">신뢰도</div>
            <div class="val" id="g-conf">--</div>
          </div>
          <div class="ai-item">
            <div class="lbl">XGBoost</div>
            <div class="val" id="xgb-label">--</div>
          </div>
        </div>
      </div>
    </div>

    <!-- 고급 설정 -->
    <div class="card">
      <details>
        <summary>고급 설정</summary>
        <div class="sliders">
          <div class="sl-item">
            <label>BASE_SPEED <span id="v-base">45</span></label>
            <input type="range" min="10" max="80" value="45"
                   oninput="updateVal('v-base',this.value);sendCfg('base_speed',+this.value)">
          </div>
          <div class="sl-item">
            <label>THRESH <span id="v-thresh">43</span></label>
            <input type="range" min="20" max="200" value="43"
                   oninput="updateVal('v-thresh',this.value);sendCfg('thresh',+this.value)">
          </div>
          <div class="sl-item">
            <label>KP <span id="v-kp">0.80</span></label>
            <input type="range" min="1" max="150" value="80"
                   oninput="updateVal('v-kp',(this.value/100).toFixed(2));sendCfg('kp',this.value/100)">
          </div>
          <div class="sl-item">
            <label>DEAD_ZONE <span id="v-dz">15</span></label>
            <input type="range" min="0" max="80" value="15"
                   oninput="updateVal('v-dz',this.value);sendCfg('dead_zone',+this.value)">
          </div>
          <div class="sl-item">
            <label>ROI_TOP <span id="v-roi">0.70</span></label>
            <input type="range" min="30" max="90" value="70"
                   oninput="updateVal('v-roi',(this.value/100).toFixed(2));sendCfg('roi_top',this.value/100)">
          </div>
          <div class="sl-item">
            <label>SPIN_SPEED <span id="v-spin">25</span></label>
            <input type="range" min="10" max="60" value="25"
                   oninput="updateVal('v-spin',this.value);sendCfg('spin_speed',+this.value)">
          </div>
        </div>
      </details>
    </div>

  </div>
</div>

<script>
let curMode  = 'safe';
let prevRoute = 'A';

function setMode(mode) {
  curMode = mode;
  fetch('/mode', {method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode})});
  document.getElementById('btn-safe').className =
    'mode-btn' + (mode==='safe' ? ' active-safe' : '');
  document.getElementById('btn-fast').className =
    'mode-btn' + (mode==='fast' ? ' active-fast' : '');
}

function startDrive() { fetch('/start', {method:'POST'}); }
function stopDrive()  { fetch('/stop',  {method:'POST'}); }
function sendCommand() {
  const text = document.getElementById('cmd-input').value.trim();
  if (!text) return;
  document.getElementById('cmd-result').textContent = 'Gemini 판단 중...';
  document.getElementById('cmd-result').style.color = 'var(--yellow)';
  fetch('/command', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({text})
  });
}
function updateVal(id, v) { document.getElementById(id).textContent = v; }
function sendCfg(k, v) {
  fetch('/config', {method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({[k]:v})});
}

function poll() {
  fetch('/state').then(r=>r.json()).then(d=>{
    // FPS
    document.getElementById('fps').textContent = (d.fps||'--') + ' fps';

    // 초음파
    const dist = d.distance;
    const distStr = dist >= 0 ? dist.toFixed(1) : '--';
    const distCls = dist >= 0 && dist < 15 ? 'danger' : dist >= 0 && dist < 30 ? 'warn' : '';
    document.getElementById('dist-tag').textContent = distStr + (dist>=0?' cm':'');
    document.getElementById('dist-tag').className   = 'dist-val' + (distCls ? ' '+distCls : '');
    const ultraEl = document.getElementById('ultra-big');
    ultraEl.textContent  = dist >= 0 ? dist.toFixed(1) : '--';
    ultraEl.style.color  = dist>=0 && dist<15 ? 'var(--red)' :
                           dist>=0 && dist<30 ? 'var(--yellow)' : 'var(--green)';

    // 상태
    document.getElementById('action-tag').textContent = d.action || '--';
    document.getElementById('err-val').textContent    = d.error  || 0;
    document.getElementById('green-val').textContent  = (d.green_area||0) + 'px';
    document.getElementById('junc-val').textContent   = d.junction_count || 0;
    document.getElementById('route-stat').textContent = d.route || '--';

    // 루트 큰 표시
    const routeBig = document.getElementById('route-big');
    const newRoute = (d.route||'A').replace('→','→');
    if (newRoute !== prevRoute) {
      routeBig.classList.add('changed');
      setTimeout(() => routeBig.classList.remove('changed'), 2000);
      prevRoute = newRoute;
    }
    routeBig.textContent = newRoute;

    // AI 판단
    const aiEl = document.getElementById('ai-status');
    const aiS  = d.ai_status || '대기중';
    aiEl.textContent = aiS;
    aiEl.className = 'ai-status-bar' +
      (aiS.includes('분석') ? ' analyzing' :
       aiS.includes('BLOCK')||aiS.includes('REROUTE') ? ' detour' :
       aiS.includes('NORMAL')||aiS.includes('CAUTIOUS') ? ' pass' : '');

    document.getElementById('g-type').textContent   = d.gemini_type   || '--';
    document.getElementById('g-height').textContent = (d.gemini_height || '--') +
      (d.gemini_height && d.gemini_height !== '--' ? ' cm' : '');
    document.getElementById('g-conf').textContent   = d.gemini_conf   || '--';
    
    // 명령 결과
    const cmdEl = document.getElementById('cmd-result');
    if (d.cmd_result && d.cmd_result !== '--') {
      cmdEl.textContent = d.cmd_result;
      cmdEl.style.color = d.cmd_result.includes('FAST') ? 'var(--orange)' :
                          d.cmd_result.includes('SAFE') ? 'var(--cyan)' : 'var(--red)';
    }
    const wEl = document.getElementById('weight-val');
    wEl.textContent = d.weight_g !== undefined ? d.weight_g.toFixed(1) + ' g' : '-- g';
    wEl.className   = 'val' + (d.weight_g > 300 ? ' danger' : d.weight_g > 150 ? ' warn' : '');
    // 모드 버튼 동기화
    if (d.user_mode) {
      document.getElementById('btn-safe').className =
        'mode-btn' + (d.user_mode==='safe' ? ' active-safe' : '');
      document.getElementById('btn-fast').className =
        'mode-btn' + (d.user_mode==='fast' ? ' active-fast' : '');
    }

    const xgbEl  = document.getElementById('xgb-label');
    const xgbLbl = d.xgb_label || '--';
    xgbEl.textContent = xgbLbl;
    xgbEl.className   = 'val ' + (xgbLbl==='pass'?'pass':xgbLbl==='cautious'?'cautious':xgbLbl==='detour'?'detour':'');

  }).catch(()=>{});
}
setInterval(poll, 200);

let activeKey = null;
window.addEventListener('keydown', function(e) {
  if (document.activeElement.tagName === 'INPUT') return;
  const keyMap = {'ArrowUp':'forward','w':'forward','ArrowDown':'backward','s':'backward',
                  'ArrowLeft':'left','a':'left','ArrowRight':'right','d':'right'};
  const dir = keyMap[e.key];
  if (dir && activeKey !== dir) {
    activeKey = dir;
    fetch('/manual_control', {method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({direction:dir})});
  }
});
window.addEventListener('keyup', function(e) {
  const keyMap = {'ArrowUp':'forward','w':'forward','ArrowDown':'backward','s':'backward',
                  'ArrowLeft':'left','a':'left','ArrowRight':'right','d':'right'};
  if (keyMap[e.key]) {
    activeKey = null;
    fetch('/manual_control', {method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({direction:'stop'})});
  }
});

</script>
</body>
</html>'''

if __name__ == '__main__':
    print("자율주행 대시보드 시작: http://192.168.0.50:5004")
    app.run(host='0.0.0.0', port=5004, threaded=True)