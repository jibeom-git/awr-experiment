# app/auto_dashboard_v3.py
# 자율 주행 대시보드 - 양방향 카메라 동시 송출 버그 완치 및 캘리브레이션 통합본
#
# 실행: python app/auto_dashboard_v3.py
# 접속: http://192.168.0.50:5003

import sys, os, time, threading, copy, warnings
from typing import Any
sys.path.insert(0, '/home/pi/insite')

# =====================================================================
# 🔑 [구글 제미나이 API 키 통합 관리 센터]
# 새로운 계정으로 키를 발급받으시면 오직 여기 한 곳만 수정해 주시면 됩니다!
# =====================================================================
# 💡 기존의 빈 문자열 대입 대신 시스템에 등록된 키를 자동으로 읽어오도록 수정합니다.
os.environ["GEMINI_API_KEY"] = os.environ.get("GEMINI_API_KEY", "").strip()

current_key_debug = os.environ.get('GEMINI_API_KEY') or ""
print(f"[Key-Debug] 현재 전역 API KEY 로드 성공: {current_key_debug[:10]}...")

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

from sensors.camera import Camera
from sensors.motor import MotorController

app = Flask(__name__)

# -- 모터 전역 인스턴스 선언
motor = MotorController()

# -- Pi 카메라 인프라 구동
try:
    cam = Camera(width=320, height=240)
    CAM_AVAILABLE = True
    print("[OK] Pi 카메라 로드 완료")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] Pi 카메라 실패: {e}")

# ── USB 웹캠 (중복 개방 방지 및 오리지널 인덱스 0, 1번 자동 탐색 기동) ───────────────────
try:
    webcam = None
    # 💡 Pi 카메라는 libcamera를 쓰므로 OpenCV 인덱스 0, 1번은 순수하게 USB 웹캠 자리가 맞습니다.
    for idx in [0, 1, 2, 4]:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            webcam = cap
            webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print(f"[OK] USB 웹캠 물리 장치 포착 성공 (시스템 인덱스: {idx}번 할당됨)")
            break
        cap.release()
        
    WEBCAM_AVAILABLE = webcam is not None if webcam else False
    if not WEBCAM_AVAILABLE:
        print("[SKIP] USB 웹캠 장치를 어디서도 개방할 수 없습니다. 포트를 재연결하세요.")
except Exception as e:
    webcam = None
    WEBCAM_AVAILABLE = False
    print(f"[SKIP] 웹캠 예외: {e}")

# -- IMU 센서 모듈 가동
try:
    from sensors.mpu6050 import MPU6050
    imu = MPU6050()
    time.sleep(0.5)
    _baseline = imu.get_accel()
    IMU_AVAILABLE = True
    print(f"[OK] IMU 초기화 성공: x_base={_baseline['x']:.3f}")
except Exception as e:
    imu = None
    _baseline = {'x': 0.0}
    IMU_AVAILABLE = False
    print(f"[SKIP] IMU 센서 예외: {e}")

# -- 초음파 거리 센서 가동
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
    print("[OK] 초음파 거리 센서 연동 성공")
except Exception as e:
    ultra = None
    ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파 센서 예외: {e}")

# -- 로드셀 센서 인터페이스 가동
try:
    from sensors.hx711 import HX711
    hx711 = HX711()
    import time as _t; _t.sleep(1)
    hx711.tare(samples=20)
    HX711_AVAILABLE = True
    print("[OK] 로드셀 HX711 초기화 완료")
except Exception as e:
    hx711 = None
    HX711_AVAILABLE = False
    print(f"[SKIP] 로드셀 예외: {e}")

def get_weight_g() -> float:
    if hx711 is None:
        return 0.0
    try:
        w = hx711.get_weight(times=3)
        return max(0.0, w)
    except:
        return 0.0

# ══════════════════════════════════════════════════════
# 라우트 하드코딩 명세 딕셔너리
# ══════════════════════════════════════════════════════
ROUTES = {
    "A":   {1: "straight", 2: "stop"},
    "B":   {1: "left",  2: "right", 3: "right", 4: "stop"},
    "C":   {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight", 6: "stop"},
    "A->B": {1: "left", 2: "right", 3: "right", 4: "stop"},
    "A->C": {1: "left", 2: "straight", 3: "right", 4: "right", 5: "straight", 6: "stop"},
    "B->C": {1: "left", 2: "right", 3: "right", 4: "straight", 5: "stop"},
}

# ══════════════════════════════════════════════════════
# 전역 스레드 동기화 락 및 상태 파라미터 트리
# ══════════════════════════════════════════════════════
lock = threading.Lock()

config = {
    "base_speed":     50,            # 평지 주행 기본 크루즈 속도 50% 단일화 잠금
    "turn_speed":     30,
    "spin_speed":     25,
    "thresh":         43,
    "roi_top":        0.7,
    "dead_zone":      50,            # 기본 공차 수치 (하단 연산 커널에서 1.5배 스케일업 가변 확장)
    "kp":             0.8,
    "ki":             0.002,
    "forward_time":   0.5,
    "slope_speed":    65,            # 경사면 조향 유지 등판 가속 속도 65% 고정
    "green_h_min":    30,
    "green_h_max":    85,
    "green_s_min":    80,
    "green_v_min":    50,
    "green_min_area": 120,
    "running":        False,         # True 시 자동 주행, False 시 수동 모드 대기
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
    "fps":            0,
    "lost":           False,
    "ai_status":      "대기중",
    "gemini_type":    "--",
    "gemini_height":  "--",
    "gemini_conf":    "--",
    "xgb_label":      "--",
    "ai_action":      "--",
    "cmd_result":     "--",
    "weight_g":       0.0,
    "has_1cm_bump":   False,         
    "bump_speed_override": 0,        
    "target_cruise_speed": 50,
    "current_direction": "stop"      # 실시간 웹 조종 방향 디스플레이용 변수
}

latest_picam  = None
latest_webcam = None

g_junction_count = 0
g_junction_cooldown = 0.0  
g_junction_lock  = threading.Lock()

def get_junction_count():
    with g_junction_lock: return g_junction_count

def increment_junction():
    global g_junction_count
    with g_junction_lock:
        g_junction_count += 1
        return g_junction_count

def reset_junction():
    global g_junction_count, g_junction_cooldown
    with g_junction_lock:
        g_junction_count = 0
        g_junction_cooldown = time.time() + 2.5

# ── 차륜 출력 다이렉트 드라이버 계층 ─────────────────────
def go_forward(speed):
    motor.set_motor(1, 1, speed); motor.set_motor(2, 1, speed)
    motor.set_motor(3, 1, speed); motor.set_motor(4, 1, speed)

def spin_left(speed):
    motor.set_motor(4,-1,speed); motor.set_motor(3,-1,speed)
    motor.set_motor(2, 1,speed); motor.set_motor(1, 1,speed)

def spin_right(speed):
    motor.set_motor(4, 1,speed); motor.set_motor(3, 1,speed)
    motor.set_motor(2,-1,speed); motor.set_motor(1,-1,speed)

# ── OpenCV 관심 영역 마스킹 및 렌더링 파이프라인 ───────────
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
    
    is_junction = cfg["green_min_area"] < green_area < 5000

    debug = frame.copy()
    cv2.line(debug, (0, roi_y), (w, roi_y), (255,180,0), 2)
    
    # [사용자 명시 조건] 2cm 가로폭 트랙 조향 유연화를 위해 데드존 경계선을 기존 대비 1.5배 확장하여 드로잉
    dz = int(cfg["dead_zone"] * 2)
    cv2.rectangle(debug, (w//2 - dz, roi_y), (w//2 + dz, h), (0, 0, 200), 1)
    cv2.line(debug, (w//2, roi_y), (w//2, h), (0, 0, 255), 2)
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
# VLM 복합 시나리오 의사결정 코어 레이어
# ══════════════════════════════════════════════════════
def run_ai_decision(cur_route: str, frame_snap):
    try:
        with lock:
            state["ai_status"] = "분석중..."

        height_cm = 0.0
        obs_type  = "none"
        try:
            import cv2 as _cv2
            from ai_core.vlm_client import VLMClient
            vlm = VLMClient()
            _, jpeg = _cv2.imencode('.jpg', frame_snap, [_cv2.IMWRITE_JPEG_QUALITY, 85])
            result = vlm.analyze(jpeg.tobytes())
            height_cm = float(result.get("height_cm", 0.0))
            obs_type  = result.get("obstacle_type", "none")
            with lock:
                state["gemini_type"]   = obs_type
                state["gemini_height"] = str(height_cm)
                state["gemini_conf"]   = f"{float(result.get('confidence',0))*100:.0f}%"
        except Exception as e:
            print(f"[VLM Engine Failure] {e}")

        mode   = config.get("user_mode", "safe").lower()
        weight = state.get("weight_g", 0.0)
        route  = cur_route.split("->")[-1] if "->" in cur_route else cur_route
        heavy  = weight >= 140.0

        if obs_type == "none" or height_cm == 0.0:
            bump = "none"
        elif height_cm <= 1.2:
            bump = "1cm"
        else:
            bump = "2cm"

        # ── 규칙표 적용 ──────────────────────────────────────
        action = "pass"
        speed  = config["base_speed"]  # 기본값: 항상 base_speed 유지

        if mode == "fast":
            if not heavy:
                if route == "A":
                    if bump == "none":   action, speed = "pass",   70   # 경사로 오르막 70%
                    elif bump == "1cm":  action = "detour"               # B로 우회
                    elif bump == "2cm":  action = "detour"               # B로 우회
                elif route == "B":
                    if bump == "none":   action = "pass"                 # 기본 속도 유지
                    elif bump == "1cm":  action, speed = "pass",   70   # 70%로 통과
                    elif bump == "2cm":  action = "detour"               # C로 우회
                elif route == "C":
                    if bump == "none":   action = "pass"                 # 기본 속도 유지
                    elif bump == "1cm":  action, speed = "pass",   60   # 60%로 통과
                    elif bump == "2cm":  action = "stop"                 # 운영자 개입
            else:  # heavy (140g 이상)
                if route == "A":
                    action = "detour"                                    # 무조건 B로 우회
                elif route == "B":
                    if bump == "none":   action, speed = "pass",   80   # 무거울때 80%
                    elif bump == "1cm":  action = "detour"               # C로 우회
                    elif bump == "2cm":  action = "detour"               # C로 우회
                elif route == "C":
                    if bump == "none":   action = "pass"                 # 기본 속도 유지
                    elif bump == "1cm":  action, speed = "pass",   70   # 70%로 통과
                    elif bump == "2cm":  action = "stop"                 # 운영자 개입

        else:  # safe (B 경로부터 시작)
            if not heavy:
                if route == "B":
                    if bump == "none":   action = "pass"                 # 기본 속도 유지
                    elif bump == "1cm":  action, speed = "pass",   60   # 60%로 통과
                    elif bump == "2cm":  action = "detour"               # C로 우회
                elif route == "C":
                    if bump == "none":   action = "pass"                 # 기본 속도 유지
                    elif bump == "1cm":  action, speed = "pass",   60   # 60%로 통과
                    elif bump == "2cm":  action = "stop"                 # 운영자 개입
            else:  # heavy (140g 이상)
                if route == "B":
                    if bump == "none":   action, speed = "pass",   60   # 오르막 60%
                    elif bump == "1cm":  action = "detour"               # C로 우회
                    elif bump == "2cm":  action = "detour"               # C로 우회
                elif route == "C":
                    if bump == "none":   action = "pass"                 # 기본 속도 유지
                    elif bump == "1cm":  action, speed = "pass",   70   # 70%로 통과
                    elif bump == "2cm":  action = "stop"                 # 운영자 개입

        print(f"[VLM Matrix Evaluated] Action: {action} / Dynamic Speed: {speed}%")

        if action == "detour":
            next_map = "B" if route == "A" else "C"
            new_route_str = f"{route}->{next_map}"
            reset_junction()
            config['route'] = new_route_str
            with lock:
                state["junction_count"] = 0
                state["route"] = new_route_str
                state["ai_status"] = f"우회결정_복귀대기"
            config["running"] = True
        elif action == "stop":
            motor.motorStop()
            config["running"] = False
            with lock:
                state["action"] = "최종정지"
                state["ai_status"] = "장애물완전단절정지"
        else:
            if bump == "1cm":
                with lock:
                    state["ai_status"] = "1cm단차극복기동"
                    state["has_1cm_bump"] = True
                    state["bump_speed_override"] = speed
                config["running"] = True
            else:
                config["running"] = True
                with lock: state["ai_status"] = "본선크루즈안착"

    except Exception as e:
        print(f"[AI Matrix Critical Crash] {e}")
        config["running"] = True
    finally:
        drive_loop._ai_running = False
        drive_loop._ai_cooldown = time.time() + 5.0

# ── 하드웨어 센서 폴링 레이어 스레드 구역 ───────────────────
def ultra_loop():
    while True:
        if ultra is not None:
            try:
                d = ultra.distance
                with lock: state["distance"] = round(d*100,1) if d else -1
            except: pass
        time.sleep(0.1)

def weight_loop():
    while True:
        w = get_weight_g()
        with lock: state["weight_g"] = round(w, 1)
        time.sleep(0.5)

threading.Thread(target=weight_loop, daemon=True, name="weight").start()

def webcam_loop():
    global latest_webcam
    # 💡 [버그 완치] 새로 장치를 열지 않고, 최상단에서 이미 성공적으로 열린 webcam 오브젝트를 그대로 공유합니다.
    # 이 방식을 통해 리눅스의 자원 독점(Resource Busy) 충돌 경고가 완벽하게 사라집니다.
    if webcam is None or not WEBCAM_AVAILABLE:
        print("[FAIL] 초기 웹캠 인스턴스가 존재하지 않아 송출 스레드를 기동할 수 없습니다.")
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

def check_traffic_signal_before_start():
    if not hasattr(check_traffic_signal_before_start, '_active'):
        check_traffic_signal_before_start._active = True
    try:
        from ai_core.signal_detector import SignalDetectorVLM
        detector = SignalDetectorVLM()
    except:
        config["running"] = True
        return

    with lock:
        state["ai_status"] = "신호등 스캔 중"
        state["action"] = "신호등 대기"

    while True:
        if not getattr(check_traffic_signal_before_start, '_active', False): return
        with lock: img_bytes = latest_picam  
        if img_bytes is not None:
            try:
                img_array = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if frame is None:
                    time.sleep(0.1)
                    continue
                h, w = frame.shape[:2]
                traffic_roi = frame[0:int(h*0.5), :] 
                _, roi_bytes = cv2.imencode('.jpg', traffic_roi, [cv2.IMWRITE_JPEG_QUALITY, 85])
                signal_suspect = detector.detect_via_cv(roi_bytes.tobytes())
                color = "green" if "green" in signal_suspect else "yellow" if "yellow" in signal_suspect else "red"
                if color == "green":
                    with lock:
                        state["action"] = "대기기동승인"
                        state["ai_status"] = "자율크루즈가동완료"
                    config["running"] = True 
                    return
            except: time.sleep(0.1)
        time.sleep(0.1)

# ══════════════════════════════════════════════════════
# 메인 하이브리드 제어 루프
# ══════════════════════════════════════════════════════
def drive_loop():
    global latest_picam, g_junction_cooldown

    CENTER_X          = 160
    lost_dir          = 1
    fps_t             = time.time()
    fps_count         = 0
    integral          = 0.0
    prev_time         = time.time()
    green_seen        = False
    stopped           = False

    while True:
        cfg = copy.deepcopy(config)
        frame = None
        if cam is not None:
            try: 
                # 🔴 [색상 완치] 불필요한 cvtColor 채널 반전을 삭제하여 태블릿 화면과 라인이 오리지널 원색으로 정상 출력
                frame = cam.capture()
            except: frame = None

        if frame is not None:
            cx, green_area, is_junction, debug = detect_line_and_green(frame, cfg)
            _, jpeg = cv2.imencode('.jpg', debug, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with lock: latest_picam = jpeg.tobytes()
        else:
            cx, green_area, is_junction = None, 0, False

        current_route = cfg["route"]

        fps_count += 1
        if time.time() - fps_t >= 1.0:
            with lock: state["fps"] = fps_count
            fps_count = 0; fps_t = time.time()

        # ── 수동 제어 모드 상태일 때 백그라운드 데이터 수집 레이어 처리 ──
        if not cfg["running"]:
            integral = 0.0
            with lock:
                state["green_area"] = green_area
                state["junction_count"] = get_junction_count()
                state["error"] = cx - CENTER_X if cx else 0
                state["lost"] = cx is None
                
                cur_dir = state["current_direction"]
                if cur_dir == "stop": state["action"] = "수동조종_대기정지"
            time.sleep(0.03)
            continue

        now = time.time()
        dt  = now - prev_time; prev_time = now
        if dt > 0.1: dt = 0.033

        # ── 다중 센서 융합 기반 라인 단절 및 장애물 확정 트리거 ──
        with lock: dist_now = state["distance"]
        is_line_cut_by_obstacle = (cx is None and 0 < dist_now < 40)
        
        if (cfg["running"] and (dist_now < 20 or is_line_cut_by_obstacle) and
                not getattr(drive_loop, '_ai_running', False) and
                time.time() > getattr(drive_loop, '_ai_cooldown', 0)):
            drive_loop._ai_running = True
            config["running"] = False
            motor.motorStop()
            time.sleep(0.5)
            frame_snap = frame.copy() if frame is not None else np.zeros((240,320,3), np.uint8)

            def _run_ai(r=current_route, f=frame_snap):
                try:
                    time.sleep(0.5)
                    run_ai_decision(r, f)
                finally: drive_loop._ai_running = False

            threading.Thread(target=_run_ai, daemon=True, name="ai").start()
            continue

        # ── 실시간 가변 속도 프로필 할당 엔진 ──
        terrain = "flat"
        if imu is not None:
            try:
                accel = imu.get_accel()
                diff_x = accel['x'] - _baseline['x']
                if diff_x > 0.15: terrain = "uphill"
            except: pass

        mode = cfg["user_mode"].lower()
        weight = state.get("weight_g", 0.0)
        heavy = weight >= 140.0
        jc = get_junction_count()
        
        if jc >= 3:
            cruise_speed = 45
            action_state = "최종정렬주행"
        elif terrain == "uphill":
            cruise_speed = 65
            action_state = "경사로추종가속"
        elif state.get("has_1cm_bump"):
            cruise_speed = state.get("bump_speed_override", 70)
            action_state = "단차극복기동"
        else:
            if mode == "fast" and not heavy and current_route == "A":
                cruise_speed = 60
            else:
                cruise_speed = 50
            action_state = "평지정속주행"

        with lock:
            state["target_cruise_speed"] = cruise_speed

        # -- 분기점(노드) 포착 타이밍 스케줄러 (친구분 복구 오리지널 밸런스 기반) --
        if is_junction and not green_seen and now > g_junction_cooldown:
            green_seen = True
            jc = increment_junction()
            print(f"[Junction Event] 노드 검출 포착 #{jc} | 현재 선로: {current_route}")
            with lock:
                state["junction_count"] = jc
                state["action"] = f"노드#{jc}_물리정렬"
            
            go_forward(45)
            time.sleep(cfg["forward_time"])
            motor.motorStop()
            time.sleep(0.1)
            
            action_at = ROUTES[current_route].get(jc, "straight")
            if action_at == "stop":
                go_forward(45)
                time.sleep(0.2)
                motor.motorStop()
                config['running'] = False
                continue
            elif action_at == "straight":
                go_forward(45)
                time.sleep(0.4)
            elif action_at in ("left", "right"):
                spin_fn = spin_left if action_at == "left" else spin_right
                spin_fn(35)
                time.sleep(0.55)
                
                fine_timeout = time.time() + 3.0
                while time.time() < fine_timeout:
                    if cam is not None: frame_sub = cam.capture()
                    else: break
                    cx_sub, _, _, _ = detect_line_and_green(frame_sub, cfg)
                    if cx_sub is not None and abs(cx_sub - 160) < int(cfg["dead_zone"] * 2 * 0.8):
                        motor.motorStop()
                        break
                    spin_fn(34)
                    time.sleep(0.02)
                    
            now = time.time()
            g_junction_cooldown = now + 4.0
            green_seen = True
            continue

        if not is_junction:
            green_seen = False

        # -- 고정밀 차동 조향 보정 커널 매핑 구역 (요청사항 반영 수정 구역) --
        if cx is None:
            # 💡 라인을 완전히 잃었을 때의 탐색 회전 속도도 35에서 23으로 하향 조절
            spin_right(23) if lost_dir > 0 else spin_left(23)
            with lock: state["action"] = "라인유실_탐색모드"
        else:
            error = cx - CENTER_X
            current_dead_zone = int(cfg["dead_zone"] * 2)
            lost_dir = 1 if error > 0 else (-1 if error < 0 else lost_dir)

            # 1. 초록점이 빨간 박스 내부(데드존)에 있을 때 -> 정선 직진
            if abs(error) < current_dead_zone:
                go_forward(cruise_speed)
                with lock: state["action"] = f"{action_state}(박스내부:직진)"
            
            # 2. 초록점이 빨간 박스 외부로 나갔을 때 -> 중앙 빨간선까지 스핀 회전
            else:
                # 💡 cfg에서 값이 없을 때 적용할 기본 대피 속도를 35에서 22로 감속 수정!
                spin_speed = cfg.get("spin_speed", 22) 
                if error > 0:
                    spin_right(spin_speed)
                    with lock: state["action"] = f"{action_state}(우향 스핀복귀:{abs(error)}px)"
                else:
                    spin_left(spin_speed)
                    with lock: state["action"] = f"{action_state}(좌향 스핀복귀:{abs(error)}px)"

        with lock:
            state["error"]  = cx - CENTER_X if cx else 0
            state["lost"]   = cx is None
            state["route"]  = current_route
            state["green_area"] = green_area

        time.sleep(0.03)

threading.Thread(target=drive_loop, daemon=True, name="drive").start()

# ══════════════════════════════════════════════════════
# 키보드 실시간 매핑 조종 핸들러 라우트
# ══════════════════════════════════════════════════════
@app.route('/manual_control', methods=['POST'])
def manual_control():
    data = request.get_json(force=True)
    direction = data.get('direction', 'stop')
    
    with lock:
        cruise_speed = state["target_cruise_speed"]
        state["current_direction"] = direction
        if direction != "stop":
            config["running"] = False

    if direction == "forward":
        go_forward(cruise_speed) 
    elif direction == "backward":
        motor.set_motor(1, -1, 45); motor.set_motor(2, -1, 45)
        motor.set_motor(3, -1, 45); motor.set_motor(4, -1, 45)
    elif direction == "left":
        spin_left(35)
    elif direction == "right":
        spin_right(35)
    else:
        motor.motorStop()
        
    return jsonify({'status': 'ok', 'direction': direction, 'applied_speed': cruise_speed})

# ── Flask 웹 서빙 인터페이스 및 API 통신 엔드포인트 ───────
def gen_stream(get_fn):
    while True:
        with lock: frame = get_fn()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)

@app.route('/picam_feed')
def picam_feed(): return Response(gen_stream(lambda: latest_picam), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/webcam_feed')
def webcam_feed(): return Response(gen_stream(lambda: latest_webcam), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/state')
def get_state():
    with lock: return jsonify(state)

@app.route('/start', methods=['POST'])
def start():
    reset_junction()
    mode  = config.get('user_mode', 'safe')
    weight = get_weight_g()
    heavy = weight >= 140.0
    
    if mode == "fast": route = "B" if heavy else "A"
    else: route = "B"
        
    config['route'] = route
    config['running'] = False
    
    with lock:
        state["has_1cm_bump"] = False
        state["bump_speed_override"] = 0
        state["route"]          = route
        state["junction_count"] = 0
        state["ai_status"]      = "신호등 스캔 중"
        state["action"]         = "신호대기"

    check_traffic_signal_before_start._active = True
    threading.Thread(target=check_traffic_signal_before_start, daemon=True, name="signal_check").start()
    return jsonify({'status': 'running'})

@app.route('/stop', methods=['POST'])
def stop():
    config['running'] = False
    check_traffic_signal_before_start._active = False
    motor.motorStop()
    with lock: state["current_direction"] = "stop"
    return jsonify({'status': 'stopped'})

# ── Flask 웹 서빙 인터페이스 중 /command 라우트 내부 스레드 구역 수정 ──
@app.route('/command', methods=['POST'])
def command():
    data = request.get_json(force=True)
    user_text = data.get('text', '').strip()
    if not user_text: 
        return jsonify({'status': 'error', 'msg': '텍스트 누락'})

    def _parse():
        global g_junction_cooldown  
        try:
            from ai_core.commander import CommanderVLM
            cmd = CommanderVLM()
            result = cmd.parse(user_text)
            mode  = result.get('mode', 'safe').lower()
            route = result.get('route', 'B').replace("ROUTE_", "").strip().upper()
            reason= result.get('reason', '')
            
            if mode == "safe":
                route = "B"
                reason = "안전 주행 모드가 활성화되어 기준 경로인 B루트로 진입합니다."
            elif mode == "fast":
                weight = state.get("weight_g", 0.0)
                route = "B" if weight >= 140.0 else "A"
                reason = f"빠른 주행 모드가 활성화되어 화물 중량별 최적 경로인 {route}루트로 진입합니다."
            
            config['user_mode'] = mode
            config['route'] = route
            with lock:
                state['route'] = route
                state['cmd_result'] = f"[{mode.upper()}] {reason}"
                state['user_mode'] = mode
                
            with g_junction_lock:
                g_junction_cooldown = time.time() - 1.0  
            
            config["running"] = True 

        except Exception as e:
            fallback_mode = config.get('user_mode', 'safe')
            fallback_route = "B" if fallback_mode == "safe" else "A"
            with lock: 
                state['cmd_result'] = f"시스템 라우팅 예외 복구 적용 -> {fallback_route}"
                state['route'] = fallback_route
            config['route'] = fallback_route
            
            with g_junction_lock:
                g_junction_cooldown = time.time() - 1.0
            config["running"] = True
                        
    threading.Thread(target=_parse, daemon=True).start()
    return jsonify({'status': 'ok', 'msg': '분석 진행 중'})

@app.route('/mode', methods=['POST'])
def set_mode():
    data = request.get_json(force=True)
    mode = data.get('mode', 'safe').lower()
    if mode in ('fast', 'safe'): config['user_mode'] = mode
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
def index(): return render_template_string(HTML)

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AWR Hyper Tuning Dashboard</title>
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
  header{
    position:relative;z-index:10;
    background:var(--panel);border-bottom:1px solid var(--border);
    padding:14px 24px;display:flex;align-items:center;justify-content:space-between;
  }
  .logo{font-family:'Syne',sans-serif;font-size:1.1rem;font-weight:800;color:var(--cyan);letter-spacing:3px;}
  .header-right{display:flex;align-items:center;gap:20px;}
  .fps-badge{font-size:.7rem;color:var(--dim);}
  .mode-toggle{display:flex;gap:4px;background:rgba(255,255,255,.04);border:1px solid var(--border);border-radius:8px;padding:4px;}
  .mode-btn{padding:7px 18px;border:none;border-radius:6px;font-family:'JetBrains Mono',monospace;font-size:.75rem;font-weight:700;cursor:pointer;transition:all .2s;background:transparent;color:var(--sub);text-transform:uppercase;letter-spacing:1px;}
  .mode-btn.active-safe{background:var(--cyan);color:#000;box-shadow:0 0 20px rgba(0,229,255,.3);}
  .mode-btn.active-fast{background:var(--orange);color:#000;box-shadow:0 0 20px rgba(255,149,0,.3);}
  .main{position:relative;z-index:1;display:grid;grid-template-columns:1fr 1fr 340px;gap:12px;padding:12px;}
  .card{background:var(--panel);border:1px solid var(--border);border-radius:12px;padding:16px;position:relative;overflow:hidden;}
  .card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:linear-gradient(90deg,transparent,var(--cyan),transparent);opacity:.3;}
  .card-title{font-size:.65rem;color:var(--sub);letter-spacing:2px;text-transform:uppercase;margin-bottom:12px;}
  .cam-wrap{position:relative;border-radius:8px;overflow:hidden;background:#000;border:1px solid var(--border);}
  .cam-wrap img{width:100%;max-height:200px;object-fit:contain;display:block;}
  .cam-overlay{position:absolute;bottom:8px;left:8px;right:8px;display:flex;justify-content:space-between;align-items:flex-end;}
  .dist-tag{background:rgba(0,0,0,.8);border:1px solid var(--border);border-radius:6px;padding:4px 10px;font-size:.8rem;}
  .dist-val{font-weight:700;color:var(--green);}
  .dist-val.warn{color:var(--yellow);}
  .dist-val.danger{color:var(--red);animation:blink .3s step-end infinite;}
  @keyframes blink{50%{opacity:.2}}
  .side{display:flex;flex-direction:column;gap:12px;}
  .route-display{text-align:center;padding:16px;}
  .route-label{font-size:.65rem;color:var(--sub);letter-spacing:2px;margin-bottom:8px;}
  .route-val{font-family:'Syne',sans-serif;font-size:3.5rem;font-weight:800;color:var(--cyan);line-height:1;transition:all .3s;}
  .route-val.changed{color:var(--orange);text-shadow:0 0 20px rgba(255,149,0,.5);}
  .ctrl-row{display:flex;gap:8px;}
  .btn-start{flex:1;padding:14px;border:none;border-radius:8px;cursor:pointer;background:linear-gradient(135deg,#006644,var(--green));color:#000;font-family:inherit;font-size:.9rem;font-weight:700;letter-spacing:1px;transition:all .2s;}
  .btn-start:hover{filter:brightness(1.1);}
  .btn-stop{flex:1;padding:14px;border:none;border-radius:8px;cursor:pointer;background:linear-gradient(135deg,#660011,var(--red));color:#fff;font-family:inherit;font-size:.9rem;font-weight:700;letter-spacing:1px;transition:all .2s;}
  .btn-stop:hover{filter:brightness(1.1);}
  .ai-section{display:flex;flex-direction:column;gap:8px;}
  .ai-status-bar{padding:8px 12px;border-radius:6px;font-size:.75rem;font-weight:700;text-align:center;letter-spacing:1px;background:rgba(0,229,255,.08);border:1px solid rgba(0,229,255,.2);color:var(--cyan);transition:all .3s;}
  .ai-status-bar.analyzing{background:rgba(255,214,0,.08);border-color:rgba(255,214,0,.3);color:var(--yellow);}
  .ai-status-bar.detour{background:rgba(255,61,87,.08);border-color:rgba(255,61,87,.3);color:var(--red);}
  .ai-status-bar.pass{background:rgba(0,255,157,.08);border-color:rgba(0,255,157,.3);color:var(--green);}
  .ai-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;}
  .ai-item{background:rgba(0,0,0,.3);border:1px solid var(--border);border-radius:8px;padding:10px;}
  .ai-item .lbl{font-size:.6rem;color:var(--sub);letter-spacing:.5px;margin-bottom:4px;}
  .ai-item .val{font-size:.9rem;font-weight:600;color:var(--cyan);}
  .ai-item .val.pass{color:var(--green);}
  .ai-item .val.cautious{color:var(--yellow);}
  .ai-item .val.detour{color:var(--red);}
  .status-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px;}
  .stat{background:rgba(0,0,0,.25);border:1px solid var(--border);border-radius:8px;padding:10px;}
  .stat .lbl{font-size:.65rem;color:var(--sub);margin-bottom:4px;}
  .stat .val{font-size:1rem;font-weight:700;color:var(--green);}
  .sliders{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px;}
  .sl-item label{font-size:.65rem;color:var(--sub);display:block;margin-bottom:3px;}
  .sl-item input{width:100%;accent-color:var(--cyan);}
  .sl-item span{font-size:.75rem;color:var(--cyan);}
  details{margin-top:12px;}
  summary{font-size:.7rem;color:var(--sub);cursor:pointer;padding:8px 0;letter-spacing:1px;text-transform:uppercase;}
  
  .keyboard-guide {
    margin-top: 12px;
    background: rgba(255,255,255,0.02);
    border: 1px dashed var(--border);
    border-radius: 8px;
    padding: 12px;
    font-size: 0.75rem;
    color: var(--sub);
  }
  .key-cap {
    display: inline-block;
    padding: 2px 6px;
    border: 1px solid var(--border);
    border-radius: 4px;
    background: var(--panel);
    color: var(--cyan);
    font-weight: 700;
  }
</style>
</head>
<body>
<header>
  <div class="logo">AWR TUNING COMMAND CENTER</div>
  <div class="header-right">
    <div class="mode-toggle">
      <button class="mode-btn active-safe" id="btn-safe" onclick="setMode('safe')">SAFE</button>
      <button class="mode-btn" id="btn-fast" onclick="setMode('fast')">FAST</button>
    </div>
    <span class="fps-badge" id="fps">-- fps</span>
  </div>
</header>
<div class="main">
  <div class="card">
    <div class="card-title">Pi Camera · 1.5x DeadZone Expanded Follower</div>
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
      <div class="stat"><div class="lbl">분기점 카운트</div><div class="val" id="junc-val">0</div></div>
      <div class="stat"><div class="lbl">현재 루트</div><div class="val" id="route-stat">A</div></div>
    </div>
    <div class="stat" style="grid-column:1/-1; margin-top:8px;">
        <div class="lbl">적재 무게</div>
        <div class="val" id="weight-val">-- g</div>
    </div>
  </div>
  <div class="card">
    <div class="card-title">USB Webcam · Obstacle Tracker</div>
    <div class="cam-wrap">
      <img src="/webcam_feed" alt="Webcam">
    </div>
  </div>
  <div class="side">
    <div class="card">
      <div class="route-display">
        <div class="route-label">CURRENT ROUTE</div>
        <div class="route-val" id="route-big">A</div>
      </div>
      <div class="ctrl-row">
        <button class="btn-start" onclick="startDrive()">자동 트래킹 가동</button>
        <button class="btn-stop"  onclick="stopDrive()">강제 비상 제동</button>
      </div>
    </div>
    <div class="card" style="text-align:center;padding:16px;">
      <div class="card-title">Ultrasonic Distance</div>
      <div style="font-size:2.6rem;font-weight:700;color:var(--green);line-height:1;transition:color .2s;" id="ultra-big">--</div>
      <div style="font-size:.7rem;color:var(--sub);margin-top:4px;">cm</div>
    </div>
    <div class="card">
      <div class="card-title">자연어 라우팅 명령</div>
      <div style="display:flex;gap:8px;margin-bottom:8px;">
        <input type="text" id="cmd-input" placeholder="예: b경로 고정하고 안전 모드로 전환해" style="flex:1;background:rgba(0,0,0,.3);border:1px solid var(--border);border-radius:6px;padding:8px 12px;color:var(--text);font-family:inherit;font-size:.8rem;outline:none;" onkeydown="if(event.key==='Enter')sendCommand()">
        <button onclick="sendCommand()" style="padding:8px 16px;border:none;border-radius:6px;background:var(--cyan);color:#000;cursor:pointer;font-family:inherit;font-size:.8rem;font-weight:700;">전송</button>
      </div>
      <div style="font-size:.75rem;min-height:20px;" id="cmd-result">--</div>
    </div>
    <div class="card">
      <div class="card-title">시나리오 타겟 연산 지표</div>
      <div class="ai-section">
        <div class="ai-status-bar" id="ai-status">대기중</div>
        <div class="ai-grid">
          <div class="ai-item"><div class="lbl">Gemini 장애물</div><div class="val" id="g-type">--</div></div>
          <div class="ai-item"><div class="lbl">추정 높이</div><div class="val" id="g-height">-- cm</div></div>
          <div class="ai-item"><div class="lbl">신뢰율</div><div class="val" id="g-conf">--</div></div>
          <div class="ai-item"><div class="lbl">연산 크루즈 출력</div><div class="val" id="v-target-cruise" style="color:var(--orange);">50 %</div></div>
        </div>
      </div>
    </div>
    <div class="card">
      <details>
        <summary>고급 가인 레지스터 튜닝</summary>
        <div class="sliders">
          <div class="sl-item"><label>BASE_SPEED <span id="v-base">50</span></label><input type="range" min="10" max="80" value="50" oninput="updateVal('v-base',this.value);sendCfg('base_speed',+this.value)"></div>
          <div class="sl-item"><label>THRESH <span id="v-thresh">43</span></label><input type="range" min="20" max="200" value="43" oninput="updateVal('v-thresh',this.value);sendCfg('thresh',+this.value)"></div>
          <div class="sl-item"><label>KP <span id="v-kp">0.80</span></label><input type="range" min="1" max="150" value="80" oninput="updateVal('v-kp',(this.value/100).toFixed(2));sendCfg('kp',this.value/100)"></div>
          <div class="sl-item"><label>DEAD_ZONE <span id="v-dz">15</span></label><input type="range" min="0" max="80" value="15" oninput="updateVal('v-dz',this.value);sendCfg('dead_zone',+this.value)"></div>
          <div class="sl-item"><label>ROI_TOP <span id="v-roi">0.70</span></label><input type="range" min="30" max="90" value="70" oninput="updateVal('v-roi',(this.value/100).toFixed(2));sendCfg('roi_top',this.value/100)"></div>
          <div class="sl-item"><label>SPIN_SPEED <span id="v-spin">25</span></label><input type="range" min="10" max="60" value="25" oninput="updateVal('v-spin',this.value);sendCfg('spin_speed',+this.value)"></div>
        </div>
      </details>
    </div>
  </div>
</div>
<script>
let curMode  = 'safe';
let prevRoute = 'A';
let activeKey = null;

function setMode(mode) {
  curMode = mode;
  fetch('/mode', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode})});
  document.getElementById('btn-safe').className = 'mode-btn' + (mode==='safe' ? ' active-safe' : '');
  document.getElementById('btn-fast').className = 'mode-btn' + (mode==='fast' ? ' active-fast' : '');
}
function startDrive() { fetch('/start', {method:'POST'}); }
function stopDrive()  { fetch('/stop',  {method:'POST'}); }
function sendCommand() {
  const text = document.getElementById('cmd-input').value.trim();
  if (!text) return;
  document.getElementById('cmd-result').textContent = 'Gemini 파싱 연산 중...';
  document.getElementById('cmd-result').style.color = 'var(--yellow)';
  fetch('/command', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({text})});
}

window.addEventListener('keydown', function(e) {
  if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA') return;
  if (["ArrowUp", "KeyW", "w", "W"].includes(e.key) && activeKey !== "forward") {
    activeKey = "forward"; sendMove("forward");
  } else if (["ArrowDown", "KeyS", "s", "S"].includes(e.key) && activeKey !== "backward") {
    activeKey = "backward"; sendMove("backward");
  } else if (["ArrowLeft", "KeyA", "a", "A"].includes(e.key) && activeKey !== "left") {
    activeKey = "left"; sendMove("left");
  } else if (["ArrowRight", "KeyD", "d", "D"].includes(e.key) && activeKey !== "right") {
    activeKey = "right"; sendMove("right");
  }
});

window.addEventListener('keyup', function(e) {
  if (["ArrowUp", "KeyW", "w", "W", "ArrowDown", "KeyS", "s", "S", "ArrowLeft", "KeyA", "a", "A", "ArrowRight", "KeyD", "d", "D"].includes(e.key)) {
    activeKey = null; sendMove("stop");
  }
});

function sendMove(dir) {
  fetch('/manual_control', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({direction: dir})
  }).catch(()=>{});
}

function updateVal(id, v) { document.getElementById(id).textContent = v; }
function sendCfg(k, v) { fetch('/config', {method:'POST',headers:{'Content-Type':'application/json'}, body:JSON.stringify({[k]:v})}); }
function poll() {
  fetch('/state').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent = (d.fps||'--') + ' fps';
    const dist = d.distance;
    const distStr = dist >= 0 ? dist.toFixed(1) : '--';
    const distCls = dist >= 0 && dist < 15 ? 'danger' : dist >= 0 && dist < 30 ? 'warn' : '';
    document.getElementById('dist-tag').textContent = distStr + (dist>=0?' cm':'');
    document.getElementById('dist-tag').className   = 'dist-val' + (distCls ? ' '+distCls : '');
    const ultraEl = document.getElementById('ultra-big');
    ultraEl.textContent  = dist >= 0 ? dist.toFixed(1) : '--';
    ultraEl.style.color  = dist>=0 && dist<15 ? 'var(--red)' : dist>=0 && dist<30 ? 'var(--yellow)' : 'var(--green)';
    document.getElementById('action-tag').textContent = d.action || '--';
    document.getElementById('err-val').textContent    = d.error  || 0;
    document.getElementById('green-val').textContent  = (d.green_area||0) + 'px';
    document.getElementById('junc-val').textContent   = d.junction_count || 0;
    document.getElementById('route-stat').textContent = d.route || '--';
    document.getElementById('v-target-cruise').textContent = (d.target_cruise_speed || 50) + ' %';
    const routeBig = document.getElementById('route-big');
    const newRoute = (d.route||'A');
    if (newRoute !== prevRoute) {
      routeBig.classList.add('changed');
      setTimeout(() => routeBig.classList.remove('changed'), 2000);
      prevRoute = newRoute;
    }
    routeBig.textContent = newRoute;
    const aiEl = document.getElementById('ai-status');
    const aiS  = d.ai_status || '대기중';
    aiEl.textContent = aiS;
    aiEl.className = 'ai-status-bar' + (aiS.includes('분석') ? ' analyzing' : aiS.includes('BLOCK')||aiS.includes('REROUTE')||aiS.includes('RUN')||aiS.includes('우회') ? ' detour' : aiS.includes('NORMAL')||aiS.includes('주행')||aiS.includes('안착') ? ' pass' : '');
    document.getElementById('g-type').textContent   = d.gemini_type   || '--';
    document.getElementById('g-height').textContent = (d.gemini_height || '--') + (d.gemini_height && d.gemini_height !== '--' ? ' cm' : '');
    document.getElementById('g-conf').textContent   = d.gemini_conf   || '--';
    const cmdEl = document.getElementById('cmd-result');
    if (d.cmd_result && d.cmd_result !== '--') {
      cmdEl.textContent = d.cmd_result;
      cmdEl.style.color = d.cmd_result.includes('FAST') ? 'var(--orange)' : d.cmd_result.includes('SAFE') ? 'var(--cyan)' : 'var(--red)';
    }
    const wEl = document.getElementById('weight-val');
    wEl.textContent = d.weight_g !== undefined ? d.weight_g.toFixed(1) + ' g' : '-- g';
    wEl.className   = 'val' + (d.weight_g > 300 ? ' danger' : d.weight_g > 140 ? ' warn' : '');
    if (d.user_mode) {
      document.getElementById('btn-safe').className = 'mode-btn' + (d.user_mode==='safe' ? ' active-safe' : '');
      document.getElementById('btn-fast').className = 'mode-btn' + (d.user_mode==='fast' ? ' active-fast' : '');
    }
  }).catch(()=>{});
}
setInterval(poll, 200);
</script>
</body>
</html>'''

if __name__ == '__main__':
    print("수치 캘리브레이션 전용 하이브리드 대시보드 V2 기동: http://192.168.0.50:5003")
    app.run(host='0.0.0.0', port=5003, threaded=True)