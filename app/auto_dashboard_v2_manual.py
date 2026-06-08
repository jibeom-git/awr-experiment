# app/auto_dashboard_v2_manual.py
# 자율 주행 대시보드 - 키보드 수동 조종 및 자동 AI 시나리오 결합본

import sys, os, time, threading, copy, warnings
from typing import Any
sys.path.insert(0, '/home/pi/insite')

# =====================================================================
# [구글 제미나이 API 키 통합 관리 센터]
# 새로운 계정으로 키를 발급받으시면 오직 여기 한 곳만 수정해 주시면 됩니다!
# =====================================================================
os.environ["GEMINI_API_KEY"] = "제미나이 API 키".strip()

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

from sensors.camera import Camera
from sensors.motor import MotorController

app = Flask(__name__)

# -- 모터 전역 인스턴스 선언
motor = MotorController()

# -- Pi 카메라
try:
    cam = Camera(width=320, height=240)
    CAM_AVAILABLE = True
    print("[OK] Pi 카메라")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] Pi 카메라: {e}")

# -- USB 웹캠 (중복 자원 점유 방지 및 시스템 실측 인덱스 선제 할당)
try:
    webcam = None
    # 💡 libcamera와 비디오 장치 충돌을 원천 차단하기 위해 안전한 순서대로 개방을 검증합니다.
    for idx in [0, 1, 2, 4]:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            webcam = cap
            webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print(f"[OK] USB 웹캠 장치 활성화 성공 (시스템 바인딩 인덱스: {idx}번)")
            break
        cap.release()

    WEBCAM_AVAILABLE = webcam is not None if webcam else False
    if not WEBCAM_AVAILABLE:
        print("[SKIP] 웹캠 자원을 획득하지 못했습니다. 물리 포트 연결을 확인하세요.")
except Exception as e:
    webcam = None
    WEBCAM_AVAILABLE = False
    print(f"[SKIP] 웹캠 예외 발생: {e}")

# -- IMU
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

# -- 초음파
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

# -- 로드셀 HX711
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

# ══════════════════════════════════════════════════════
# 라우트 정의
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
# 전역 공유 상태 및 시나리오 연동 변수 구조체
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
    "has_1cm_bump":   False,         # 1cm 장애물 통과 활성화 플래그
    "bump_speed_override": 0,        # 장애물 돌파 시 지정 속도 매핑 변수
    "target_cruise_speed": 45,       # 실시간 연산된 타겟 크루즈 속도
    "current_direction": "stop"      # 실시간 키보드 인가 방향 로그
}

latest_picam  = None
latest_webcam = None

# ── 분기점 카운트 전역 뮤텍스 관리 ───────────────────────
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

# ── 저수준 모터 기본 구동 드라이버 ───────────────────────
def go_forward(speed):
    motor.set_motor(1, 1, speed); motor.set_motor(2, 1, speed)
    motor.set_motor(3, 1, speed); motor.set_motor(4, 1, speed)

def spin_left(speed):
    motor.set_motor(4,-1,speed); motor.set_motor(3,-1,speed)
    motor.set_motor(2, 1,speed); motor.set_motor(1, 1,speed)

def spin_right(speed):
    motor.set_motor(4, 1,speed); motor.set_motor(3, 1,speed)
    motor.set_motor(2,-1,speed); motor.set_motor(1,-1,speed)

# ── 비전 라인 픽셀 연산 커널 ────────────────────────────
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
# VLM 기반 수동 주행 연동 최적화 의사결정 매트릭스
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
            print(f"[VLM Engine] 피드백 지연: {e}")

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

        action = "pass"
        speed = config["base_speed"]
        
        # 친구분이 새로 명시한 4차원 시나리오 조건 수식 정밀 주입
        if mode == "fast":
            if not heavy:
                if route == "A":
                    if bump == "none": action, speed = "pass", 60
                    else: action = "detour"
                elif route == "B":
                    if bump == "none": action, speed = "pass", 45
                    elif bump == "1cm": action, speed = "pass", 70
                    elif bump == "2cm": action = "detour"
                elif route == "C":
                    if bump == "none": action, speed = "pass", 45
                    elif bump == "1cm": action, speed = "pass", 70
                    elif bump == "2cm": action = "stop"
            else:
                if route == "A": action = "detour"
                elif route == "B":
                    if bump == "none": action, speed = "pass", 45
                    else: action = "detour"
                elif route == "C":
                    if bump == "none": action, speed = "pass", 45
                    elif bump == "1cm": action, speed = "pass", 70
                    elif bump == "2cm": action = "stop"
        else: # safe mode
            if route == "A":
                action = "detour"
            elif route == "B":
                if bump == "none": action, speed = "pass", 45
                elif bump == "1cm":
                    if not heavy: action, speed = "pass", 70
                    else: action = "detour"
                elif bump == "2cm": action = "detour"
            elif route == "C":
                if bump == "none": action, speed = "pass", 45
                elif bump == "1cm": action, speed = "pass", 70
                elif bump == "2cm": action = "stop"

        print(f"[Matrix Determination] 결정 행동: {action} / 설정 속도: {speed}%")

        if action == "detour":
            next_map = "B" if route == "A" else "C"
            new_route_str = f"{route}->{next_map}"
            # 수동 모드이므로 물리 복귀를 생략하고 논리 타임스탬프와 경로 타겟팅만 강제 락킹
            reset_junction()
            config['route'] = new_route_str
            with lock:
                state["junction_count"] = 0
                state["route"] = new_route_str
                state["ai_status"] = f"우회결정->{next_map}"
            config["running"] = True
        elif action == "stop":
            motor.motorStop()
            config["running"] = False
            with lock:
                state["action"] = "최종정지"
                state["ai_status"] = "장애물차단정지"
        else:
            if bump == "1cm":
                with lock:
                    state["ai_status"] = "1cm단차구간(속도 가변)"
                    state["has_1cm_bump"] = True
                    state["bump_speed_override"] = speed
                config["running"] = True
            else:
                config["running"] = True
                with lock: state["ai_status"] = "정상주행판단"

    except Exception as e:
        print(f"[AI Exception] {e}")
        config["running"] = True
    finally:
        drive_loop._ai_running = False
        drive_loop._ai_cooldown = time.time() + 5.0

# ── 백그라운드 하드웨어 스레드 계층 ───────────────────────
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
    # 💡 [자원 잠금 완치] 루프 내부에서 카메라를 새로 열지 않고, 상단에서 검증 완료된 단일 인스턴스(webcam)를 그대로 상속받아 사용합니다.
    # 이 구조를 통해 리눅스의 'Device or resource busy' 락 현상이 완벽하게 해결됩니다.
    if webcam is None or not WEBCAM_AVAILABLE:
        print("[FAIL] 초기 마운트된 웹캠 인스턴스가 없어 송출 프로세스를 생략합니다.")
        return

    while True:
        ret, frame = webcam.read()
        if ret:
            # 🔴 대시보드 화면상에서 R-B 색상이 뒤바뀌어 파랗게 나오지 않도록 별도의 역전 필터 없이 원본 데이터 그대로 스트림에 태웁니다.
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
                        state["action"] = "수동 조종 대기"
                        state["ai_status"] = "정상 주행 승인"
                    config["running"] = True 
                    return
            except: time.sleep(0.1)
        time.sleep(0.1)

# ══════════════════════════════════════════════════════
# 주행 인지 루프 (자동 조향 차단 및 시나리오 텔레메트리 보존)
# ══════════════════════════════════════════════════════
def drive_loop():
    global latest_picam, g_junction_cooldown

    CENTER_X          = 160
    fps_t             = time.time()
    fps_count         = 0
    prev_time         = time.time()
    green_seen        = False

    while True:
        cfg = copy.deepcopy(config)
        frame = None
        if cam is not None:
            try: frame = cam.capture()
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

        if not cfg["running"]:
            with lock:
                state["green_area"] = green_area
                state["junction_count"] = get_junction_count()
            time.sleep(0.05)
            continue

        now = time.time()
        dt  = now - prev_time; prev_time = now
        if dt > 0.1: dt = 0.033

        # -- 전방 장애물 및 선 차단(Line Cut) 연동 트리거 구역 --
        with lock: dist_now = state["distance"]
        is_line_cut_by_obstacle = (cx is None and 0 < dist_now < 40)
        
        if (cfg["running"] and (dist_now < 20 or is_line_cut_by_obstacle) and
                not getattr(drive_loop, '_ai_running', False) and
                time.time() > getattr(drive_loop, '_ai_cooldown', 0)):
            drive_loop._ai_running = True
            config["running"] = False
            motor.motorStop()
            with lock: state["current_direction"] = "stop"
            time.sleep(0.5)
            frame_snap = frame.copy() if frame is not None else np.zeros((240,320,3), np.uint8)

            def _run_ai(r=current_route, f=frame_snap):
                try:
                    time.sleep(0.5)
                    run_ai_decision(r, f)
                finally: drive_loop._ai_running = False

            threading.Thread(target=_run_ai, daemon=True, name="ai").start()
            continue

        # ── 실시간 시나리오 가변 크루즈 속도 연산 ──
        mode = cfg["user_mode"].lower()
        weight = state.get("weight_g", 0.0)
        heavy = weight >= 140.0
        jc = get_junction_count()
        
        # 3번째 노드부터는 종착지 정렬을 위해 45% 잠금
        if jc >= 3:
            cruise_speed = 45
            action_state = "최종정렬주행"
        elif state.get("has_1cm_bump"):
            cruise_speed = state.get("bump_speed_override", 70)
            action_state = "단차극복주행"
        else:
            if mode == "fast" and not heavy and current_route == "A":
                cruise_speed = 60
            else:
                cruise_speed = 45
            action_state = "평지정속"

        with lock:
            state["target_cruise_speed"] = cruise_speed

        # -- 분기점(노드) 수동 조종 대응형 자동 카운트 맵 레이어 --
        if is_junction and not green_seen and now > g_junction_cooldown:
            green_seen = True
            jc = increment_junction()
            print(f"[Junction Event Localized] 노드 검출 완료 #{jc} | 현재 경로: {current_route}")
            with lock:
                state["junction_count"] = jc
            
            g_junction_cooldown = time.time() + 2.5
            green_seen = True
            continue

        if not is_junction:
            green_seen = False

        # -- 수동 조종 기동 로그 상태 매핑 렌더링 구역 --
        with lock:
            state["error"]  = cx - CENTER_X if cx else 0
            state["lost"]   = cx is None
            state["route"]  = current_route
            state["green_area"] = green_area
            
            cur_dir = state["current_direction"]
            if cur_dir == "forward": state["action"] = f"{action_state}(수동전진:{cruise_speed}%)"
            elif cur_dir == "backward": state["action"] = f"{action_state}(수동후진:45%)"
            elif cur_dir == "left": state["action"] = f"{action_state}(수동좌회전:35%)"
            elif cur_dir == "right": state["action"] = f"{action_state}(수동우회전:35%)"
            else: state["action"] = "수동조종_대기정지"

        time.sleep(0.03)

threading.Thread(target=drive_loop, daemon=True, name="drive").start()

# ══════════════════════════════════════════════════════
# [신설] 웹 대시보드 키보드 이벤트 연동형 조종 핸들러 라우트
# ══════════════════════════════════════════════════════
@app.route('/manual_control', methods=['POST'])
def manual_control():
    data = request.get_json(force=True)
    direction = data.get('direction', 'stop')
    
    with lock:
        cruise_speed = state["target_cruise_speed"]
        state["current_direction"] = direction

    # 실시간으로 자동 연산되는 시나리오 목표 속도를 수동 제어 전진 속도에 즉각 동기화
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

# ── Flask 웹 서비스 인프라 엔드포인트 구역 ───────────────────
def gen_stream(get_fn):
    while True:
        with lock: frame = get_fn()
        if frame: yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
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
    
    if mode == "fast":
        route = "B" if heavy else "A"
    else:
        route = "B"
        
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

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json(force=True)
    user_text = data.get('text', '').strip()
    if not user_text: return jsonify({'status': 'error', 'msg': '텍스트 누락'})

    def _parse():
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
            if not config['running']:
                config['route'] = route
                with lock: state['route'] = route
            with lock:
                state['cmd_result'] = f"[{mode.upper()}] {reason}"
                state['user_mode'] = mode
            print(f"[Commander Routing Synced] mode={mode} route={route}")
        except Exception as e:
            fallback_mode = config.get('user_mode', 'safe')
            fallback_route = "B" if fallback_mode == "safe" else "A"
            with lock: 
                state['cmd_result'] = "시스템 라우팅 예외 복구 적용"
                state['route'] = fallback_route
            config['route'] = fallback_route
                        
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
<title>AWR Auto Drive (Manual Key Control)</title>
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
  .cam-wrap img{width:100%;display:block;}
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
  <div class="logo">AWR MANUAL COMMAND</div>
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
    <div class="card-title">Pi Camera · Line Target Monitor</div>
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
      <div class="stat"><div class="lbl">분기점 인지</div><div class="val" id="junc-val">0</div></div>
      <div class="stat"><div class="lbl">현재 루트</div><div class="val" id="route-stat">A</div></div>
    </div>
    <div class="stat" style="grid-column:1/-1; margin-top:8px;">
        <div class="lbl">적재 무게</div>
        <div class="val" id="weight-val">-- g</div>
    </div>
  </div>
  <div class="card">
    <div class="card-title">USB Webcam · Obstacle View</div>
    <div class="cam-wrap">
      <img src="/webcam_feed" alt="Webcam">
    </div>
    <div class="keyboard-guide">
      <p style="margin-bottom: 6px; font-weight: 600; color: var(--text);">[영상 촬영용 키보드 수동 조종 가이드]</p>
      <p>대시보드 화면을 클릭한 후 키보드를 누르고 있으면 로봇이 기동합니다.</p>
      <p style="margin-top: 4px;"><span class="key-cap">W</span> 전진 (시나리오 속도 연동) | <span class="key-cap">S</span> 후진 (45% 고정)</p>
      <p><span class="key-cap">A</span> 제자리 좌회전 | <span class="key-cap">D</span> 제자리 우회전</p>
      <p style="margin-top: 4px; color: var(--orange);">키를 떼면 모터가 즉시 제동 정지합니다.</p>
    </div>
  </div>
  <div class="side">
    <div class="card">
      <div class="route-display">
        <div class="route-label">CURRENT ROUTE</div>
        <div class="route-val" id="route-big">A</div>
      </div>
      <div class="ctrl-row">
        <button class="btn-start" onclick="startDrive()">자율 시스템 가동</button>
        <button class="btn-stop"  onclick="stopDrive()">강제 제동정지</button>
      </div>
    </div>
    <div class="card" style="text-align:center;padding:16px;">
      <div class="card-title">Ultrasonic Distance</div>
      <div style="font-size:2.6rem;font-weight:700;color:var(--green);line-height:1;transition:color .2s;" id="ultra-big">--</div>
      <div style="font-size:.7rem;color:var(--sub);margin-top:4px;">cm</div>
    </div>
    <div class="card">
      <div class="card-title">자연어 명령</div>
      <div style="display:flex;gap:8px;margin-bottom:8px;">
        <input type="text" id="cmd-input" placeholder="예: b경로로 안전하게 세팅해줘" style="flex:1;background:rgba(0,0,0,.3);border:1px solid var(--border);border-radius:6px;padding:8px 12px;color:var(--text);font-family:inherit;font-size:.8rem;outline:none;" onkeydown="if(event.key==='Enter')sendCommand()">
        <button onclick="sendCommand()" style="padding:8px 16px;border:none;border-radius:6px;background:var(--cyan);color:#000;cursor:pointer;font-family:inherit;font-size:.8rem;font-weight:700;">전송</button>
      </div>
      <div style="font-size:.75rem;min-height:20px;" id="cmd-result">--</div>
    </div>
    <div class="card">
      <div class="card-title">AI 전역 시나리오 판단 로그</div>
      <div class="ai-section">
        <div class="ai-status-bar" id="ai-status">대기중</div>
        <div class="ai-grid">
          <div class="ai-item"><div class="lbl">Gemini 장애물</div><div class="val" id="g-type">--</div></div>
          <div class="ai-item"><div class="lbl">추정 높이</div><div class="val" id="g-height">-- cm</div></div>
          <div class="ai-item"><div class="lbl">신뢰도</div><div class="val" id="g-conf">--</div></div>
          <div class="ai-item"><div class="lbl">타겟 크루즈 속도</div><div class="val" id="v-target-cruise" style="color:var(--orange);">45 %</div></div>
        </div>
      </div>
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
  document.getElementById('cmd-result').textContent = 'Gemini 판단 중...';
  document.getElementById('cmd-result').style.color = 'var(--yellow)';
  fetch('/command', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({text})});
}

window.addEventListener('keydown', function(e) {
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
    document.getElementById('v-target-cruise').textContent = (d.target_cruise_speed || 45) + ' %';
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
    aiEl.className = 'ai-status-bar' + (aiS.includes('분석') ? ' analyzing' : aiS.includes('BLOCK')||aiS.includes('REROUTE')||aiS.includes('RUN')||aiS.includes('우회') ? ' detour' : aiS.includes('NORMAL')||aiS.includes('주행') ? ' pass' : '');
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
    print("자율주행 대시보드 V2 (수동 키보드 제어판) 기동: http://192.168.0.50:5003")
    app.run(host='0.0.0.0', port=5003, threaded=True)