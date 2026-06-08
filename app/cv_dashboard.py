# app/cv_dashboard.py
# OpenCV 라인 추종 + 초록 분기점 표식 기반 경로 선택 대시보드
#
# 실행: python app/cv_dashboard.py
# 접속: http://192.168.0.50:5001

import sys, os, time, threading, copy
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

from sensors.camera import Camera
from sensors.motor  import MotorController

app   = Flask(__name__)
cam   = Camera(width=320, height=240)
motor = MotorController()

ROUTES = {
    "A": {1:"straight", 2: "stop"},
    "B": {1: "left",  2: "right", 3: "right", 4:"stop"},
    "C": {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight",6:"stop"},
}

config = {
    "base_speed":     45,
    "turn_speed":     30,
    "spin_speed":     25,
    "thresh":         43,
    "roi_top":        0.65,
    "dead_zone":      15,
    "kp":             0.8,
    "ki":             0.002,
    "running":        False,
    "route":          "A",
    "green_h_min":    30,
    "green_h_max":    85,
    "green_s_min":    80,
    "green_v_min":    50,
    "green_min_area": 200,
    "forward_time":   0.5,
}

lock  = threading.Lock()
state = {
    "cx": None, "error": 0, "correction": 0,
    "action": "정지", "lost": False, "fps": 0,
    "junction_count": 0, "route": "A", "green_area": 0,
}
latest_debug = None

# ══════════════════════════════════════════════════════
# 전역 분기점 카운트
# ══════════════════════════════════════════════════════
g_junction_count = 0
g_junction_lock  = threading.Lock()

def get_junction_count():
    with g_junction_lock:
        return g_junction_count

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
# 모터
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
    if frame is None or len(frame.shape) < 3 or frame.shape[2] != 3:
        return None, 0, False, frame

    h, w  = frame.shape[:2]
    roi_y = int(h * cfg["roi_top"])
    roi   = frame[roi_y:h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, cfg["thresh"], 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    M  = cv2.moments(binary)
    cx = None
    cy = roi_y
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) + roi_y

    # 초록 감지 (ROI 안에서만)
    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower = np.array([cfg["green_h_min"], cfg["green_s_min"], cfg["green_v_min"]])
    upper = np.array([cfg["green_h_max"], 255, 255])
    green_mask = cv2.inRange(roi_hsv, lower, upper)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    green_area  = int(np.sum(green_mask > 0))
    is_junction = green_area > cfg["green_min_area"]

    green_cx = green_cy = None
    if is_junction:
        Mg = cv2.moments(green_mask)
        if Mg['m00'] > 0:
            green_cx = int(Mg['m10'] / Mg['m00'])
            green_cy = int(Mg['m01'] / Mg['m00'])+ roi_y

    debug = frame.copy()
    cv2.line(debug, (0, roi_y), (w, roi_y), (255, 180, 0), 2)
    cv2.line(debug, (w//2, roi_y), (w//2, h), (0, 80, 255), 1)
    dz = cfg["dead_zone"]
    cv2.line(debug, (w//2 - dz, roi_y), (w//2 - dz, h), (80, 80, 255), 1)
    cv2.line(debug, (w//2 + dz, roi_y), (w//2 + dz, h), (80, 80, 255), 1)

    if cx is not None:
        cv2.circle(debug, (cx, cy), 8, (0, 255, 80), -1)
        cv2.line(debug, (cx, roi_y), (cx, h), (0, 255, 80), 2)
        err = cx - w // 2
        cv2.putText(debug, f"err={err:+d}", (5, roi_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 80), 1)

    if is_junction and green_cx is not None:
        cnts, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, cnts, -1, (0, 255, 0), 2)
        cv2.circle(debug, (green_cx, green_cy), 12, (0, 255, 0), 3)
        cv2.putText(debug, f"GREEN! {green_area}px", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(debug, f"green={green_area}px", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return cx, green_area, is_junction, debug

# ══════════════════════════════════════════════════════
# 분기점 행동
# ══════════════════════════════════════════════════════
def execute_junction(action, cfg):
    print(f"[Junction] 행동: {action}")

    # 분기점 표식 위를 지나칠 때까지 직진
    go_forward(cfg["base_speed"])
    time.sleep(cfg["forward_time"])
    motor.motorStop()
    time.sleep(0.1)

    if action == "stop":
        go_forward(cfg["base_speed"])
        time.sleep(1.5)
        motor.motorStop()
        config['running'] = False
        print("[Junction] 목적지 도착 — 정지")
        return

    if action == "straight":
        go_forward(cfg["base_speed"])
        time.sleep(0.4)
        return

    # elif action in ("left", "right"):
    #     spin_fn = spin_left if action == "left" else spin_right

    #     # 90도 고정 회전 (속도 30, 1.8초)
    #     spin_fn(30)
    #     time.sleep(2.0)
    #     motor.motorStop()
    #     time.sleep(0.1)

    #     # 미세 조정: action 방향으로만 고정 회전 (lost_dir 무시)
    #     fine_timeout = time.time() + 2.0
    #     while time.time() < fine_timeout:
    #         frame = cam.capture()
    #         frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #         h, w  = frame.shape[:2]
    #         roi_y = int(h * cfg["roi_top"])
    #         roi   = frame[roi_y:h, :]
    #         gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    #         _, binary = cv2.threshold(gray, cfg["thresh"], 255, cv2.THRESH_BINARY_INV)
    #         M = cv2.moments(binary)

    #         if M['m00'] > 0:
    #             cx = int(M['m10'] / M['m00'])
    #             if abs(cx - w // 2) < 20:
    #                 motor.motorStop()
    #                 print(f"[Junction] {action} 완료")
    #                 return

    #         # lost_dir 무시하고 항상 action 방향으로만 회전
    #         spin_fn(20)
    #         time.sleep(0.03)

    #     motor.motorStop()
    #     print(f"[Junction] {action} 타임아웃")
    elif action in ("left", "right"):
        spin_fn = spin_left if action == "left" else spin_right

        # 먼저 60도 고정 회전 (원래 선 벗어나기)
        spin_fn(35)
        time.sleep(1.5)

        # 그 다음 선이 중앙에 올 때까지 계속 회전 (최대 4초)
        fine_timeout = time.time() + 2.0
        while time.time() < fine_timeout:
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
                if abs(cx - w // 2) < 20:
                    motor.motorStop()
                    print(f"[Junction] {action} 완료")
                    return
            spin_fn(30)
            time.sleep(0.03)

        motor.motorStop()
        print(f"[Junction] {action} 타임아웃")

# ══════════════════════════════════════════════════════
# 주행 루프
# ══════════════════════════════════════════════════════
def drive_loop():
    global latest_debug

    CENTER_X          = 160
    lost_dir          = 1
    fps_t             = time.time()
    fps_count         = 0
    integral          = 0.0
    prev_time         = time.time()
    green_seen        = False
    junction_cooldown = 0.0
    current_route     = "A"
    stopped           = False

    while True:
        cfg = copy.deepcopy(config)

        frame = cam.capture()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cx, green_area, is_junction, debug_frame = detect_line_and_green(frame, cfg)

        encode_target = debug_frame if (debug_frame is not None and
                                        len(debug_frame.shape) == 3 and
                                        debug_frame.shape[2] == 3) else frame
        if encode_target is not None:
            _, dbg_jpeg = cv2.imencode('.jpg', encode_target, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with lock:
                latest_debug = dbg_jpeg.tobytes()

        with lock:
            current_route = cfg["route"]

        fps_count += 1
        if time.time() - fps_t >= 1.0:
            with lock:
                state["fps"] = fps_count
            fps_count = 0
            fps_t = time.time()

        if not cfg["running"]:
            if not stopped:
                motor.motorStop()
                stopped = True
            integral = 0.0
            with lock:
                state["action"]         = "정지"
                state["cx"]             = cx
                state["green_area"]     = green_area
                state["junction_count"] = get_junction_count()
            time.sleep(0.05)
            continue
        else:
            stopped = False

        now       = time.time()
        dt        = now - prev_time
        prev_time = now
        if dt > 0.1:
            dt = 0.033

        # 분기점 처리
        if is_junction and not green_seen and now > junction_cooldown:
            green_seen        = True
            junction_cooldown = now + 3.0
            junction_count    = increment_junction()
            integral          = 0.0

            action_at_junc = ROUTES[current_route].get(junction_count, "straight")
            print(f"\n[Junction] #{junction_count} | 루트 {current_route} | 행동: {action_at_junc}")

            with lock:
                state["junction_count"] = junction_count
                state["action"]         = f"분기점#{junction_count} {action_at_junc}"

            execute_junction(action_at_junc, cfg)
            continue

        if not is_junction:
            green_seen = False

        # 라인 추종 (PI 제어)
        correction = 0
        action     = "직진"

        if cx is None:
            integral = 0.0
            action   = f"탐색({'우' if lost_dir > 0 else '좌'})"
            if lost_dir > 0:
                spin_right(cfg["spin_speed"])
            else:
                spin_left(cfg["spin_speed"])
        else:
            error     = cx - CENTER_X
            integral += error * dt
            integral  = max(min(integral, 200), -200)

            correction = int(error * cfg["kp"] + integral * cfg["ki"])
            correction = max(min(correction, cfg["base_speed"] - 10), -(cfg["base_speed"] - 10))

            lost_dir = 1 if error > 0 else (-1 if error < 0 else lost_dir)

            if abs(error) < cfg["dead_zone"]:
                go_forward(cfg["base_speed"])
                action = "직진"
            elif correction > 0:
                right_speed = max(cfg["base_speed"] - abs(correction), cfg["turn_speed"])
                turn_right(cfg["base_speed"], right_speed)
                action = f"우회전 ({abs(correction)})"
            else:
                left_speed = max(cfg["base_speed"] - abs(correction), cfg["turn_speed"])
                turn_left(left_speed, cfg["base_speed"])
                action = f"좌회전 ({abs(correction)})"

        with lock:
            state["cx"]             = cx
            state["error"]          = cx - CENTER_X if cx else 0
            state["correction"]     = abs(correction)
            state["action"]         = action
            state["lost"]           = cx is None
            state["route"]          = current_route
            state["green_area"]     = green_area
            state["junction_count"] = get_junction_count()

        time.sleep(0.03)

# ══════════════════════════════════════════════════════
# Flask
# ══════════════════════════════════════════════════════
def gen_stream(get_fn):
    while True:
        with lock:
            frame = get_fn()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)

@app.route('/debug_feed')
def debug_feed():
    return Response(gen_stream(lambda: latest_debug),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/state')
def get_state():
    with lock:
        return jsonify(state)

@app.route('/config', methods=['GET', 'POST'])
def handle_config():
    if request.method == 'POST':
        data = request.get_json(force=True)
        for k, v in data.items():
            if k in config:
                config[k] = v
        return jsonify({'status': 'ok'})
    return jsonify(config)

@app.route('/start', methods=['POST'])
def start():
    config['running'] = True
    return jsonify({'status': 'running'})

@app.route('/stop', methods=['POST'])
def stop():
    config['running'] = False
    motor.motorStop()
    reset_junction()
    with lock:
        state["junction_count"] = 0
    return jsonify({'status': 'stopped'})

@app.route('/reset', methods=['POST'])
def reset():
    reset_junction()
    with lock:
        state["junction_count"] = 0
    return jsonify({'status': 'ok'})

@app.route('/')
def index():
    return render_template_string(HTML)

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Line Follow Dashboard</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #0f0f13; color: #e0e0e0;
         font-family: 'Courier New', monospace; min-height: 100vh; }
  header { background: #18181f; border-bottom: 1px solid #2a2a3a;
           padding: 12px 24px; display: flex;
           align-items: center; justify-content: space-between; }
  header h1 { font-size: 1rem; color: #4ade80; letter-spacing: 2px; }
  .fps { color: #666; font-size: 0.8rem; }
  .layout { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; padding: 12px; }
  .card { background: #18181f; border: 1px solid #2a2a3a; border-radius: 8px; padding: 14px; }
  .card h2 { font-size: 0.75rem; color: #666; letter-spacing: 2px; margin-bottom: 10px; }
  img.feed { width: 100%; border-radius: 4px; background: #000; display: block; }
  .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .stat { background: #0f0f13; border-radius: 6px; padding: 10px; }
  .stat .label { font-size: 0.7rem; color: #555; margin-bottom: 4px; }
  .stat .value { font-size: 1.1rem; color: #4ade80; font-weight: bold; }
  .stat .value.warn   { color: #f59e0b; }
  .stat .value.danger { color: #f87171; }
  .error-bar-bg { background: #0f0f13; border-radius: 4px;
                  height: 16px; position: relative; overflow: hidden; margin-top: 8px; }
  .error-bar-center { position: absolute; left: 50%; top: 0; width: 2px; height: 100%; background: #333; }
  .error-bar-fill { position: absolute; top: 0; height: 100%; transition: all 0.1s; }
  .route-row { display: flex; gap: 8px; margin-bottom: 12px; }
  .btn-route { flex: 1; padding: 12px; border: 2px solid #2a2a3a;
               background: #0f0f13; color: #666; border-radius: 6px;
               cursor: pointer; font-family: inherit; font-size: 0.9rem;
               font-weight: bold; letter-spacing: 1px; transition: all 0.15s; }
  .btn-route.active { border-color: #4ade80; color: #4ade80; background: #0f2010; }
  .btn-row { display: flex; gap: 10px; margin-bottom: 12px; }
  .btn-start { flex:1; padding: 12px; border: none; border-radius: 6px;
               cursor: pointer; background: #4ade80; color: #000;
               font-family: inherit; font-size: 0.85rem; }
  .btn-stop  { flex:1; padding: 12px; border: none; border-radius: 6px;
               cursor: pointer; background: #f87171; color: #000;
               font-family: inherit; font-size: 0.85rem; }
  .btn-reset { padding: 8px 16px; border: 1px solid #444; background: #1a1a22;
               color: #888; border-radius: 6px; cursor: pointer;
               font-family: inherit; font-size: 0.75rem; }
  .sliders { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
  .slider-item label { font-size: 0.7rem; color: #666; display: block; margin-bottom: 4px; }
  .slider-item input { width: 100%; accent-color: #4ade80; }
  .slider-item span  { font-size: 0.8rem; color: #4ade80; }
  .controls { grid-column: 1 / -1; }
  .junction-badge { display: inline-block; background: #0f2030;
                    border: 1px solid #0af; border-radius: 4px;
                    padding: 2px 8px; font-size: 0.75rem; color: #0af; margin-left: 8px; }
  .green-section { border-top: 1px solid #2a2a3a; padding-top: 12px; margin-top: 12px; }
</style>
</head>
<body>
<header>
  <h1>▶ LINE FOLLOW DASHBOARD (GREEN MARKER)</h1>
  <span class="fps" id="fps">-- fps</span>
</header>
<div class="layout">
  <div class="card" style="grid-column: 1 / -1;">
    <h2>CAMERA VIEW (CSI)</h2>
    <div style="max-width:640px;margin:0 auto;">
      <img class="feed" src="/debug_feed">
    </div>
  </div>
  <div class="card">
    <h2>STATUS</h2>
    <div class="status-grid">
      <div class="stat"><div class="label">ACTION</div><div class="value" id="action">--</div></div>
      <div class="stat"><div class="label">ERROR (px)</div><div class="value" id="error">--</div></div>
      <div class="stat"><div class="label">GREEN AREA</div><div class="value" id="green">--</div></div>
      <div class="stat">
        <div class="label">분기점 <span class="junction-badge" id="junc">0</span></div>
        <div class="value" id="route-state">--</div>
      </div>
    </div>
    <div class="error-bar-bg">
      <div class="error-bar-center"></div>
      <div class="error-bar-fill" id="error-bar" style="left:50%;width:0"></div>
    </div>
  </div>
  <div class="card controls">
    <h2>CONTROL</h2>
    <div style="font-size:0.7rem;color:#666;margin-bottom:6px">경로 선택</div>
    <div class="route-row">
      <button class="btn-route active" id="btn-A" onclick="selectRoute('A')">
        A<br><span style="font-size:0.65rem;font-weight:normal">직진만</span>
      </button>
      <button class="btn-route" id="btn-B" onclick="selectRoute('B')">
        B<br><span style="font-size:0.65rem;font-weight:normal">1좌→2우→5우</span>
      </button>
      <button class="btn-route" id="btn-C" onclick="selectRoute('C')">
        C<br><span style="font-size:0.65rem;font-weight:normal">1좌→2직→3우→4우→5직</span>
      </button>
    </div>
    <div class="btn-row">
      <button class="btn-start" onclick="startDrive()">▶ 주행 시작</button>
      <button class="btn-stop"  onclick="stopDrive()">■ 정지</button>
      <button class="btn-reset" onclick="resetJunc()">분기점 리셋</button>
    </div>
    <div class="sliders">
      <div class="slider-item">
        <label>BASE_SPEED <span id="v-base">35</span></label>
        <input type="range" min="10" max="80" value="35"
               oninput="updateVal('v-base',this.value);sendConfig('base_speed',+this.value)">
      </div>
      <div class="slider-item">
        <label>THRESH <span id="v-thresh">80</span></label>
        <input type="range" min="20" max="200" value="80"
               oninput="updateVal('v-thresh',this.value);sendConfig('thresh',+this.value)">
      </div>
      <div class="slider-item">
        <label>KP <span id="v-kp">0.70</span></label>
        <input type="range" min="1" max="80" value="70"
               oninput="updateVal('v-kp',(this.value/100).toFixed(2));sendConfig('kp',this.value/100)">
      </div>
      <div class="slider-item">
        <label>KI <span id="v-ki">0.002</span></label>
        <input type="range" min="0" max="20" value="2"
               oninput="updateVal('v-ki',(this.value/1000).toFixed(3));sendConfig('ki',this.value/1000)">
      </div>
      <div class="slider-item">
        <label>DEAD_ZONE <span id="v-dz">15</span></label>
        <input type="range" min="0" max="80" value="15"
               oninput="updateVal('v-dz',this.value);sendConfig('dead_zone',+this.value)">
      </div>
      <div class="slider-item">
        <label>ROI_TOP <span id="v-roi">0.65</span></label>
        <input type="range" min="30" max="90" value="65"
               oninput="updateVal('v-roi',(this.value/100).toFixed(2));sendConfig('roi_top',this.value/100)">
      </div>
      <div class="slider-item">
        <label>FORWARD_TIME <span id="v-fwd">1.5</span>s</label>
        <input type="range" min="1" max="30" value="15"
               oninput="updateVal('v-fwd',(this.value/10).toFixed(1));sendConfig('forward_time',this.value/10)">
      </div>
      <div class="slider-item">
        <label>SPIN_SPEED <span id="v-spin">25</span></label>
        <input type="range" min="10" max="60" value="25"
               oninput="updateVal('v-spin',this.value);sendConfig('spin_speed',+this.value)">
      </div>
    </div>
    <div class="green-section">
      <div style="font-size:0.7rem;color:#666;margin-bottom:6px">초록 분기점 인식 설정</div>
      <div class="sliders">
        <div class="slider-item">
          <label>H_MIN <span id="v-hmin">30</span></label>
          <input type="range" min="20" max="60" value="30"
                 oninput="updateVal('v-hmin',this.value);sendConfig('green_h_min',+this.value)">
        </div>
        <div class="slider-item">
          <label>H_MAX <span id="v-hmax">85</span></label>
          <input type="range" min="60" max="100" value="85"
                 oninput="updateVal('v-hmax',this.value);sendConfig('green_h_max',+this.value)">
        </div>
        <div class="slider-item">
          <label>S_MIN <span id="v-smin">80</span></label>
          <input type="range" min="30" max="200" value="80"
                 oninput="updateVal('v-smin',this.value);sendConfig('green_s_min',+this.value)">
        </div>
        <div class="slider-item">
          <label>V_MIN <span id="v-vmin">50</span></label>
          <input type="range" min="30" max="200" value="50"
                 oninput="updateVal('v-vmin',this.value);sendConfig('green_v_min',+this.value)">
        </div>
        <div class="slider-item">
          <label>MIN_AREA <span id="v-area">200</span></label>
          <input type="range" min="50" max="2000" value="200"
                 oninput="updateVal('v-area',this.value);sendConfig('green_min_area',+this.value)">
        </div>
      </div>
    </div>
  </div>
</div>
<script>
function updateVal(id, val) { document.getElementById(id).textContent = val; }
function startDrive() { fetch('/start', {method:'POST'}); }
function stopDrive()  { fetch('/stop',  {method:'POST'}); }
function resetJunc()  { fetch('/reset', {method:'POST'}); }
function selectRoute(r) {
  ['A','B','C'].forEach(x => {
    document.getElementById('btn-'+x).classList.toggle('active', x === r);
  });
  sendConfig('route', r);
}
function sendConfig(key, value) {
  fetch('/config', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({[key]: value})});
}
function updateState() {
  fetch('/state').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent    = (d.fps||'--')+' fps';
    document.getElementById('action').textContent = d.action||'--';
    document.getElementById('green').textContent  = (d.green_area||0)+' px';
    document.getElementById('junc').textContent   = d.junction_count || 0;
    document.getElementById('route-state').textContent = '루트 ' + (d.route||'--');
    const err = d.error || 0;
    const errEl = document.getElementById('error');
    errEl.textContent = (err > 0 ? '+' : '') + err;
    errEl.className = 'value' + (Math.abs(err) > 60 ? ' danger' : Math.abs(err) > 20 ? ' warn' : '');
    const pct = Math.min(Math.abs(err) / 160 * 50, 50);
    const bar = document.getElementById('error-bar');
    if (err > 0) { bar.style.left='50%'; bar.style.width=pct+'%'; bar.style.background='#f59e0b'; }
    else { bar.style.left=(50-pct)+'%'; bar.style.width=pct+'%'; bar.style.background='#4ade80'; }
  }).catch(()=>{});
}
setInterval(updateState, 150);
</script>
</body>
</html>'''

if __name__ == '__main__':
    threading.Thread(target=drive_loop, daemon=True, name="drive").start()
    print("대시보드 시작: http://192.168.0.50:5001")
    app.run(host='0.0.0.0', port=5001, threaded=True)