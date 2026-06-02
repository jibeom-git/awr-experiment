# app/cv_dashboard.py
# OpenCV 라인 추종 + 분기점 경로 선택 대시보드
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

# ══════════════════════════════════════════════════════
# 경로 정의
# ══════════════════════════════════════════════════════
ROUTES = {
    "A": {},
    "B": {1: "left",  2: "right", 5: "right"},
    "C": {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight"},
}

# ══════════════════════════════════════════════════════
# 설정값
# ══════════════════════════════════════════════════════
config = {
    "base_speed":   45,
    "turn_speed":   20,
    "spin_speed":   30,
    "thresh":       80,
    "roi_top":      0.6,
    "dead_zone":    150,    # 낮게 설정 → 작은 오차도 보정
    "kp":           0.20,
    "ki":           0.002, # 적분 게인
    "running":      False,
    "route":        "A",
}

lock  = threading.Lock()
state = {
    "cx": None, "error": 0, "correction": 0,
    "action": "정지", "lost": False, "fps": 0,
    "junction_count": 0, "route": "A",
}
latest_raw   = None
latest_debug = None

# ══════════════════════════════════════════════════════
# 모터
# M4(왼앞) M2(오른앞)
# M3(왼뒤) M1(오른뒤)
# motor.py가 DIR 보정 처리 → direction=1 이 전진
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
# 라인 감지 + 분기점 감지
# ══════════════════════════════════════════════════════
def detect_line(frame, cfg):
    """
    검은 선 중심 및 분기점 감지.
    Returns:
        cx: 선 중심 X (없으면 None)
        is_split: 분기점 여부 (윤곽선 2개 이상)
        left_cx: 왼쪽 덩어리 중심 (분기점 시)
        right_cx: 오른쪽 덩어리 중심 (분기점 시)
        debug: 시각화 프레임
    """
    h, w  = frame.shape[:2]
    roi_y = int(h * cfg["roi_top"])
    roi   = frame[roi_y:h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, cfg["thresh"], 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # 윤곽선 찾기
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 작은 노이즈 제거 (면적 50 이하)
    contours = [c for c in contours if cv2.contourArea(c) > 50]

    cx        = None
    is_split  = False
    left_cx   = None
    right_cx  = None
    cy        = roi_y

    if len(contours) == 0:
        pass  # 선 없음

    elif len(contours) == 1:
        # 직선 구간
        M = cv2.moments(contours[0])
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00']) + roi_y

    else:
        # 분기점: 윤곽선 2개 이상
        is_split = True
        centers = []
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 0:
                centers.append(int(M['m10'] / M['m00']))

        centers.sort()
        left_cx  = centers[0]
        right_cx = centers[-1]
        cx = (left_cx + right_cx) // 2  # 전체 중심

    # 디버그 프레임
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

    if is_split:
        cv2.putText(debug, "JUNCTION!", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        if left_cx:
            cv2.circle(debug, (left_cx, cy), 6, (255, 100, 0), -1)
        if right_cx:
            cv2.circle(debug, (right_cx, cy), 6, (0, 100, 255), -1)
    else:
        cv2.putText(debug, "LINE LOST" if cx is None else "TRACKING",
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 60, 255) if cx is None else (0, 255, 80), 1)

    return cx, is_split, left_cx, right_cx, debug

# ══════════════════════════════════════════════════════
# 주행 루프
# ══════════════════════════════════════════════════════
def drive_loop():
    global latest_raw, latest_debug

    CENTER_X       = 160   # 320px 기준 중앙
    lost_dir       = 1
    fps_t          = time.time()
    fps_count      = 0

    # PI 제어 변수
    integral       = 0.0
    prev_time      = time.time()

    # 분기점 상태
    junction_count = 0
    junc_start     = None   # 분기점 첫 감지 시각
    JUNC_CONFIRM   = 0.2    # 0.3초 이상 지속 시 분기점 확정
    in_junction    = False  # 분기점 처리 중 여부

    current_route  = "A"

    while True:
        cfg   = copy.deepcopy(config)
        frame = cam.capture()

        _, raw_jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
        cx, is_split, left_cx, right_cx, debug_frame = detect_line(frame, cfg)
        _, dbg_jpeg = cv2.imencode('.jpg', debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 65])

        with lock:
            latest_raw   = raw_jpeg.tobytes()
            latest_debug = dbg_jpeg.tobytes()
            current_route = cfg["route"]

        fps_count += 1
        if time.time() - fps_t >= 1.0:
            with lock:
                state["fps"] = fps_count
            fps_count = 0
            fps_t = time.time()

        if not cfg["running"]:
            motor.stop()
            integral = 0.0
            with lock:
                state["action"] = "정지"
                state["cx"]     = cx
            time.sleep(0.05)
            continue

        # 시간 간격
        now      = time.time()
        dt       = now - prev_time
        prev_time = now
        if dt > 0.1:
            dt = 0.033

        correction = 0
        action     = "직진"

        # ── 분기점 감지 ───────────────────────────────
        if is_split and not in_junction:
            if junc_start is None:
                junc_start = time.time()
            elif time.time() - junc_start >= JUNC_CONFIRM:
                # 분기점 확정
                junction_count += 1
                in_junction     = True
                junc_start      = None
                integral        = 0.0   # 적분 리셋

                action_at_junc = ROUTES[current_route].get(junction_count, "straight")
                print(f"[Junction] #{junction_count} | 루트 {current_route} | 행동: {action_at_junc}")

                with lock:
                    state["junction_count"] = junction_count
                    state["action"]         = f"분기점#{junction_count} {action_at_junc}"

                # 행동 실행
                if action_at_junc == "straight":
                    go_forward(cfg["base_speed"])
                    time.sleep(0.4)

                elif action_at_junc == "left" and left_cx is not None:
                    # 왼쪽 덩어리 중심 따라가기
                    err = left_cx - CENTER_X
                    while True:
                        frame2 = cam.capture()
                        cx2, is_split2, lx2, rx2, _ = detect_line(frame2, cfg)
                        if not is_split2:
                            in_junction = False
                            break
                        if lx2 is not None:
                            err2 = lx2 - CENTER_X
                            if abs(err2) < cfg["dead_zone"]:
                                go_forward(cfg["base_speed"])
                            elif err2 > 0:
                                turn_right(cfg["base_speed"],
                                           max(cfg["base_speed"] - int(abs(err2)*cfg["kp"]), cfg["turn_speed"]))
                            else:
                                turn_left(max(cfg["base_speed"] - int(abs(err2)*cfg["kp"]), cfg["turn_speed"]),
                                          cfg["base_speed"])
                        time.sleep(0.03)

                elif action_at_junc == "right" and right_cx is not None:
                    # 오른쪽 덩어리 중심 따라가기
                    while True:
                        frame2 = cam.capture()
                        cx2, is_split2, lx2, rx2, _ = detect_line(frame2, cfg)
                        if not is_split2:
                            in_junction = False
                            break
                        if rx2 is not None:
                            err2 = rx2 - CENTER_X
                            if abs(err2) < cfg["dead_zone"]:
                                go_forward(cfg["base_speed"])
                            elif err2 > 0:
                                turn_right(cfg["base_speed"],
                                           max(cfg["base_speed"] - int(abs(err2)*cfg["kp"]), cfg["turn_speed"]))
                            else:
                                turn_left(max(cfg["base_speed"] - int(abs(err2)*cfg["kp"]), cfg["turn_speed"]),
                                          cfg["base_speed"])
                        time.sleep(0.03)

                in_junction = False
                continue

        elif not is_split:
            junc_start  = None
            in_junction = False

        # ── 라인 추종 (PI 제어) ───────────────────────
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
            # 적분 와인드업 방지
            integral  = max(min(integral, 200), -200)

            p_term     = error    * cfg["kp"]
            i_term     = integral * cfg["ki"]
            correction = int(p_term + i_term)
            correction = max(min(correction, cfg["base_speed"] - 10), -(cfg["base_speed"] - 10))

            if error > 0:
                lost_dir = 1
            elif error < 0:
                lost_dir = -1

            if abs(error) < cfg["dead_zone"]:
                go_forward(cfg["base_speed"])
                action = "직진"
            elif correction > 0:
                # 오른쪽으로 치우침 → 우회전
                right_speed = max(cfg["base_speed"] - abs(correction), cfg["turn_speed"])
                turn_right(cfg["base_speed"], right_speed)
                action = f"우회전 ({abs(correction)})"
            else:
                # 왼쪽으로 치우침 → 좌회전
                left_speed = max(cfg["base_speed"] - abs(correction), cfg["turn_speed"])
                turn_left(left_speed, cfg["base_speed"])
                action = f"좌회전 ({abs(correction)})"

        with lock:
            state["cx"]         = cx
            state["error"]      = cx - CENTER_X if cx else 0
            state["correction"] = abs(correction)
            state["action"]     = action
            state["lost"]       = cx is None
            state["route"]      = current_route

        time.sleep(0.03)

# ══════════════════════════════════════════════════════
# Flask 라우트
# ══════════════════════════════════════════════════════
def gen_stream(get_fn):
    while True:
        with lock:
            frame = get_fn()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)

@app.route('/raw_feed')
def raw_feed():
    return Response(gen_stream(lambda: latest_raw),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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
    motor.stop()
    return jsonify({'status': 'stopped'})

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
  .layout { display: grid; grid-template-columns: 1fr 1fr;
            gap: 12px; padding: 12px; }
  .card { background: #18181f; border: 1px solid #2a2a3a;
          border-radius: 8px; padding: 14px; }
  .card h2 { font-size: 0.75rem; color: #666;
             letter-spacing: 2px; margin-bottom: 10px; }
  img.feed { width: 100%; border-radius: 4px; background: #000; display: block; }
  .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .stat { background: #0f0f13; border-radius: 6px; padding: 10px; }
  .stat .label { font-size: 0.7rem; color: #555; margin-bottom: 4px; }
  .stat .value { font-size: 1.1rem; color: #4ade80; font-weight: bold; }
  .stat .value.warn   { color: #f59e0b; }
  .stat .value.danger { color: #f87171; }
  .error-bar-bg { background: #0f0f13; border-radius: 4px;
                  height: 16px; position: relative; overflow: hidden; margin-top: 8px; }
  .error-bar-center { position: absolute; left: 50%; top: 0;
                      width: 2px; height: 100%; background: #333; }
  .error-bar-fill { position: absolute; top: 0; height: 100%;
                    transition: all 0.1s; }
  /* 경로 선택 버튼 */
  .route-row { display: flex; gap: 8px; margin-bottom: 12px; }
  .btn-route { flex: 1; padding: 12px; border: 2px solid #2a2a3a;
               background: #0f0f13; color: #666; border-radius: 6px;
               cursor: pointer; font-family: inherit; font-size: 0.9rem;
               font-weight: bold; letter-spacing: 1px; transition: all 0.15s; }
  .btn-route.active { border-color: #4ade80; color: #4ade80; background: #0f2010; }
  .btn-route:hover  { border-color: #4ade80; color: #4ade80; }
  .btn-row { display: flex; gap: 10px; margin-bottom: 12px; }
  .btn-start { flex:1; padding: 12px; border: none; border-radius: 6px;
               cursor: pointer; font-family: inherit; font-size: 0.85rem;
               background: #4ade80; color: #000; letter-spacing: 1px; }
  .btn-stop  { flex:1; padding: 12px; border: none; border-radius: 6px;
               cursor: pointer; font-family: inherit; font-size: 0.85rem;
               background: #f87171; color: #000; letter-spacing: 1px; }
  .sliders { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
  .slider-item label { font-size: 0.7rem; color: #666; display: block; margin-bottom: 4px; }
  .slider-item input { width: 100%; accent-color: #4ade80; }
  .slider-item span  { font-size: 0.8rem; color: #4ade80; }
  .controls { grid-column: 1 / -1; }
  .junction-badge { display: inline-block; background: #0f2030;
                    border: 1px solid #0af; border-radius: 4px;
                    padding: 2px 8px; font-size: 0.75rem; color: #0af;
                    margin-left: 8px; }
</style>
</head>
<body>
<header>
  <h1>▶ LINE FOLLOW DASHBOARD</h1>
  <span class="fps" id="fps">-- fps</span>
</header>
<div class="layout">
  <div class="card" style="grid-column: 1 / -1;">
    <h2>CAMERA VIEW</h2>
    <div style="max-width:640px;margin:0 auto;">
      <img class="feed" src="/debug_feed">
    </div>
  </div>
  <div class="card">
    <h2>STATUS</h2>
    <div class="status-grid">
      <div class="stat">
        <div class="label">ACTION</div>
        <div class="value" id="action">--</div>
      </div>
      <div class="stat">
        <div class="label">ERROR (px)</div>
        <div class="value" id="error">--</div>
      </div>
      <div class="stat">
        <div class="label">LINE CX</div>
        <div class="value" id="cx">--</div>
      </div>
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

    <!-- 경로 선택 -->
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

    <!-- 주행 버튼 -->
    <div class="btn-row">
      <button class="btn-start" onclick="startDrive()">▶ 주행 시작</button>
      <button class="btn-stop"  onclick="stopDrive()">■ 정지</button>
    </div>

    <!-- 슬라이더 -->
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
        <label>ROI_TOP <span id="v-roi">0.6</span></label>
        <input type="range" min="30" max="90" value="60"
               oninput="updateVal('v-roi',(this.value/100).toFixed(2));sendConfig('roi_top',this.value/100)">
      </div>
      <div class="slider-item">
        <label>KP <span id="v-kp">0.20</span></label>
        <input type="range" min="1" max="80" value="20"
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
    </div>
  </div>
</div>
<script>
function updateVal(id, val) { document.getElementById(id).textContent = val; }
function startDrive() { fetch('/start', {method:'POST'}); }
function stopDrive()  { fetch('/stop',  {method:'POST'}); }

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
    document.getElementById('cx').textContent     = d.cx !== null ? d.cx : '--';
    document.getElementById('junc').textContent   = d.junction_count || 0;
    document.getElementById('route-state').textContent = '루트 ' + (d.route||'--');

    const err = d.error || 0;
    const errEl = document.getElementById('error');
    errEl.textContent = (err > 0 ? '+' : '') + err;
    errEl.className = 'value' + (Math.abs(err) > 60 ? ' danger' : Math.abs(err) > 20 ? ' warn' : '');

    const pct = Math.min(Math.abs(err) / 160 * 50, 50);
    const bar = document.getElementById('error-bar');
    if (err > 0) {
      bar.style.left = '50%'; bar.style.width = pct+'%';
      bar.style.background = '#f59e0b';
    } else {
      bar.style.left = (50-pct)+'%'; bar.style.width = pct+'%';
      bar.style.background = '#4ade80';
    }
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