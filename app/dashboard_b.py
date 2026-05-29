# app/dashboard_b.py
# 경로학습 전용 경량 대시보드 (A/B/C 경로 분리 저장/재생)
#
# 실행: python app/dashboard_b.py
# 접속: http://192.168.0.50:5001

import sys, os, time, threading, json
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

app = Flask(__name__)

WAYPOINT_DIR = '/home/pi/insite'

# ══════════════════════════════════════════════════════
# 센서 초기화
# ══════════════════════════════════════════════════════

# ── RGB 카메라 ────────────────────────────────────────
try:
    from sensors.camera import Camera
    cam = Camera(width=640, height=480)
    CAM_AVAILABLE = True
    print("[OK]   RGB 카메라")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] RGB 카메라: {e}")

# ── 모터 ──────────────────────────────────────────────
try:
    from sensors.motor import MotorController
    motor = MotorController()
    MOTOR_AVAILABLE = True
    print("[OK]   모터")
except Exception as e:
    motor = None
    MOTOR_AVAILABLE = False
    print(f"[SKIP] 모터: {e}")

# ── IMU ───────────────────────────────────────────────
try:
    from sensors.mpu6050 import MPU6050
    from core.heading_tracker import HeadingTracker
    imu = MPU6050()
    heading = HeadingTracker(imu)
    heading.start()
    IMU_AVAILABLE = True
    print("[OK]   MPU-6050")
except Exception as e:
    imu = None
    heading = None
    IMU_AVAILABLE = False
    print(f"[SKIP] MPU-6050: {e}")

# ── 초음파 ────────────────────────────────────────────
try:
    import warnings, signal

    def _timeout_handler(signum, frame):
        raise TimeoutError("초음파 초기화 타임아웃")

    signal.signal(signal.SIGALRM, _timeout_handler)
    signal.alarm(3)

    from gpiozero import DistanceSensor
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)

    signal.alarm(0)
    ULTRA_AVAILABLE = True
    print("[OK]   초음파")
except Exception as e:
    ultra = None
    ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파: {e}")

# ── 경로학습 모듈 ─────────────────────────────────────
from core.waypoint_recorder import WaypointRecorder
from core.waypoint_runner   import WaypointRunner

recorder = WaypointRecorder()

print()
print("┌──────────────────────────┐")
print("│  경로학습 대시보드 준비   │")
print("├──────────────────────────┤")
print(f"│ RGB 카메라 : {'✓ OK  ' if CAM_AVAILABLE   else '✗ SKIP'} │")
print(f"│ 모터       : {'✓ OK  ' if MOTOR_AVAILABLE else '✗ SKIP'} │")
print(f"│ IMU        : {'✓ OK  ' if IMU_AVAILABLE   else '✗ SKIP'} │")
print(f"│ 초음파     : {'✓ OK  ' if ULTRA_AVAILABLE else '✗ SKIP'} │")
print("└──────────────────────────┘")
print()

# ══════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════
lock       = threading.Lock()
latest_rgb = None
is_running = False

# ══════════════════════════════════════════════════════
# 백그라운드 스레드
# ══════════════════════════════════════════════════════

def rgb_loop():
    global latest_rgb
    if not CAM_AVAILABLE or cam is None:
        return
    while True:
        try:
            frame = cam.capture()
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_rgb = jpeg.tobytes()
        except Exception as e:
            print(f"[RGB] {e}")
            time.sleep(1)

threading.Thread(target=rgb_loop, daemon=True, name="rgb").start()

# ══════════════════════════════════════════════════════
# HTML 템플릿
# ══════════════════════════════════════════════════════

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>경로학습 대시보드</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #0d1117; color: #e6edf3;
         font-family: 'Courier New', monospace;
         display: flex; flex-direction: column; align-items: center;
         padding: 20px; gap: 16px; }

  header { width: 100%; max-width: 960px;
           display: flex; align-items: center; justify-content: space-between;
           border-bottom: 1px solid #30363d; padding-bottom: 12px; }
  header h1 { font-size: 1.1rem; color: #58a6ff; letter-spacing: 2px; }

  .main { display: flex; gap: 16px; width: 100%; max-width: 960px; }

  .cam-box { flex: 1; background: #161b22;
             border: 1px solid #30363d; border-radius: 8px; overflow: hidden; }
  .cam-box img { width: 100%; display: block; }
  .cam-label { padding: 8px 12px; font-size: 0.75rem;
               color: #8b949e; border-top: 1px solid #21262d; }

  .panel { display: flex; flex-direction: column; gap: 12px; width: 280px; }

  .card { background: #161b22; border: 1px solid #30363d;
          border-radius: 8px; padding: 14px; }
  .card h2 { font-size: 0.7rem; color: #8b949e;
             letter-spacing: 2px; margin-bottom: 10px; }

  .status-row { display: flex; justify-content: space-between;
                align-items: center; margin-bottom: 6px; font-size: 0.85rem; }
  .badge { padding: 2px 8px; border-radius: 4px; font-size: 0.7rem; font-weight: bold; }
  .badge.ok   { background: #1a4a1a; color: #3fb950; }
  .badge.skip { background: #3a1a1a; color: #f85149; }

  /* 경로 선택 버튼 */
  .route-row { display: flex; gap: 6px; margin-bottom: 10px; }
  .btn-route { flex: 1; background: #21262d; border: 2px solid #30363d;
               border-radius: 6px; color: #8b949e;
               font-family: 'Courier New'; font-size: 1.1rem;
               font-weight: bold; cursor: pointer; padding: 10px;
               transition: all 0.15s; }
  .btn-route.selected  { border-color: #58a6ff; color: #58a6ff; background: #1a2a3a; }
  .btn-route.has-data  { border-color: #3fb950; color: #3fb950; background: #1a4a1a; }
  .btn-route.has-data.selected { border-color: #58a6ff; color: #58a6ff; background: #1a2a3a; }

  #rec-status { text-align: center; padding: 10px; border-radius: 6px;
                font-size: 0.85rem; background: #21262d; color: #8b949e;
                margin-bottom: 8px; transition: all 0.3s; }
  #rec-status.recording { background: #3a1a1a; color: #f85149; animation: blink 1s infinite; }
  #rec-status.done      { background: #1a4a1a; color: #3fb950; }
  #rec-status.running   { background: #1a3a4a; color: #58a6ff; animation: blink 0.5s infinite; }
  #rec-status.countdown { background: #2a2a1a; color: #e3b341; animation: blink 0.5s infinite; }
  @keyframes blink { 50% { opacity: 0.5; } }

  #wp-count { text-align: center; font-size: 2.2rem; font-weight: bold;
              color: #58a6ff; padding: 6px 0; }
  .wp-label { text-align: center; font-size: 0.7rem;
              color: #8b949e; margin-bottom: 10px; }

  .btn-group { display: flex; flex-direction: column; gap: 6px; }
  .btn { padding: 10px; border: none; border-radius: 6px;
         font-family: 'Courier New'; font-size: 0.82rem;
         cursor: pointer; transition: all 0.15s;
         font-weight: bold; letter-spacing: 1px; }
  .btn:disabled { opacity: 0.3; cursor: not-allowed; }
  .btn-rec   { background: #f85149; color: #fff; }
  .btn-rec:hover:not(:disabled)   { background: #da3633; }
  .btn-save  { background: #30363d; color: #e6edf3; }
  .btn-save:hover:not(:disabled)  { background: #484f58; }
  .btn-run   { background: #238636; color: #fff; }
  .btn-run:hover:not(:disabled)   { background: #2ea043; }
  .btn-abort { background: #9e6a03; color: #fff; }
  .btn-abort:hover:not(:disabled) { background: #bb8009; }

  /* 저장된 파일 목록 */
  .file-list { font-size: 0.75rem; margin-bottom: 8px; }
  .file-item { display: flex; justify-content: space-between;
               padding: 4px 0; border-bottom: 1px solid #21262d; }
  .file-item:last-child { border-bottom: none; }
  .file-item .fname  { color: #e6edf3; }
  .file-item .fcount { color: #3fb950; }

  /* 조종 패드 */
  .dpad { display: grid; grid-template-columns: repeat(3, 54px);
          grid-template-rows: repeat(3, 54px);
          gap: 5px; justify-content: center; margin-top: 4px; }
  .dpad button { background: #21262d; border: 1px solid #30363d;
                 border-radius: 8px; color: #58a6ff;
                 font-size: 1.1rem; cursor: pointer; transition: all 0.1s; }
  .dpad button:hover, .dpad button.active { background: #58a6ff; color: #0d1117; }
  .dpad .empty { visibility: hidden; }

  .speed-row { display: flex; align-items: center; gap: 8px;
               justify-content: center; margin-top: 6px; }
  .speed-row label { font-size: 0.75rem; color: #8b949e; }
  input[type=range] { width: 110px; accent-color: #58a6ff; }
  #speed-val { color: #58a6ff; font-size: 0.85rem; font-weight: bold; min-width: 36px; }
</style>
</head>
<body>

<header>
  <h1>[ AWR WAYPOINT TRAINER ]</h1>
  <span id="ts" style="color:#8b949e;font-size:0.75rem;"></span>
</header>

<div class="main">
  <!-- 카메라 -->
  <div class="cam-box">
    <img src="/wp/rgb_feed" alt="RGB">
    <div class="cam-label">RGB CAMERA — 640×480</div>
  </div>

  <!-- 오른쪽 패널 -->
  <div class="panel">

    <!-- 센서 상태 -->
    <div class="card">
      <h2>SENSOR STATUS</h2>
      <div class="status-row">
        <span>RGB CAM</span>
        <span class="badge {{ 'ok' if cam_ok else 'skip' }}">{{ 'OK' if cam_ok else 'SKIP' }}</span>
      </div>
      <div class="status-row">
        <span>MOTOR</span>
        <span class="badge {{ 'ok' if motor_ok else 'skip' }}">{{ 'OK' if motor_ok else 'SKIP' }}</span>
      </div>
      <div class="status-row">
        <span>IMU</span>
        <span class="badge {{ 'ok' if imu_ok else 'skip' }}">{{ 'OK' if imu_ok else 'SKIP' }}</span>
      </div>
      <div class="status-row">
        <span>ULTRASONIC</span>
        <span class="badge {{ 'ok' if ultra_ok else 'skip' }}">{{ 'OK' if ultra_ok else 'SKIP' }}</span>
      </div>
    </div>

    <!-- 경로 기록 카드 -->
    <div class="card">
      <h2>WAYPOINT RECORDER</h2>
      <div class="route-row">
        <button class="btn-route" id="route-A" onclick="selectRoute('A')">A</button>
        <button class="btn-route" id="route-B" onclick="selectRoute('B')">B</button>
        <button class="btn-route" id="route-C" onclick="selectRoute('C')">C</button>
      </div>
      <div id="selected-route" style="text-align:center;font-size:0.75rem;
           color:#8b949e;margin-bottom:8px;">경로를 선택하세요</div>

      <div id="rec-status">대기 중</div>
      <div id="wp-count">—</div>
      <div class="wp-label">WAYPOINTS RECORDED</div>

      <div class="btn-group">
        <button class="btn btn-rec"  id="btn-rec"  onclick="startRec()" disabled>⏺ 기록 시작</button>
        <button class="btn btn-save" id="btn-srec" onclick="stopRec()"  disabled>⏹ 기록 저장</button>
      </div>
    </div>

    <!-- 경로 재생 카드 -->
    <div class="card">
      <h2>WAYPOINT RUNNER</h2>
      <div class="route-row">
        <button class="btn-route" id="play-A" onclick="selectPlay('A')">A</button>
        <button class="btn-route" id="play-B" onclick="selectPlay('B')">B</button>
        <button class="btn-route" id="play-C" onclick="selectPlay('C')">C</button>
      </div>
      <div id="selected-play" style="text-align:center;font-size:0.75rem;
           color:#8b949e;margin-bottom:8px;">재생할 경로를 선택하세요</div>

      <div class="file-list" id="file-list">
        <div style="color:#8b949e;">확인 중...</div>
      </div>

      <div class="btn-group">
        <button class="btn btn-run"  id="btn-run"  onclick="runWp()"   disabled>▶ 경로 재생</button>
        <button class="btn btn-abort"id="btn-abort"onclick="abortWp()" disabled>✕ 재생 중단</button>
      </div>
    </div>

    <!-- 조종 패드 -->
    <div class="card">
      <h2>MANUAL CONTROL</h2>
      <div class="dpad">
        <div class="empty"></div>
        <button id="btn-forward"  onclick="sendCmd('forward')">▲</button>
        <div class="empty"></div>
        <button id="btn-left"     onclick="sendCmd('left')">◀</button>
        <button id="btn-stop"     onclick="sendCmd('stop')">■</button>
        <button id="btn-right"    onclick="sendCmd('right')">▶</button>
        <div class="empty"></div>
        <button id="btn-backward" onclick="sendCmd('backward')">▼</button>
        <div class="empty"></div>
      </div>
      <div class="speed-row">
        <label>SPD</label>
        <input type="range" id="speed" min="20" max="100" value="50"
               oninput="document.getElementById('speed-val').textContent=this.value+'%'">
        <span id="speed-val">50%</span>
      </div>
    </div>

  </div>
</div>

<script>
let wpCount      = 0;
let selectedRec  = null;
let selectedPlay = null;

// ── 기록용 경로 선택 ───────────────────────────────────
function selectRoute(r) {
  selectedRec = r;
  ['A','B','C'].forEach(x =>
    document.getElementById('route-'+x).classList.remove('selected'));
  document.getElementById('route-'+r).classList.add('selected');
  document.getElementById('selected-route').textContent = '기록 경로: ' + r;
  document.getElementById('selected-route').style.color = '#58a6ff';
  document.getElementById('btn-rec').disabled = false;
  document.getElementById('wp-count').textContent = '0';
}

// ── 재생용 경로 선택 ───────────────────────────────────
function selectPlay(r) {
  selectedPlay = r;
  ['A','B','C'].forEach(x =>
    document.getElementById('play-'+x).classList.remove('selected'));
  document.getElementById('play-'+r).classList.add('selected');
  document.getElementById('selected-play').textContent = '재생 경로: ' + r;
  document.getElementById('selected-play').style.color = '#58a6ff';
  document.getElementById('btn-run').disabled = false;
}

// ── 기록 시작 (3초 카운트다운) ─────────────────────────
function startRec() {
  if (!selectedRec) return;

  document.getElementById('btn-rec').disabled  = true;
  document.getElementById('btn-srec').disabled = true;

  let count = 3;
  document.getElementById('rec-status').className   = 'countdown';
  document.getElementById('rec-status').textContent = count + '초 후 시작...';
  document.getElementById('wp-count').textContent   = count;

  const timer = setInterval(() => {
    count--;
    if (count > 0) {
      document.getElementById('rec-status').textContent = count + '초 후 시작...';
      document.getElementById('wp-count').textContent   = count;
    } else {
      clearInterval(timer);
      fetch('/wp/record/start', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({route: selectedRec})
      }).then(r => r.json()).then(d => {
        wpCount = 0;
        document.getElementById('wp-count').textContent   = '0';
        document.getElementById('rec-status').className   = 'recording';
        document.getElementById('rec-status').textContent = '● 경로 ' + selectedRec + ' 기록 중...';
        document.getElementById('btn-srec').disabled      = false;
      });
    }
  }, 1000);
}

// ── 기록 저장 ──────────────────────────────────────────
function stopRec() {
  fetch('/wp/record/stop', {method: 'POST'}).then(r => r.json()).then(d => {
    document.getElementById('rec-status').className   = 'done';
    document.getElementById('rec-status').textContent
      = '✓ 경로 ' + d.route + ' 저장 완료 (' + d.count + '개)';
    document.getElementById('btn-rec').disabled  = false;
    document.getElementById('btn-srec').disabled = true;
    refreshFileList();
    document.getElementById('route-' + d.route).classList.add('has-data');
    document.getElementById('play-'  + d.route).classList.add('has-data');
  });
}

// ── 경로 재생 ──────────────────────────────────────────
function runWp() {
  if (!selectedPlay) return;
  fetch('/wp/run', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({route: selectedPlay})
  }).then(r => r.json()).then(d => {
    if (d.status === 'error') { alert(d.msg); return; }
    document.getElementById('rec-status').className   = 'running';
    document.getElementById('rec-status').textContent = '▶ 경로 ' + selectedPlay + ' 재생 중...';
    document.getElementById('btn-run').disabled   = true;
    document.getElementById('btn-abort').disabled = false;
    document.getElementById('btn-rec').disabled   = true;
  });
}

// ── 재생 중단 ──────────────────────────────────────────
function abortWp() {
  fetch('/wp/abort', {method: 'POST'}).then(r => r.json()).then(() => {
    document.getElementById('rec-status').className   = '';
    document.getElementById('rec-status').textContent = '대기 중';
    document.getElementById('btn-run').disabled   = false;
    document.getElementById('btn-abort').disabled = true;
    document.getElementById('btn-rec').disabled   = false;
  });
}

// ── 저장된 파일 목록 갱신 ──────────────────────────────
function refreshFileList() {
  fetch('/wp/files').then(r => r.json()).then(d => {
    const el = document.getElementById('file-list');
    if (d.files.length === 0) {
      el.innerHTML = '<div style="color:#8b949e;font-size:0.75rem;padding:4px 0;">저장된 경로 없음</div>';
      return;
    }
    el.innerHTML = d.files.map(f =>
      '<div class="file-item">' +
        '<span class="fname">경로 ' + f.route + '</span>' +
        '<span class="fcount">' + f.count + '개 waypoint</span>' +
      '</div>'
    ).join('');
    d.files.forEach(f => {
      document.getElementById('route-' + f.route).classList.add('has-data');
      document.getElementById('play-'  + f.route).classList.add('has-data');
    });
  });
}

// ── 모터 조종 ──────────────────────────────────────────
function sendCmd(cmd) {
  const speed = document.getElementById('speed').value;
  fetch('/wp/control', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({cmd, speed})
  }).then(r => r.json()).then(d => {
    if (d.wp_count !== undefined) {
      wpCount = d.wp_count;
      document.getElementById('wp-count').textContent = wpCount;
    }
  });
  document.querySelectorAll('.dpad button').forEach(b => b.classList.remove('active'));
  const btn = document.getElementById('btn-' + cmd);
  if (btn) btn.classList.add('active');
}

// ── 키보드 ─────────────────────────────────────────────
const keyMap = {
  'ArrowUp':'forward',   'w':'forward',
  'ArrowDown':'backward','s':'backward',
  'ArrowLeft':'left',    'a':'left',
  'ArrowRight':'right',  'd':'right',
  ' ':'stop'
};
const pressed = {};
window.addEventListener('keydown', e => { if (keyMap[e.key]) e.preventDefault(); });
document.addEventListener('keydown', e => {
  if (!keyMap[e.key] || pressed[e.key]) return;
  pressed[e.key] = true;
  sendCmd(keyMap[e.key]);
});
document.addEventListener('keyup', e => {
  if (!keyMap[e.key]) return;
  pressed[e.key] = false;
  if (keyMap[e.key] !== 'stop') sendCmd('stop');
});

// ── 재생 완료 감지 ─────────────────────────────────────
setInterval(() => {
  fetch('/wp/status').then(r => r.json()).then(d => {
    if (!d.is_running && document.getElementById('btn-abort').disabled === false) {
      document.getElementById('rec-status').className   = 'done';
      document.getElementById('rec-status').textContent = '✓ 재생 완료';
      document.getElementById('btn-run').disabled   = false;
      document.getElementById('btn-abort').disabled = true;
      document.getElementById('btn-rec').disabled   = false;
    }
  });
}, 500);

// ── 시계 ───────────────────────────────────────────────
setInterval(() => {
  document.getElementById('ts').textContent = new Date().toLocaleTimeString();
}, 1000);

// ── 초기 파일 목록 로드 ────────────────────────────────
refreshFileList();
</script>
</body>
</html>'''

# ══════════════════════════════════════════════════════
# Flask 라우트
# ══════════════════════════════════════════════════════

def gen_stream(get_frame_fn):
    while True:
        with lock:
            frame = get_frame_fn()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)


@app.route('/')
def index():
    return render_template_string(HTML,
        cam_ok   = CAM_AVAILABLE,
        motor_ok = MOTOR_AVAILABLE,
        imu_ok   = IMU_AVAILABLE,
        ultra_ok = ULTRA_AVAILABLE,
    )

@app.route('/wp/rgb_feed')
def rgb_feed():
    return Response(gen_stream(lambda: latest_rgb),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/wp/status')
def wp_status():
    return jsonify({'is_running': is_running})

@app.route('/wp/files')
def wp_files():
    """저장된 waypoint 파일 목록 반환"""
    result = []
    for route in ['A', 'B', 'C']:
        path = os.path.join(WAYPOINT_DIR, f'waypoints_{route}.json')
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
            result.append({'route': route, 'count': len(data), 'path': path})
    return jsonify({'files': result})

@app.route('/wp/control', methods=['POST'])
def control():
    if not MOTOR_AVAILABLE or motor is None:
        return jsonify({'status': 'error', 'msg': 'motor not available'})

    data  = request.get_json(force=True)
    cmd   = data.get('cmd', 'stop')
    speed = int(data.get('speed', 50))

    try:
        if cmd == 'forward':
            motor.forward(speed)
        elif cmd == 'backward':
            motor.backward(speed)
        elif cmd == 'left':
            motor.set_motor(1,  1, speed)
            motor.set_motor(2,  1, speed)
            motor.set_motor(3, -1, speed)
            motor.set_motor(4, -1, speed)
        elif cmd == 'right':
            motor.set_motor(1, -1, speed)
            motor.set_motor(2, -1, speed)
            motor.set_motor(3,  1, speed)
            motor.set_motor(4,  1, speed)
        elif cmd == 'stop':
            motor.stop()
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)})

    if recorder.recording:
        recorder.record(cmd, speed, heading.get() if heading else 0.0)

    return jsonify({
        'status':   'ok',
        'cmd':      cmd,
        'wp_count': len(recorder.waypoints),
    })

@app.route('/wp/record/start', methods=['POST'])
def record_start():
    data  = request.get_json(force=True)
    route = data.get('route', 'A').upper()
    recorder.start()
    recorder.current_route = route
    print(f"[Recorder] 경로 {route} 기록 시작")
    return jsonify({'status': 'ok', 'route': route})

@app.route('/wp/record/stop', methods=['POST'])
def record_stop():
    route = getattr(recorder, 'current_route', 'A')
    path  = os.path.join(WAYPOINT_DIR, f'waypoints_{route}.json')
    recorder.stop(path)
    return jsonify({'status': 'ok', 'route': route,
                    'count': len(recorder.waypoints), 'path': path})

@app.route('/wp/run', methods=['POST'])
def run_waypoints():
    global is_running
    if is_running:
        return jsonify({'status': 'error', 'msg': '이미 재생 중'})

    data  = request.get_json(force=True)
    route = data.get('route', 'A').upper()
    path  = os.path.join(WAYPOINT_DIR, f'waypoints_{route}.json')

    if not os.path.exists(path):
        return jsonify({'status': 'error',
                        'msg': f'경로 {route} 파일 없음 — 먼저 기록하세요'})

    def _run():
        global is_running
        is_running = True
        print(f"[Runner] 경로 {route} 재생 시작")
        try:
            runner = WaypointRunner(motor, ultra, heading)
            runner.run(path)
        finally:
            is_running = False
            print(f"[Runner] 경로 {route} 재생 완료")

    threading.Thread(target=_run, daemon=True).start()
    return jsonify({'status': 'ok', 'route': route})

@app.route('/wp/abort', methods=['POST'])
def abort_waypoints():
    global is_running
    is_running = False
    if MOTOR_AVAILABLE and motor:
        motor.stop()
    return jsonify({'status': 'ok'})


if __name__ == '__main__':
    print("경로학습 대시보드 시작: http://192.168.0.50:5001")
    app.run(host='0.0.0.0', port=5001, threaded=True)