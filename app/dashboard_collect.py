# python app/dashboard_collect.py
# 장애물 데이터 수집 전용 대시보드
#
# 실행: python app/dashboard_collect.py
# 접속: http://192.168.0.50:5002
#
# 저장 위치:
#   이미지 → /home/pi/insite/data/images/obstacle_NNN.jpg
#   데이터 → /home/pi/insite/data/obstacle_data.json

import sys, os, time, threading, json, base64
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

app = Flask(__name__)

DATA_DIR   = '/home/pi/insite/data'
IMAGE_DIR  = os.path.join(DATA_DIR, 'images')
DATA_FILE  = os.path.join(DATA_DIR, 'obstacle_data.json')

os.makedirs(IMAGE_DIR, exist_ok=True)
if not os.path.exists(DATA_FILE):
    with open(DATA_FILE, 'w') as f:
        json.dump([], f)

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
    imu = MPU6050()

    # gyro_offset.txt 로드
    OFFSET_FILE = '/home/pi/insite/gyro_offset.txt'
    if os.path.exists(OFFSET_FILE):
        with open(OFFSET_FILE) as f:
            vals = [float(v) for v in f.read().strip().split(',')]
        imu.gyro_offset = {'x': vals[0], 'y': vals[1], 'z': vals[2]}
        print(f"  gyro offset 로드: {imu.gyro_offset}")
    else:
        print("  gyro_offset.txt 없음 — offset 0으로 사용")

    IMU_AVAILABLE = True
    print("[OK]   MPU-6050")
except Exception as e:
    imu = None
    IMU_AVAILABLE = False
    print(f"[SKIP] MPU-6050: {e}")

# ── HX711 로드셀 ──────────────────────────────────────
try:
    from sensors.hx711 import HX711
    hx711 = HX711()
    time.sleep(1)
    hx711.REF_UNIT_A = -262.5
    hx711.tare(samples=20)
    HX711_AVAILABLE = True
    print("[OK]   HX711 로드셀")
except Exception as e:
    hx711 = None
    HX711_AVAILABLE = False
    print(f"[SKIP] HX711: {e}")

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

print()
print("┌──────────────────────────┐")
print("│  데이터 수집 대시보드     │")
print("├──────────────────────────┤")
print(f"│ RGB 카메라 : {'✓ OK  ' if CAM_AVAILABLE   else '✗ SKIP'} │")
print(f"│ 모터       : {'✓ OK  ' if MOTOR_AVAILABLE else '✗ SKIP'} │")
print(f"│ IMU        : {'✓ OK  ' if IMU_AVAILABLE   else '✗ SKIP'} │")
print(f"│ 로드셀     : {'✓ OK  ' if HX711_AVAILABLE else '✗ SKIP'} │")
print(f"│ 초음파     : {'✓ OK  ' if ULTRA_AVAILABLE else '✗ SKIP'} │")
print("└──────────────────────────┘")
print(f"데이터 저장 위치: {DATA_FILE}")
print()

# ══════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════
lock         = threading.Lock()
latest_rgb   = None
latest_frame = None

# 통과 시도 중 IMU 기록용
imu_recording   = False
imu_log         = []   # {'ax','ay','az','gx','gy','gz'} 리스트

# ══════════════════════════════════════════════════════
# 백그라운드 스레드
# ══════════════════════════════════════════════════════

def rgb_loop():
    global latest_rgb, latest_frame
    if not CAM_AVAILABLE or cam is None:
        return
    while True:
        try:
            frame = cam.capture()
            with lock:
                latest_frame = frame.copy()
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with lock:
                latest_rgb = jpeg.tobytes()
        except Exception as e:
            print(f"[RGB] {e}")
            time.sleep(1)

def imu_loop():
    """통과 시도 중 100Hz로 IMU 기록"""
    global imu_log, imu_recording
    if not IMU_AVAILABLE or imu is None:
        return
    while True:
        if imu_recording:
            try:
                accel = imu.get_accel()
                gyro  = imu.get_gyro()
                imu_log.append({
                    'ax': accel['x'], 'ay': accel['y'], 'az': accel['z'],
                    'gx': gyro['x'],  'gy': gyro['y'],  'gz': gyro['z'],
                })
            except Exception as e:
                print(f"[IMU] {e}")
        time.sleep(0.01)  # 100Hz

threading.Thread(target=rgb_loop, daemon=True, name="rgb").start()
threading.Thread(target=imu_loop, daemon=True, name="imu").start()

# ══════════════════════════════════════════════════════
# 유틸
# ══════════════════════════════════════════════════════

def get_next_id():
    with open(DATA_FILE) as f:
        data = json.load(f)
    return len(data) + 1

def load_data():
    with open(DATA_FILE) as f:
        return json.load(f)

def save_entry(entry):
    data = load_data()
    data.append(entry)
    with open(DATA_FILE, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def read_sensors():
    weight   = 0.0
    distance = -1.0
    accel    = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    gyro     = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    if HX711_AVAILABLE and hx711:
        try:
            weight = round(hx711.get_grams(), 1)
        except:
            pass
    if ULTRA_AVAILABLE and ultra:
        try:
            d = ultra.distance
            distance = round(d * 100, 1) if d else -1
        except:
            pass
    if IMU_AVAILABLE and imu:
        try:
            accel = imu.get_accel()
            gyro  = imu.get_gyro()
        except:
            pass

    return weight, distance, accel, gyro

def summarize_imu(log):
    """통과 시도 중 IMU 로그 → 최대/평균값 요약"""
    if not log:
        return {}

    ax = [d['ax'] for d in log]
    ay = [d['ay'] for d in log]
    az = [d['az'] for d in log]
    gx = [d['gx'] for d in log]
    gy = [d['gy'] for d in log]
    gz = [d['gz'] for d in log]

    return {
        # 가속도 최대 변화량 (충격 감지)
        'accel_x_max':  round(max(ax, key=abs), 4),
        'accel_y_max':  round(max(ay, key=abs), 4),
        'accel_z_max':  round(max(az, key=abs), 4),
        'accel_x_mean': round(sum(ax) / len(ax), 4),
        'accel_y_mean': round(sum(ay) / len(ay), 4),
        'accel_z_mean': round(sum(az) / len(az), 4),
        # 자이로 최대 (기울기 변화율)
        'gyro_x_max':   round(max(gx, key=abs), 4),
        'gyro_y_max':   round(max(gy, key=abs), 4),
        'gyro_z_max':   round(max(gz, key=abs), 4),
        'samples':      len(log),
    }

# ══════════════════════════════════════════════════════
# HTML
# ══════════════════════════════════════════════════════

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>장애물 데이터 수집</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #0d1117; color: #e6edf3;
         font-family: 'Courier New', monospace;
         padding: 20px; }

  header { display: flex; align-items: center; justify-content: space-between;
           border-bottom: 1px solid #30363d; padding-bottom: 12px; margin-bottom: 16px; }
  header h1 { font-size: 1.1rem; color: #f0883e; letter-spacing: 2px; }
  #total-count { background: #2a1f0e; border: 1px solid #f0883e;
                 border-radius: 6px; padding: 4px 12px;
                 color: #f0883e; font-size: 0.85rem; }

  .layout { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }

  .left { display: flex; flex-direction: column; gap: 12px; }

  .cam-box { background: #161b22; border: 1px solid #30363d;
             border-radius: 8px; overflow: hidden; }
  .cam-box img { width: 100%; display: block; }
  #snap-preview { width: 100%; display: none; }
  .cam-label { padding: 6px 12px; font-size: 0.72rem; color: #8b949e;
               border-top: 1px solid #21262d;
               display: flex; justify-content: space-between; }

  /* 센서 수치 */
  .sensor-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; }
  .sensor-item { background: #161b22; border: 1px solid #30363d;
                 border-radius: 8px; padding: 8px; text-align: center; }
  .sensor-item .s-label { font-size: 0.65rem; color: #8b949e; margin-bottom: 3px; }
  .sensor-item .s-value { font-size: 1rem; font-weight: bold; color: #f0883e; }

  /* IMU 상세 */
  .imu-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
  .imu-item { background: #0d1117; border: 1px solid #21262d;
              border-radius: 6px; padding: 6px; text-align: center; }
  .imu-item .i-label { font-size: 0.62rem; color: #8b949e; }
  .imu-item .i-value { font-size: 0.85rem; color: #58a6ff; font-weight: bold; }

  .right { display: flex; flex-direction: column; gap: 12px; }

  .card { background: #161b22; border: 1px solid #30363d;
          border-radius: 8px; padding: 14px; }
  .card h2 { font-size: 0.7rem; color: #8b949e;
             letter-spacing: 2px; margin-bottom: 10px; }

  .form-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 8px; }
  .form-item { display: flex; flex-direction: column; gap: 4px; }
  .form-item label { font-size: 0.72rem; color: #8b949e; }
  .form-item select, .form-item input {
    background: #0d1117; border: 1px solid #30363d; border-radius: 6px;
    color: #e6edf3; padding: 8px; font-family: 'Courier New';
    font-size: 0.85rem; outline: none; }
  .form-item select:focus, .form-item input:focus { border-color: #f0883e; }
  .form-item.full { grid-column: 1 / -1; }

  .btn { width: 100%; padding: 12px; border: none; border-radius: 6px;
         font-family: 'Courier New'; font-size: 0.88rem;
         cursor: pointer; font-weight: bold; letter-spacing: 1px;
         transition: all 0.15s; margin-bottom: 6px; }
  .btn:disabled { opacity: 0.3; cursor: not-allowed; }
  .btn-snap    { background: #1f6feb; color: #fff; }
  .btn-snap:hover:not(:disabled)    { background: #388bfd; }
  .btn-try     { background: #f0883e; color: #0d1117; }
  .btn-try:hover:not(:disabled)     { background: #ffa657; }
  .btn-success { background: #238636; color: #fff; }
  .btn-success:hover:not(:disabled) { background: #2ea043; }
  .btn-fail    { background: #da3633; color: #fff; }
  .btn-fail:hover:not(:disabled)    { background: #f85149; }
  .btn-reset   { background: #21262d; color: #8b949e; font-size:0.78rem; }
  .btn-reset:hover { background: #30363d; color: #e6edf3; }
  .btn-group-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .btn-group-2 .btn { margin-bottom: 0; }

  #state-box { text-align: center; padding: 12px; border-radius: 6px;
               font-size: 0.88rem; background: #21262d; color: #8b949e;
               margin-bottom: 8px; transition: all 0.3s; }
  #state-box.snapped  { background: #1a2a4a; color: #58a6ff; }
  #state-box.trying   { background: #2a1f0e; color: #f0883e; animation: blink 0.8s infinite; }
  #state-box.success  { background: #1a4a1a; color: #3fb950; }
  #state-box.fail     { background: #3a1a1a; color: #f85149; }
  @keyframes blink { 50% { opacity: 0.5; } }

  /* IMU 결과 요약 */
  #imu-result { background: #0d1117; border: 1px solid #21262d;
                border-radius: 6px; padding: 10px; margin-bottom: 8px;
                font-size: 0.75rem; color: #8b949e; display: none; }
  #imu-result.visible { display: block; }
  #imu-result .imu-title { color: #58a6ff; margin-bottom: 6px; font-size: 0.72rem; }

  /* 로그 테이블 */
  .log-wrap { overflow-x: auto; max-height: 200px; overflow-y: auto; }
  .log-table { width: 100%; border-collapse: collapse; font-size: 0.72rem; white-space: nowrap; }
  .log-table th { color: #8b949e; border-bottom: 1px solid #30363d;
                  padding: 4px 8px; text-align: left; position: sticky; top: 0;
                  background: #161b22; }
  .log-table td { padding: 5px 8px; border-bottom: 1px solid #21262d; }
  .log-table tr:last-child td { border-bottom: none; }
  .tag-s { color: #3fb950; font-weight: bold; }
  .tag-f { color: #f85149; font-weight: bold; }
</style>
</head>
<body>

<header>
  <h1>[ OBSTACLE DATA COLLECTOR ]</h1>
  <span id="total-count">총 0개 수집</span>
</header>

<div class="layout">

  <!-- 왼쪽 -->
  <div class="left">

    <!-- 카메라 -->
    <div class="cam-box">
      <img id="live-feed" src="/dc/rgb_feed" alt="LIVE">
      <img id="snap-preview" alt="SNAPSHOT">
      <div class="cam-label">
        <span id="cam-label-text">● LIVE</span>
        <span id="ts"></span>
      </div>
    </div>

    <!-- 센서 수치 (4개) -->
    <div class="sensor-grid">
      <div class="sensor-item">
        <div class="s-label">WEIGHT (g)</div>
        <div class="s-value" id="val-weight">--</div>
      </div>
      <div class="sensor-item">
        <div class="s-label">DISTANCE (cm)</div>
        <div class="s-value" id="val-distance">--</div>
      </div>
      <div class="sensor-item">
        <div class="s-label">ACCEL Y (g)</div>
        <div class="s-value" id="val-ay">--</div>
      </div>
      <div class="sensor-item">
        <div class="s-label">GYRO X (°/s)</div>
        <div class="s-value" id="val-gx">--</div>
      </div>
    </div>

    <!-- IMU 상세 (실시간) -->
    <div class="card">
      <h2>IMU LIVE</h2>
      <div class="imu-grid">
        <div class="imu-item"><div class="i-label">Accel X</div><div class="i-value" id="imu-ax">--</div></div>
        <div class="imu-item"><div class="i-label">Accel Y</div><div class="i-value" id="imu-ay">--</div></div>
        <div class="imu-item"><div class="i-label">Accel Z</div><div class="i-value" id="imu-az">--</div></div>
        <div class="imu-item"><div class="i-label">Gyro X</div><div class="i-value" id="imu-gx">--</div></div>
        <div class="imu-item"><div class="i-label">Gyro Y</div><div class="i-value" id="imu-gy">--</div></div>
        <div class="imu-item"><div class="i-label">Gyro Z</div><div class="i-value" id="imu-gz">--</div></div>
      </div>
    </div>

    <!-- 촬영 버튼 -->
    <button class="btn btn-snap" id="btn-snap" onclick="doSnap()">
      📸 촬영 + 센서 기록
    </button>

  </div>

  <!-- 오른쪽 -->
  <div class="right">

    <!-- 상태 -->
    <div id="state-box">장애물 앞에 로봇을 세우고 촬영하세요</div>

    <!-- IMU 결과 요약 (통과 시도 후 표시) -->
    <div id="imu-result">
      <div class="imu-title">▶ 통과 시도 중 IMU 기록</div>
      <div id="imu-summary-text"></div>
    </div>

    <!-- 조건 입력 -->
    <div class="card">
      <h2>OBSTACLE CONDITIONS</h2>
      <div class="form-grid">
        <div class="form-item">
          <label>장애물 종류</label>
          <select id="obs-type">
            <option value="slope">경사로</option>
            <option value="bump">방지턱</option>
            <option value="floor">바닥 표면</option>
          </select>
        </div>
        <div class="form-item">
          <label>바닥 상태</label>
          <select id="floor-type">
            <option value="smooth">매끄러움</option>
            <option value="rough">요철</option>
          </select>
        </div>
        <div class="form-item">
          <label>경사/높이</label>
          <select id="obs-level">
            <option value="none">없음</option>
            <option value="1cm">방지턱 1cm</option>
            <option value="3cm">방지턱 3cm</option>
            <option value="5cm">방지턱 5cm</option>
          </select>
        </div>
        <div class="form-item full">
            <label>진입 속도: <span id="speed-label">50%</span></label>
            <input type="range" id="try-speed" value="50" min="20" max="100"
                    oninput="document.getElementById('speed-label').textContent=this.value+'%'"
                    style="width:100%; accent-color:#f0883e;">
        </div>
        <div class="form-item full">
          <label>메모 (선택)</label>
          <input type="text" id="memo" placeholder="특이사항 입력...">
        </div>
      </div>
    </div>

    <!-- 통과 시도 -->
    <div class="card">
      <h2>TRIAL</h2>
      <button class="btn btn-try" id="btn-try" onclick="doTry()" disabled>
        🚗 통과 시도 (IMU 기록 시작)
      </button>
      <div class="btn-group-2">
        <button class="btn btn-success" id="btn-success" onclick="doResult('success')" disabled>
          ✓ 성공
        </button>
        <button class="btn btn-fail" id="btn-fail" onclick="doResult('fail')" disabled>
          ✗ 실패
        </button>
      </div>
      <button class="btn btn-reset" onclick="doReset()">↺ 초기화 (새 데이터)</button>
    </div>

    <!-- 수집 로그 -->
    <div class="card" style="flex:1;">
      <h2>COLLECTED DATA</h2>
      <div class="log-wrap">
        <table class="log-table">
          <thead>
            <tr>
              <th>#</th><th>종류</th><th>조건</th>
              <th>무게</th><th>거리</th>
              <th>accel_y_max</th><th>gyro_x_max</th>
              <th>속도</th><th>결과</th>
            </tr>
          </thead>
          <tbody id="log-body">
            <tr><td colspan="9" style="color:#8b949e;text-align:center;">없음</td></tr>
          </tbody>
        </table>
      </div>
    </div>

  </div>
</div>

<script>
let currentSnap = null;
let currentId   = null;
let imuSummary  = null;

// ── 촬영 ───────────────────────────────────────────────
function doSnap() {
  fetch('/dc/snap', {method: 'POST'}).then(r => r.json()).then(d => {
    if (d.status !== 'ok') { alert(d.msg); return; }

    currentId   = d.id;
    currentSnap = d;

    document.getElementById('snap-preview').src = 'data:image/jpeg;base64,' + d.image_b64;
    document.getElementById('snap-preview').style.display = 'block';
    document.getElementById('live-feed').style.display    = 'none';
    document.getElementById('cam-label-text').textContent = '📸 SNAPSHOT #' + d.id;

    document.getElementById('val-weight').textContent   = d.weight_g + 'g';
    document.getElementById('val-distance').textContent = d.distance_cm + 'cm';
    document.getElementById('val-ay').textContent       = d.accel.y.toFixed(3);
    document.getElementById('val-gx').textContent       = d.gyro.x.toFixed(3);

    document.getElementById('state-box').className   = 'snapped';
    document.getElementById('state-box').textContent
      = '📸 촬영 완료 (#' + d.id + ') — 조건 확인 후 통과 시도하세요';

    document.getElementById('btn-try').disabled     = false;
    document.getElementById('btn-success').disabled = true;
    document.getElementById('btn-fail').disabled    = true;
    document.getElementById('imu-result').className = '';
  });
}

// ── 통과 시도 ──────────────────────────────────────────
function doTry() {
  const speed = parseInt(document.getElementById('try-speed').value);
  document.getElementById('state-box').className   = 'trying';
  document.getElementById('state-box').textContent = '🚗 통과 시도 중... (IMU 기록 중)';
  document.getElementById('btn-try').disabled      = true;

  fetch('/dc/try', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({speed})
  }).then(r => r.json()).then(d => {
    imuSummary = d.imu_summary;

    // IMU 결과 표시
    const s = d.imu_summary;
    if (s && s.samples > 0) {
      document.getElementById('imu-summary-text').innerHTML =
        `accel_x_max: <b>${s.accel_x_max}</b> &nbsp;
         accel_y_max: <b>${s.accel_y_max}</b> &nbsp;
         accel_z_max: <b>${s.accel_z_max}</b><br>
         gyro_x_max: <b>${s.gyro_x_max}</b> &nbsp;
         gyro_y_max: <b>${s.gyro_y_max}</b> &nbsp;
         샘플수: <b>${s.samples}</b>`;
      document.getElementById('imu-result').className = 'visible';
    }

    document.getElementById('state-box').textContent
      = '통과 시도 완료 — 성공/실패를 선택하세요';
    document.getElementById('btn-success').disabled = false;
    document.getElementById('btn-fail').disabled    = false;
  });
}

// ── 결과 저장 ──────────────────────────────────────────
function doResult(result) {
  if (!currentSnap) { alert('먼저 촬영하세요'); return; }

  const payload = {
    id:          currentId,
    image:       currentSnap.image_filename,
    obs_type:    document.getElementById('obs-type').value,
    obs_level:   document.getElementById('obs-level').value,
    floor_type:  document.getElementById('floor-type').value,
    speed:       parseInt(document.getElementById('try-speed').value),
    weight_g:    currentSnap.weight_g,
    distance_cm: currentSnap.distance_cm,
    accel_snap:  currentSnap.accel,
    gyro_snap:   currentSnap.gyro,
    imu_trial:   imuSummary,
    result:      result,
    memo:        document.getElementById('memo').value,
  };

  fetch('/dc/save', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  }).then(r => r.json()).then(d => {
    const cls   = result === 'success' ? 'success' : 'fail';
    const icon  = result === 'success' ? '✓ 성공 저장 완료' : '✗ 실패 저장 완료';
    document.getElementById('state-box').className   = cls;
    document.getElementById('state-box').textContent = icon + ' (#' + currentId + ')';
    document.getElementById('btn-success').disabled  = true;
    document.getElementById('btn-fail').disabled     = true;
    refreshLog();
    updateCount();
  });
}

// ── 초기화 ─────────────────────────────────────────────
function doReset() {
  currentSnap = null;
  currentId   = null;
  imuSummary  = null;
  document.getElementById('snap-preview').style.display = 'none';
  document.getElementById('live-feed').style.display    = 'block';
  document.getElementById('cam-label-text').textContent = '● LIVE';
  document.getElementById('state-box').className        = '';
  document.getElementById('state-box').textContent      = '장애물 앞에 로봇을 세우고 촬영하세요';
  document.getElementById('btn-snap').disabled          = false;
  document.getElementById('btn-try').disabled           = true;
  document.getElementById('btn-success').disabled       = true;
  document.getElementById('btn-fail').disabled          = true;
  document.getElementById('memo').value                 = '';
  document.getElementById('imu-result').className       = '';
}

// ── 로그 갱신 ──────────────────────────────────────────
function refreshLog() {
  fetch('/dc/data').then(r => r.json()).then(d => {
    const tbody = document.getElementById('log-body');
    if (d.length === 0) {
      tbody.innerHTML = '<tr><td colspan="9" style="color:#8b949e;text-align:center;">없음</td></tr>';
      return;
    }
    tbody.innerHTML = d.slice().reverse().map(row => {
      const cls   = row.result === 'success' ? 'tag-s' : 'tag-f';
      const label = row.result === 'success' ? '✓ 성공' : '✗ 실패';
      const t     = row.imu_trial || {};
      return '<tr>' +
        '<td>#' + row.id + '</td>' +
        '<td>' + (row.obs_type  || '-') + '</td>' +
        '<td>' + (row.obs_level || '-') + '</td>' +
        '<td>' + (row.weight_g  || 0)  + 'g</td>' +
        '<td>' + (row.distance_cm || '-') + 'cm</td>' +
        '<td>' + (t.accel_y_max !== undefined ? t.accel_y_max : '-') + '</td>' +
        '<td>' + (t.gyro_x_max  !== undefined ? t.gyro_x_max  : '-') + '</td>' +
        '<td>' + (row.speed || '-') + '%</td>' +
        '<td class="' + cls + '">' + label + '</td>' +
        '</tr>';
    }).join('');
  });
}

function updateCount() {
  fetch('/dc/data').then(r => r.json()).then(d => {
    document.getElementById('total-count').textContent = '총 ' + d.length + '개 수집';
  });
}

// ── IMU 실시간 폴링 ────────────────────────────────────
setInterval(() => {
  fetch('/dc/sensors').then(r => r.json()).then(d => {
    // 라이브 중일 때만 센서값 업데이트
    if (document.getElementById('live-feed').style.display !== 'none') {
      document.getElementById('val-weight').textContent   = d.weight_g    + 'g';
      document.getElementById('val-distance').textContent = d.distance_cm + 'cm';
      document.getElementById('val-ay').textContent       = d.accel.y.toFixed(3);
      document.getElementById('val-gx').textContent       = d.gyro.x.toFixed(3);
    }
    // IMU는 항상 업데이트
    document.getElementById('imu-ax').textContent = d.accel.x.toFixed(3);
    document.getElementById('imu-ay').textContent = d.accel.y.toFixed(3);
    document.getElementById('imu-az').textContent = d.accel.z.toFixed(3);
    document.getElementById('imu-gx').textContent = d.gyro.x.toFixed(3);
    document.getElementById('imu-gy').textContent = d.gyro.y.toFixed(3);
    document.getElementById('imu-gz').textContent = d.gyro.z.toFixed(3);
  });
  document.getElementById('ts').textContent = new Date().toLocaleTimeString();
}, 200);

// ── 초기 로드 ──────────────────────────────────────────
refreshLog();
updateCount();
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
    return render_template_string(HTML)

@app.route('/dc/rgb_feed')
def rgb_feed():
    return Response(gen_stream(lambda: latest_rgb),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/dc/sensors')
def sensors():
    w, d, accel, gyro = read_sensors()
    return jsonify({
        'weight_g':    w,
        'distance_cm': d,
        'accel':       accel,
        'gyro':        gyro,
    })

@app.route('/dc/snap', methods=['POST'])
def snap():
    with lock:
        frame = latest_frame.copy() if latest_frame is not None else None

    if frame is None:
        return jsonify({'status': 'error', 'msg': '카메라 없음'})

    nid      = get_next_id()
    filename = f'obstacle_{nid:03d}.jpg'
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, frame)

    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    b64    = base64.b64encode(buf.tobytes()).decode('utf-8')

    weight, distance, accel, gyro = read_sensors()

    return jsonify({
        'status':         'ok',
        'id':             nid,
        'image_filename': filename,
        'image_b64':      b64,
        'weight_g':       weight,
        'distance_cm':    distance,
        'accel':          accel,
        'gyro':           gyro,
    })

@app.route('/dc/try', methods=['POST'])
def try_obstacle():
    global imu_recording, imu_log

    if not MOTOR_AVAILABLE or motor is None:
        return jsonify({'status': 'error', 'msg': '모터 없음'})

    data  = request.get_json(force=True)
    speed = int(data.get('speed', 50))

    # IMU 기록 시작
    imu_log       = []
    imu_recording = True

    # 모터 전진 1초
    motor.forward(speed)
    time.sleep(1.0)
    motor.stop()

    # IMU 기록 종료
    imu_recording = False
    summary       = summarize_imu(imu_log)

    print(f"[Trial] speed={speed} IMU samples={summary.get('samples', 0)}")

    return jsonify({'status': 'ok', 'imu_summary': summary})

@app.route('/dc/save', methods=['POST'])
def save():
    entry = request.get_json(force=True)
    entry['timestamp'] = time.strftime('%Y-%m-%d %H:%M:%S')
    save_entry(entry)
    print(f"[Collect] #{entry['id']} {entry.get('obs_type')} "
          f"{entry.get('obs_level')} → {entry['result']}")
    return jsonify({'status': 'ok'})

@app.route('/dc/data')
def get_data():
    return jsonify(load_data())


if __name__ == '__main__':
    print("데이터 수집 대시보드 시작: http://192.168.0.50:5002")
    app.run(host='0.0.0.0', port=5002, threaded=True)