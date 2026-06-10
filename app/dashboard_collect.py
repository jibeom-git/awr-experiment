# tools/capture_reference_dash.py
# 레퍼런스 사진 촬영 대시보드
# 실행: python app/dashboard_collect.py
# 접속: http://192.168.0.50:5010

import sys, os, time, threading
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np
from sensors.camera import Camera


app = Flask(__name__)

os.makedirs("ai_core/references", exist_ok=True)

# ── 웹캠 초기화 ─────────────────────────────────────
try:
    cam = Camera(width=640, height=480)
    CAM_AVAILABLE = True
    print("[OK] Pi 카메라")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] Pi 카메라: {e}")

lock         = threading.Lock()
latest_frame = None
saved_status = {}

lock         = threading.Lock()
latest_frame = None
saved_status = {}  # {"flat": True, "1cm": False, "2cm": False}

LABELS = {
    "flat": "평지 (장애물 없음)",
    "1cm":  "1cm 방지턱",
    "2cm":  "2cm 방지턱",
}

# ── 캡처 루프 ────────────────────────────────────────
def capture_loop():
    global latest_frame
    while True:
        try:
            frame = cam.capture()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with lock:
                latest_frame = jpeg.tobytes()
        except Exception as e:
            print(f"[캡처] {e}")
        time.sleep(0.033)

threading.Thread(target=capture_loop, daemon=True).start()

# ── 스트림 ───────────────────────────────────────────
def gen_stream():
    while True:
        with lock:
            frame = latest_frame
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)

@app.route('/video_feed')
def video_feed():
    return Response(gen_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture', methods=['POST'])
def capture():
    data  = request.get_json(force=True)
    label = data.get('label')
    if label not in LABELS:
        return jsonify({'status': 'error', 'msg': '잘못된 레이블'})
    if not CAM_AVAILABLE:  # 이렇게
        return jsonify({'status': 'error', 'msg': '웹캠 없음'})

    with lock:
        frame_bytes = latest_frame
    if frame_bytes is None:
        return jsonify({'status': 'error', 'msg': '캡처 실패'})
    
    img_array = np.frombuffer(frame_bytes, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    if frame is None:
        return jsonify({'status': 'error', 'msg': '캡처 실패'})

    path = f"ai_core/references/{label}.jpg"
    cv2.imwrite(path, frame)
    saved_status[label] = True
    print(f"[저장] {path}")
    return jsonify({'status': 'ok', 'path': path})

@app.route('/status')
def status():
    result = {}
    for key in LABELS:
        path = f"ai_core/references/{key}.jpg"
        result[key] = os.path.exists(path)
    return jsonify(result)

@app.route('/')
def index():
    return render_template_string(HTML)

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Reference Capture</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #0a0a0f; color: #e0e0e0;
         font-family: 'Courier New', monospace; min-height: 100vh;
         display: flex; flex-direction: column; align-items: center;
         padding: 24px; gap: 20px; }

  h1 { color: #00e5ff; font-size: 1rem; letter-spacing: 3px;
       text-transform: uppercase; }

  .cam-wrap { position: relative; border: 2px solid #182038;
              border-radius: 10px; overflow: hidden; width: 100%; max-width: 640px; }
  .cam-wrap img { width: 100%; display: block; }

  .overlay-label {
    position: absolute; top: 12px; left: 12px;
    background: rgba(0,0,0,0.7); border: 1px solid #00e5ff;
    color: #00e5ff; padding: 4px 12px; border-radius: 6px;
    font-size: 0.75rem; letter-spacing: 1px;
  }

  .btn-grid { display: flex; gap: 12px; width: 100%; max-width: 640px; }

  .btn-capture {
    flex: 1; padding: 18px 10px; border: 2px solid #2a3a5a;
    background: #0d1220; color: #5a6a8a;
    border-radius: 10px; cursor: pointer;
    font-family: inherit; font-size: 0.8rem;
    font-weight: bold; letter-spacing: 1px;
    transition: all 0.2s; text-align: center;
  }
  .btn-capture:hover { border-color: #00e5ff; color: #00e5ff; background: #0a1628; }
  .btn-capture.saved { border-color: #00ff9d; color: #00ff9d; background: #001a10; }
  .btn-capture.saving { border-color: #ffd600; color: #ffd600; }

  .btn-label { font-size: 0.65rem; color: inherit; margin-top: 6px; }
  .check { font-size: 1.2rem; }

  .guide {
    width: 100%; max-width: 640px;
    background: #0d1220; border: 1px solid #182038;
    border-radius: 10px; padding: 16px;
    font-size: 0.75rem; color: #5a6a8a; line-height: 1.8;
  }
  .guide span { color: #00e5ff; }
</style>
</head>
<body>

<h1>📸 Reference Capture</h1>

<div class="cam-wrap">
  <img src="/video_feed" alt="webcam">
  <div class="overlay-label" id="overlay">LIVE</div>
</div>

<div class="btn-grid">
  <button class="btn-capture" id="btn-flat" onclick="capture('flat')">
    <div class="check" id="check-flat">○</div>
    <div>FLAT</div>
    <div class="btn-label">평지</div>
  </button>
  <button class="btn-capture" id="btn-1cm" onclick="capture('1cm')">
    <div class="check" id="check-1cm">○</div>
    <div>1 cm</div>
    <div class="btn-label">방지턱</div>
  </button>
  <button class="btn-capture" id="btn-2cm" onclick="capture('2cm')">
    <div class="check" id="check-2cm">○</div>
    <div>2 cm</div>
    <div class="btn-label">방지턱</div>
  </button>
</div>

<div class="guide">
  <span>사용법</span><br>
  1. 로봇을 장애물 앞 실제 주행 위치에 놓기<br>
  2. 웹캠 앵글 확인 후 버튼 클릭<br>
  3. 세 장 모두 ✓ 되면 완료<br><br>
  <span>저장 위치</span> ai_core/references/{flat,1cm,2cm}.jpg
</div>

<script>
function capture(label) {
  const btn = document.getElementById('btn-' + label);
  const overlay = document.getElementById('overlay');
  btn.className = 'btn-capture saving';
  overlay.textContent = '촬영중...';

  fetch('/capture', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({label})
  }).then(r => r.json()).then(d => {
    if (d.status === 'ok') {
      btn.className = 'btn-capture saved';
      document.getElementById('check-' + label).textContent = '✓';
      overlay.textContent = '저장완료: ' + d.path;
      setTimeout(() => overlay.textContent = 'LIVE', 2000);
    } else {
      btn.className = 'btn-capture';
      overlay.textContent = '오류: ' + d.msg;
      setTimeout(() => overlay.textContent = 'LIVE', 2000);
    }
  });
}

// 저장 상태 폴링
function pollStatus() {
  fetch('/status').then(r => r.json()).then(d => {
    for (const [key, saved] of Object.entries(d)) {
      const btn = document.getElementById('btn-' + key);
      const chk = document.getElementById('check-' + key);
      if (saved && btn && !btn.classList.contains('saved')) {
        btn.className = 'btn-capture saved';
        chk.textContent = '✓';
      }
    }
  });
}
setInterval(pollStatus, 2000);
pollStatus();
</script>
</body>
</html>'''

if __name__ == '__main__':
    print("레퍼런스 촬영 대시보드: http://192.168.0.50:5010")
    app.run(host='0.0.0.0', port=5010, threaded=True)