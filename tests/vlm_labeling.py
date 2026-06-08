# experiment/vlm_labeling.py
# VLM 라벨링 대시보드
#
# 실행: python tests/vlm_labeling.py
# 접속: http://192.168.0.50:5002
#
# 기능:
#   - USB 웹캠 실시간 스트리밍
#   - 초음파 거리 표시
#   - 캡처 → Gemini 분석 → 정답값 입력 → CSV 저장

import sys, os, time, json, csv, threading, copy, warnings
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template_string, jsonify, request
import cv2
import numpy as np

BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_CSV = os.path.join(BASE_DIR, "experiment", "vlm_labels.csv")
PHOTO_DIR  = os.path.join(BASE_DIR, "experiment", "obstacle_photos")
os.makedirs(PHOTO_DIR, exist_ok=True)

app  = Flask(__name__)
lock = threading.Lock()

# ── USB 웹캠 ──────────────────────────────────────────
try:
    webcam = cv2.VideoCapture(0)
    webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    WEBCAM_AVAILABLE = webcam.isOpened()
    print(f"[{'OK' if WEBCAM_AVAILABLE else 'SKIP'}] USB 웹캠")
except Exception as e:
    webcam = None
    WEBCAM_AVAILABLE = False
    print(f"[SKIP] 웹캠: {e}")

# ── 초음파 ────────────────────────────────────────────
try:
    from gpiozero import DistanceSensor
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
    ULTRA_AVAILABLE = True
    print("[OK] 초음파")
except Exception as e:
    ultra = None
    ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파: {e}")

# ── VLM ───────────────────────────────────────────────
try:
    from ai_core.BY.vlm_client_by import VLMClient
    vlm = VLMClient()
    VLM_AVAILABLE = True
except Exception as e:
    vlm = None
    VLM_AVAILABLE = False
    print(f"[SKIP] VLM: {e}")

# ── 공유 상태 ─────────────────────────────────────────
state = {
    "distance":   -1,
    "vlm_result": None,
    "last_photo": None,   # base64 jpeg
    "count":      0,
    "analyzing":  False,
}

CSV_FIELDS = [
    "timestamp", "photo_path",
    "gemini_type", "gemini_height_cm", "gemini_slope_deg", "gemini_surface",
    "actual_height_cm", "actual_slope_deg", "actual_surface",
    "label", "note"
]

LABEL_OPTIONS = {
    "normal": "정상 주행",
    "cautious_pass": "조심히 통과",
    "hill_fail": "언덕 실패",
    "impact": "충격",
    "slip": "미끄러짐",
}

# ── CSV 초기화 ────────────────────────────────────────
def init_csv():
    if not os.path.exists(OUTPUT_CSV):
        with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=CSV_FIELDS).writeheader()

init_csv()

# ── 초음파 백그라운드 ─────────────────────────────────
def ultra_loop():
    while True:
        if ULTRA_AVAILABLE:
            try:
                d = ultra.distance
                dist = round(d * 100, 1) if d else -1
            except:
                dist = -1
        else:
            dist = -1
        with lock:
            state["distance"] = dist
        time.sleep(0.1)

threading.Thread(target=ultra_loop, daemon=True).start()

# ── 웹캠 프레임 ───────────────────────────────────────
latest_frame = None

def webcam_loop():
    global latest_frame
    while True:
        if WEBCAM_AVAILABLE and webcam:
            ret, frame = webcam.read()
            if ret:
                # 초음파 오버레이
                with lock:
                    dist = state["distance"]
                dist_txt = f"{dist:.1f}cm" if dist >= 0 else "N/A"
                color = (0, 60, 255) if (dist >= 0 and dist < 30) else (180, 180, 180)
                cv2.putText(frame, f"ULTRA: {dist_txt}",
                            (frame.shape[1]-160, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                with lock:
                    latest_frame = jpeg.tobytes()
        time.sleep(0.033)

threading.Thread(target=webcam_loop, daemon=True).start()

# ── Flask 라우트 ──────────────────────────────────────
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

@app.route('/status')
def status():
    with lock:
        return jsonify({
            "distance":  state["distance"],
            "count":     state["count"],
            "analyzing": state["analyzing"],
            "vlm_result":state["vlm_result"],
        })

@app.route('/capture', methods=['POST'])
def capture():
    """웹캠 캡처 → Gemini 분석"""
    if not WEBCAM_AVAILABLE:
        return jsonify({"status": "error", "msg": "웹캠 없음"})
    if not VLM_AVAILABLE:
        return jsonify({"status": "error", "msg": "VLM 없음"})

    with lock:
        if state["analyzing"]:
            return jsonify({"status": "error", "msg": "분석 중..."})
        state["analyzing"] = True

    def do_analyze():
        try:
            ret, frame = webcam.read()
            if not ret:
                with lock:
                    state["analyzing"] = False
                return

            # 사진 저장
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename  = f"obstacle_{timestamp}.jpg"
            filepath  = os.path.join(PHOTO_DIR, filename)
            cv2.imwrite(filepath, frame)

            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            jpeg_bytes = jpeg.tobytes()

            # Gemini 분석
            result = vlm.analyze(jpeg_bytes)
            result["_filepath"]  = filepath
            result["_timestamp"] = timestamp

            with lock:
                state["vlm_result"] = result
                state["analyzing"]  = False

        except Exception as e:
            print(f"[캡처오류] {e}")
            with lock:
                state["analyzing"] = False

    threading.Thread(target=do_analyze, daemon=True).start()
    return jsonify({"status": "ok", "msg": "분석 중..."})

@app.route('/save', methods=['POST'])
def save():
    """정답값 저장"""
    data = request.get_json(force=True)

    with lock:
        vlm_result = copy.deepcopy(state["vlm_result"])

    if not vlm_result:
        return jsonify({"status": "error", "msg": "먼저 캡처하세요"})

    row = {
        "timestamp":        vlm_result.get("_timestamp", ""),
        "photo_path":       vlm_result.get("_filepath", ""),
        "gemini_type":      vlm_result.get("obstacle_type", ""),
        "gemini_height_cm": vlm_result.get("height_cm", 0.0),
        "gemini_slope_deg": vlm_result.get("slope_deg", 0.0),
        "gemini_surface":   vlm_result.get("surface_type", "normal"),
        "actual_height_cm": float(data.get("height_cm", vlm_result.get("height_cm", 0.0))),
        "actual_slope_deg": float(data.get("slope_deg", vlm_result.get("slope_deg", 0.0))),
        "actual_surface":   data.get("surface", vlm_result.get("surface_type", "normal")),
        "label":            data.get("label", "normal"),
        "note":             data.get("note", ""),
    }

    with open(OUTPUT_CSV, "a", newline="", encoding="utf-8") as f:
        csv.DictWriter(f, fieldnames=CSV_FIELDS).writerow(row)

    with lock:
        state["count"] += 1
        state["vlm_result"] = None

    print(f"[저장] {row['label']} / 실제높이={row['actual_height_cm']}cm")
    return jsonify({"status": "ok", "count": state["count"]})

@app.route('/stats')
def stats():
    """라벨 분포"""
    import collections
    labels = []
    if os.path.exists(OUTPUT_CSV):
        with open(OUTPUT_CSV, "r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                labels.append(row["label"])
    return jsonify(dict(collections.Counter(labels)))

@app.route('/')
def index():
    return render_template_string(HTML)

HTML = '''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>VLM 라벨링 도구</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&display=swap');
  :root {
    --bg: #0c0f18; --panel: #131720; --border: #1e2740;
    --cyan: #00e5ff; --green: #00ff9d; --yellow: #ffd600;
    --red: #ff3d57; --dim: #3a4a6b; --text: #cdd6f4; --sub: #6b7fa8;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text);
         font-family: 'JetBrains Mono', monospace; min-height: 100vh; }
  body::before {
    content:''; position:fixed; inset:0; pointer-events:none; z-index:0;
    background-image: linear-gradient(rgba(0,229,255,.025) 1px,transparent 1px),
                      linear-gradient(90deg,rgba(0,229,255,.025) 1px,transparent 1px);
    background-size: 36px 36px;
  }
  header {
    position:relative; z-index:10;
    background: var(--panel); border-bottom: 1px solid var(--border);
    padding: 14px 24px; display:flex; align-items:center; justify-content:space-between;
  }
  .logo { font-size:1rem; font-weight:700; color:var(--cyan); letter-spacing:2px; }
  .badge {
    display:flex; align-items:center; gap:6px;
    font-size:.7rem; color:var(--sub);
  }
  .dot { width:7px; height:7px; border-radius:50%;
         background:var(--green); box-shadow:0 0 8px var(--green);
         animation: pulse 2s infinite; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.3} }

  .layout { position:relative; z-index:1;
             display:grid; grid-template-columns:1fr 380px;
             gap:12px; padding:12px; }

  .card { background:var(--panel); border:1px solid var(--border);
          border-radius:12px; padding:16px; }
  .card-title { font-size:.7rem; color:var(--sub); letter-spacing:2px;
                text-transform:uppercase; margin-bottom:12px; }

  /* 카메라 */
  .cam-wrap { position:relative; border-radius:8px; overflow:hidden;
              background:#000; border:1px solid var(--border); }
  .cam-wrap img { width:100%; display:block; }
  .dist-overlay {
    position:absolute; top:10px; left:10px;
    background:rgba(0,0,0,.7); border:1px solid var(--border);
    border-radius:6px; padding:6px 12px; font-size:.85rem;
  }
  .dist-val { color:var(--green); font-weight:700; }
  .dist-val.warn  { color:var(--yellow); }
  .dist-val.danger{ color:var(--red); animation:blink .4s step-end infinite; }
  @keyframes blink { 50%{opacity:.2} }

  /* 캡처 버튼 */
  .btn-capture {
    width:100%; margin-top:12px; padding:14px;
    background: linear-gradient(135deg, #0077aa, #00e5ff);
    border:none; border-radius:8px; color:#000;
    font-family:inherit; font-size:.9rem; font-weight:700;
    cursor:pointer; letter-spacing:1px;
    transition: all .2s;
  }
  .btn-capture:hover { filter:brightness(1.15); }
  .btn-capture:disabled { background:var(--dim); color:var(--sub); cursor:not-allowed; }

  /* 사이드패널 */
  .side { display:flex; flex-direction:column; gap:12px; }

  /* 초음파 카드 */
  .ultra-big { text-align:center; padding:20px 16px; }
  .ultra-num { font-size:2.8rem; font-weight:700; color:var(--green);
               line-height:1; transition:color .2s; }
  .ultra-num.warn   { color:var(--yellow); }
  .ultra-num.danger { color:var(--red); }
  .ultra-unit { font-size:.75rem; color:var(--sub); margin-top:4px; }

  /* Gemini 결과 */
  .gemini-result { display:none; }
  .gemini-result.show { display:block; }
  .analyzing-spinner {
    display:none; text-align:center; padding:20px;
    color:var(--cyan); font-size:.8rem;
  }
  .analyzing-spinner.show { display:block; }
  .spin { display:inline-block; animation:spin 1s linear infinite; }
  @keyframes spin { to{transform:rotate(360deg)} }

  .result-row { display:flex; justify-content:space-between;
                align-items:center; padding:7px 0;
                border-bottom:1px solid var(--border); font-size:.8rem; }
  .result-row:last-child { border-bottom:none; }
  .result-key { color:var(--sub); }
  .result-val { color:var(--cyan); font-weight:600; }

  /* 입력 폼 */
  .form-group { margin-bottom:10px; }
  .form-group label { font-size:.7rem; color:var(--sub);
                      display:block; margin-bottom:4px; }
  .form-group input, .form-group select, .form-group textarea {
    width:100%; background:rgba(0,0,0,.3);
    border:1px solid var(--border); border-radius:6px;
    padding:8px 10px; color:var(--text);
    font-family:inherit; font-size:.8rem;
    outline:none; transition:border-color .2s;
  }
  .form-group input:focus, .form-group select:focus { border-color:var(--cyan); }
  .form-group textarea { height:56px; resize:none; }

  .btn-save {
    width:100%; padding:12px;
    background: linear-gradient(135deg, #006644, #00ff9d);
    border:none; border-radius:8px; color:#000;
    font-family:inherit; font-size:.85rem; font-weight:700;
    cursor:pointer; letter-spacing:1px; transition:all .2s;
  }
  .btn-save:hover { filter:brightness(1.1); }
  .btn-save:disabled { background:var(--dim); color:var(--sub); cursor:not-allowed; }

  /* 통계 */
  .stat-bar { margin-bottom:6px; }
  .stat-label { display:flex; justify-content:space-between;
                font-size:.7rem; margin-bottom:3px; }
  .stat-label span:first-child { color:var(--sub); }
  .stat-label span:last-child  { color:var(--text); font-weight:600; }
  .bar-bg { background:rgba(255,255,255,.06); border-radius:3px; height:5px; }
  .bar-fill { height:5px; border-radius:3px; transition:width .4s; }

  .toast {
    position:fixed; bottom:24px; right:24px; z-index:100;
    background:var(--panel); border:1px solid var(--green);
    border-radius:8px; padding:12px 20px;
    font-size:.8rem; color:var(--green);
    opacity:0; transform:translateY(10px);
    transition:all .3s; pointer-events:none;
  }
  .toast.show { opacity:1; transform:none; }
</style>
</head>
<body>
<header>
  <div class="logo">VLM LABELING</div>
  <div class="badge">
    <div class="dot"></div>
    <span id="count-badge">0개 수집</span>
  </div>
</header>

<div class="layout">
  <!-- 왼쪽: 카메라 -->
  <div>
    <div class="card">
      <div class="card-title">USB Webcam</div>
      <div class="cam-wrap">
        <img src="/video_feed" alt="webcam">
        <div class="dist-overlay">
          초음파 <span class="dist-val" id="dist-overlay">--</span>
        </div>
      </div>
      <button class="btn-capture" id="btn-capture" onclick="doCapture()">
        📷 캡처 + Gemini 분석
      </button>
    </div>

    <!-- 라벨 통계 -->
    <div class="card" style="margin-top:12px;">
      <div class="card-title">수집 현황</div>
      <div id="stats-area"></div>
    </div>
  </div>

  <!-- 오른쪽: 사이드 -->
  <div class="side">

    <!-- 초음파 -->
    <div class="card ultra-big">
      <div class="card-title" style="text-align:center">Ultrasonic Distance</div>
      <div class="ultra-num" id="ultra-num">--</div>
      <div class="ultra-unit">cm</div>
    </div>

    <!-- Gemini 결과 -->
    <div class="card">
      <div class="card-title">Gemini 분석 결과</div>
      <div class="analyzing-spinner" id="spinner">
        <div class="spin">⟳</div> 분석 중...
      </div>
      <div class="gemini-result" id="gemini-result">
        <div class="result-row">
          <span class="result-key">장애물 종류</span>
          <span class="result-val" id="r-type">--</span>
        </div>
        <div class="result-row">
          <span class="result-key">추정 높이</span>
          <span class="result-val" id="r-height">--</span>
        </div>
        <div class="result-row">
          <span class="result-key">경사각</span>
          <span class="result-val" id="r-slope">--</span>
        </div>
        <div class="result-row">
          <span class="result-key">표면</span>
          <span class="result-val" id="r-surface">--</span>
        </div>
        <div class="result-row">
          <span class="result-key">신뢰도</span>
          <span class="result-val" id="r-conf">--</span>
        </div>
        <div class="result-row" style="border:none;padding-top:8px;">
          <span class="result-key" style="font-size:.65rem;color:var(--dim)" id="r-desc"></span>
        </div>
      </div>
    </div>

    <!-- 정답 입력 폼 -->
    <div class="card">
      <div class="card-title">정답값 입력</div>
      <div class="form-group">
        <label>실제 높이 (cm)</label>
        <input type="number" id="f-height" step="0.5" min="0" placeholder="예: 2.0">
      </div>
      <div class="form-group">
        <label>실제 경사각 (도)</label>
        <input type="number" id="f-slope" step="1" min="0" placeholder="예: 10">
      </div>
      <div class="form-group">
        <label>표면 종류</label>
        <select id="f-surface">
          <option value="normal">normal (일반)</option>
          <option value="vinyl">vinyl (비닐)</option>
          <option value="rough">rough (요철)</option>
        </select>
      </div>
      <div class="form-group">
        <label>실험 결과 레이블</label>
        <select id="f-label">
          <option value="normal">normal (정상 주행)</option>
          <option value="cautious_pass">cautious_pass (조심히 통과)</option>
          <option value="hill_fail">hill_fail (언덕 실패)</option>
          <option value="impact">impact (충격)</option>
          <option value="slip">slip (미끄러짐)</option>
        </select>
      </div>
      <div class="form-group">
        <label>메모 (선택)</label>
        <textarea id="f-note" placeholder="예: 2cm 비닐 방지턱, 무게 3kg"></textarea>
      </div>
      <button class="btn-save" id="btn-save" onclick="doSave()" disabled>
        💾 저장
      </button>
    </div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
let hasResult = false;

// ── 폴링 ─────────────────────────────────────────────
function poll() {
  fetch('/status').then(r=>r.json()).then(d => {
    // 초음파
    const dist = d.distance;
    const numEl = document.getElementById('ultra-num');
    const ovEl  = document.getElementById('dist-overlay');
    if (dist < 0) {
      numEl.textContent = '--';
      numEl.className   = 'ultra-num';
      ovEl.textContent  = 'N/A';
      ovEl.className    = 'dist-val';
    } else {
      numEl.textContent = dist.toFixed(1);
      ovEl.textContent  = dist.toFixed(1) + 'cm';
      const cls = dist < 15 ? 'danger' : dist < 30 ? 'warn' : '';
      numEl.className = 'ultra-num' + (cls ? ' '+cls : '');
      ovEl.className  = 'dist-val'  + (cls ? ' '+cls : '');
    }

    // 분석 중
    const spinner = document.getElementById('spinner');
    const result  = document.getElementById('gemini-result');
    if (d.analyzing) {
      spinner.classList.add('show');
      result.classList.remove('show');
      document.getElementById('btn-capture').disabled = true;
    } else {
      spinner.classList.remove('show');
      document.getElementById('btn-capture').disabled = false;
    }

    // Gemini 결과
    if (d.vlm_result && !d.analyzing) {
      const v = d.vlm_result;
      document.getElementById('r-type').textContent    = v.obstacle_type || '--';
      document.getElementById('r-height').textContent  = (v.height_cm ?? '--') + ' cm';
      document.getElementById('r-slope').textContent   = (v.slope_deg  ?? '--') + ' 도';
      document.getElementById('r-surface').textContent = v.surface_type || '--';
      document.getElementById('r-conf').textContent    = v.confidence != null
        ? (v.confidence * 100).toFixed(0) + '%' : '--';
      document.getElementById('r-desc').textContent    = v.description || '';

      // 폼 기본값 채우기
      if (!hasResult) {
        document.getElementById('f-height').value  = v.height_cm ?? '';
        document.getElementById('f-slope').value   = v.slope_deg ?? '';
        const surf = document.getElementById('f-surface');
        for (let opt of surf.options) if (opt.value === v.surface_type) opt.selected = true;
        hasResult = true;
      }

      result.classList.add('show');
      document.getElementById('btn-save').disabled = false;
    }

    // 카운트
    document.getElementById('count-badge').textContent = d.count + '개 수집';
  }).catch(()=>{});
}
setInterval(poll, 300);

// ── 통계 ─────────────────────────────────────────────
const LABEL_COLORS = {
  normal:'#00ff9d', cautious_pass:'#ffd600',
  hill_fail:'#ff7043', impact:'#ff3d57', slip:'#aa00ff'
};
function updateStats() {
  fetch('/stats').then(r=>r.json()).then(d => {
    const area  = document.getElementById('stats-area');
    const total = Object.values(d).reduce((a,b)=>a+b, 0);
    if (total === 0) { area.innerHTML = '<div style="color:var(--dim);font-size:.75rem;text-align:center;padding:8px">아직 데이터 없음</div>'; return; }
    area.innerHTML = Object.entries(d).map(([label, cnt]) => {
      const pct  = Math.round(cnt / total * 100);
      const color= LABEL_COLORS[label] || '#0af';
      return `<div class="stat-bar">
        <div class="stat-label"><span>${label}</span><span>${cnt}개 (${pct}%)</span></div>
        <div class="bar-bg"><div class="bar-fill" style="width:${pct}%;background:${color}"></div></div>
      </div>`;
    }).join('');
  });
}
setInterval(updateStats, 2000);
updateStats();

// ── 캡처 ─────────────────────────────────────────────
function doCapture() {
  hasResult = false;
  document.getElementById('btn-save').disabled = true;
  document.getElementById('gemini-result').classList.remove('show');
  fetch('/capture', {method:'POST'}).then(r=>r.json()).then(d=>{
    if (d.status !== 'ok') showToast('❌ ' + d.msg, true);
  });
}

// ── 저장 ─────────────────────────────────────────────
function doSave() {
  const payload = {
    height_cm: parseFloat(document.getElementById('f-height').value) || 0,
    slope_deg: parseFloat(document.getElementById('f-slope').value)  || 0,
    surface:   document.getElementById('f-surface').value,
    label:     document.getElementById('f-label').value,
    note:      document.getElementById('f-note').value,
  };
  fetch('/save', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(payload)
  }).then(r=>r.json()).then(d=>{
    if (d.status === 'ok') {
      showToast(`✓ 저장 완료 — 총 ${d.count}개`);
      document.getElementById('f-note').value = '';
      document.getElementById('btn-save').disabled = true;
      document.getElementById('gemini-result').classList.remove('show');
      hasResult = false;
      updateStats();
    } else {
      showToast('❌ ' + d.msg, true);
    }
  });
}

// ── 토스트 ────────────────────────────────────────────
function showToast(msg, isErr=false) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.borderColor = isErr ? 'var(--red)' : 'var(--green)';
  t.style.color       = isErr ? 'var(--red)' : 'var(--green)';
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2500);
}
</script>
</body>
</html>'''

if __name__ == '__main__':
    print("VLM 라벨링 대시보드 시작: http://192.168.0.50:5002")
    app.run(host='0.0.0.0', port=5002, threaded=True)