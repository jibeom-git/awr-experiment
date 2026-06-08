#!/usr/bin/env python3
# experiment/visualize.py
# experiment_v2.csv 데이터를 분석하여 HTML 분석 리포트(report.html)를 생성하는 독립 스크립트
# 실행: python experiment/visualize.py  →  experiment/data/report.html 생성 (브라우저로 열람)

import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

INSITE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(INSITE_ROOT))

DATA_DIR    = Path(__file__).resolve().parent / "data"
V2_CSV      = DATA_DIR / "experiment_v2.csv"
REPORT_PATH = DATA_DIR / "report.html"

MODELS_DIR       = INSITE_ROOT / "models"
ROUTE_MODEL_PATH = MODELS_DIR / "xgboost_route.json"
SPEED_MODEL_PATH = MODELS_DIR / "xgboost_speed.json"
IF_MODEL_PATH    = MODELS_DIR / "isolation_forest.pkl"
HISTORY_CSV      = INSITE_ROOT / "data" / "real_agv_history.csv"

ANOMALY_THRESHOLD = 0.6

# result 라벨별 표시 색상 (파이차트 / 산점도 공용)
RESULT_COLORS = {
    "normal":        "#3fb950",
    "slip_risk":     "#d29922",
    "hill_fail":     "#f85149",
    "rollover_risk": "#d29922",
    "rollover":      "#da3633",
    "bump_pass":     "#58a6ff",
    "bump_fail":     "#db6d28",
}
DEFAULT_COLOR = "#8b949e"

# 산점도(차트 3) 전용 색상 — 명세에 명시된 4종만 표시
SCATTER_COLORS = {
    "normal":    "#3fb950",
    "slip_risk": "#d29922",
    "hill_fail": "#f85149",
    "bump_fail": "#db6d28",
}


# ─────────────────────────────────────────────────────────────────────────────
# 데이터 로드
# ─────────────────────────────────────────────────────────────────────────────

def load_rows() -> list:
    """experiment_v2.csv 를 읽어 숫자 열을 float로 변환한 딕셔너리 리스트로 반환"""
    if not V2_CSV.exists():
        print(f"[ERROR] 파일 없음: {V2_CSV}")
        return []
    rows = []
    with open(V2_CSV, newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            try:
                rows.append({
                    "timestamp":     raw.get("timestamp", ""),
                    "session_id":    raw.get("session_id", ""),
                    "route":         raw.get("route", ""),
                    "phase":         raw.get("phase", ""),
                    "speed_cmd":     float(raw.get("speed_cmd") or 0),
                    "weight_g":      float(raw.get("weight_g") or 0),
                    "obstacle":      raw.get("obstacle", "none"),
                    "pitch":         float(raw.get("pitch") or 0),
                    "sonic_cm":      float(raw.get("sonic_cm") or 0),
                    "accel_x":       float(raw.get("accel_x") or 0),
                    "accel_y":       float(raw.get("accel_y") or 0),
                    "accel_z":       float(raw.get("accel_z") or 0),
                    "pitch_delta":   float(raw.get("pitch_delta") or 0),
                    "accel_z_delta": float(raw.get("accel_z_delta") or 0),
                    "result":        raw.get("result", ""),
                })
            except (TypeError, ValueError):
                continue
    return rows


# ─────────────────────────────────────────────────────────────────────────────
# 차트 1: 세션별 실시간 센서 시계열
# ─────────────────────────────────────────────────────────────────────────────

def build_session_series(rows: list, max_sessions: int = 30, min_rows: int = 3) -> dict:
    """
    행 수가 min_rows 이상인 세션을 행 수 내림차순으로 max_sessions개까지 추려
    시계열 데이터로 구성한다 (단일 행짜리 레거시 세션은 시계열로서 의미가 없어 제외).
    """
    by_session = defaultdict(list)
    for r in rows:
        by_session[r["session_id"]].append(r)

    candidates = [(sid, recs) for sid, recs in by_session.items() if len(recs) >= min_rows]
    candidates.sort(key=lambda kv: len(kv[1]), reverse=True)
    candidates = candidates[:max_sessions]

    series = {}
    for sid, recs in candidates:
        series[sid] = {
            "labels":        [r["timestamp"][11:] for r in recs],  # 시:분:초.밀리초
            "pitch":         [r["pitch"] for r in recs],
            "accel_z":       [r["accel_z"] for r in recs],
            "accel_z_delta": [r["accel_z_delta"] for r in recs],
            "sonic_cm":      [r["sonic_cm"] for r in recs],
            # 정상(normal)/기록 중(recording)이 아닌 행을 이상 구간으로 표시
            "anomaly":       [r["result"] not in ("normal", "recording", "") for r in recs],
        }
    return series


# ─────────────────────────────────────────────────────────────────────────────
# 차트 2: result 분포
# ─────────────────────────────────────────────────────────────────────────────

def build_result_distribution(rows: list) -> Counter:
    return Counter(r["result"] for r in rows if r["result"])


# ─────────────────────────────────────────────────────────────────────────────
# 차트 3: pitch vs speed 산점도 (오르막 데이터만)
# ─────────────────────────────────────────────────────────────────────────────

def build_pitch_speed_scatter(rows: list) -> dict:
    datasets = defaultdict(list)
    for r in rows:
        if r["phase"] != "up" or r["result"] not in SCATTER_COLORS:
            continue
        datasets[r["result"]].append({"x": r["speed_cmd"], "y": r["pitch"]})
    return dict(datasets)


# ─────────────────────────────────────────────────────────────────────────────
# 차트 4: XGBoost 교차검증 정확도 (real_agv_history.csv 기준)
# ─────────────────────────────────────────────────────────────────────────────

def compute_xgboost_accuracy():
    if not (ROUTE_MODEL_PATH.exists() and SPEED_MODEL_PATH.exists() and HISTORY_CSV.exists()):
        return None
    try:
        import xgboost as xgb
        from sklearn.model_selection import cross_val_score
        from ai_core import trainer as tr
    except ImportError as e:
        print(f"[WARN] XGBoost 정확도 계산 건너뜀 — 모듈 없음: {e}")
        return None

    X, y_route, y_speed = [], [], []
    with open(HISTORY_CSV, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                route  = row.get("Assigned_Route", "")
                speed  = int(row.get("Applied_Speed", 40))
                obs    = (row.get("Obstacle_Type", "none") or "none").lower()
                status = row.get("Pass_Status", "")
                if route not in tr.ROUTE_ENC:
                    continue
                speed_key = min(tr.SPEED_VALUES, key=lambda s: abs(s - speed))
                if speed_key not in tr.SPEED_ENC:
                    continue

                pitch    = float(row.get("pitch", 0.0) or 0.0)
                weight   = float(row.get("weight", 0.0) or 0.0)
                sonic    = float(row.get("sonic", 400.0) or 400.0)
                impact_z = float(row.get("Impact_Z", 0.0) or 0.0)
                conf     = float(row.get("Gemini_Confidence", 0.5) or 0.5)
                obs_enc  = tr.OBS_ENC.get(obs, 0)
                passable = 0 if status in ("PATH_BLOCKED", "FATAL_DEADLOCK") else 1
                mode_enc = 1 if "FAST" in (row.get("System_Message", "") or "") else 0

                X.append([pitch, weight, sonic, obs_enc, mode_enc, passable, conf, impact_z])
                y_route.append(tr.ROUTE_ENC[route])
                y_speed.append(tr.SPEED_ENC[speed_key])
            except Exception:
                continue

    if len(X) < 10:
        return None

    X       = np.array(X, dtype=np.float32)
    y_route = np.array(y_route)
    y_speed = np.array(y_speed)
    cv      = min(5, len(X))

    try:
        route_scores = cross_val_score(
            xgb.XGBClassifier(n_estimators=50, max_depth=4), X, y_route, cv=cv)
        speed_scores = cross_val_score(
            xgb.XGBClassifier(n_estimators=50, max_depth=4), X, y_speed, cv=cv)
    except Exception as e:
        print(f"[WARN] XGBoost 교차검증 실패: {e}")
        return None

    return {
        "route_acc": round(float(route_scores.mean()), 3),
        "speed_acc": round(float(speed_scores.mean()), 3),
        "n_samples": len(X),
        "cv":        cv,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 차트 5: Isolation Forest 이상 점수 분포
# ─────────────────────────────────────────────────────────────────────────────

def compute_if_anomaly_scores(rows: list):
    if not IF_MODEL_PATH.exists():
        return None
    try:
        import joblib
    except ImportError:
        return None
    try:
        if_model = joblib.load(IF_MODEL_PATH)
    except Exception as e:
        print(f"[WARN] IF 모델 로드 실패: {e}")
        return None

    groups = {"normal": [], "hill_fail": [], "bump_fail": []}
    for r in rows:
        if r["result"] not in groups:
            continue
        try:
            # ai_core.trainer.Trainer.anomaly_score 와 동일한 피처 구성/정규화 방식 사용
            x   = np.array([[r["pitch"], r["weight_g"], r["sonic_cm"], r["accel_z"]]], dtype=np.float32)
            raw = float(if_model.decision_function(x)[0])
            score = max(0.0, min(1.0, (-raw) / 1.5))
            groups[r["result"]].append(round(score, 3))
        except Exception:
            continue

    if not any(groups.values()):
        return None
    return groups


def _histogram(values: list, bins: int = 12, rng=(0.0, 1.0)):
    """0~1 구간을 균등 분할한 히스토그램 (counts, bin_edges) 반환"""
    if not values:
        return [0] * bins, list(np.linspace(rng[0], rng[1], bins + 1))
    counts, edges = np.histogram(values, bins=bins, range=rng)
    return counts.tolist(), edges.tolist()


# ─────────────────────────────────────────────────────────────────────────────
# HTML 리포트 렌더링
# ─────────────────────────────────────────────────────────────────────────────

def render_report(rows):
    session_series = build_session_series(rows)
    result_dist    = build_result_distribution(rows)
    scatter_data   = build_pitch_speed_scatter(rows)
    xgb_acc        = compute_xgboost_accuracy()
    if_scores      = compute_if_anomaly_scores(rows)

    # ── 차트 2: 파이차트 데이터 ──
    result_labels = list(result_dist.keys())
    result_counts = [result_dist[k] for k in result_labels]
    result_colors = [RESULT_COLORS.get(k, DEFAULT_COLOR) for k in result_labels]

    # ── 차트 3: 산점도 데이터셋 ──
    scatter_datasets = [
        {"label": label, "color": SCATTER_COLORS[label], "data": pts}
        for label, pts in scatter_data.items()
    ]

    # ── 차트 5: 히스토그램 ──
    if_hist = None
    if if_scores:
        if_hist = {}
        for key in ("normal", "hill_fail", "bump_fail"):
            counts, edges = _histogram(if_scores.get(key, []))
            if_hist[key] = {"counts": counts, "edges": edges, "n": len(if_scores.get(key, []))}

    # ── 해석 텍스트용 통계 ──
    up_normal   = [r for r in rows if r["phase"] == "up" and r["result"] == "normal"]
    up_fail     = [r for r in rows if r["phase"] == "up" and r["result"] == "hill_fail"]
    fail_pitch_avg   = round(float(np.mean([r["pitch"] for r in up_fail])), 2) if up_fail else None
    normal_pitch_avg = round(float(np.mean([r["pitch"] for r in up_normal])), 2) if up_normal else None
    total_rows = len(rows)
    n_sessions = len(set(r["session_id"] for r in rows if r["session_id"]))

    data = {
        "session_series":   session_series,
        "result_labels":    result_labels,
        "result_counts":    result_counts,
        "result_colors":    result_colors,
        "scatter_datasets": scatter_datasets,
        "xgb_acc":          xgb_acc,
        "if_hist":          if_hist,
        "anomaly_threshold": ANOMALY_THRESHOLD,
        "stats": {
            "total_rows":       total_rows,
            "n_sessions":       n_sessions,
            "fail_pitch_avg":   fail_pitch_avg,
            "normal_pitch_avg": normal_pitch_avg,
        },
    }
    data_json = json.dumps(data, ensure_ascii=False)

    html = _HTML_TEMPLATE.replace("__REPORT_DATA__", data_json)
    REPORT_PATH.write_text(html, encoding="utf-8")
    print(f"리포트 생성 완료 → {REPORT_PATH} ({total_rows}행, 세션 {n_sessions}개)")


_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>경사로 학습 데이터 분석 리포트</title>
<script src="https://cdn.tailwindcss.com"></script>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-annotation"></script>
<style>
  body { background:#0d1117; color:#c9d1d9; font-family:'Pretendard','Apple SD Gothic Neo',sans-serif; }
  .card { background:#161b22; border:1px solid #30363d; border-radius:10px; padding:18px; margin-bottom:24px; }
  .note { font-size:0.85rem; color:#8b949e; margin-top:10px; line-height:1.6; }
  h1 { font-size:1.4rem; font-weight:700; margin-bottom:6px; }
  h2 { font-size:1.05rem; font-weight:600; margin-bottom:12px; color:#58a6ff; }
  .empty { color:#8b949e; font-size:0.9rem; padding:24px; text-align:center; }
  select { background:#0d1117; color:#c9d1d9; border:1px solid #30363d; border-radius:6px; padding:6px 10px; }
</style>
</head>
<body class="p-6 max-w-5xl mx-auto">
  <h1>🚜 경사로 학습 데이터 분석 리포트</h1>
  <p class="note" id="summary-note"></p>

  <!-- 차트 1: 세션별 실시간 센서 시계열 -->
  <div class="card">
    <h2>1. 세션별 센서 시계열</h2>
    <div class="mb-3">
      <label class="text-sm text-gray-400">세션 선택:
        <select id="session-select"></select>
      </label>
    </div>
    <div id="session-chart-area"><canvas id="sessionChart" height="110"></canvas></div>
    <p class="note">
      이 차트는 선택한 세션의 시간 흐름에 따른 pitch(피치각), accel_z(수직 가속도),
      accel_z_delta(수직 가속도 변화량 — 충격/이상 감지 핵심 지표), sonic_cm(초음파 거리)의
      변화를 보여줍니다. 배경이 붉게 표시된 구간은 result가 normal/recording이 아닌
      이상(슬립/전복/실패 등) 구간입니다.
    </p>
  </div>

  <!-- 차트 2: result 분포 -->
  <div class="card">
    <h2>2. 결과(result) 분포</h2>
    <div class="max-w-md mx-auto"><canvas id="resultPieChart"></canvas></div>
    <p class="note">
      이 차트는 전체 수집 데이터에서 각 result 라벨이 차지하는 비율을 보여줍니다.
      normal 비중이 과도하게 높으면 이상 상황(슬립, 전복, 방지턱 실패 등) 학습 데이터가
      부족하다는 의미이므로, 다양한 라벨의 데이터를 균형 있게 수집할 필요가 있습니다.
    </p>
  </div>

  <!-- 차트 3: pitch vs speed 산점도 -->
  <div class="card">
    <h2>3. 오르막 pitch vs speed_cmd 산점도</h2>
    <canvas id="scatterChart" height="100"></canvas>
    <p class="note" id="scatter-note">
      이 차트는 오르막(phase=up) 구간에서 주행 속도(speed_cmd)와 측정된 경사각(pitch)에 따라
      결과가 어떻게 갈리는지 보여줍니다 (정상=초록, 슬립위험=노랑, 통과못함=빨강,
      방지턱실패=주황). 특정 속도 구간에서 빨강/주황 점이 밀집한다면, 그 속도로는
      해당 경사각을 통과하기 어렵다는 패턴으로 해석할 수 있습니다.
    </p>
  </div>

  <!-- 차트 4: XGBoost 교차검증 정확도 -->
  <div class="card">
    <h2>4. XGBoost 학습 모델 교차검증 정확도</h2>
    <div id="xgb-area"></div>
    <p class="note" id="xgb-note"></p>
  </div>

  <!-- 차트 5: Isolation Forest 이상 점수 분포 -->
  <div class="card">
    <h2>5. Isolation Forest 이상 점수(anomaly_score) 분포</h2>
    <div id="if-area"></div>
    <p class="note" id="if-note"></p>
  </div>

<script>
const REPORT = __REPORT_DATA__;

document.getElementById('summary-note').textContent =
  `총 ${REPORT.stats.total_rows.toLocaleString()}행 · 세션 ${REPORT.stats.n_sessions.toLocaleString()}개 — ` +
  `experiment_v2.csv 기준으로 자동 생성된 리포트입니다.`;

Chart.defaults.color = '#c9d1d9';
Chart.defaults.borderColor = '#30363d';

// ── 차트 1: 세션 시계열 ──────────────────────────────────────────────────
const sessionIds = Object.keys(REPORT.session_series);
const select = document.getElementById('session-select');
let sessionChart = null;

function anomalyBg(ctx, color) {
  // 이상(anomaly) 포인트만 강조 색상, 정상은 라인 색 그대로
  const idx = ctx.dataIndex;
  const series = REPORT.session_series[select.value];
  return (series && series.anomaly[idx]) ? '#f85149' : color;
}

function drawSessionChart(sid) {
  const s = REPORT.session_series[sid];
  if (sessionChart) sessionChart.destroy();
  if (!s) return;
  sessionChart = new Chart(document.getElementById('sessionChart'), {
    type: 'line',
    data: {
      labels: s.labels,
      datasets: [
        { label: 'pitch',         data: s.pitch,         borderColor: '#58a6ff', pointBackgroundColor: ctx => anomalyBg(ctx, '#58a6ff'), tension: 0.15 },
        { label: 'accel_z',       data: s.accel_z,       borderColor: '#3fb950', pointBackgroundColor: ctx => anomalyBg(ctx, '#3fb950'), tension: 0.15, hidden: true },
        { label: 'accel_z_delta', data: s.accel_z_delta, borderColor: '#d29922', pointBackgroundColor: ctx => anomalyBg(ctx, '#d29922'), tension: 0.15 },
        { label: 'sonic_cm',      data: s.sonic_cm,      borderColor: '#a371f7', pointBackgroundColor: ctx => anomalyBg(ctx, '#a371f7'), tension: 0.15, hidden: true },
      ]
    },
    options: {
      responsive: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { position: 'top' } },
      scales: { x: { ticks: { maxTicksLimit: 12 } } },
    }
  });
}

if (sessionIds.length === 0) {
  document.getElementById('session-chart-area').innerHTML =
    '<div class="empty">시계열로 표시할 만큼 행 수가 충분한(3행 이상) 세션이 없습니다.<br>' +
    '경사로 학습을 진행하여 세션 데이터를 수집한 뒤 다시 생성해 주세요.</div>';
  document.querySelector('label[for=""]')?.remove();
  select.style.display = 'none';
} else {
  sessionIds.forEach(sid => {
    const opt = document.createElement('option');
    opt.value = sid;
    opt.textContent = `${sid} (${REPORT.session_series[sid].labels.length}행)`;
    select.appendChild(opt);
  });
  select.addEventListener('change', () => drawSessionChart(select.value));
  drawSessionChart(sessionIds[0]);
}

// ── 차트 2: result 분포 파이차트 ─────────────────────────────────────────
if (REPORT.result_labels.length > 0) {
  new Chart(document.getElementById('resultPieChart'), {
    type: 'pie',
    data: {
      labels: REPORT.result_labels,
      datasets: [{ data: REPORT.result_counts, backgroundColor: REPORT.result_colors }]
    },
    options: { plugins: { legend: { position: 'right' } } }
  });
} else {
  document.getElementById('resultPieChart').replaceWith(
    Object.assign(document.createElement('div'), { className: 'empty', textContent: '표시할 result 데이터가 없습니다.' }));
}

// ── 차트 3: pitch vs speed 산점도 ────────────────────────────────────────
if (REPORT.scatter_datasets.length > 0) {
  new Chart(document.getElementById('scatterChart'), {
    type: 'scatter',
    data: {
      datasets: REPORT.scatter_datasets.map(d => ({
        label: d.label, data: d.data,
        backgroundColor: d.color, pointRadius: 4,
      }))
    },
    options: {
      scales: {
        x: { title: { display: true, text: 'speed_cmd (%)' } },
        y: { title: { display: true, text: 'pitch (°)' } },
      }
    }
  });
} else {
  document.getElementById('scatterChart').replaceWith(
    Object.assign(document.createElement('div'), { className: 'empty', textContent: '오르막(phase=up) 결과 데이터가 없어 산점도를 표시할 수 없습니다.' }));
  document.getElementById('scatter-note').textContent = '오르막 학습 데이터가 충분히 쌓이면 속도-경사각-결과 패턴 분석이 가능합니다.';
}

// ── 차트 4: XGBoost 교차검증 정확도 ──────────────────────────────────────
const xgbArea = document.getElementById('xgb-area');
const xgbNote = document.getElementById('xgb-note');
if (REPORT.xgb_acc) {
  const c = document.createElement('canvas');
  c.id = 'xgbChart'; c.height = 90;
  xgbArea.appendChild(c);
  new Chart(c, {
    type: 'bar',
    data: {
      labels: ['Route 분류 정확도', 'Speed 분류 정확도'],
      datasets: [{
        data: [REPORT.xgb_acc.route_acc, REPORT.xgb_acc.speed_acc],
        backgroundColor: ['#58a6ff', '#3fb950'],
      }]
    },
    options: {
      indexAxis: 'y',
      scales: { x: { min: 0, max: 1 } },
      plugins: { legend: { display: false } },
    }
  });
  xgbNote.textContent =
    `이 차트는 models/xgboost_route.json, xgboost_speed.json 학습에 사용된 ` +
    `real_agv_history.csv 데이터(${REPORT.xgb_acc.n_samples}건)로 ${REPORT.xgb_acc.cv}-fold 교차검증한 ` +
    `평균 정확도입니다. Route 정확도 ${(REPORT.xgb_acc.route_acc*100).toFixed(1)}%, ` +
    `Speed 정확도 ${(REPORT.xgb_acc.speed_acc*100).toFixed(1)}% — ` +
    `값이 낮다면 더 다양한 주행 상황 데이터를 누적해 재학습하는 것을 권장합니다.`;
} else {
  xgbArea.innerHTML = '<div class="empty">모델 없음 — models/xgboost_route.json, xgboost_speed.json 또는 학습 데이터(real_agv_history.csv)를 찾을 수 없습니다.</div>';
  xgbNote.textContent = '';
}

// ── 차트 5: Isolation Forest 이상 점수 히스토그램 ────────────────────────
const ifArea = document.getElementById('if-area');
const ifNote = document.getElementById('if-note');
if (REPORT.if_hist) {
  const c = document.createElement('canvas');
  c.id = 'ifChart'; c.height = 100;
  ifArea.appendChild(c);
  const edges = REPORT.if_hist.normal.edges;
  const labels = edges.slice(0, -1).map((e, i) => `${e.toFixed(2)}~${edges[i+1].toFixed(2)}`);
  new Chart(c, {
    type: 'bar',
    data: {
      labels: labels,
      datasets: [
        { label: `normal (n=${REPORT.if_hist.normal.n})`,    data: REPORT.if_hist.normal.counts,    backgroundColor: '#3fb95099' },
        { label: `hill_fail (n=${REPORT.if_hist.hill_fail.n})`, data: REPORT.if_hist.hill_fail.counts, backgroundColor: '#f8514999' },
        { label: `bump_fail (n=${REPORT.if_hist.bump_fail.n})`, data: REPORT.if_hist.bump_fail.counts, backgroundColor: '#db6d2899' },
      ]
    },
    options: {
      scales: { x: { title: { display: true, text: 'anomaly_score 구간' } } },
      plugins: {
        annotation: {
          annotations: {
            thLine: {
              type: 'line',
              scaleID: 'x',
              value: labels.findIndex(l => parseFloat(l) >= REPORT.anomaly_threshold),
              borderColor: '#f85149', borderWidth: 2, borderDash: [6, 4],
              label: { display: true, content: `임계값 ${REPORT.anomaly_threshold}`, position: 'start', color: '#f85149' }
            }
          }
        }
      }
    }
  });
  ifNote.textContent =
    `이 차트는 Isolation Forest 모델(models/isolation_forest.pkl)을 normal / hill_fail / bump_fail ` +
    `각 데이터에 적용해 계산한 anomaly_score(0~1, 높을수록 이상) 분포입니다. 점선은 VLM 호출 ` +
    `기준 임계값(${REPORT.anomaly_threshold})입니다. normal 분포가 왼쪽(낮은 점수)에, ` +
    `hill_fail/bump_fail 분포가 오른쪽(높은 점수)에 모일수록 모델이 이상 상황을 잘 구분하고 있다는 뜻입니다.`;
} else {
  ifArea.innerHTML = '<div class="empty">모델 없음 또는 normal/hill_fail/bump_fail 데이터가 부족하여 분포를 계산할 수 없습니다.</div>';
}
</script>
</body>
</html>
"""


if __name__ == "__main__":
    data_rows = load_rows()
    if not data_rows:
        print("[ERROR] experiment_v2.csv 에 표시할 데이터가 없습니다.")
        sys.exit(1)
    render_report(data_rows)
