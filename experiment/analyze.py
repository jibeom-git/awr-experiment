#!/usr/bin/env python3
# experiment/analyze.py
# 수집 데이터 분석 및 임계값 자동 추출
# 결과를 experiment/data/thresholds.json 및 analysis_report.html 로 저장

import sys
import csv
import json
import math
from pathlib import Path
from datetime import datetime

INSITE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(INSITE_ROOT))

DATA_DIR     = Path(__file__).resolve().parent / "data"
CSV_PATH     = DATA_DIR / "raw_experiment.csv"
JSON_PATH    = DATA_DIR / "thresholds.json"
REPORT_PATH  = DATA_DIR / "analysis_report.html"

CSV_FIELDS = [
    "timestamp", "label", "speed_cmd", "pitch", "weight",
    "sonic", "accel_x", "accel_y", "accel_z", "pitch_delta", "result",
]


# ─────────────────────────────────────────────────────────────────────────────
# CSV 로드
# ─────────────────────────────────────────────────────────────────────────────

def load_csv(path: Path) -> list:
    if not path.exists():
        print(f"[ERROR] CSV 파일 없음: {path}")
        return []
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f, fieldnames=CSV_FIELDS):
            try:
                rows.append({
                    "timestamp":   row.get("timestamp", ""),
                    "label":       row.get("label", ""),
                    "speed_cmd":   float(row.get("speed_cmd", 0) or 0),
                    "pitch":       float(row.get("pitch", 0) or 0),
                    "weight":      float(row.get("weight", 0) or 0),
                    "sonic":       float(row.get("sonic", 0) or 0),
                    "accel_x":     float(row.get("accel_x", 0) or 0),
                    "accel_y":     float(row.get("accel_y", 0) or 0),
                    "accel_z":     float(row.get("accel_z", 0) or 0),
                    "pitch_delta": float(row.get("pitch_delta", 0) or 0),
                    "result":      row.get("result", "normal").strip(),
                })
            except ValueError:
                continue
    return rows


# ─────────────────────────────────────────────────────────────────────────────
# 기초 통계 (stdlib only, pandas 없음)
# ─────────────────────────────────────────────────────────────────────────────

def _stats(vals: list) -> dict:
    if not vals:
        return {"count": 0, "mean": 0, "std": 0, "min": 0, "max": 0, "p95": 0}
    n    = len(vals)
    mean = sum(vals) / n
    std  = math.sqrt(sum((v - mean) ** 2 for v in vals) / n) if n > 1 else 0
    s    = sorted(vals)
    p95  = s[int(0.95 * n)]
    return {
        "count": n,
        "mean":  round(mean, 4),
        "std":   round(std, 4),
        "min":   round(s[0], 4),
        "max":   round(s[-1], 4),
        "p95":   round(p95, 4),
    }


def _median(vals: list) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def _filter(rows: list, result: str) -> list:
    return [r for r in rows if r["result"] == result]


# ─────────────────────────────────────────────────────────────────────────────
# 1. 등판 실패 패턴 분석 (hill_fail)
# ─────────────────────────────────────────────────────────────────────────────

def analyze_hill_fail(rows: list) -> dict:
    fails = _filter(rows, "hill_fail")
    if not fails:
        return {"note": "hill_fail 데이터 없음", "pitch_delta_threshold": 3.0, "min_hill_speed": 30}

    pd_vals  = [abs(r["pitch_delta"]) for r in fails]
    spd_vals = [r["speed_cmd"] for r in fails]

    pd_stats  = _stats(pd_vals)
    spd_stats = _stats(spd_vals)

    pd_thr  = round(pd_stats["p95"], 3)
    spd_min = round(spd_stats["min"], 1)

    return {
        "pitch_delta_stats":      pd_stats,
        "speed_cmd_stats":        spd_stats,
        "pitch_delta_threshold":  max(pd_thr, 0.5),
        "min_hill_speed":         max(spd_min, 10),
    }


# ─────────────────────────────────────────────────────────────────────────────
# 2. 슬립 감지 패턴 분석 (slip)
# ─────────────────────────────────────────────────────────────────────────────

def analyze_slip(rows: list) -> dict:
    slips = _filter(rows, "slip")
    if not slips:
        return {"note": "slip 데이터 없음", "slip_accel_ratio_threshold": 0.3}

    # 슬립 비율: speed_cmd 대비 accel_x 변화량이 작을수록 슬립
    # ratio = |accel_x| / (speed_cmd / 100 + 1e-6)
    ratios = []
    for r in slips:
        ratio = abs(r["accel_x"]) / (r["speed_cmd"] / 100.0 + 1e-6)
        ratios.append(ratio)

    st  = _stats(ratios)
    thr = round(_median(ratios), 4)

    return {
        "slip_ratio_stats":             st,
        "slip_accel_ratio_threshold":   round(max(thr, 0.05), 4),
    }


# ─────────────────────────────────────────────────────────────────────────────
# 3. 방지턱 충격 패턴 분석 (impact)
# ─────────────────────────────────────────────────────────────────────────────

def analyze_impact(rows: list) -> dict:
    impacts = _filter(rows, "impact")
    if not impacts:
        return {"note": "impact 데이터 없음", "impact_z_threshold": 12.0}

    az_vals = [abs(r["accel_z"]) for r in impacts]
    st      = _stats(az_vals)
    thr     = round(st["p95"], 3)

    return {
        "accel_z_stats":       st,
        "impact_z_threshold":  round(max(thr, 9.81 + 1.0), 3),
    }


# ─────────────────────────────────────────────────────────────────────────────
# 4. 서행 통과 패턴 분석 (cautious_pass)
# ─────────────────────────────────────────────────────────────────────────────

def analyze_cautious_pass(rows: list) -> dict:
    cautious = _filter(rows, "cautious_pass")
    if not cautious:
        return {"note": "cautious_pass 데이터 없음", "cautious_pass_speed_avg": 0.0}

    spd_vals = [r["speed_cmd"] for r in cautious]
    st = _stats(spd_vals)

    return {
        "speed_cmd_stats":        st,
        "cautious_pass_speed_avg": st["mean"],
    }


# ─────────────────────────────────────────────────────────────────────────────
# 전체 분석 실행
# ─────────────────────────────────────────────────────────────────────────────

def run_analysis():
    print("=" * 60)
    print("  INSITE 실험 데이터 분석")
    print(f"  CSV : {CSV_PATH}")
    print("=" * 60)

    rows = load_csv(CSV_PATH)
    if not rows:
        print("[WARN] 분석할 데이터 없음. 먼저 manual_drive.py로 데이터를 수집하세요.")
        return

    total = len(rows)
    result_counts = {}
    for r in rows:
        result_counts[r["result"]] = result_counts.get(r["result"], 0) + 1

    print(f"\n총 {total}행 로드됨")
    for k, v in sorted(result_counts.items()):
        print(f"  {k:20s}: {v}행")

    # ── 분석 실행 ─────────────────────────────────────────────────────────────
    hill     = analyze_hill_fail(rows)
    slip     = analyze_slip(rows)
    impact   = analyze_impact(rows)
    cautious = analyze_cautious_pass(rows)

    # ── 임계값 병합 (ai_core/config.py 변수명과 일치하는 키 사용) ─────────────
    # 키 이름이 config.py 전역 변수명과 동일해야 _load_thresholds() 에서 자동 반영됨
    thresholds = {
        "generated_at":         datetime.now().isoformat(),
        "total_rows":           total,
        "result_counts":        result_counts,
        # 등판 실패 — config.TH_PITCH_DELTA_FAIL
        "TH_PITCH_DELTA_FAIL":  hill.get("pitch_delta_threshold", 3.0),
        "min_hill_speed":       hill.get("min_hill_speed", 30),
        # 슬립 — config.TH_SLIP_ACCEL
        "TH_SLIP_ACCEL":        slip.get("slip_accel_ratio_threshold", 0.3),
        # 방지턱 충격 — config.TH_IMPACT_Z
        "TH_IMPACT_Z":          impact.get("impact_z_threshold", 12.0),
        # 서행 통과
        "cautious_pass_speed_avg": cautious.get("cautious_pass_speed_avg", 0.0),
        # 상세 통계
        "_detail": {
            "hill_fail":     hill,
            "slip":          slip,
            "impact":        impact,
            "cautious_pass": cautious,
        },
    }

    # ── JSON 저장 ─────────────────────────────────────────────────────────────
    DATA_DIR.mkdir(exist_ok=True)
    with open(JSON_PATH, "w") as f:
        json.dump(thresholds, f, ensure_ascii=False, indent=2)
    print(f"\n[SAVED] {JSON_PATH}")

    # ── 콘솔 출력 ─────────────────────────────────────────────────────────────
    print("\n── 등판 실패 임계값 ──────────────────────────────────────")
    print(f"  TH_PITCH_DELTA_FAIL : {thresholds['TH_PITCH_DELTA_FAIL']}")
    print(f"  min_hill_speed      : {thresholds['min_hill_speed']}")
    print("\n── 슬립 감지 임계값 ──────────────────────────────────────")
    print(f"  TH_SLIP_ACCEL : {thresholds['TH_SLIP_ACCEL']}")
    print("\n── 방지턱 충격 임계값 ────────────────────────────────────")
    print(f"  TH_IMPACT_Z : {thresholds['TH_IMPACT_Z']}")
    print("\n── 서행 통과 통계 ────────────────────────────────────────")
    print(f"  cautious_pass_speed_avg : {thresholds['cautious_pass_speed_avg']}")

    # ── HTML 리포트 생성 ──────────────────────────────────────────────────────
    _write_html_report(thresholds, rows)
    print(f"\n[SAVED] {REPORT_PATH}")
    print("\n분석 완료.")


# ─────────────────────────────────────────────────────────────────────────────
# HTML 리포트 생성
# ─────────────────────────────────────────────────────────────────────────────

def _write_html_report(thr: dict, rows: list):
    detail = thr.get("_detail", {})

    def fmt_stats(st: dict) -> str:
        if not st or not st.get("count"):
            return "<em>데이터 없음</em>"
        return (f"count={st['count']}, mean={st['mean']}, std={st['std']}, "
                f"min={st['min']}, max={st['max']}, p95={st['p95']}")

    # pitch_delta 히스토그램 데이터 (버킷 20개)
    pd_all   = [r["pitch_delta"] for r in rows]
    az_all   = [r["accel_z"]     for r in rows]

    def hist_buckets(vals, n=20):
        if not vals:
            return [], []
        lo, hi = min(vals), max(vals)
        if lo == hi:
            return [str(round(lo, 2))], [len(vals)]
        step = (hi - lo) / n
        counts = [0] * n
        for v in vals:
            idx = min(int((v - lo) / step), n - 1)
            counts[idx] += 1
        labels = [str(round(lo + i * step, 2)) for i in range(n)]
        return labels, counts

    pd_labels, pd_counts = hist_buckets(pd_all)
    az_labels, az_counts = hist_buckets(az_all)

    html = f"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>Insite 분석 리포트</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  body {{ font-family: 'Segoe UI', sans-serif; background: #0f1117; color: #e2e8f0; padding: 20px; }}
  h1 {{ color: #7dd3fc; margin-bottom: 16px; }}
  h2 {{ color: #94a3b8; font-size: 0.9rem; text-transform: uppercase; margin: 20px 0 8px; letter-spacing: 1px; }}
  .grid {{ display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; margin-bottom: 20px; }}
  .card {{ background: #1e2433; border-radius: 10px; padding: 14px; }}
  .card .key {{ font-size: 0.72rem; color: #64748b; }}
  .card .val {{ font-size: 1.3rem; font-weight: bold; color: #7dd3fc; }}
  .stat-row {{ font-size: 0.78rem; color: #94a3b8; margin-top: 6px; }}
  .chart-wrap {{ background: #1e2433; border-radius: 10px; padding: 14px; margin-bottom: 12px; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.8rem; }}
  th {{ background: #2d3748; padding: 6px 10px; text-align: left; }}
  td {{ padding: 5px 10px; border-bottom: 1px solid #2d3748; }}
</style>
</head>
<body>
<h1>📈 INSITE 실험 데이터 분석 리포트</h1>
<p style="color:#64748b;font-size:0.8rem;">생성: {thr['generated_at']} &nbsp;|&nbsp; 총 {thr['total_rows']}행</p>

<h2>추천 임계값 요약</h2>
<div class="grid">
  <div class="card">
    <div class="key">TH_PITCH_DELTA_FAIL (등판 실패)</div>
    <div class="val">{thr['TH_PITCH_DELTA_FAIL']}</div>
    <div class="stat-row">{fmt_stats(detail.get('hill_fail', {}).get('pitch_delta_stats'))}</div>
  </div>
  <div class="card">
    <div class="key">min_hill_speed (등판 최저 속도)</div>
    <div class="val">{thr['min_hill_speed']}</div>
  </div>
  <div class="card">
    <div class="key">TH_SLIP_ACCEL (슬립)</div>
    <div class="val">{thr['TH_SLIP_ACCEL']}</div>
    <div class="stat-row">{fmt_stats(detail.get('slip', {}).get('slip_ratio_stats'))}</div>
  </div>
  <div class="card">
    <div class="key">TH_IMPACT_Z (방지턱 충격)</div>
    <div class="val">{thr['TH_IMPACT_Z']}</div>
    <div class="stat-row">{fmt_stats(detail.get('impact', {}).get('accel_z_stats'))}</div>
  </div>
  <div class="card">
    <div class="key">cautious_pass_speed_avg (서행 통과 평균 속도)</div>
    <div class="val">{thr['cautious_pass_speed_avg']}</div>
    <div class="stat-row">{fmt_stats(detail.get('cautious_pass', {}).get('speed_cmd_stats'))}</div>
  </div>
</div>

<h2>레이블 분포</h2>
<table>
  <tr><th>result</th><th>행 수</th></tr>
  {"".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k,v in thr['result_counts'].items())}
</table>

<h2>pitch_delta 분포 히스토그램</h2>
<div class="chart-wrap" style="max-width:700px;">
  <canvas id="ch-pd" height="120"></canvas>
</div>

<h2>accel_z 분포 히스토그램</h2>
<div class="chart-wrap" style="max-width:700px;">
  <canvas id="ch-az" height="120"></canvas>
</div>

<script>
new Chart(document.getElementById("ch-pd"), {{
  type: "bar",
  data: {{
    labels: {json.dumps(pd_labels)},
    datasets: [{{ data: {json.dumps(pd_counts)}, backgroundColor: "#3b82f6", borderWidth: 0 }}],
  }},
  options: {{
    plugins: {{ legend: {{ display: false }} }},
    scales: {{
      x: {{ ticks: {{ color: "#94a3b8", font: {{ size: 9 }} }}, grid: {{ color: "#2d3748" }} }},
      y: {{ ticks: {{ color: "#94a3b8", font: {{ size: 9 }} }}, grid: {{ color: "#2d3748" }} }},
    }},
  }},
}});
new Chart(document.getElementById("ch-az"), {{
  type: "bar",
  data: {{
    labels: {json.dumps(az_labels)},
    datasets: [{{ data: {json.dumps(az_counts)}, backgroundColor: "#a855f7", borderWidth: 0 }}],
  }},
  options: {{
    plugins: {{ legend: {{ display: false }} }},
    scales: {{
      x: {{ ticks: {{ color: "#94a3b8", font: {{ size: 9 }} }}, grid: {{ color: "#2d3748" }} }},
      y: {{ ticks: {{ color: "#94a3b8", font: {{ size: 9 }} }}, grid: {{ color: "#2d3748" }} }},
    }},
  }},
}});
</script>
</body>
</html>"""

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write(html)


if __name__ == "__main__":
    run_analysis()
