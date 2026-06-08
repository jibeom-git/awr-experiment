#!/usr/bin/env python3
# experiment/label_tool.py
# 수집 후 CSV 구간별 레이블 후처리 웹 UI (포트 5002)
# Chart.js 기반 드래그 구간 선택 → result 컬럼 일괄 변경

import os
import sys
import csv
import json
from pathlib import Path

INSITE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(INSITE_ROOT))

from flask import Flask, render_template, request, jsonify
from flask_cors import CORS

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
app = Flask(__name__, template_folder=str(TEMPLATE_DIR))
CORS(app)

DATA_DIR = Path(__file__).resolve().parent / "data"
CSV_PATH = DATA_DIR / "raw_experiment.csv"

VALID_RESULTS = {"normal", "hill_fail", "slip", "impact", "cautious_pass"}


def _load_csv():
    """CSV를 읽어 리스트 of dict 반환. 파일 없으면 빈 리스트."""
    if not CSV_PATH.exists():
        return []
    rows = []
    try:
        with open(CSV_PATH, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(dict(row))
    except Exception as e:
        print(f"[CSV LOAD] {e}")
    return rows


def _save_csv(rows: list):
    """리스트 of dict를 CSV에 덮어씀"""
    if not rows:
        return
    try:
        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    except Exception as e:
        print(f"[CSV SAVE] {e}")


@app.route("/")
def index():
    return render_template("label.html")


@app.route("/api/data")
def api_data():
    """전체 CSV 데이터를 JSON으로 반환"""
    rows = _load_csv()
    return jsonify({"rows": rows, "count": len(rows)})


@app.route("/api/label_range", methods=["POST"])
def api_label_range():
    """
    선택 구간의 result 일괄 변경.
    Body: { "start_idx": int, "end_idx": int, "result": str }
    """
    body      = request.get_json(force=True)
    start_idx = int(body.get("start_idx", 0))
    end_idx   = int(body.get("end_idx", 0))
    result    = str(body.get("result", "normal"))

    if result not in VALID_RESULTS:
        return jsonify({"error": f"허용되지 않는 result 값: {result}"}), 400

    rows = _load_csv()
    if not rows:
        return jsonify({"error": "CSV 파일 없음"}), 404

    changed = 0
    for i in range(start_idx, min(end_idx + 1, len(rows))):
        rows[i]["result"] = result
        changed += 1

    _save_csv(rows)
    return jsonify({"ok": True, "changed": changed})


@app.route("/api/save", methods=["POST"])
def api_save():
    """프론트에서 전체 수정된 rows를 받아 저장"""
    body = request.get_json(force=True)
    rows = body.get("rows", [])
    if not rows:
        return jsonify({"error": "빈 데이터"}), 400
    _save_csv(rows)
    return jsonify({"ok": True, "saved": len(rows)})


if __name__ == "__main__":
    print("=" * 60)
    print("  INSITE CSV 레이블링 도구")
    print("  URL : http://192.168.0.50:5002")
    print(f"  CSV : {CSV_PATH}")
    print("=" * 60)
    app.run(host="0.0.0.0", port=5002, debug=False)
