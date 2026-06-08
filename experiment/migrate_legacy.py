#!/usr/bin/env python3
# experiment/migrate_legacy.py
# 기존 raw_experiment.csv(레거시 포맷) 데이터를 experiment_v2.csv 형식으로 변환하는 독립 스크립트
# 실행: python experiment/migrate_legacy.py

import csv
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent / "data"
RAW_CSV  = DATA_DIR / "raw_experiment.csv"
V2_CSV   = DATA_DIR / "experiment_v2.csv"

# raw_experiment.csv 는 헤더 없이 저장되며 열 순서는 다음과 같다 (experiment/analyze.py CSV_FIELDS 참고)
RAW_FIELDS = [
    "timestamp", "label", "speed_cmd", "pitch", "weight",
    "sonic", "accel_x", "accel_y", "accel_z", "pitch_delta", "result",
]

V2_FIELDS = [
    "timestamp", "session_id", "route", "phase", "speed_cmd",
    "weight_g", "obstacle",
    "pitch", "sonic_cm", "accel_x", "accel_y", "accel_z",
    "pitch_delta", "accel_z_delta", "result",
]

# 레거시 한국어 라벨 → (phase, route, obstacle) 매핑
LABEL_MAP = {
    "정상 평지":   ("flat", "flat", "none"),
    "20도 언덕":   ("up",   "A",    "none"),
    "10도 언덕":   ("up",   "B",    "none"),
    "20도 내리막": ("down", "A",    "none"),
    "10도 내리막": ("down", "B",    "none"),
    "1cm 방지턱":  ("flat", "flat", "bump_1cm"),
    "2cm 방지턱":  ("flat", "flat", "bump_2cm"),
}
DEFAULT_MAPPING = ("flat", "flat", "none")  # 매핑 표에 없는 라벨은 기본값 사용


def _already_migrated() -> bool:
    """experiment_v2.csv 에 legacy_ 세션이 이미 있으면 True (중복 마이그레이션 방지)"""
    if not V2_CSV.exists() or V2_CSV.stat().st_size == 0:
        return False
    with open(V2_CSV, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if (row.get("session_id") or "").startswith("legacy_"):
                return True
    return False


def migrate():
    if not RAW_CSV.exists():
        print(f"[ERROR] 원본 파일 없음: {RAW_CSV}")
        return

    if _already_migrated():
        print("[SKIP] 이미 마이그레이션된 legacy_ 세션이 존재합니다 — 건너뜀")
        return

    # 파일이 없거나 비어 있으면 헤더부터 작성
    write_header = (not V2_CSV.exists()) or V2_CSV.stat().st_size == 0

    converted = 0
    with open(RAW_CSV, newline="", encoding="utf-8") as fin, \
         open(V2_CSV, "a", newline="", encoding="utf-8") as fout:

        writer = csv.DictWriter(fout, fieldnames=V2_FIELDS)
        if write_header:
            writer.writeheader()

        for idx, row in enumerate(csv.DictReader(fin, fieldnames=RAW_FIELDS)):
            label = (row.get("label") or "").strip()
            phase, route, obstacle = LABEL_MAP.get(label, DEFAULT_MAPPING)

            writer.writerow({
                "timestamp":     row.get("timestamp", ""),
                "session_id":    f"legacy_{idx:04d}",
                "route":         route,
                "phase":         phase,
                "speed_cmd":     row.get("speed_cmd", ""),
                "weight_g":      row.get("weight", ""),
                "obstacle":      obstacle,
                "pitch":         row.get("pitch", ""),
                "sonic_cm":      row.get("sonic", ""),
                "accel_x":       row.get("accel_x", ""),
                "accel_y":       row.get("accel_y", ""),
                "accel_z":       row.get("accel_z", ""),
                "pitch_delta":   row.get("pitch_delta", ""),
                "accel_z_delta": 0.0,  # 레거시 데이터에는 없는 열 — 0.0으로 채움
                "result":        row.get("result", ""),
            })
            converted += 1

    print(f"{converted}행 변환 완료 → {V2_CSV.name}")


if __name__ == "__main__":
    migrate()
