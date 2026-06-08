# ai_core/logger.py
# 블랙박스 CSV 로그 + 장애물 이미지 DB 누적 관리 모듈

import os
import csv
import json
import time
import threading
from pathlib import Path

from . import config


# CSV 헤더 정의 (신규 형식 — Impact_Z, Gemini_Confidence 추가)
_CSV_HEADER = [
    "Timestamp", "Current_Node", "Assigned_Route", "Obstacle_Type",
    "Pass_Status", "Applied_Speed", "System_Message", "VLM_Reason",
    "Impact_Z", "Gemini_Confidence"
]

# Pass_Status 유효값 집합
VALID_STATUS = {
    "PASSED", "CAUTIOUS_SLOWDOWN", "PATH_BLOCKED",
    "FATAL_DEADLOCK", "HILL_FAIL", "SLIP_DETECTED", "IMPACT_EXCEEDED"
}


class BlackboxLogger:
    """
    모든 FSM 이벤트를 CSV에 누적 기록하고
    장애물 이미지를 data/obstacles/ 에 저장하는 블랙박스 모듈.
    """

    def __init__(self):
        self._lock = threading.Lock()
        os.makedirs(config.LOG_ROOT, exist_ok=True)
        os.makedirs(config.OBSTACLES_DIR, exist_ok=True)

        self.log_path    = os.path.join(config.LOG_ROOT, config.DRIVING_LOG_NAME)
        self.obs_db_path = config.OBSTACLE_DB_FILE

        self._init_csv()
        self._obstacle_db = self._load_obstacle_db()

    # ─────────────────────────────────────────────────────────────────────────
    # CSV 초기화: 헤더 불일치 시 구버전 백업 후 신규 생성
    # ─────────────────────────────────────────────────────────────────────────

    def _init_csv(self):
        if not os.path.exists(self.log_path):
            self._write_csv_header()
            return
        try:
            with open(self.log_path, 'r', encoding='utf-8') as f:
                existing_header = next(csv.reader(f), [])
            if existing_header == _CSV_HEADER:
                return  # 헤더 일치, 그대로 사용
            # 헤더 불일치: 구버전 백업 후 신규 생성
            backup = self.log_path.replace('.csv', '_backup.csv')
            os.rename(self.log_path, backup)
            print(f"[Logger] 구버전 CSV 백업: {backup}")
        except Exception:
            pass
        self._write_csv_header()

    def _write_csv_header(self):
        with open(self.log_path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(_CSV_HEADER)

    # ─────────────────────────────────────────────────────────────────────────
    # 이벤트 로그 기록
    # ─────────────────────────────────────────────────────────────────────────

    def log_event(
        self,
        node: int,
        route: str,
        obs_type: str,
        status: str,
        speed: int,
        msg: str,
        vlm_reason: str = "N/A",
        impact_z: float = 0.0,
        gemini_confidence: float = 0.0,
    ):
        """FSM 상태 천이 이벤트를 블랙박스 CSV에 실시간 누적 기록"""
        row = [
            time.strftime('%Y-%m-%d %H:%M:%S'),
            node, route, obs_type, status, speed,
            msg, vlm_reason,
            round(impact_z, 3),
            round(gemini_confidence, 3),
        ]
        with self._lock:
            try:
                with open(self.log_path, 'a', newline='', encoding='utf-8') as f:
                    csv.writer(f).writerow(row)
            except Exception as e:
                print(f"[Logger] CSV 기록 오류: {e}")
        print(f"[LOG] Node:{node} Route:{route} Obs:{obs_type} "
              f"Status:{status} Speed:{speed}% ImpactZ:{impact_z:.2f}")

    # ─────────────────────────────────────────────────────────────────────────
    # 장애물 이미지 저장 및 DB 관리
    # ─────────────────────────────────────────────────────────────────────────

    def save_obstacle(
        self,
        frame,                      # numpy 이미지 프레임 (None 가능)
        gemini_judgment: str,       # "passable" | "blocked"
        gemini_confidence: float,
        actual_result: str,         # "passed" | "hill_fail" | "slip" | "impact_exceeded"
        impact_z: float,
        speed_cmd: int,
        pitch: float,
        weight: float,
        obstacle_type: str,
    ) -> str:
        """장애물 이미지를 obs_NNN.jpg로 저장하고 obstacle_db.json에 append"""
        import cv2
        with self._lock:
            idx = len(self._obstacle_db) + 1
            img_filename = f"obs_{idx:03d}.jpg"
            img_path = os.path.join(config.OBSTACLES_DIR, img_filename)

            # 이미지 저장 (frame이 없으면 빈 파일로 생략)
            if frame is not None:
                try:
                    cv2.imwrite(img_path, frame)
                except Exception as e:
                    print(f"[Logger] 이미지 저장 실패: {e}")
                    img_path = ""
            else:
                img_path = ""

            entry = {
                "timestamp":          time.strftime('%Y-%m-%d %H:%M:%S'),
                "image_path":         img_path,
                "gemini_judgment":    gemini_judgment,
                "gemini_confidence":  round(gemini_confidence, 3),
                "actual_result":      actual_result,
                "impact_z":           round(impact_z, 3),
                "speed_cmd":          speed_cmd,
                "pitch":              round(pitch, 2),
                "weight":             round(weight, 1),
                "obstacle_type":      obstacle_type,
            }
            self._obstacle_db.append(entry)
            self._flush_obstacle_db()
            return img_path

    def update_last_obstacle_result(self, actual_result: str, impact_z: float):
        """장애물 통과 후 실제 결과를 마지막 DB 엔트리에 업데이트"""
        with self._lock:
            if not self._obstacle_db:
                return
            self._obstacle_db[-1]["actual_result"] = actual_result
            self._obstacle_db[-1]["impact_z"]      = round(impact_z, 3)
            self._flush_obstacle_db()

    def _flush_obstacle_db(self):
        """obstacle_db를 JSON 파일에 즉시 저장 (락 내부에서 호출)"""
        try:
            with open(self.obs_db_path, 'w', encoding='utf-8') as f:
                json.dump(self._obstacle_db, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"[Logger] obstacle_db.json 저장 오류: {e}")

    def _load_obstacle_db(self) -> list:
        """obstacle_db.json 로드 (없으면 빈 리스트)"""
        if not os.path.exists(self.obs_db_path):
            return []
        try:
            with open(self.obs_db_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return []

    def get_obstacle_db(self) -> list:
        """DB 전체 반환 (대시보드 API용)"""
        with self._lock:
            return list(self._obstacle_db)

    def get_csv_row_count(self) -> int:
        """CSV에 실제 기록된 데이터 행 수 반환 (헤더 제외)"""
        try:
            with open(self.log_path, 'r', encoding='utf-8') as f:
                return max(0, sum(1 for _ in f) - 1)
        except Exception:
            return 0
