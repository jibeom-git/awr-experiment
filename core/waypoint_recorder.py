# core/waypoint_recorder.py
import time, json, os

# 절대경로 고정
WAYPOINT_PATH = os.path.join(os.path.dirname(__file__), '..', '/home/pi/insite/waypoints.json')
WAYPOINT_PATH = os.path.abspath(WAYPOINT_PATH)  # /home/pi/insite/waypoints.json

class WaypointRecorder:
    def __init__(self, path=WAYPOINT_PATH):
        self.path      = path
        self.recording = False
        self.waypoints = []
        self._last_time = None

    def start(self):
        self.waypoints  = []
        self.recording  = True
        self._last_time = time.time()
        self._stopped   = True   # ← 추가
        print("[Recorder] 기록 시작")

    def record(self, cmd, speed, heading):
        if not self.recording:
            return

        now = time.time()
        duration = now - self._last_time
        self._last_time = now

        if cmd == 'stop':
            # 직전 명령 duration 확정
            if self.waypoints:
                self.waypoints[-1]['duration'] += round(duration, 3)
            self._last_time = now  # stop 시점으로 리셋
            self._stopped = True   # 정지 상태 표시
            return

        if self.waypoints and self.waypoints[-1]['cmd'] == cmd:
            self.waypoints[-1]['duration'] += round(duration, 3)
            self._stopped = False
            return

        # 새 명령 — 정지 상태에서 왔으면 duration 0으로 시작
        self.waypoints.append({
            "cmd":      cmd,
            "speed":    speed,
            "duration": 0.0 if getattr(self, '_stopped', True) else round(duration, 3),
            "heading":  round(heading, 2),
        })
        self._stopped = False

    def stop(self, path=None):
        self.recording = False
        save_path = path or self.path
        with open(save_path, "w") as f:
            json.dump(self.waypoints, f, indent=2, ensure_ascii=False)
        print(f"[Recorder] {len(self.waypoints)}개 waypoint 저장 → {save_path}")