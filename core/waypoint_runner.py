# core/waypoint_runner.py
import json, time

class WaypointRunner:
    def __init__(self, motor, ultra, heading_tracker):
        self.motor   = motor
        self.ultra   = ultra
        self.tracker = heading_tracker

    def run(self, path="waypoints.json", obstacle_cm=30.0):
        with open(path) as f:
            waypoints = json.load(f)

        print(f"[Runner] {len(waypoints)}개 waypoint 재생 시작")

        for i, wp in enumerate(waypoints):
            print(f"  [{i+1}/{len(waypoints)}] {wp['cmd']} "
                  f"speed={wp['speed']} dur={wp['duration']:.2f}s")

            # ── 장애물 감지 ──────────────────────────────
            if self.ultra:
                dist = round(self.ultra.distance * 100, 1)
                if dist < obstacle_cm:
                    print(f"  ⚠ 장애물 {dist}cm — 대기 중...")
                    self.motor.stop()
                    while dist < obstacle_cm:
                        time.sleep(0.1)
                        dist = round(self.ultra.distance * 100, 1)
                    print("  ✓ 장애물 해제, 재개")

            # ── 명령 실행 ────────────────────────────────
            cmd   = wp['cmd']
            speed = wp['speed']

            if cmd == 'forward':
                self.motor.forward(speed)
            elif cmd == 'backward':
                self.motor.backward(speed)
            elif cmd == 'left':
                self.motor.set_motor(1,  1, speed)
                self.motor.set_motor(2,  1, speed)
                self.motor.set_motor(3, -1, speed)
                self.motor.set_motor(4, -1, speed)
            elif cmd == 'right':
                self.motor.set_motor(1, -1, speed)
                self.motor.set_motor(2, -1, speed)
                self.motor.set_motor(3,  1, speed)
                self.motor.set_motor(4,  1, speed)
            elif cmd == 'stop':
                self.motor.stop()

            time.sleep(wp['duration'])

        self.motor.stop()
        print("[Runner] 경로 완료")