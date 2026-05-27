# tests/test_waypoint_recorder.py
# 실행: python tests/waypoint_recorder.py
# 로봇 없이 PC에서도 실행 가능

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.waypoint_recorder import WaypointRecorder

if __name__ == "__main__":
    recorder = WaypointRecorder()
    recorder.start()

    # 가짜 명령 시퀀스 (실제 대시보드 조종을 흉내냄)
    commands = [
        ('forward', 50),
        ('forward', 50),
        ('left',    40),
        ('forward', 50),
        ('stop',     0),
    ]

    print("가짜 명령 5개 기록 중...")
    for cmd, speed in commands:
        time.sleep(0.5)   # 각 명령 간격 0.5초
        recorder.record(cmd=cmd, speed=speed, heading=0.0)
        print(f"  기록: {cmd} speed={speed}")

    recorder.stop('test_waypoints.json')

    # 결과 확인
    import json
    with open('test_waypoints.json') as f:
        data = json.load(f)
    print(f"\n저장된 waypoint 수: {len(data)}")
    for i, wp in enumerate(data):
        print(f"  [{i+1}] {wp}")