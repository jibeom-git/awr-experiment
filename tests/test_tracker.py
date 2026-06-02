# tests/test_tracker.py
import sys, time
sys.path.insert(0, '/home/pi/insite')
from sensors.tracker import LineTracker

tracker = LineTracker()

print("=" * 40)
print("  라인트래킹 센서 테스트")
print("  Ctrl+C 로 종료")
print("=" * 40)

try:
    while True:
        data = tracker.read()
        error = tracker.get_error()
        junction = tracker.is_junction()

        print(
            f"L={data['left']}  C={data['center']}  R={data['right']}  "
            f"패턴={data['pattern']}  오차={error}  "
            f"교차점={'YES' if junction else ' no'}  "
            f"선위={'YES' if data['on_line'] else ' NO'}"
        )
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n종료")
finally:
    tracker.close()