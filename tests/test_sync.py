# tests/test_sync.py
# 멀티센서 동기화 테스트
# 실행: python tests/test_sync.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.sync import SensorSynchronizer
import time

if __name__ == "__main__":
    sync = SensorSynchronizer()
    sync.start(fps=10)

    print("10초간 동기화 데이터 출력 (Ctrl+C로 종료)")
    try:
        while True:
            frame = sync.output_queue.get(timeout=2.0)
            print(f"[{frame.timestamp:.3f}]"
                  f" RGB={frame.rgb is not None}"
                  f" Depth={frame.depth is not None}"
                  f" Accel={frame.accel}"
                  f" Weight={frame.weight:.0f}" if frame.weight else ""
                  f" Dist={frame.distance:.1f}cm" if frame.distance else "")
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        sync.stop()