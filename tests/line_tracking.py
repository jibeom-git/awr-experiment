# python tests/line_tracking.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.tracker import LineTracker
from sensors.motor   import MotorController
import time

BASE_SPEED   = 35
TURN_SPEED   = 25
SPIN_SPEED   = 35
SEARCH_SPEED = 20
LOOP_HZ      = 50

tracker = LineTracker()
motor   = MotorController()

# 채널 매핑
# 채널 1 → 오른뒤
# 채널 2 → 오른앞
# 채널 3 → 왼뒤
# 채널 4 → 왼앞

def spin_right(speed=SPIN_SPEED):
    """왼쪽 바퀴 후진, 오른쪽 바퀴 전진 → 좌회전"""
    motor.set_motor(4, -1, speed)   # 왼앞 후진
    motor.set_motor(3, -1, speed)   # 왼뒤 후진
    motor.set_motor(2,  1, speed)   # 오른앞 전진
    motor.set_motor(1,  1, speed)   # 오른뒤 전진

def spin_left(speed=SPIN_SPEED):
    """오른쪽 바퀴 후진, 왼쪽 바퀴 전진 → 우회전"""
    motor.set_motor(4,  1, speed)   # 왼앞 전진
    motor.set_motor(3,  1, speed)   # 왼뒤 전진
    motor.set_motor(2, -1, speed)   # 오른앞 후진
    motor.set_motor(1, -1, speed)   # 오른뒤 후진

def follow_line():
    interval        = 1.0 / LOOP_HZ
    lost_last_error = 1
    lost_count      = 0
    searching       = False

    print("[Follow] 시작 — Ctrl+C로 종료")

    while True:
        t0 = time.time()

        state   = tracker.read()
        l, c, r = state['left'], state['center'], state['right']
        pattern = state['pattern']

        if l or c or r:
            if searching:
                motor.stop()
                time.sleep(0.05)
                searching  = False
                lost_count = 0

        # ── 교차점 ────────────────────────────────────
        if l and c and r:
            motor.forward(BASE_SPEED)

        # ── 정중앙 ────────────────────────────────────
        elif not l and c and not r:
            motor.forward(BASE_SPEED)

        # ── 약간 왼쪽 → 오른쪽 바퀴 빠르게 ──────────
        elif l and c and not r:
            lost_last_error = -1
            motor.set_motor(4, 1, TURN_SPEED)   # 왼앞 느리게
            motor.set_motor(3, 1, TURN_SPEED)   # 왼뒤 느리게
            motor.set_motor(2, 1, BASE_SPEED)   # 오른앞 빠르게
            motor.set_motor(1, 1, BASE_SPEED)   # 오른뒤 빠르게

        # ── 약간 오른쪽 → 왼쪽 바퀴 빠르게 ──────────
        elif not l and c and r:
            lost_last_error = 1
            motor.set_motor(4, 1, BASE_SPEED)   # 왼앞 빠르게
            motor.set_motor(3, 1, BASE_SPEED)   # 왼뒤 빠르게
            motor.set_motor(2, 1, TURN_SPEED)   # 오른앞 느리게
            motor.set_motor(1, 1, TURN_SPEED)   # 오른뒤 느리게

        # ── 많이 왼쪽 → 제자리 좌회전 ────────────────
        elif l and not c and not r:
            lost_last_error = -1
            spin_left()

        # ── 많이 오른쪽 → 제자리 우회전 ──────────────
        elif not l and not c and r:
            lost_last_error = 1
            spin_right()

        # ── 양끝만 ────────────────────────────────────
        elif l and not c and r:
            motor.forward(BASE_SPEED)

        # ── 선 이탈 ───────────────────────────────────
        else:
            searching   = True
            lost_count += 1

            if lost_last_error == 1:
                spin_right(SEARCH_SPEED)
            else:
                spin_left(SEARCH_SPEED)

            if lost_count > LOOP_HZ * 2:
                lost_last_error *= -1
                lost_count       = 0
                print("[Follow] 탐색 방향 전환")

        if int(time.time() * LOOP_HZ) % 10 == 0:
            print(f"[{pattern}] searching={searching} dir={lost_last_error}")

        elapsed = time.time() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    print("라인 트래킹 시작")
    print("로봇을 검은 선 위에 올려놓으세요")
    print("3초 후 시작...")
    time.sleep(3)

    try:
        follow_line()
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        motor.stop()
        motor.close()
        tracker.close()