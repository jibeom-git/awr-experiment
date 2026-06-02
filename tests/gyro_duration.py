# python tests/gyro_duration.py
# 자이로 기반 좌회전 테스트
# 1. 85도까지 자이로로 회전
# 2. 검은 선 잡힐 때까지 천천히 계속 회전
# 3. 선 잡히면 라인 추종 시작
#
# 실행: python tests/test_turn_left.py

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.tracker import LineTracker
from sensors.motor   import MotorController
from sensors.mpu6050 import MPU6050

# ── 설정값 ──────────────────────────────────────────
BASE_SPEED      = 35
SPIN_SPEED      = 35
SEARCH_SPEED    = 20
TURN_SPEED      = 25
LOST_SPEED      = 20
TURN_TARGET_DEG = 20   # 자이로 목표 각도
GYRO_DT         = 0.01
LOOP_HZ         = 50
# ────────────────────────────────────────────────────

tracker = LineTracker()
motor   = MotorController()
imu     = MPU6050()

print("[Init] 자이로 캘리브레이션 중...")
time.sleep(1)
from tests.calibration_gyro import calibrate_gyro
imu.gyro_offset = calibrate_gyro(imu)
print("[Init] 완료")

def spin_left(speed=SPIN_SPEED):
    motor.set_motor(4, -1, speed)
    motor.set_motor(3, -1, speed)
    motor.set_motor(2,  1, speed)
    motor.set_motor(1,  1, speed)

def spin_right(speed=SPIN_SPEED):
    motor.set_motor(4,  1, speed)
    motor.set_motor(3,  1, speed)
    motor.set_motor(2, -1, speed)
    motor.set_motor(1, -1, speed)

def turn_left_90():
    """
    좌회전 90도:
    1단계 - 자이로로 TURN_TARGET_DEG까지 빠르게
    2단계 - 중앙 센서가 선 잡힐 때까지 천천히
    """
    print(f"\n[Turn] 좌회전 시작 (목표: {TURN_TARGET_DEG}도)")

    # 1단계: 자이로 기반 회전
    angle  = 0.0
    t_prev = time.time()

    while angle < TURN_TARGET_DEG:
        t_now  = time.time()
        dt     = t_now - t_prev
        t_prev = t_now

        if dt > 0.1:   # 너무 긴 dt 무시
            dt = GYRO_DT

        gyro   = imu.get_gyro()
        dangle = abs(gyro['z']) * dt
        angle += dangle

        spin_left(SPIN_SPEED)
        time.sleep(GYRO_DT)

    motor.stop()
    time.sleep(0.1)
    print(f"[Turn] 자이로 완료: {angle:.1f}도")

    # 2단계: 선 탐색 (중앙 센서 기준)
    print("[Turn] 선 탐색 중...")
    timeout = time.time() + 3.0

    while time.time() < timeout:
        state = tracker.read()
        print(f"[Turn] L={state['left']} C={state['center']} R={state['right']}")

        if state['center']:
            motor.stop()
            time.sleep(0.1)
            print("[Turn] ✓ 선 잡힘!")
            return True

        spin_left(SEARCH_SPEED)
        time.sleep(0.02)

    motor.stop()
    print("[Turn] ✗ 선 못 찾음 (3초 타임아웃)")
    return False

def follow_line():
    """선 잡힌 후 라인 추종"""
    interval        = 1.0 / LOOP_HZ
    lost_last_error = 1
    lost_count      = 0
    searching       = False

    print("\n[Follow] 라인 추종 시작 — Ctrl+C로 종료")

    while True:
        t0 = time.time()

        state   = tracker.read()
        l, c, r = state['left'], state['center'], state['right']

        if (l or c or r) and searching:
            motor.stop()
            time.sleep(0.05)
            searching  = False
            lost_count = 0

        if not l and c and not r:
            motor.forward(BASE_SPEED)
        elif l and c and not r:
            lost_last_error = -1
            motor.set_motor(4, 1, TURN_SPEED)
            motor.set_motor(3, 1, TURN_SPEED)
            motor.set_motor(2, 1, BASE_SPEED)
            motor.set_motor(1, 1, BASE_SPEED)
        elif not l and c and r:
            lost_last_error = 1
            motor.set_motor(4, 1, BASE_SPEED)
            motor.set_motor(3, 1, BASE_SPEED)
            motor.set_motor(2, 1, TURN_SPEED)
            motor.set_motor(1, 1, TURN_SPEED)
        elif l and not c and not r:
            lost_last_error = -1
            spin_left()
        elif not l and not c and r:
            lost_last_error = 1
            spin_right()
        elif l and not c and r:
            motor.forward(BASE_SPEED)
        elif l and c and r:
            motor.forward(BASE_SPEED)
        else:
            searching   = True
            lost_count += 1
            if lost_last_error == 1:
                spin_right(LOST_SPEED)
            else:
                spin_left(LOST_SPEED)
            if lost_count > LOOP_HZ * 2:
                lost_last_error *= -1
                lost_count       = 0

        if int(time.time() * 10) % 10 == 0:
            print(f"[{state['pattern']}]", end='\r')

        elapsed = time.time() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    print("=================================")
    print("  자이로 좌회전 + 라인 추종 테스트")
    print("=================================")
    print()
    print("로봇을 직선 구간 위에 올려놓으세요.")
    print("3초 후 좌회전 시작...")
    time.sleep(3)

    try:
        success = turn_left_90()
        if success:
            print("\n회전 성공! 라인 추종 시작")
            follow_line()
        else:
            print("\n회전 실패. TURN_TARGET_DEG 또는 SPIN_SPEED 조정 필요")
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        motor.stop()
        motor.close()
        tracker.close()
        imu.close()