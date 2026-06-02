# python tests/route_follow.py
# 라인트래킹 + 자이로 기반 경로 선택 주행
#
# 트랙 구조:
#   START → 1 → (루트 선택) → 6 GOAL
#
# 루트 A: 모든 분기점 직진 (1→6)
# 루트 B: 1(좌) → 2(우) → 5(우)
# 루트 C: 1(좌) → 2(직) → 3(우) → 4(우) → 5(직)
#
# 실행: python tests/test_route_follow.py

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.tracker import LineTracker
from sensors.motor   import MotorController
from sensors.mpu6050 import MPU6050
from sensors.ultra   import get_distance

# ══════════════════════════════════════════════════════
# 경로 정의
# ══════════════════════════════════════════════════════
ROUTES = {
    "A": {},
    "B": {1: "left",  2: "right", 5: "right"},
    "C": {1: "left",  2: "straight", 3: "right", 4: "right", 5: "straight"},
}

def get_next_route(current):
    order = ["A", "B", "C"]
    idx   = order.index(current)
    return order[idx + 1] if idx < len(order) - 1 else current

# ══════════════════════════════════════════════════════
# 설정값
# ══════════════════════════════════════════════════════
BASE_SPEED      = 35
TURN_SPEED      = 25
SPIN_SPEED      = 35
SEARCH_SPEED    = 20
OBSTACLE_CM     = 1
LOOP_HZ         = 50
LCR_CONFIRM_SEC = 0.15   # LCR 유지 시간 → 분기점 확정
TURN_TARGET_DEG = 40     # 자이로 회전 목표 각도
GYRO_DT         = 0.01

# ══════════════════════════════════════════════════════
# 센서 / 모터 초기화
# ══════════════════════════════════════════════════════
tracker = LineTracker()
motor   = MotorController()
imu     = MPU6050()

print("[Init] 자이로 캘리브레이션 중... (로봇 고정)")
time.sleep(1)
from tests.calibration_gyro import calibrate_gyro
imu.gyro_offset = calibrate_gyro(imu)
print("[Init] 캘리브레이션 완료")

# ══════════════════════════════════════════════════════
# 모터 기본 동작
# ══════════════════════════════════════════════════════
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

# ══════════════════════════════════════════════════════
# 자이로 기반 각도 회전
# ══════════════════════════════════════════════════════
def turn_by_angle(direction, target_deg=TURN_TARGET_DEG):
    print(f"[Turn] {direction} {target_deg}도 회전")

    # 1단계: 교차점 벗어나기
    motor.forward(BASE_SPEED)
    time.sleep(0.25)
    motor.stop()
    time.sleep(0.1)

    # 2단계: 자이로로 목표 각도까지
    angle  = 0.0
    t_prev = time.time()

    while angle < target_deg:
        t_now  = time.time()
        dt     = t_now - t_prev
        t_prev = t_now

        gyro   = imu.get_gyro()
        angle += abs(gyro['z']) * dt

        if direction == "left":
            spin_left(SPIN_SPEED)
        else:
            spin_right(SPIN_SPEED)

        time.sleep(GYRO_DT)

    motor.stop()
    time.sleep(0.1)
    print(f"[Turn] {angle:.1f}도 완료 → 라인 탐색")

    # 3단계: 중앙 센서가 선 잡힐 때까지 천천히
    timeout = time.time() + 2.0
    while time.time() < timeout:
        state = tracker.read()
        if state['center']:
            motor.stop()
            time.sleep(0.1)
            print("[Turn] 라인 정렬 완료")
            return True
        if direction == "left":
            spin_left(SEARCH_SPEED)
        else:
            spin_right(SEARCH_SPEED)
        time.sleep(0.02)

    motor.stop()
    print("[Turn] 경고: 라인 정렬 실패")
    return False

def turn_180():
    print("[Turn] 180도 회전")
    motor.stop()
    time.sleep(0.3)

    angle  = 0.0
    t_prev = time.time()

    while angle < 170:
        t_now  = time.time()
        dt     = t_now - t_prev
        t_prev = t_now
        gyro   = imu.get_gyro()
        angle += abs(gyro['z']) * dt
        spin_right(SPIN_SPEED)
        time.sleep(GYRO_DT)

    motor.stop()
    time.sleep(0.2)

    timeout = time.time() + 3.0
    while time.time() < timeout:
        state = tracker.read()
        if state['center']:
            motor.stop()
            time.sleep(0.1)
            print("[Turn] 180도 완료")
            return
        spin_right(SEARCH_SPEED)
        time.sleep(0.02)

    motor.stop()

# ══════════════════════════════════════════════════════
# 분기점 감지 (l, c, r 인자로 받음)
# ══════════════════════════════════════════════════════
def check_junction(l, c, r, lcr_start):
    """
    LCR 패턴이 LCR_CONFIRM_SEC 이상 유지되면 분기점 확정.
    센서값을 인자로 받아서 중복 읽기 방지.
    """
    if l and c and r:
        if lcr_start is None:
            lcr_start = time.time()
        elif time.time() - lcr_start >= LCR_CONFIRM_SEC:
            return True, lcr_start
    else:
        lcr_start = None
    return False, lcr_start

# ══════════════════════════════════════════════════════
# 라인 추종 (l, c, r 인자로 받음)
# ══════════════════════════════════════════════════════
def line_follow_step(l, c, r, lost_last_error, lost_count, searching):
    """
    센서값을 인자로 받아서 중복 읽기 방지.
    """
    # 선 감지 → 탐색 중이었으면 즉시 멈추고 재개
    if (l or c or r) and searching:
        motor.stop()
        time.sleep(0.05)
        searching  = False
        lost_count = 0

    # 교차점은 check_junction에서 처리하므로 여기선 직진 유지
    if l and c and r:
        motor.forward(BASE_SPEED)

    # 정중앙
    elif not l and c and not r:
        motor.forward(BASE_SPEED)

    # 약간 왼쪽
    elif l and c and not r:
        lost_last_error = -1
        motor.set_motor(4, 1, TURN_SPEED)
        motor.set_motor(3, 1, TURN_SPEED)
        motor.set_motor(2, 1, BASE_SPEED)
        motor.set_motor(1, 1, BASE_SPEED)

    # 약간 오른쪽
    elif not l and c and r:
        lost_last_error = 1
        motor.set_motor(4, 1, BASE_SPEED)
        motor.set_motor(3, 1, BASE_SPEED)
        motor.set_motor(2, 1, TURN_SPEED)
        motor.set_motor(1, 1, TURN_SPEED)

    # 많이 왼쪽
    elif l and not c and not r:
        lost_last_error = -1
        spin_left()

    # 많이 오른쪽
    elif not l and not c and r:
        lost_last_error = 1
        spin_right()

    # 양끝만
    elif l and not c and r:
        motor.forward(BASE_SPEED)

    # 선 이탈
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

    return lost_last_error, lost_count, searching

# ══════════════════════════════════════════════════════
# 메인 주행 루프
# ══════════════════════════════════════════════════════
def run(route="A"):
    junction_count  = 0
    lost_last_error = 1
    lost_count      = 0
    searching       = False
    lcr_start       = None
    current_route   = route
    interval        = 1.0 / LOOP_HZ

    print(f"\n[Main] 루트 {current_route} 주행 시작")
    print(f"[Main] 행동 계획: {ROUTES[current_route]}")

    while True:
        t0 = time.time()

        # ── 센서 한 번만 읽기 ─────────────────────────
        state   = tracker.read()
        l, c, r = state['left'], state['center'], state['right']

        # ── 장애물 감지 ───────────────────────────────
        dist = get_distance()
        if dist < OBSTACLE_CM:
            print(f"[Main] 장애물: {dist:.1f}cm")
            motor.stop()

            # TODO: VLM + XGBoost 판단 연결
            passable = False

            if not passable:
                new_route = get_next_route(current_route)
                if new_route == current_route:
                    print("[Main] 우회 루트 없음 — 정지")
                    break
                current_route   = new_route
                junction_count  = 0
                lcr_start       = None
                searching       = False
                lost_last_error = 1
                print(f"[Main] 우회: 루트 {current_route}")
                print(f"[Main] 행동 계획: {ROUTES[current_route]}")
                turn_180()
                continue

        # ── 분기점 감지 ───────────────────────────────
        is_junction, lcr_start = check_junction(l, c, r, lcr_start)

        if is_junction:
            junction_count += 1
            lcr_start = None
            print(f"\n[Main] 분기점 #{junction_count} | 루트 {current_route}")

            # GOAL 도착
            if junction_count == 6:
                motor.stop()
                print("[Main] ★ GOAL 도착! ★")
                break

            action = ROUTES[current_route].get(junction_count, "straight")
            print(f"[Main] 행동: {action}")

            if action == "straight":
                motor.forward(BASE_SPEED)
                time.sleep(0.3)
            elif action == "left":
                turn_by_angle("left")
            elif action == "right":
                turn_by_angle("right")

        else:
            # ── 라인 추종 ─────────────────────────────
            lost_last_error, lost_count, searching = line_follow_step(
                l, c, r, lost_last_error, lost_count, searching
            )

        # 루프 주기 유지
        elapsed = time.time() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

# ══════════════════════════════════════════════════════
# 실행
# ══════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 45)
    print("  경로 선택 주행")
    print("=" * 45)
    print()
    print("  A: 직진만 (1→6)")
    print("  B: 1좌→2우→5우")
    print("  C: 1좌→2직→3우→4우→5직")
    print()
    choice = input("루트 (A/B/C, 기본=A): ").strip().upper()
    if choice not in ("A", "B", "C"):
        choice = "A"

    print(f"\n루트 {choice} — 3초 후 시작")
    time.sleep(3)

    try:
        run(route=choice)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        motor.stop()
        motor.close()
        tracker.close()
        imu.close()