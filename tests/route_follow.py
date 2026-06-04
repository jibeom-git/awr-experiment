# python tests/route_follow.py
# 라인트래킹 기반 경로 선택 주행 (자이로 없음)
#
# 트랙 구조:
#   START → 1 → (루트 선택) → 6 GOAL
#
# 루트 A: 모든 분기점 직진 (1→6)
# 루트 B: 1(좌) → 2(우) → 5(우)
# 루트 C: 1(좌) → 2(직) → 3(우) → 4(우) → 5(직)

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.tracker import LineTracker
from sensors.motor   import MotorController
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
SPIN_SPEED      = 30   # 회전 속도 (너무 빠르면 선 지나침 → 낮추기)
SEARCH_SPEED    = 20
LOST_SPEED      = 20
OBSTACLE_CM     = 1
LOOP_HZ         = 50
LCR_CONFIRM_SEC = 0.15

# ══════════════════════════════════════════════════════
# 센서 / 모터 초기화
# ══════════════════════════════════════════════════════
tracker = LineTracker()
motor   = MotorController()

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
# 라인 센서 기반 회전
# ══════════════════════════════════════════════════════
def turn_until_line(direction):
    """
    라인 센서만으로 90도 회전.
    1단계: 직진으로 교차점 벗어나기
    2단계: 중앙 센서 선 사라질 때까지 회전
    3단계: 중앙 센서 선 다시 잡힐 때까지 회전
    """
    print(f"\n[Turn] {direction} 회전 시작")

    # 1단계: 교차점 벗어나기
    motor.forward(BASE_SPEED)
    time.sleep(0.3)
    motor.stop()
    time.sleep(0.1)

    # 2단계: 현재 선 벗어나기
    print(f"[Turn] 현재 선 벗어나는 중...")
    timeout = time.time() + 3.0
    while time.time() < timeout:
        state = tracker.read()
        if not state['center']:
            break
        if direction == "left":
            spin_left(SPIN_SPEED)
        else:
            spin_right(SPIN_SPEED)
        time.sleep(0.02)
    motor.stop()
    time.sleep(0.05)

    # 3단계: 새 선 탐색
    print(f"[Turn] 새 선 탐색 중...")
    timeout = time.time() + 3.0
    while time.time() < timeout:
        state = tracker.read()
        if state['center']:
            motor.stop()
            time.sleep(0.1)
            print(f"[Turn] ✓ 완료!")
            return True
        if direction == "left":
            spin_left(SPIN_SPEED)
        else:
            spin_right(SPIN_SPEED)
        time.sleep(0.02)

    motor.stop()
    print(f"[Turn] ✗ 선 못 찾음 — SPIN_SPEED 조정 필요")
    return False

def turn_180():
    """180도 회전 후 라인 탐색"""
    print("[Turn] 180도 회전")
    motor.stop()
    time.sleep(0.3)

    # 선 사라질 때까지 회전
    timeout = time.time() + 5.0
    while time.time() < timeout:
        state = tracker.read()
        if not state['center']:
            break
        spin_right(SPIN_SPEED)
        time.sleep(0.02)

    # 선 다시 잡힐 때까지 계속 회전
    timeout = time.time() + 5.0
    while time.time() < timeout:
        state = tracker.read()
        if state['center']:
            motor.stop()
            time.sleep(0.1)
            print("[Turn] 180도 완료")
            return
        spin_right(SPIN_SPEED)
        time.sleep(0.02)

    motor.stop()
    print("[Turn] 180도 완료 (라인 미감지)")

# ══════════════════════════════════════════════════════
# 분기점 감지
# ══════════════════════════════════════════════════════
def check_junction(l, c, r, lcr_start):
    if l and c and r:
        if lcr_start is None:
            lcr_start = time.time()
        elif time.time() - lcr_start >= LCR_CONFIRM_SEC:
            return True, lcr_start
    else:
        lcr_start = None
    return False, lcr_start

# ══════════════════════════════════════════════════════
# 라인 추종
# ══════════════════════════════════════════════════════
def line_follow_step(l, c, r, lost_last_error, lost_count, searching):
    if (l or c or r) and searching:
        motor.stop()
        time.sleep(0.05)
        searching  = False
        lost_count = 0

    if l and c and r:
        motor.forward(BASE_SPEED)
    elif not l and c and not r:
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
                turn_until_line("left")
            elif action == "right":
                turn_until_line("right")

        else:
            # ── 라인 추종 ─────────────────────────────
            lost_last_error, lost_count, searching = line_follow_step(
                l, c, r, lost_last_error, lost_count, searching
            )

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