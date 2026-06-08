# python tests/cv_follow.py
# OpenCV 기반 검은 선 추종 주행
#
# 동작 원리:
#   카메라 프레임 하단 ROI에서 검은 선의 X 중심을 계산
#   화면 중앙(320px) 기준으로 오차만큼 조향
#
# 실행: python tests/test_cv_linefollow.py

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from sensors.camera import Camera
from sensors.motor  import MotorController

# ══════════════════════════════════════════════════════
# 설정값
# ══════════════════════════════════════════════════════
BASE_SPEED    = 35     # 기본 직진 속도
TURN_SPEED    = 20     # 회전 시 느린 쪽 속도
SPIN_SPEED    = 30     # 제자리 회전 속도

# ROI: 이미지 하단 몇 % 만 볼지 (0.6 = 하단 40%)
ROI_TOP       = 0.6

# 검은 선 이진화 임계값 (낮을수록 더 어두운 것만 검출)
THRESH        = 80

# 화면 가로 중앙
CENTER_X      = 320

# 오차 허용 범위 (픽셀): 이 안이면 직진
DEAD_ZONE     = 40

# 오차 → 속도 보정 비율
KP            = 0.15   # 클수록 강하게 꺾음

LOOP_HZ       = 20
# ══════════════════════════════════════════════════════

cam   = Camera(width=640, height=480)
motor = MotorController()

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

def detect_line(frame):
    """
    프레임에서 검은 선의 X 중심 좌표 반환.
    Returns:
        cx: 선의 X 좌표 (없으면 None)
        debug_frame: 시각화용 프레임
    """
    h, w = frame.shape[:2]

    # 하단 ROI만 잘라서 분석 (속도 향상)
    roi = frame[int(h * ROI_TOP):h, :]

    # 흑백 변환
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # 이진화: 어두운 픽셀(검정 선) → 255, 밝은 픽셀 → 0
    _, binary = cv2.threshold(gray, THRESH, 255, cv2.THRESH_BINARY_INV)

    # 노이즈 제거
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # 검은 선 픽셀의 무게중심 계산
    M = cv2.moments(binary)
    if M['m00'] == 0:
        return None, frame   # 선 없음

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00']) + int(h * ROI_TOP)

    # 디버그용 시각화
    debug = frame.copy()
    cv2.line(debug, (0, int(h * ROI_TOP)), (w, int(h * ROI_TOP)), (255, 0, 0), 2)
    cv2.circle(debug, (cx, cy), 8, (0, 255, 0), -1)
    cv2.line(debug, (CENTER_X, 0), (CENTER_X, h), (0, 0, 255), 1)
    cv2.putText(debug, f"cx={cx} err={cx - CENTER_X}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return cx, debug

def follow_line():
    interval     = 1.0 / LOOP_HZ
    lost_count   = 0
    lost_dir     = 1   # 1=오른쪽, -1=왼쪽

    print("[Follow] OpenCV 라인 추종 시작 — Ctrl+C로 종료")
    print(f"[Follow] ROI 상단: {int(ROI_TOP*100)}% | 임계값: {THRESH} | KP: {KP}")

    while True:
        t0 = time.time()

        frame = cam.capture()
        cx, debug = detect_line(frame)

        if cx is None:
            # 선 못 찾음 → 마지막 방향으로 탐색
            lost_count += 1
            if lost_count > LOOP_HZ * 1:   # 1초 이상 못 찾으면
                print(f"[Follow] 선 이탈 — 탐색 중 (방향: {'우' if lost_dir > 0 else '좌'})")
            if lost_dir > 0:
                spin_right(SPIN_SPEED)
            else:
                spin_left(SPIN_SPEED)

        else:
            lost_count = 0
            error = cx - CENTER_X   # 양수: 선이 오른쪽, 음수: 선이 왼쪽

            # 마지막 방향 기억
            if error > 0:
                lost_dir = 1
            elif error < 0:
                lost_dir = -1

            if abs(error) < DEAD_ZONE:
                # 직진
                motor.forward(BASE_SPEED)

            elif error > 0:
                # 선이 오른쪽 → 우회전 (오른쪽 느리게)
                correction = min(int(abs(error) * KP), BASE_SPEED - 10)
                left_speed  = BASE_SPEED
                right_speed = max(BASE_SPEED - correction, TURN_SPEED)
                motor.set_motor(4, 1, left_speed)
                motor.set_motor(3, 1, left_speed)
                motor.set_motor(2, 1, right_speed)
                motor.set_motor(1, 1, right_speed)

            else:
                # 선이 왼쪽 → 좌회전 (왼쪽 느리게)
                correction = min(int(abs(error) * KP), BASE_SPEED - 10)
                left_speed  = max(BASE_SPEED - correction, TURN_SPEED)
                right_speed = BASE_SPEED
                motor.set_motor(4, 1, left_speed)
                motor.set_motor(3, 1, left_speed)
                motor.set_motor(2, 1, right_speed)
                motor.set_motor(1, 1, right_speed)

            print(f"[Follow] cx={cx} err={error:+d} correction={correction if abs(error) >= DEAD_ZONE else 0}", end='\r')

        elapsed = time.time() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    print("=================================")
    print("  OpenCV 라인 추종 테스트")
    print("=================================")
    print()
    print("조정 가능한 값:")
    print(f"  THRESH   = {THRESH}   (낮추면 더 어두운 것만 검출)")
    print(f"  ROI_TOP  = {ROI_TOP}  (낮추면 더 넓은 영역 분석)")
    print(f"  KP       = {KP}   (높이면 더 강하게 꺾음)")
    print()
    print("3초 후 시작...")
    time.sleep(3)

    try:
        follow_line()
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        motor.stop()
        motor.close()
        cam.close()