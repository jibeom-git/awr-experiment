#!/usr/bin/env python3
# dashboard.py  (루트 진입점)
# INSITE AGV AI 제어 대시보드 — FSM+XGBoost+IF 통합, 포트 5000
# python dashboard.py 로 직접 실행

import os
import sys
import csv
import math
import time
import json
import signal
import threading
from datetime import datetime
from pathlib import Path

# ── insite 루트를 sys.path 에 추가 ──────────────────────────────────────────
INSITE_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(INSITE_ROOT))

import cv2
import numpy as np
from flask import Flask, render_template, jsonify, Response, request, send_from_directory
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# ── AI 코어 임포트 ──────────────────────────────────────────────────────────
from ai_core.engine        import AGVAIEngine
from ai_core               import config as ai_cfg
from ai_core.signal_detector import detect_traffic_light

# =============================================================================
# 센서 드라이버 안전 초기화 (실패 시 mock 모드 자동 전환)
# =============================================================================

_ultra  = None
_imu    = None
_hx711  = None
_ai_cam = None

# 모터: Move.py 우선 시도, 없으면 MotorController 폴백
_move_ok = False
try:
    import Move as move  # type: ignore  # ~/insite/Move.py
    move.setup()
    _move_ok = True
    print("[INIT] Move (모터 드라이버) OK")
except Exception as e:
    print(f"[INIT] Move FAIL (mock): {e}")

# Move.py 없을 때 sensors/motor.py MotorController 로 폴백
_motor_ctrl = None
if not _move_ok:
    try:
        from sensors.motor import MotorController as _MC
        _motor_ctrl = _MC()
        print("[INIT] MotorController (폴백) OK")
    except Exception as e:
        print(f"[INIT] MotorController FAIL (완전 mock): {e}")

try:
    from sensors.ultra import UltrasonicSensor
    _ultra = UltrasonicSensor()
    print("[INIT] UltrasonicSensor OK")
except Exception as e:
    print(f"[INIT] UltrasonicSensor FAIL: {e}")

try:
    from sensors.mpu6050 import MPU6050
    _imu = MPU6050(bus_id=5, address=0x68)
    print("[INIT] MPU6050 OK")
except Exception as e:
    print(f"[INIT] MPU6050 FAIL: {e}")

try:
    from sensors.hx711 import HX711
    _hx711 = HX711(dout=5, pd_sck=6)
    print("[INIT] HX711 OK")
except Exception as e:
    print(f"[INIT] HX711 FAIL: {e}")

try:
    from sensors.camera import USBCamera
    _ai_cam = USBCamera(device_index=0, width=640, height=480)
    print("[INIT] AI Camera OK")
except Exception as e:
    print(f"[INIT] AI Camera FAIL: {e}")

# =============================================================================
# Flask / SocketIO 설정
# =============================================================================

TEMPLATE_DIR = INSITE_ROOT / "templates"
app = Flask(__name__, template_folder=str(TEMPLATE_DIR))
app.config['SECRET_KEY'] = 'insite_agv_ai_2025'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# =============================================================================
# 전역 공유 상태 레지스터
# =============================================================================

_state_lock   = threading.Lock()
_driving      = False       # 주행 플래그
_mock_obs     = "None"      # 가상 장애물 주입값
_node_flag    = False       # 단발성 노드 감지 플래그
_deadlock     = False       # 데드락 상태
_current_spd  = 0           # 현재 적용 속도
_fsm_state    = "IDLE"      # FSM 상태 문자열
_active_route = "UNKNOWN"   # 배정 경로
_signal_state = "UNKNOWN"   # 신호등 상태
_waiting_signal = False     # 신호 대기 중 플래그
_lc_tare_done = False

# 이상 감지 관련 전역 변수
_prev_pitch   = 0.0
_pitch_delta  = 0.0
_prev_accel_z = 0.0
_accel_z_delta = 0.0
_anomaly_score = 0.0
_gemini_confidence = 0.0
_last_vlm_obs_type = "none"
_last_vlm_reason   = "N/A"

# 센서 캐시
_cache_lock   = threading.Lock()
_sensor_cache = {
    'distance_cm':  400.0,
    'pitch':        0.0,
    'weight_g':     0.0,
    'accel_z':      9.81,
    'accel_x':      0.0,
    'imu_roll':     0.0,
    'imu_yaw':      0.0,
}

# AI 카메라 프레임 버퍼 (raw numpy + JPEG bytes)
_frame_lock = threading.Lock()
_frame_raw  = np.zeros((480, 640, 3), dtype=np.uint8)
_frame_jpg  = None

# AI 엔진 인스턴스
_ai_engine = AGVAIEngine(mode="SAFE")

# ── 경사로 학습 전역 변수 ─────────────────────────────────────────────────────
_slope_learning   = False   # 경사로 학습 활성화 플래그
_slope_paused     = False   # 정상에서 속도 변경 시 일시 중지
_slope_data       = []      # 수집된 데이터 리스트
_slope_route      = "A"     # 학습 중인 경로
_slope_phase      = "up"    # "up" 오르막 | "down" 내리막
_slope_up_label   = None    # 오르막 결과 라벨
_slope_down_label = None    # 내리막 결과 라벨
_slope_up_speed   = 55      # 오르막 속도
_slope_down_speed = 20      # 내리막 속도
_slope_up_result  = None    # 오르막 결과 ("normal"/"slip_risk"/"hill_fail")
_slope_obstacle   = "none"  # 내리막 장애물 선택 ("none"/"bump_1cm"/"bump_2cm")
_session_counter  = 0       # 세션 ID 카운터 (파일 저장 시 고유 ID 생성용)

# =============================================================================
# 백그라운드 스레드 정의
# =============================================================================

def _ai_camera_loop():
    """전방 카메라 프레임을 raw + JPEG 버퍼에 지속 유지 (20fps)"""
    global _frame_raw, _frame_jpg
    while True:
        try:
            if _ai_cam is not None:
                frame = _ai_cam.capture()
            else:
                # 카메라 미연결 시 안내 화면
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "AI CAM OFFLINE", (120, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (60, 60, 60), 2)

            if frame is not None and frame.size > 0:
                ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with _frame_lock:
                    _frame_raw = frame.copy()
                    if ok:
                        _frame_jpg = buf.tobytes()
        except Exception as e:
            print(f"[AI_CAM] {e}")
        time.sleep(0.05)  # ~20fps


def _imu_loop():
    """MPU-6050 상보 필터 폴링 (20Hz), accel_z 추출"""
    global _prev_accel_z, _accel_z_delta
    while True:
        try:
            if _imu is not None:
                data = _imu.get_all()
                pitch  = round(float(data.get('pitch', 0.0)), 1)
                roll   = round(float(data.get('roll',  0.0)), 1)
                yaw    = round(float(data.get('yaw',   0.0)), 1)
                # accel_z: get_all()에 없으면 get_accel() 호출
                accel  = _imu.get_accel()
                accel_z = round(float(accel.get('z', 1.0)) * 9.81, 3)
                accel_x = round(float(accel.get('x', 0.0)) * 9.81, 3)
            else:
                t = time.time()
                pitch   = round(math.cos(t * 0.2) * 3.0, 1)
                roll    = round(math.sin(t * 0.4) * 8.0, 1)
                yaw     = round((t * 5.0) % 360.0, 1)
                accel_z = round(9.81 + math.sin(t * 1.2) * 0.1, 3)
                accel_x = round(math.sin(t * 0.5) * 0.3, 3)

            with _cache_lock:
                _sensor_cache['pitch']    = pitch
                _sensor_cache['imu_roll'] = roll
                _sensor_cache['imu_yaw']  = yaw
                _sensor_cache['accel_z']  = accel_z
                _sensor_cache['accel_x']  = accel_x
        except Exception as e:
            print(f"[IMU] {e}")
        time.sleep(0.05)  # ~20Hz


def _sensor_loop():
    """초음파 + 로드셀 폴링 및 SocketIO 브로드캐스트 (10Hz)"""
    global _lc_tare_done, _prev_pitch, _pitch_delta, _prev_accel_z, _accel_z_delta
    import random

    while True:
        # ── 초음파 ──────────────────────────────────────────────────────────
        try:
            if _ultra is not None:
                d = _ultra.get_distance()
                dist = float(d) if (d is not None and d > 0) else 400.0
            else:
                dist = round(random.uniform(50, 300), 1)
            with _cache_lock:
                _sensor_cache['distance_cm'] = dist
        except Exception as e:
            print(f"[SENSOR ultra] {e}")

        # ── 로드셀 (첫 사이클 영점 교정) ────────────────────────────────────
        try:
            if _hx711 is not None:
                if not _lc_tare_done:
                    _hx711.tare(samples=20)
                    _lc_tare_done = True
                w = _hx711.get_weight()
                with _cache_lock:
                    _sensor_cache['weight_g'] = round(float(w), 1) if w is not None else 0.0
            else:
                with _cache_lock:
                    _sensor_cache['weight_g'] = round(random.uniform(0, 50), 1)
        except Exception as e:
            print(f"[SENSOR hx711] {e}")

        # ── pitch_delta / accel_z_delta 계산 ────────────────────────────────
        with _cache_lock:
            snap = dict(_sensor_cache)

        _pitch_delta   = round(snap['pitch'] - _prev_pitch, 3)
        _accel_z_delta = round(snap['accel_z'] - _prev_accel_z, 3)
        _prev_pitch    = snap['pitch']
        _prev_accel_z  = snap['accel_z']

        # ── 이상 감지 경보 리스트 ────────────────────────────────────────────
        alerts = []
        if abs(_pitch_delta) > ai_cfg.TH_PITCH_DELTA_FAIL:
            alerts.append("⚠ pitch 급변 — 등판 실패 의심")
        if abs(_accel_z_delta) > ai_cfg.TH_IMPACT_Z:
            alerts.append("⚠ Z축 충격 스파이크 — 방지턱")
        if snap['distance_cm'] <= ai_cfg.TH_SONIC_CRITICAL:
            alerts.append("⚠ 전방 장애물 비상 제동 거리!")
        if _current_spd > 40 and abs(snap['accel_x']) < ai_cfg.TH_SLIP_ACCEL:
            alerts.append("⚠ 슬립 의심 — 가속도 불일치")

        # ── 경사로 학습 데이터 수집 ──────────────────────────────────────────────
        with _state_lock:
            _sl_active = _slope_learning
            _sl_paused = _slope_paused
            _sl_phase  = _slope_phase
            _sl_route  = _slope_route
            _sl_uspd   = _slope_up_speed
            _sl_dspd   = _slope_down_speed
            _sl_obs    = _slope_obstacle  # 내리막 장애물 정보 읽기

        if _sl_active and not _sl_paused:
            _slope_data.append({
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "session_id": f"{_sl_route}_{_sl_phase}_{_session_counter:03d}",
                "route":     _sl_route,
                "phase":     _sl_phase,
                "speed_cmd": _sl_uspd if _sl_phase == "up" else _sl_dspd,
                "weight_g":  snap["weight_g"],
                "obstacle":  _sl_obs if _sl_phase == "down" else "none",  # 내리막에만 장애물 기록
                "pitch":     snap["pitch"],
                "sonic_cm":  snap["distance_cm"],
                "accel_x":   snap["accel_x"],
                "accel_y":   0.0,
                "accel_z":   snap["accel_z"],
                "pitch_delta":   _pitch_delta,
                "accel_z_delta": _accel_z_delta,  # 신규 추가
                "result":    "recording",
            })

        # ── SocketIO 브로드캐스트 ─────────────────────────────────────────────
        with _state_lock:
            state      = _fsm_state
            route      = _active_route
            speed      = _current_spd
            mode       = _ai_engine.mode
            drv        = _driving
            dead       = _deadlock
            obs        = _mock_obs
            sig        = _signal_state
            wsig       = _waiting_signal
            anoms      = _anomaly_score
            gc         = _gemini_confidence
            vobs       = _last_vlm_obs_type
            vrsn       = _last_vlm_reason
            # 경사로 학습 상태 (UI 표시용)
            sl_learning  = _slope_learning
            sl_paused    = _slope_paused
            sl_phase     = _slope_phase
            sl_obstacle  = _slope_obstacle
            sl_up_cnt    = len([r for r in _slope_data if r.get("phase") == "up"])
            sl_down_cnt  = len([r for r in _slope_data if r.get("phase") == "down"])

        # pitch / accel_z_delta / anomaly_score → index.html 실시간 센서 그래프(Chart.js)에서 사용
        socketio.emit('sensor_update', {
            'distance_cm':      snap['distance_cm'],
            'pitch':            snap['pitch'],
            'weight_g':         snap['weight_g'],
            'accel_z':          snap['accel_z'],
            'accel_x':          snap['accel_x'],
            'imu_roll':         snap.get('imu_roll', 0.0),
            'imu_yaw':          snap.get('imu_yaw', 0.0),
            'pitch_delta':      _pitch_delta,
            'accel_z_delta':    _accel_z_delta,
            'fsm_state':        state,
            'route':            route,
            'speed':            speed,
            'drive_mode':       mode,
            'driving':          drv,
            'deadlock':         dead,
            'mock_obs':         obs,
            'signal':           sig,
            'waiting_signal':   wsig,
            'anomaly_score':    anoms,
            'gemini_confidence': gc,
            'vlm_obs_type':     vobs,
            'vlm_reason':       vrsn,
            'alerts':           alerts,
            'slope_learning':   sl_learning,
            'slope_paused':     sl_paused,
            'slope_phase':      sl_phase,
            'slope_obstacle':   sl_obstacle,
            'slope_up_cnt':     sl_up_cnt,
            'slope_down_cnt':   sl_down_cnt,
        })
        time.sleep(0.1)  # 10Hz


def _signal_loop():
    """
    주행 시작 버튼 클릭 후 신호등 감지 루프.
    "GO" 신호 확인 시 drive_loop 활성화.
    """
    global _signal_state, _waiting_signal, _driving

    # [수정 3] 연속 GO 카운트: 3회 연속 GO 감지 시 확정 (오탐 방지)
    green_count = 0

    while True:
        with _state_lock:
            waiting = _waiting_signal

        if not waiting:
            green_count = 0  # 대기 중 아닐 때 카운트 초기화
            time.sleep(0.1)  # 기존 0.5초 → 0.1초 (5배 빠른 폴링)
            continue

        # [수정 3] 0.1초마다 신호 감지 (기존 0.5초에서 5배 빠르게)
        with _frame_lock:
            frame = _frame_raw.copy()

        sig = detect_traffic_light(frame)

        with _state_lock:
            _signal_state = sig

        socketio.emit('signal_update', {'signal': sig})

        if sig == "GO":
            green_count += 1
            # 연속 3회 GO 신호 감지 시 확정 (단발 오탐 무시)
            if green_count >= 3:
                green_count = 0
                with _state_lock:
                    _waiting_signal = False
                    _driving        = True
                # cv_dashboard 라인트래킹 주행 시작
                _ai_engine.start_tracking()
                socketio.emit('toast', {'msg': '신호 GO — 주행 시작!', 'type': 'ok'})
                print("[SIGNAL] GO 신호 3회 연속 확정 → 주행 활성화 + cv_dashboard 기동")
        else:
            green_count = 0  # GO 아닌 신호 감지 시 카운트 초기화

        time.sleep(0.1)  # 기존 0.5초 → 0.1초


def _drive_loop():
    """
    AGV 주행 루프 (0.1초 간격).
    global _mock_obs
    _driving 여부와 무관하게 노드 / 장애물 처리.
    경로 변경 시 cv_dashboard(포트 5001)에 stop → reset → config → start 전송.
    """
    global _current_spd, _fsm_state, _active_route, _deadlock, _node_flag
    global _anomaly_score, _gemini_confidence, _last_vlm_obs_type, _last_vlm_reason
    global _mock_obs  # UnboundLocalError 방지: 루프 내 _mock_obs 초기화 시 필요

    while True:
        # _driving 여부와 무관하게 플래그 항상 읽기 (핵심 버그 수정)
        with _state_lock:
            node_trig  = _node_flag
            _node_flag = False  # 단발성 플래그 소비
            mock       = _mock_obs
            driving    = _driving
            dead       = _deadlock

        # 데드락 상태: 항상 모터 정지 유지
        if dead:
            _motor_stop()
            time.sleep(0.1)
            continue

        # 주행 중 초음파 장애물 구간 자동 감지
        in_obs_zone = False
        if driving:
            with _cache_lock:
                _sonic_now = _sensor_cache['distance_cm']
            if 0 < _sonic_now <= ai_cfg.TH_SONIC_SLOWDOWN:
                in_obs_zone = True

        # _driving 여부와 무관하게 노드/장애물 처리
        if node_trig or mock != "None" or in_obs_zone:
            _motor_stop()

            # 장애물 구간에서만 후진 후 캡처 (노드 트리거 단독이면 생략)
            if in_obs_zone or mock != "None":
                _do_obstacle_backup()

            with _cache_lock:
                sensors = dict(_sensor_cache)
            with _frame_lock:
                frame = _frame_raw.copy()

            sonic = sensors['distance_cm']
            sensor_data = dict(sensors)
            sensor_data['mock_obs'] = mock
            sensor_data['frame']    = frame
            sensor_data['sonic']    = sonic

            with _state_lock:
                mode       = _ai_engine.mode
                node_id    = _ai_engine.current_node_count + (1 if node_trig else 0)
                prev_route = _active_route

            try:
                result = _ai_engine.evaluate(sensor_data, mode, node_id)
            except Exception as e:
                print(f"[DRIVE] engine.evaluate 오류: {e}")
                result = {"route": "UNKNOWN", "speed": 0,
                          "fsm_state": "ERROR", "reason": str(e),
                          "anomaly_score": 0.0, "gemini_confidence": 0.0}

            speed   = result.get('speed', 0)
            state   = result.get('fsm_state', 'UNKNOWN')
            route   = result.get('route', 'UNKNOWN')
            anoms   = result.get('anomaly_score', 0.0)
            gc      = result.get('gemini_confidence', 0.0)
            reason  = result.get('reason', 'N/A')
            vlm_obs = _ai_engine.last_vlm_result.get('obstacle_type', 'none')

            with _state_lock:
                _current_spd       = speed
                _fsm_state         = state
                _active_route      = route
                _anomaly_score     = anoms
                _gemini_confidence = gc
                _last_vlm_obs_type = vlm_obs
                _last_vlm_reason   = reason

            print(f"[DRIVE] 노드 처리: route={route} speed={speed} fsm={state}")

            # 데드락 감지 → 경고 오버레이 + 각 경로 차단 이유 수집
            if route == ai_cfg.DEADLOCK:
                with _state_lock:
                    if not _deadlock:
                        _deadlock = True
                _motor_stop()
                avail = _ai_engine.route_availability
                blocked = {}
                if not avail.get('A', True):
                    blocked['A'] = 'ROUTE_A 차단: 전방 장애물 통과 불가 판정'
                if not avail.get('B', True):
                    blocked['B'] = 'ROUTE_B 차단: 전방 장애물 통과 불가 판정'
                if not avail.get('C', True):
                    blocked['C'] = 'ROUTE_C 차단: 전방 장애물 통과 불가 판정'
                socketio.emit('deadlock_alert', {
                    'active': True,
                    'msg': '모든 경로 차단 — 관리자 개입 필요',
                    'blocked_reasons': blocked,
                })
                time.sleep(0.1)
                continue

            # [수정 2] 경로 변경 시 cv_dashboard 통신 방식 분기
            # PATH_BLOCKED(장애물 차단) → /reroute 호출 (cv_dashboard가 stop/reset/restart 내부 처리)
            # 일반 경로 변경 → 기존 stop→reset→config→start 시퀀스 유지
            if route not in ('UNKNOWN',) and route != prev_route:
                is_reroute = (state == "PATH_BLOCKED")
                if is_reroute:
                    # 장애물로 인한 우회 경로 변경: /reroute 한 번으로 처리
                    _ai_engine.send_route_and_speed(route, speed, reroute=True)
                else:
                    # 일반 경로 변경: 기존 시퀀스
                    _ai_engine.stop_tracking()
                    _ai_engine.reset_tracking()
                    _ai_engine.send_route_and_speed(route, speed, reroute=False)
                    if driving:  # 주행 중일 때만 재기동
                        _ai_engine.start_tracking()

            # _driving=False 상태에서 모터가 간헐적으로 구동되는 문제 방지
            if driving and speed > 0:
                _motor_forward(speed)
            else:
                _motor_stop()  # _driving=False 면 항상 정지

            # 처리 완료 후 트리거 강제 초기화 (무한 반복 방지)
            # mock_obs가 "None"이 아닌 상태로 남아 루프가 계속 돌던 버그 수정
            with _state_lock:
                _mock_obs  = "None"   # 장애물 주입값 1회 소비 후 초기화
                _node_flag = False    # 노드 플래그 소비 확인
            time.sleep(0.3)  # 동일 트리거 연속 발생 방지 (0.5→0.3으로 단축)
            continue

        # 주행 중 아님 + 트리거 없음 → 대기
        if not driving:
            time.sleep(0.1)
            continue

        # 정상 주행 구간: 비상 제동 체크
        with _cache_lock:
            sonic = _sensor_cache['distance_cm']

        if 0 < sonic <= ai_cfg.TH_SONIC_CRITICAL:
            _motor_stop()
            with _state_lock:
                _current_spd  = 0
                _fsm_state    = "CRITICAL_STOP"
            time.sleep(0.1)
            continue

        time.sleep(0.1)  # 10Hz


# =============================================================================
# 모터 제어 헬퍼 (Move.py 우선, mock 폴백)
# =============================================================================

def _motor_forward(speed: int):
    """전방 직진 명령"""
    if _move_ok:
        try:
            move.move(speed, 1, "mid")
        except Exception as e:
            print(f"[MOTOR] forward 오류: {e}")
    elif _motor_ctrl is not None:
        _motor_ctrl.forward(speed)
    else:
        print(f"[MOTOR MOCK] forward @ {speed}%")


def _motor_stop():
    """정지 명령"""
    if _move_ok:
        try:
            move.motorStop()
        except Exception as e:
            print(f"[MOTOR] stop 오류: {e}")
    elif _motor_ctrl is not None:
        _motor_ctrl.stop()


def _do_obstacle_backup():
    """장애물 확인용 후진: 40% 속도로 1초 후진 후 정지"""
    # cv_dashboard 정지 + 0.2초 대기
    _ai_engine.backup_for_obstacle()

    # 로컬 모터 후진
    if _move_ok:
        try:
            move.move(ai_cfg.OBSTACLE_BACKUP_SPEED, -1, "mid")
        except Exception as e:
            print(f"[MOTOR] 후진 오류: {e}")
    elif _motor_ctrl is not None:
        _motor_ctrl.backward(ai_cfg.OBSTACLE_BACKUP_SPEED)
    else:
        print(f"[MOTOR MOCK] 후진 @ {ai_cfg.OBSTACLE_BACKUP_SPEED}%")

    time.sleep(ai_cfg.OBSTACLE_BACKUP_SEC)
    _motor_stop()
    print("[BACKUP] 후진 완료 — 카메라 이미지 캡처 준비")


# =============================================================================
# MJPEG 스트리밍
# =============================================================================

def _gen_mjpeg():
    """전방 카메라 MJPEG 제너레이터"""
    while True:
        with _frame_lock:
            jpg = _frame_jpg
        if jpg:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
        time.sleep(0.066)  # ~15fps


# =============================================================================
# Flask 라우트
# =============================================================================

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """전방 카메라 MJPEG 스트리밍"""
    return Response(_gen_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/sensor')
def api_sensor():
    """실시간 센서값 JSON"""
    with _cache_lock:
        snap = dict(_sensor_cache)
    with _state_lock:
        return jsonify({**snap,
                        'pitch_delta':   _pitch_delta,
                        'accel_z_delta': _accel_z_delta,
                        'anomaly_score': _anomaly_score})


@app.route('/api/status')
def api_status():
    """FSM 상태, 경로, 속도, 신호 JSON"""
    with _state_lock:
        return jsonify({
            'fsm_state':         _fsm_state,
            'route':             _active_route,
            'speed':             _current_spd,
            'drive_mode':        _ai_engine.mode,
            'driving':           _driving,
            'deadlock':          _deadlock,
            'signal':            _signal_state,
            'waiting_signal':    _waiting_signal,
            'anomaly_score':     _anomaly_score,
            'gemini_confidence': _gemini_confidence,
            'vlm_obs_type':      _last_vlm_obs_type,
            'vlm_reason':        _last_vlm_reason,
            'row_count':         _ai_engine.logger.get_csv_row_count(),
        })


@app.route('/api/mock_obstacle', methods=['POST'])
def api_mock_obstacle():
    """가상 장애물 주입 (HTTP API)"""
    global _mock_obs
    body    = request.get_json(silent=True) or {}
    obs_key = str(body.get('type', 'reset'))
    # 1cm/2cm 방지턱 기준으로 업데이트
    obs_map = {
        'bump_1cm': 'Bump_1cm',
        'bump_2cm': 'Bump_2cm',
        'vinyl':    'Vinyl',
        'reset':    'None',
    }
    with _state_lock:
        _mock_obs = obs_map.get(obs_key, 'None')
    print(f"[API] 장애물 주입: {_mock_obs}")
    socketio.emit('toast', {'msg': f'장애물 주입: {obs_key}', 'type': 'warn'})
    return jsonify({'ok': True, 'mock_obs': _mock_obs})


@app.route('/api/node_trigger', methods=['POST'])
def api_node_trigger():
    """가상 노드 감지 자극"""
    global _node_flag
    with _state_lock:
        _node_flag = True
    print("[API] 노드 감지 자극")
    socketio.emit('toast', {'msg': '노드 감지 자극 주입', 'type': 'info'})
    return jsonify({'ok': True})


@app.route('/api/operator_cmd', methods=['POST'])
def api_operator_cmd():
    """관리자 채팅 명령 수신 (HTTP) — DEADLOCK 해제 + cv_dashboard 리셋"""
    global _deadlock, _ai_engine, _driving, _mock_obs
    body = request.get_json(silent=True) or {}
    cmd  = str(body.get('command', '')).strip()
    print(f"[API] 관리자 명령: {cmd}")

    with _state_lock:
        _deadlock = False
        _mock_obs = "None"
        mode      = _ai_engine.mode

    _ai_engine = AGVAIEngine(mode=mode)
    _ai_engine.stop_tracking()
    _ai_engine.reset_tracking()
    socketio.emit('deadlock_alert', {'active': False})
    socketio.emit('toast', {'msg': f'[ADMIN] 데드락 해제: "{cmd}"', 'type': 'ok'})
    return jsonify({'ok': True})


@app.route('/api/retrain', methods=['POST'])
def api_retrain():
    """XGBoost 수동 재학습 트리거"""
    def _do_retrain():
        acc = _ai_engine.trainer.retrain(_ai_engine.logger.log_path)
        socketio.emit('toast', {
            'msg': f'재학습 완료 — Route 정확도: {acc:.3f}' if acc >= 0 else '데이터 부족 (< 20건)',
            'type': 'ok' if acc >= 0 else 'warn',
        })
        socketio.emit('retrain_done', {'accuracy': acc})

    threading.Thread(target=_do_retrain, daemon=True, name='retrain').start()
    return jsonify({'ok': True, 'msg': '재학습 시작 (백그라운드)'})


@app.route('/api/obstacle_db')
def api_obstacle_db():
    """누적 장애물 DB 조회"""
    db = _ai_engine.logger.get_obstacle_db()
    return jsonify(db)


@app.route('/obstacle_image/<path:filename>')
def obstacle_image(filename):
    """data/obstacles/ 에서 이미지 파일 서빙"""
    return send_from_directory(
        os.path.join(str(INSITE_ROOT), 'data', 'obstacles'), filename
    )


# =============================================================================
# SocketIO 이벤트 핸들러
# =============================================================================

@socketio.on('connect')
def on_connect():
    """클라이언트 연결 시 현재 전체 상태 즉시 전송"""
    with _cache_lock:
        snap = dict(_sensor_cache)
    with _state_lock:
        # pitch / accel_z_delta / anomaly_score → index.html 실시간 센서 그래프(Chart.js)에서 사용
        emit('sensor_update', {
            'distance_cm':      snap['distance_cm'],
            'pitch':            snap['pitch'],
            'weight_g':         snap['weight_g'],
            'accel_z':          snap['accel_z'],
            'accel_x':          snap.get('accel_x', 0.0),
            'imu_roll':         snap.get('imu_roll', 0.0),
            'imu_yaw':          snap.get('imu_yaw', 0.0),
            'pitch_delta':      _pitch_delta,
            'accel_z_delta':    _accel_z_delta,
            'fsm_state':        _fsm_state,
            'route':            _active_route,
            'speed':            _current_spd,
            'drive_mode':       _ai_engine.mode,
            'driving':          _driving,
            'deadlock':         _deadlock,
            'mock_obs':         _mock_obs,
            'signal':           _signal_state,
            'waiting_signal':   _waiting_signal,
            'anomaly_score':    _anomaly_score,
            'gemini_confidence': _gemini_confidence,
            'vlm_obs_type':     _last_vlm_obs_type,
            'vlm_reason':       _last_vlm_reason,
            'alerts':           [],
        })
        if _deadlock:
            emit('deadlock_alert', {'active': True, 'msg': 'DEADLOCK ACTIVE'})


@socketio.on('set_drive_mode')
def on_set_drive_mode(data):
    """주행 모드 전환 (FAST / SAFE) — 엔진 재초기화"""
    global _ai_engine, _driving, _mock_obs, _deadlock, _waiting_signal, _signal_state

    mode = str(data.get('mode', 'SAFE')).upper()
    if mode not in ('FAST', 'SAFE'):
        return

    with _state_lock:
        _driving        = False
        _mock_obs       = "None"
        _deadlock       = False
        _waiting_signal = False
        _signal_state   = "UNKNOWN"

    _motor_stop()
    _ai_engine = AGVAIEngine(mode=mode)
    print(f"[CMD] 모드 전환: {mode}")
    socketio.emit('toast', {'msg': f'모드 전환 → {mode}', 'type': 'info'})


@socketio.on('start_drive')
def on_start_drive():
    """
    주행 시작:
    1. 현재 중량 + 모드로 최적 경로/속도 결정
    2. cv_dashboard에 경로/속도 전송
    3. 신호등 감지 루프 시작 (GO 신호 감지 시 start_tracking 호출)
    """
    global _waiting_signal, _signal_state, _active_route, _current_spd

    # 현재 센서 중량과 주행 모드 읽기
    with _cache_lock:
        weight = _sensor_cache['weight_g']
    with _state_lock:
        mode = _ai_engine.mode

    # 중량 + 모드 → 최적 경로/속도 결정
    if mode == ai_cfg.MODE_FAST:
        if weight >= ai_cfg.TH_LOAD_HEAVY:
            # FAST + 고중량 → ROUTE_A 전복 위험 → ROUTE_B
            route = ai_cfg.ROUTE_B
            speed = ai_cfg.SPEED_MEDIUM_POWER
        else:
            route = ai_cfg.ROUTE_A
            speed = ai_cfg.SPEED_HIGH_POWER
    else:  # SAFE
        route = ai_cfg.ROUTE_C
        speed = ai_cfg.SPEED_DEFAULT

    with _state_lock:
        _active_route = route
        _current_spd  = speed

    # cv_dashboard에 경로/속도 사전 전송
    _ai_engine.send_route_and_speed(route, speed)

    print(f"[CMD] 신호 대기 모드 시작 — route={route}, speed={speed}")

    # 신호 대기 루프 활성화 (GO 감지 시 start_tracking 호출)
    with _state_lock:
        _waiting_signal = True
        _signal_state   = "UNKNOWN"

    socketio.emit('toast', {
        'msg': f'신호 대기 중... 경로={route} 속도={speed}%', 'type': 'info'
    })
    socketio.emit('signal_update', {'signal': 'UNKNOWN'})


@socketio.on('stop_drive')
def on_stop_drive():
    """주행 정지 — 로컬 모터 + cv_dashboard 동시 정지"""
    global _driving, _waiting_signal
    with _state_lock:
        _driving        = False
        _waiting_signal = False
    _motor_stop()
    _ai_engine.stop_tracking()  # cv_dashboard 정지
    print("[CMD] 주행 정지")
    socketio.emit('toast', {'msg': '주행 정지', 'type': 'info'})


@socketio.on('inject_obstacle')
def on_inject_obstacle(data):
    """가상 장애물 SocketIO 이벤트 (HTML 버튼) — 1cm/2cm 방지턱 기준"""
    global _mock_obs
    obs_key = str(data.get('obs', 'reset'))
    obs_map = {
        'bump_1cm': 'Bump_1cm',
        'bump_2cm': 'Bump_2cm',
        'vinyl':    'Vinyl',
        'reset':    'None',
    }
    label_map = {
        'bump_1cm': '1cm 방지턱',
        'bump_2cm': '2cm 방지턱',
        'vinyl':    '비닐 노면',
        'reset':    '장애물 리셋',
    }
    with _state_lock:
        _mock_obs = obs_map.get(obs_key, 'None')
    socketio.emit('toast', {'msg': f'장애물: {label_map.get(obs_key, obs_key)}', 'type': 'warn'})


@socketio.on('trigger_node')
def on_trigger_node():
    """가상 노드 감지 자극"""
    global _node_flag
    with _state_lock:
        _node_flag = True
    socketio.emit('toast', {'msg': '노드 감지 자극 주입', 'type': 'info'})


@socketio.on('admin_command')
def on_admin_command(data):
    """데드락 해제 + 관리자 명령 → cv_dashboard 리셋 후 재기동"""
    global _deadlock, _ai_engine, _driving, _mock_obs

    cmd = str(data.get('command', '')).strip()
    with _state_lock:
        _deadlock = False
        _mock_obs = "None"
        mode      = _ai_engine.mode

    # AI 엔진 재초기화 (경로 가용성 초기화)
    _ai_engine = AGVAIEngine(mode=mode)
    # cv_dashboard 리셋
    _ai_engine.stop_tracking()
    _ai_engine.reset_tracking()
    socketio.emit('deadlock_alert', {'active': False})
    socketio.emit('toast', {'msg': f'[ADMIN] 데드락 해제: "{cmd}"', 'type': 'ok'})


@socketio.on('tare_loadcell')
def on_tare_loadcell():
    """로드셀 수동 영점"""
    global _lc_tare_done
    if _hx711 is not None:
        try:
            _hx711.tare(samples=50)
            _lc_tare_done = True
            emit('toast', {'msg': '로드셀 TARE 완료', 'type': 'ok'})
        except Exception as e:
            emit('toast', {'msg': f'TARE 실패: {e}', 'type': 'err'})
    else:
        emit('toast', {'msg': 'HX711 미연결 (mock)', 'type': 'info'})


@socketio.on('manual_move')
def on_manual_move(data):
    """방향키 수동 모터 제어 — keydown/keyup 에서 호출 (W/A/S/D 미사용)"""
    direction = str(data.get('direction', 'stop'))
    speed     = max(0, min(100, int(data.get('speed', 40))))

    if _move_ok:
        try:
            if direction == 'forward':
                move.move(speed, 1, "mid")
            elif direction == 'backward':
                move.move(speed, -1, "mid")
            elif direction == 'left':
                move.move(speed, 1, "left")
            elif direction == 'right':
                move.move(speed, 1, "right")
            else:
                move.motorStop()
        except Exception as e:
            print(f"[MOTOR manual] {e}")
    elif _motor_ctrl is not None:
        if direction == 'forward':
            _motor_ctrl.forward(speed)
        elif direction == 'backward':
            _motor_ctrl.backward(speed)
        elif direction == 'left':
            _motor_ctrl.rotate_left(speed)
        elif direction == 'right':
            _motor_ctrl.rotate_right(speed)
        else:
            _motor_ctrl.stop()
    else:
        if direction != 'stop':
            print(f"[MOTOR MOCK manual] {direction} @ {speed}%")


@socketio.on('manual_retrain')
def on_manual_retrain():
    """XGBoost 수동 재학습 (SocketIO)"""
    def _do():
        acc = _ai_engine.trainer.retrain(_ai_engine.logger.log_path)
        socketio.emit('retrain_done', {'accuracy': acc})
        socketio.emit('toast', {
            'msg': f'재학습 완료 — 정확도: {acc:.3f}' if acc >= 0 else '데이터 부족',
            'type': 'ok' if acc >= 0 else 'warn',
        })
    threading.Thread(target=_do, daemon=True, name='retrain').start()


# =============================================================================
# 경사로 학습 SocketIO 이벤트 핸들러
# =============================================================================

def _save_experiment_data(data: list, session_id: str):
    """
    학습 데이터를 experiment/data/experiment_v2.csv 에 추가 저장.
    헤더가 없으면 자동으로 추가.
    """
    csv_path = os.path.join(str(INSITE_ROOT), 'experiment', 'data', 'experiment_v2.csv')
    # 새 열 구조: weight_g를 obstacle 앞으로 이동, accel_z_delta 추가
    fieldnames = [
        "timestamp", "session_id", "route", "phase", "speed_cmd",
        "weight_g", "obstacle",
        "pitch", "sonic_cm", "accel_x", "accel_y", "accel_z",
        "pitch_delta", "accel_z_delta", "result"
    ]
    # 파일이 없으면 헤더 포함해서 생성
    write_header = not os.path.exists(csv_path)
    with open(csv_path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        for row in data:
            writer.writerow(row)
    print(f"[EXPERIMENT] {len(data)}행 저장 완료 → experiment_v2.csv (session: {session_id})")


@socketio.on('slope_start')
def on_slope_start(data):
    """경사로 학습 시작: 오르막 데이터 수집 개시"""
    global _slope_learning, _slope_paused, _slope_data
    global _slope_route, _slope_phase, _slope_up_speed, _slope_down_speed
    global _session_counter, _slope_up_result, _slope_obstacle  # 신규 변수 초기화
    with _state_lock:
        _slope_learning   = True
        _slope_paused     = False
        _slope_data       = []
        _slope_route      = data.get("route", "A")
        _slope_phase      = "up"
        _slope_up_speed   = int(data.get("up_speed", 55))
        _slope_down_speed = int(data.get("down_speed", 20))
        _slope_up_result  = None    # 오르막 결과 초기화
        _slope_obstacle   = "none"  # 내리막 장애물 초기화
        _session_counter  += 1  # 새 경사로 세션마다 고유 ID 부여
    _ai_engine._cv_post("/config", {"base_speed": _slope_up_speed})
    _ai_engine._cv_post("/start")
    socketio.emit('toast', {
        'msg': f'경사로 학습 시작 — 오르막 {_slope_up_speed}%', 'type': 'ok'
    })


@socketio.on('slope_pause')
def on_slope_pause(data):
    """오르막 완료: 일시 중지 + 오르막 라벨 소급 적용 + hill_fail이면 즉시 저장 종료"""
    global _slope_paused, _slope_phase, _slope_up_label, _slope_up_result

    label = str(data.get("label", "normal"))
    with _state_lock:
        _slope_paused    = True
        _slope_up_label  = label
        _slope_up_result = label
        _slope_phase     = "up_done"

    # 오르막 데이터 result 소급 적용
    for row in _slope_data:
        if row["phase"] == "up":
            row["result"] = label

    _ai_engine._cv_post("/stop")

    # 통과 못함이면 즉시 저장 후 종료 (내리막 없음)
    if label == "hill_fail":
        session_id = f"{_slope_route}_up_{_session_counter:03d}"
        try:
            _save_experiment_data(_slope_data, session_id)
        except Exception as e:
            print(f"[SLOPE] 저장 실패: {e}")
        with _state_lock:
            _slope_learning = False
        socketio.emit('toast', {
            'msg': f'오르막 실패 — {len(_slope_data)}행 저장 완료', 'type': 'warn'
        })
        socketio.emit('slope_finished', {'reason': 'hill_fail', 'session_id': session_id})
        return

    up_count = sum(1 for r in _slope_data if r["phase"] == "up")
    socketio.emit('toast', {
        'msg': f'오르막 완료 ({label}) — {up_count}행. 장애물을 선택하세요.',
        'type': 'info',
    })
    # 클라이언트에 장애물 선택 팝업 표시 신호 전송 (수정 1)
    socketio.emit('show_obstacle_modal', {'up_result': label})


@socketio.on('slope_set_obstacle')
def on_slope_set_obstacle(data):
    """장애물 선택 + 내리막 자동 시작 (별도 재개 이벤트 없이 바로 진행, 수정 2)"""
    global _slope_obstacle, _slope_paused, _slope_phase, _slope_down_speed
    with _state_lock:
        _slope_obstacle = str(data.get("obstacle", "none"))
        _slope_phase    = "down"
        _slope_paused   = False

    _ai_engine._cv_post("/config", {"base_speed": _slope_down_speed})
    _ai_engine._cv_post("/start")
    socketio.emit('toast', {
        'msg': f'내리막 시작 — {_slope_down_speed}% / 장애물: {_slope_obstacle}',
        'type': 'ok'
    })


@socketio.on('slope_bump_fail')
def on_slope_bump_fail():
    """내리막 중 방지턱 통과 실패 → 즉시 저장 후 종료 (수정 3, 신규)"""
    global _slope_learning

    # 내리막 데이터 전체에 bump_fail 소급 적용
    for row in _slope_data:
        if row["phase"] == "down":
            row["result"] = "bump_fail"

    _ai_engine._cv_post("/stop")

    session_id = f"{_slope_route}_{_session_counter:03d}"
    try:
        _save_experiment_data(_slope_data, session_id)
    except Exception as e:
        print(f"[SLOPE] 저장 실패: {e}")

    with _state_lock:
        _slope_learning = False

    total = len(_slope_data)
    socketio.emit('toast', {
        'msg': f'방지턱 실패 — {total}행 저장 완료', 'type': 'warn'
    })
    socketio.emit('slope_finished', {'reason': 'bump_fail', 'session_id': session_id})


@socketio.on('slope_stop')
def on_slope_stop(data):
    """경사로 학습 종료: 내리막 라벨 적용 + CSV 저장 + IF 재학습"""
    global _slope_learning, _slope_down_label
    label = str(data.get("label", "normal"))
    with _state_lock:
        _slope_down_label = label
        _slope_learning   = False

    # 내리막 데이터에 라벨 소급 적용 (recording 상태인 행만 — bump_fail은 이미 처리됨, 수정 4)
    for row in _slope_data:
        if row["phase"] == "down" and row["result"] == "recording":
            row["result"] = label

    _ai_engine._cv_post("/stop")

    session_id = f"{_slope_route}_{_session_counter:03d}"
    try:
        _save_experiment_data(_slope_data, session_id)
    except Exception as e:
        print(f"[SLOPE] 저장 실패: {e}")

    # 정상/슬립위험 데이터로 IF 재학습
    if_data = [
        [r["pitch"], r["accel_z"], r["accel_x"], r["weight_g"], r["sonic_cm"]]
        for r in _slope_data
        if r["result"] in ("normal", "slip_risk")
    ]
    if len(if_data) >= 10:
        threading.Thread(
            target=_ai_engine.trainer.retrain_isolation_forest,
            args=(if_data,), daemon=True
        ).start()

    total = len(_slope_data)
    socketio.emit('toast', {
        'msg': f'학습 완료 — {total}행 저장 / IF 재학습 중', 'type': 'ok'
    })
    socketio.emit('slope_finished', {'reason': 'done', 'session_id': session_id})


@socketio.on('slope_delete_session')
def on_slope_delete_session(data):
    """session_id 기준으로 experiment_v2.csv 에서 해당 세션 데이터만 삭제"""
    session_id = str(data.get("session_id", ""))
    if not session_id:
        socketio.emit('toast', {'msg': '삭제할 세션 ID 없음', 'type': 'warn'})
        return

    csv_path = os.path.join(str(INSITE_ROOT), 'experiment', 'data', 'experiment_v2.csv')
    if not os.path.exists(csv_path):
        socketio.emit('toast', {'msg': 'CSV 파일 없음', 'type': 'warn'})
        return

    try:
        rows = []
        deleted = 0
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames
            for row in reader:
                if row["session_id"] == session_id:
                    deleted += 1
                else:
                    rows.append(row)

        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        socketio.emit('toast', {
            'msg': f'세션 {session_id} 삭제 완료 — {deleted}행 제거', 'type': 'ok'
        })
        print(f"[SLOPE] 세션 삭제: {session_id} ({deleted}행)")
    except Exception as e:
        socketio.emit('toast', {'msg': f'삭제 실패: {e}', 'type': 'err'})


# =============================================================================
# 종료 핸들러
# =============================================================================

def _cleanup(signum, frame):
    print("\n[SHUTDOWN] 하드웨어 자원 반환 중...")
    _motor_stop()
    for dev, name in [(_ultra, 'ultra'), (_ai_cam, 'cam'), (_imu, 'imu')]:
        if dev is not None:
            try:
                dev.close()
            except Exception:
                pass
    print("[SHUTDOWN] 완료.")
    os._exit(0)


signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

# =============================================================================
# 진입점
# =============================================================================

if __name__ == '__main__':
    print("=" * 65)
    print("  INSITE AGV AI DASHBOARD  —  XGBoost + Isolation Forest + VLM")
    print(f"  URL  : http://192.168.0.50:5000")
    print(f"  VLM  : {ai_cfg.GEMINI_MODEL_NAME}")
    print(f"  LOG  : {_ai_engine.logger.log_path}")
    print(f"  MODE : {_ai_engine.mode}")
    print("=" * 65)

    # 백그라운드 스레드 시작
    threading.Thread(target=_ai_camera_loop, daemon=True, name='aicam').start()
    threading.Thread(target=_imu_loop,       daemon=True, name='imu').start()
    threading.Thread(target=_sensor_loop,    daemon=True, name='sensor').start()
    threading.Thread(target=_signal_loop,    daemon=True, name='signal').start()
    threading.Thread(target=_drive_loop,     daemon=True, name='drive').start()

    socketio.run(
        app, host='0.0.0.0', port=5000,
        debug=False, allow_unsafe_werkzeug=True,
    )
