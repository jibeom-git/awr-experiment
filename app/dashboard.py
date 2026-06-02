#!/usr/bin/env python3
# =============================================================================
# File    : ~/insite/app/dashboard.py
# Project : Insite — Waypoint Recording Dashboard
# Run     : source ~/insite/.venv/bin/activate
#           python3 ~/insite/app/dashboard.py
# Access  : http://192.168.0.50:5001
# =============================================================================
# 디렉토리 구조:
#   ~/insite/app/dashboard.py          ← 이 파일 (서버 엔트리포인트)
#   ~/insite/app/templates/index.html  ← 대시보드 UI
#   ~/insite/sensors/                  ← 하드웨어 드라이버
#   ~/insite/core/waypoint_graph.json  ← 웨이포인트/경로 저장
#   ~/insite/loadcell_cal.txt          ← 로드셀 캘리브레이션
# =============================================================================

import os
import sys
import json
import time
import signal
import threading
from pathlib import Path

import cv2
import numpy as np

from flask import Flask, Response, send_from_directory, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# ── 프로젝트 경로 — sensors/, core/ import 전에 반드시 먼저 설정 ────────────────
INSITE_ROOT = Path.home() / 'insite'
sys.path.insert(0, str(INSITE_ROOT))

# 웨이포인트 네비게이터 — 실제 주행 로직은 core/waypoint_navigator.py 에서 수정
try:
    from core.waypoint_navigator import WaypointNavigator
    _NAV_AVAILABLE = True
except Exception as e:
    print(f'[INIT] WaypointNavigator FAIL: {e}')
    WaypointNavigator = None
    _NAV_AVAILABLE = False

GRAPH_PATH  = INSITE_ROOT / 'core' / 'waypoint_graph.json'

# =============================================================================
# 센서 초기화
# =============================================================================

# ── OV5647 RGB 카메라 ─────────────────────────────────────────────────────────
try:
    from sensors.camera import Camera
    _cam = Camera(width=1280, height=720)
    print("[INIT] Camera (OV5647) OK")
except Exception as e:
    _cam = None
    print(f"[INIT] Camera FAIL: {e}")

# ── 초음파 ────────────────────────────────────────────────────────────────────
try:
    from sensors.ultra import UltrasonicSensor
    _ultra = UltrasonicSensor()
    print("[INIT] UltrasonicSensor OK")
except Exception as e:
    _ultra = None
    print(f"[INIT] UltrasonicSensor FAIL: {e}")

# ── IMU ───────────────────────────────────────────────────────────────────────
try:
    from sensors.mpu6050 import MPU6050
    _imu = MPU6050(bus_id=5, address=0x68)
    print("[INIT] MPU6050 OK")
except Exception as e:
    _imu = None
    print(f"[INIT] MPU6050 FAIL: {e}")

# ── 로드셀 ────────────────────────────────────────────────────────────────────
try:
    from sensors.hx711 import HX711
    _hx711 = HX711(dout=5, pd_sck=6, gain=128)
    print("[INIT] HX711 OK")
except Exception as e:
    _hx711 = None
    print(f"[INIT] HX711 FAIL: {e}")

# ── 모터 ──────────────────────────────────────────────────────────────────────
try:
    from sensors.motor import MotorController
    _motor = MotorController()
    print("[INIT] MotorController OK")
except Exception as e:
    _motor = None
    print(f"[INIT] MotorController FAIL: {e}")

# ── LED ───────────────────────────────────────────────────────────────────────
try:
    from sensors.led import LEDController
    _led = LEDController()
    _led.set_running()
    print("[INIT] LEDController OK")
except Exception as e:
    _led = None
    print(f"[INIT] LEDController FAIL: {e}")

# ── HAT 배터리 (ADS7830, I2C 0x48) ───────────────────────────────────────────
# Voltage.py 원본 BatteryLevelMonitor 로직 그대로 이식
_ADCVref   = 5.2
_R15, _R17 = 3000, 1000
_DIV_RATIO = _R17 / (_R15 + _R17)   # 0.25
_V_FULL    = 8.4
_V_WARN    = 6.0
_hat_vol_buf = []   # 이동평균 버퍼 maxlen=10

try:
    import smbus as _smbus_mod
    _hat_adc = _smbus_mod.SMBus(1)   # ADS7830 전용 SMBus 인스턴스
    HAT_BAT_OK = True
    print("[INIT] HAT Battery (ADS7830) OK")
except Exception as e:
    _hat_adc   = None
    HAT_BAT_OK = False
    print(f"[INIT] HAT Battery FAIL: {e}")

def _read_hat_battery():
    """Voltage.py: analogRead(ch=0) → A0Voltage → actual_battery_voltage → %"""
    global _hat_vol_buf
    if not HAT_BAT_OK or _hat_adc is None:
        return 0.0, 0.0
    try:
        import statistics
        # Voltage.py: cmd=0x84, chn=0
        # value = bus.read_byte_data(0x48, cmd|(((chn<<2|chn>>1)&0x07)<<4))
        adc_val  = _hat_adc.read_byte_data(0x48, 0x84 | (((0 << 2 | 0 >> 1) & 0x07) << 4))
        a0_v     = adc_val / 255.0 * _ADCVref          # Voltage.py: A0Voltage
        actual_v = a0_v / _DIV_RATIO                   # Voltage.py: actual_battery_voltage
        _hat_vol_buf.append(actual_v)
        if len(_hat_vol_buf) > 10:
            _hat_vol_buf.pop(0)
        med      = statistics.median(_hat_vol_buf)
        filtered = [v for v in _hat_vol_buf if abs(v - med) < 1]
        avg_v    = sum(filtered) / len(filtered) if filtered else actual_v
        pct      = max(0.0, min(100.0, (avg_v - _V_WARN) / (_V_FULL - _V_WARN) * 100.0))
        return round(avg_v, 2), round(pct, 1)
    except Exception as e:
        print(f"[BAT] HAT 읽기 오류: {e}")
        return 0.0, 0.0

# ── Pi UPS 배터리 (52PI, I2C 0x36 MAX17040) ───────────────────────────────────
_ups_fail_count = 0
_UPS_MAX_FAIL   = 3

try:
    _ups_bus   = _smbus_mod.SMBus(1)   # UPS 전용 SMBus 인스턴스 (HAT와 분리)
    UPS_BAT_OK = True
    print("[INIT] Pi UPS Battery (MAX17040) OK")
except Exception as e:
    _ups_bus   = None
    UPS_BAT_OK = False
    print(f"[INIT] Pi UPS Battery FAIL: {e}")

def _read_ups_battery():
    global UPS_BAT_OK, _ups_fail_count
    if not UPS_BAT_OK or _ups_bus is None:
        return 0.0, 0.0
    try:
        d   = _ups_bus.read_i2c_block_data(0x36, 0x02, 2)
        v   = ((d[0] << 4) | (d[1] >> 4)) * 0.00125
        s   = _ups_bus.read_i2c_block_data(0x36, 0x04, 2)
        pct = min(100.0, s[0] + s[1] / 256.0)
        _ups_fail_count = 0
        return round(v, 2), round(pct, 1)
    except Exception as e:
        _ups_fail_count += 1
        if _ups_fail_count <= _UPS_MAX_FAIL:
            print(f"[BAT] UPS 읽기 오류 ({_ups_fail_count}/{_UPS_MAX_FAIL}): {e}")
        if _ups_fail_count >= _UPS_MAX_FAIL:
            UPS_BAT_OK = False
            print("[BAT] UPS 연속 실패 — 이후 읽기 비활성화")
        return 0.0, 0.0

# =============================================================================
# Flask / SocketIO
# =============================================================================
app = Flask(__name__,
            template_folder='templates',  # ~/insite/app/templates/
            static_folder=None)
app.config['SECRET_KEY'] = 'insite_2025'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# =============================================================================
# 상태 전역 변수
# =============================================================================
SPEED_SET   = 40
robot_state = 'IDLE'

_acc_lock  = threading.Lock()
# 방향별 누적 구동 시간 (직전 웨이포인트 이후)
_seg_fwd   = 0.0   # 전진 누적 (초)
_seg_bwd   = 0.0   # 후진 누적 (초)
_seg_left  = 0.0   # 좌회전 누적 (초)
_seg_right = 0.0   # 우회전 누적 (초)
_move_start = None
_move_dir   = None  # 현재 이동 방향 'fwd'|'bwd'|'left'|'right'|None

_lc_tare_done = False

# =============================================================================
# 센서 캐시
# =============================================================================
_sensor_cache = {
    'imu':         {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'yaw_rate': 0.0, 'pos_x': 0.0, 'pos_y': 0.0},
    'ultrasonic':  {'distance': -1.0},
    'loadcell':    {'weight': 0.0},
    'hat_battery': {'voltage': 0.0, 'percent': 0.0},
    'pi_battery':  {'voltage': 0.0, 'percent': 0.0},
    'robot_state': 'IDLE',
}
_cache_lock = threading.Lock()

# =============================================================================
# 카메라 MJPEG 스트림 — OV5647 직접 서빙
# =============================================================================
_cam_lock  = threading.Lock()
_cam_frame = None   # 최신 JPEG bytes

def _camera_loop():
    """
    Camera.capture() → BGR 반환 → JPEG 인코딩 → _cam_frame 갱신 (20fps)
    sensors/camera.py: RGB888 캡처 후 내부에서 RGB→BGR 변환 완료하여 반환.
    dashboard에서 추가 변환 불필요.
    """
    global _cam_frame
    while True:
        try:
            if _cam is not None:
                frame = _cam.capture()                  # BGR (H,W,3) — camera.py 내부 변환 완료
                if frame is not None and frame.size > 0:
                    ok, buf = cv2.imencode(
                        '.jpg', frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                    )
                    if ok:
                        with _cam_lock:
                            _cam_frame = buf.tobytes()
        except Exception as e:
            print(f"[CAM] {e}")
        time.sleep(0.05)   # 20fps


def _gen_mjpeg():
    """MJPEG multipart 스트림 제너레이터"""
    while True:
        with _cam_lock:
            frame = _cam_frame
        if frame:
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            )
        time.sleep(0.066)


@app.route('/video_feed')
def video_feed():
    return Response(
        _gen_mjpeg(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

# =============================================================================
# 센서 폴링 루프
# =============================================================================

def _imu_fast_loop():
    """
    IMU ~30Hz 폴링 (33ms 주기)
    sensors/mpu6050.py: get_all() → roll/pitch/yaw/yaw_rate/pos_x/pos_y
    """
    import math
    while True:
        try:
            if _imu is not None:
                d = _imu.get_all()   # sensors/mpu6050.py: get_all() → roll/pitch/yaw/yaw_rate/pos_x/pos_y
                entry = {
                    'roll':     round(float(d.get('roll',     0.0)), 1),
                    'pitch':    round(float(d.get('pitch',    0.0)), 1),
                    'yaw':      round(float(d.get('yaw',      0.0)), 1),
                    'yaw_rate': round(float(d.get('yaw_rate', 0.0)), 1),
                    'pos_x':    round(float(d.get('pos_x',    0.0)), 1),
                    'pos_y':    round(float(d.get('pos_y',    0.0)), 1),
                }
            else:
                t = time.time()
                entry = {
                    'roll':     round(math.sin(t * 0.4) * 8.0,  1),
                    'pitch':    round(math.cos(t * 0.3) * 4.0,  1),
                    'yaw':      round((t * 8.0) % 360.0,        1),
                    'yaw_rate': round(math.sin(t * 1.2) * 30.0, 1),
                    'pos_x':    0.0,
                    'pos_y':    0.0,
                }
            with _cache_lock:
                _sensor_cache['imu'] = entry
        except Exception as e:
            print(f"[IMU LOOP] {e}")
        time.sleep(0.05)   # 20Hz — 소프트웨어 I2C 버스 안정성 확보 (30Hz에서 간헐 오류 발생)


def _sensor_loop():
    """초음파/로드셀 20Hz, 배터리 5초, SocketIO emit 10Hz"""
    global _lc_tare_done
    import random

    BAT_INTERVAL    = 5.0
    EMIT_INTERVAL   = 0.1
    SENSOR_INTERVAL = 0.05

    _last_bat  = 0.0
    _last_emit = 0.0

    # ── 시작 즉시 배터리 첫 읽기 (5초 대기 없이 초기값 확보) ─────────────
    hv, hp = _read_hat_battery()
    uv, up = _read_ups_battery()
    with _cache_lock:
        _sensor_cache['hat_battery'] = {'voltage': hv, 'percent': hp}
        _sensor_cache['pi_battery']  = {'voltage': uv, 'percent': up}
    _last_bat = time.time()   # 이후 5초 후에 다시 읽음

    while True:
        now = time.time()

        # ── 초음파 ────────────────────────────────────────────────────────────
        try:
            if _ultra is not None:
                dist = _ultra.get_distance()
                with _cache_lock:
                    _sensor_cache['ultrasonic']['distance'] = (
                        float(dist) if dist is not None else -1.0
                    )
            else:
                with _cache_lock:
                    _sensor_cache['ultrasonic']['distance'] = round(random.uniform(15, 250), 1)
        except Exception as e:
            print(f"[SENSOR] 초음파: {e}")

        # ── 로드셀 ────────────────────────────────────────────────────────────
        try:
            if _hx711 is not None:
                if not _lc_tare_done:
                    print("[HX711] 초기 tare 실행 (바구니 무게 영점)...")
                    _hx711.tare(samples=20)
                    _lc_tare_done = True
                    print("[HX711] tare 완료")
                w = _hx711.get_weight()   # sensors/hx711.py: get_weight() = get_grams() 별칭
                with _cache_lock:
                    _sensor_cache['loadcell']['weight'] = (
                        round(float(w), 1) if w is not None else 0.0
                    )
            else:
                with _cache_lock:
                    _sensor_cache['loadcell']['weight'] = round(random.uniform(0, 300), 1)
        except Exception as e:
            print(f"[SENSOR] 로드셀: {e}")

        # ── 배터리 (5초) ──────────────────────────────────────────────────────
        if now - _last_bat >= BAT_INTERVAL:
            _last_bat = now
            hv, hp = _read_hat_battery()
            uv, up = _read_ups_battery()
            with _cache_lock:
                _sensor_cache['hat_battery'] = {'voltage': hv, 'percent': hp}
                _sensor_cache['pi_battery']  = {'voltage': uv, 'percent': up}

        # ── 로봇 상태 ─────────────────────────────────────────────────────────
        with _cache_lock:
            _sensor_cache['robot_state'] = robot_state

        # ── SocketIO emit (10Hz) ──────────────────────────────────────────────
        if now - _last_emit >= EMIT_INTERVAL:
            _last_emit = now
            with _cache_lock:
                snap = {
                    'imu':         dict(_sensor_cache['imu']),
                    'ultrasonic':  dict(_sensor_cache['ultrasonic']),
                    'loadcell':    dict(_sensor_cache['loadcell']),
                    'hat_battery': dict(_sensor_cache['hat_battery']),
                    'pi_battery':  dict(_sensor_cache['pi_battery']),
                    'robot_state': _sensor_cache['robot_state'],
                }
            socketio.emit('sensor_update', snap)

        time.sleep(SENSOR_INTERVAL)

# =============================================================================
# 모터 제어
# =============================================================================
def _motor_cmd(direction):
    global robot_state, _move_start, _move_dir
    global _seg_fwd, _seg_bwd, _seg_left, _seg_right

    with _acc_lock:
        # 이전 이동 구간 시간 확정
        if _move_dir is not None and _move_start is not None:
            elapsed = time.time() - _move_start
            if   _move_dir == 'fwd':   _seg_fwd   += elapsed
            elif _move_dir == 'bwd':   _seg_bwd   += elapsed
            elif _move_dir == 'left':  _seg_left  += elapsed
            elif _move_dir == 'right': _seg_right += elapsed
        _move_start = time.time() if direction is not None else None
        _move_dir   = direction

    if _motor is None:
        robot_state = 'MOVING' if direction else 'IDLE'
        return
    try:
        if direction == 'fwd':
            _motor.move(SPEED_SET, 1, 'mid');  robot_state = 'MOVING'
        elif direction == 'bwd':
            _motor.move(SPEED_SET, -1, 'mid'); robot_state = 'MOVING'
        elif direction == 'left':
            _motor.rotate_left(SPEED_SET);     robot_state = 'MOVING'
        elif direction == 'right':
            _motor.rotate_right(SPEED_SET);    robot_state = 'MOVING'
        else:
            _motor.stop();                     robot_state = 'IDLE'
    except Exception as e:
        print(f"[MOTOR] {e}")

# =============================================================================
# 웨이포인트 그래프 유틸
# =============================================================================
def _load_graph():
    if GRAPH_PATH.exists():
        try:
            return json.loads(GRAPH_PATH.read_text(encoding='utf-8'))
        except Exception:
            pass
    return {'waypoints': {}, 'paths': {}}

def _save_graph(graph):
    GRAPH_PATH.parent.mkdir(parents=True, exist_ok=True)
    GRAPH_PATH.write_text(
        json.dumps(graph, ensure_ascii=False, indent=2), encoding='utf-8'
    )

def _next_wp_id(graph):
    if not graph['waypoints']:
        return 1
    return max(int(k) for k in graph['waypoints']) + 1

# =============================================================================
# SocketIO 이벤트
# =============================================================================

@socketio.on('connect')
def on_connect():
    print("[WS] 클라이언트 연결")
    with _cache_lock:
        emit('sensor_update', dict(_sensor_cache))
    emit('graph_update', _load_graph())
    if not _lc_tare_done:
        emit('toast', {'type': 'info', 'msg': '로드셀 초기 영점 설정 중...'})
    if _led:
        try: _led.set_running()
        except: pass


@socketio.on('disconnect')
def on_disconnect():
    print("[WS] 클라이언트 연결 해제")
    _motor_cmd(None)


@socketio.on('key_down')
def on_key_down(data):
    km = {'ArrowUp': 'fwd', 'ArrowDown': 'bwd',
          'ArrowLeft': 'left', 'ArrowRight': 'right'}
    d = km.get(data.get('key', ''))
    if d:
        _motor_cmd(d)


@socketio.on('key_up')
def on_key_up(data):
    if data.get('key', '') in ('ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'):
        _motor_cmd(None)


@socketio.on('save_waypoint')
def on_save_waypoint(data):
    global _seg_fwd, _seg_bwd, _seg_left, _seg_right, _move_start, _move_dir

    # 현재 이동 중이면 구간 시간 마감
    with _acc_lock:
        if _move_dir is not None and _move_start is not None:
            elapsed = time.time() - _move_start
            if   _move_dir == 'fwd':   _seg_fwd   += elapsed
            elif _move_dir == 'bwd':   _seg_bwd   += elapsed
            elif _move_dir == 'left':  _seg_left  += elapsed
            elif _move_dir == 'right': _seg_right += elapsed
        # 방향별 시간 스냅샷
        seg = {
            'fwd':   round(_seg_fwd,   3),
            'bwd':   round(_seg_bwd,   3),
            'left':  round(_seg_left,  3),
            'right': round(_seg_right, 3),
        }

    with _cache_lock:
        snap_imu   = dict(_sensor_cache['imu'])
        snap_ultra = dict(_sensor_cache['ultrasonic'])

    graph   = _load_graph()
    wp_id   = _next_wp_id(graph)
    comment = data.get('comment', '').strip()

    # 지배 방향: 가장 오래 이동한 방향
    dominant = max(seg, key=lambda k: seg[k]) if any(seg.values()) else 'fwd'

    graph['waypoints'][str(wp_id)] = {
        'comment':     comment if comment else f'Point {wp_id}',
        'heading':     snap_imu.get('yaw',      0.0),
        'pitch':       snap_imu.get('pitch',     0.0),
        'ultra_front': snap_ultra.get('distance', -1.0),
        # 구간 이동 정보: 재생 시 이 데이터로 동일 동작 재현
        'move': {
            'dominant': dominant,   # 주 이동 방향
            'fwd':   seg['fwd'],
            'bwd':   seg['bwd'],
            'left':  seg['left'],
            'right': seg['right'],
        },
    }
    _save_graph(graph)

    # 누적 초기화
    with _acc_lock:
        _seg_fwd = _seg_bwd = _seg_left = _seg_right = 0.0
        _move_start = time.time() if _move_dir else None

    print(f"[WP] #{wp_id} 저장 | yaw={snap_imu.get('yaw')} ultra={snap_ultra.get('distance')}")
    socketio.emit('graph_update', graph)
    socketio.emit('waypoint_saved', {'id': wp_id})

    if _led:
        try:
            _led.set_thinking(); time.sleep(0.3); _led.set_running()
        except: pass


@socketio.on('save_path')
def on_save_path(data):
    """data = {"path_name": str, "node_ids": [int, ...]}"""
    path_name = data.get('path_name', '').strip()
    node_ids  = data.get('node_ids', [])
    if not path_name:
        emit('toast', {'type': 'error', 'msg': '경로 이름을 입력하세요.'}); return
    if not node_ids:
        emit('toast', {'type': 'error', 'msg': '노드 번호를 입력하세요.'}); return
    graph = _load_graph()
    graph['paths'][path_name] = [int(n) for n in node_ids]
    _save_graph(graph)
    print(f"[PATH] '{path_name}': {node_ids}")
    socketio.emit('graph_update', graph)
    socketio.emit('toast', {'type': 'ok', 'msg': f'경로 "{path_name}" 저장 완료'})


@socketio.on('cancel_session')
def on_cancel_session():
    _motor_cmd(None)
    if _led:
        try: _led.set_running()
        except: pass
    emit('toast', {'type': 'info', 'msg': '세션이 취소되었습니다.'})


@socketio.on('set_speed')
def on_set_speed(data):
    global SPEED_SET
    SPEED_SET = max(10, min(100, int(data.get('speed', 40))))


@socketio.on('reset_yaw')
def on_reset_yaw():
    if _imu is not None:
        try:
            _imu.reset_yaw()   # sensors/mpu6050.py: yaw + pos_x + pos_y 모두 초기화
            emit('toast', {'type': 'info', 'msg': 'Yaw 초기화 완료'})
        except Exception as e:
            emit('toast', {'type': 'error', 'msg': f'초기화 실패: {e}'})
    else:
        emit('toast', {'type': 'error', 'msg': 'IMU 미연결'})


@socketio.on('tare_loadcell')
def on_tare_loadcell():
    global _lc_tare_done
    if _hx711 is not None:
        try:
            emit('toast', {'type': 'info', 'msg': '영점 설정 중... (~5초)'})
            _hx711.tare(samples=50)
            _lc_tare_done = True
            emit('toast', {'type': 'ok', 'msg': '로드셀 영점 완료'})
        except Exception as e:
            emit('toast', {'type': 'error', 'msg': f'Tare 실패: {e}'})
    else:
        emit('toast', {'type': 'error', 'msg': 'HX711 미연결'})


@socketio.on('calibrate_loadcell')
def on_calibrate_loadcell(data):
    """
    로드셀 캘리브레이션.
    sensors/hx711.py의 calibrate(known_grams) 호출.
    순서: tare 완료 → 추 올리기 → grams 입력 → 호출
    """
    if _hx711 is None:
        emit('toast', {'type': 'error', 'msg': 'HX711 미연결'}); return
    grams = float(data.get('grams', 0.0))
    if grams <= 0:
        emit('toast', {'type': 'error', 'msg': '무게(g)를 양수로 입력하세요.'}); return
    try:
        # sensors/hx711.py: calibrate(known_grams) — tare 후 추 올리고 호출
        _hx711.calibrate(grams)
        emit('toast', {'type': 'ok', 'msg': f'캘리브레이션 완료 (REF={_hx711.REF_UNIT_A:.3f})'})
    except Exception as e:
        emit('toast', {'type': 'error', 'msg': f'캘리브레이션 실패: {e}'})


# ── 삭제 이벤트 ───────────────────────────────────────────────────────────────

@socketio.on('delete_waypoint')
def on_delete_waypoint(data):
    """웨이포인트 개별 삭제. 해당 노드를 참조하는 경로에서도 제거."""
    wp_id = str(data.get('id', ''))
    graph = _load_graph()
    if wp_id not in graph['waypoints']:
        emit('toast', {'type': 'error', 'msg': f'#{wp_id} 없음'}); return
    del graph['waypoints'][wp_id]
    for k in graph['paths']:
        graph['paths'][k] = [n for n in graph['paths'][k] if str(n) != wp_id]
    _save_graph(graph)
    socketio.emit('graph_update', graph)
    socketio.emit('toast', {'type': 'info', 'msg': f'웨이포인트 #{wp_id} 삭제'})


@socketio.on('delete_path')
def on_delete_path(data):
    """경로 개별 삭제. 웨이포인트 데이터는 유지."""
    name  = data.get('name', '').strip()
    graph = _load_graph()
    if name not in graph['paths']:
        emit('toast', {'type': 'error', 'msg': f'경로 "{name}" 없음'}); return
    del graph['paths'][name]
    _save_graph(graph)
    socketio.emit('graph_update', graph)
    socketio.emit('toast', {'type': 'info', 'msg': f'경로 "{name}" 삭제'})


@socketio.on('clear_all_waypoints')
def on_clear_all_waypoints():
    """웨이포인트 전체 삭제 (경로 노드 목록도 비움, 경로 이름 유지)"""
    graph = _load_graph()
    graph['waypoints'] = {}
    for k in graph['paths']:
        graph['paths'][k] = []
    _save_graph(graph)
    socketio.emit('graph_update', graph)
    socketio.emit('toast', {'type': 'info', 'msg': '웨이포인트 전체 삭제'})


@socketio.on('clear_all_paths')
def on_clear_all_paths():
    """경로 전체 삭제 (웨이포인트 데이터 유지)"""
    graph = _load_graph()
    graph['paths'] = {}
    _save_graph(graph)
    socketio.emit('graph_update', graph)
    socketio.emit('toast', {'type': 'info', 'msg': '경로 전체 삭제'})


# =============================================================================
# 경로 실행 — core/waypoint_navigator.py 에 위임
# 판단 파라미터(PARAMS), 도달 판정, 장애물/언덕 로직은 모두 navigator.py 에서 수정
# =============================================================================






# ── navigator 인스턴스 (센서 초기화 후 생성) ──────────────────────────────────
_navigator = None

def _get_or_create_navigator():
    """
    WaypointNavigator 인스턴스 반환.
    센서가 준비된 시점에서 처음 호출될 때 생성.
    실제 주행 로직은 core/waypoint_navigator.py 에서 수정.
    """
    global _navigator
    if _navigator is not None:
        return _navigator
    if not _NAV_AVAILABLE or WaypointNavigator is None:
        return None
    _navigator = WaypointNavigator(
        motor=_motor,
        imu=_imu,
        ultra=_ultra,
        speed=SPEED_SET,
    )
    # 상태 변경 시 SocketIO로 브로드캐스트
    def _nav_status_cb(status: dict):
        socketio.emit('run_status', {
            'running':    status.get('running', False),
            'path':       status.get('path',    ''),
            'step':       status.get('step',    ''),
            'current_wp': status.get('current_wp'),
            'state':      status.get('state',   'IDLE'),
        })
        msg = status.get('message', '')
        if msg:
            typ = 'ok' if '완료' in msg else 'error' if '오류' in msg or '장애물' in msg or '타임아웃' in msg else 'info'
            socketio.emit('toast', {'type': typ, 'msg': msg})
    _navigator.set_status_callback(_nav_status_cb)
    return _navigator


@socketio.on('run_path')
def on_run_path(data):
    """
    저장된 경로 실행.
    실제 주행 로직: core/waypoint_navigator.py 의 WaypointNavigator.run_path()
    """
    path_name = data.get('path_name', '').strip()
    if not path_name:
        emit('toast', {'type': 'error', 'msg': '경로 이름을 입력하세요.'}); return

    nav = _get_or_create_navigator()
    if nav is None:
        emit('toast', {'type': 'error', 'msg': 'Navigator 초기화 실패 (센서 확인)'}); return

    # 속도 동기화
    nav.set_speed(SPEED_SET)

    if nav.is_running():
        emit('toast', {'type': 'error', 'msg': '이미 경로 실행 중. 먼저 중단하세요.'}); return

    ok = nav.run_path(path_name)
    if not ok:
        emit('toast', {'type': 'error', 'msg': f'경로 "{path_name}" 시작 실패'})


@socketio.on('stop_path')
def on_stop_path():
    """실행 중인 경로 즉시 중단."""
    nav = _get_or_create_navigator()
    if nav:
        nav.stop()
    elif _motor:
        _motor.stop()
    emit('toast', {'type': 'info', 'msg': '경로 실행 중단'})


# =============================================================================
# HTTP 라우트
# =============================================================================
@app.route('/')
@app.route('/dashboard.html')
def index():
    # Flask가 template_folder='templates' 기준으로 index.html 렌더
    # → ~/insite/app/templates/index.html
    return render_template('index.html')

# =============================================================================
# 정상 종료 핸들러
# 문제: daemon 스레드가 stdout 락을 잡은 채 인터프리터가 종료되면
#       "could not acquire lock for BufferedWriter" 오류 발생
# 해결: SIGINT/SIGTERM 수신 시 하드웨어를 먼저 정리하고 os._exit(0)으로 즉시 종료
# =============================================================================
def _cleanup(signum, frame):
    print("\n[SHUTDOWN] 정리 중...")
    try:
        if _motor: _motor.stop()
    except: pass
    try:
        if _led:   _led.off()
    except: pass
    try:
        if _ultra: _ultra.close()
    except: pass
    try:
        if _cam:   _cam.close()
    except: pass
    try:
        if _imu:   _imu.close()
    except: pass
    print("[SHUTDOWN] 완료")
    os._exit(0)   # daemon 스레드 stdout 락 오류 없이 즉시 종료

signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

# =============================================================================
# 진입점
# =============================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  INSITE Dashboard Server")
    print("  http://192.168.0.50:5001")
    print("  Camera : /video_feed (OV5647)")
    print("  UI     : ~/insite/app/templates/index.html")
    print("=" * 60)

    threading.Thread(target=_camera_loop,  daemon=True, name='cam').start()
    threading.Thread(target=_imu_fast_loop, daemon=True, name='imu').start()
    threading.Thread(target=_sensor_loop,   daemon=True, name='sensor').start()

    socketio.run(
        app,
        host='0.0.0.0',
        port=5001,
        debug=False,
        allow_unsafe_werkzeug=True,
    )