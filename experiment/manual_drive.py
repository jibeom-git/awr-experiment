#!/usr/bin/env python3
# experiment/manual_drive.py
# 수동 조종 + 실시간 센서 로깅 메인 서버 (포트 5001)
# XGBoost/Isolation Forest 학습용 실험 데이터 수집 전용

import os
import sys
import csv
import time
import math
import signal
import threading
from pathlib import Path
from datetime import datetime

# insite 루트를 sys.path 에 추가하여 sensors/* 및 Move import 가능하게
INSITE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(INSITE_ROOT))

from flask import Flask, render_template, jsonify, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# ─────────────────────────────────────────────────────────────────────────────
# [수정 1] 모터 드라이버: sensors/motor.py → Move.py 직접 import
# ─────────────────────────────────────────────────────────────────────────────

_move_ok = False  # Move.py 로드 성공 플래그
try:
    import Move as move   # ~/insite/Move.py 직접 사용
    move.setup()          # GPIO 핀 초기화
    _move_ok = True
    print("[INIT] Move (모터 드라이버) OK")
except Exception as e:
    print(f"[INIT] Move FAIL (mock 모드): {e}")

# ─────────────────────────────────────────────────────────────────────────────
# 나머지 센서 드라이버 로드 (하드웨어 없으면 자동 mock 전환)
# ─────────────────────────────────────────────────────────────────────────────

_imu   = None
_hx711 = None
_ultra = None

try:
    from sensors.mpu6050 import MPU6050
    _imu = MPU6050(bus_id=5, address=0x68)
    print("[INIT] MPU6050 OK")
except Exception as e:
    print(f"[INIT] MPU6050 FAIL (mock): {e}")

try:
    from sensors.hx711 import HX711
    _hx711 = HX711(dout=5, pd_sck=6)
    print("[INIT] HX711 OK")
except Exception as e:
    print(f"[INIT] HX711 FAIL (mock): {e}")

try:
    from sensors.ultra import UltrasonicSensor
    _ultra = UltrasonicSensor()
    print("[INIT] Ultrasonic OK")
except Exception as e:
    print(f"[INIT] Ultrasonic FAIL (mock): {e}")

# ─────────────────────────────────────────────────────────────────────────────
# [수정 4] 웹캠 초기화 (OpenCV MJPEG 스트리밍용)
# 실패하거나 카메라 없으면 _camera = None 유지, 서버는 계속 동작
# ─────────────────────────────────────────────────────────────────────────────

cv2 = None          # OpenCV 모듈 레퍼런스 (import 성공 시만 할당)
_camera = None      # VideoCapture 객체
_frame_lock = threading.Lock()    # 최신 프레임 버퍼 보호 락
_latest_frame = None              # 카메라 루프가 갱신하는 JPEG 바이트 버퍼

try:
    import cv2 as _cv2_module
    cv2 = _cv2_module
    # /dev/video0 부터 순서대로 탐색
    for _cam_idx in range(4):
        _cap = cv2.VideoCapture(_cam_idx)
        if _cap.isOpened():
            _camera = _cap
            print(f"[INIT] 웹캠 OK (/dev/video{_cam_idx})")
            break
        _cap.release()
    if _camera is None:
        print("[INIT] 웹캠 없음 (0~3번 장치 탐색 실패)")
except ImportError:
    print("[INIT] OpenCV 없음 — 웹캠 비활성화")
except Exception as e:
    print(f"[INIT] 웹캠 초기화 실패: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# Flask / SocketIO
# ─────────────────────────────────────────────────────────────────────────────

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
app = Flask(__name__, template_folder=str(TEMPLATE_DIR))
app.config['SECRET_KEY'] = 'insite_exp_2025'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ─────────────────────────────────────────────────────────────────────────────
# CSV 경로 설정
# ─────────────────────────────────────────────────────────────────────────────

DATA_DIR = Path(__file__).resolve().parent / "data"
DATA_DIR.mkdir(exist_ok=True)
CSV_PATH = DATA_DIR / "raw_experiment.csv"
CSV_HEADER = ["timestamp", "label", "speed_cmd", "pitch", "weight",
              "sonic", "accel_x", "accel_y", "accel_z", "pitch_delta", "result"]

# 파일이 없으면 헤더 작성
if not CSV_PATH.exists():
    with open(CSV_PATH, "w", newline="") as f:
        csv.writer(f).writerow(CSV_HEADER)

# ─────────────────────────────────────────────────────────────────────────────
# 전역 상태 레지스터 (모든 스레드 공유)
# ─────────────────────────────────────────────────────────────────────────────

_lock = threading.Lock()

# 센서 캐시
_sensor = {
    "pitch":    0.0,
    "weight":   0.0,
    "sonic":    200.0,
    "accel_x":  0.0,
    "accel_y":  0.0,
    "accel_z":  0.0,
}
_prev_pitch    = 0.0    # pitch_delta 계산용 이전 pitch
_pitch_delta   = 0.0
# [수정 2] Z축 스파이크 감지: 이전 샘플 대비 순간 변화량 추적
_prev_accel_z  = 0.0   # 이전 샘플의 accel_z (순간 변화량 계산용)
_accel_z_delta = 0.0   # accel_z 순간 변화량 (브로드캐스트 포함)

# 기록/조종 상태
_recording      = False
_current_label  = "정상 평지"
_current_result = "normal"  # 결과 마킹 버튼이 덮어씀
_speed_cmd      = 50        # 현재 속도 명령 (0~100)
_row_count      = 0         # 이번 세션에서 기록된 행 수

# HX711 영점 완료 플래그
_hx711_tared = False

# [수정 3] 교정 상태 전역 변수
_calibrating  = False  # 교정 진행 중 여부
_calib_result = {}     # 교정 완료 결과 딕셔너리

# ─────────────────────────────────────────────────────────────────────────────
# 백그라운드 스레드: 센서 폴링 (10 Hz)
# ─────────────────────────────────────────────────────────────────────────────

def _sensor_loop():
    global _prev_pitch, _pitch_delta, _hx711_tared
    global _prev_accel_z, _accel_z_delta  # [수정 2] Z축 변화량 계산용
    import random

    while True:
        # ── MPU-6050 ─────────────────────────────────────────────────────────
        try:
            if _imu is not None:
                data = _imu.get_all()
                pitch   = float(data.get("pitch", 0.0))
                accel_x = float(data.get("accel_x", 0.0)) if "accel_x" in data else 0.0
                accel_y = float(data.get("accel_y", 0.0)) if "accel_y" in data else 0.0
                accel_z = float(data.get("accel_z", 0.0)) if "accel_z" in data else 0.0
                # get_all()이 accel을 직접 안 줄 경우 get_accel() 호출
                if accel_x == 0.0 and accel_y == 0.0 and accel_z == 0.0:
                    try:
                        accel = _imu.get_accel()
                        accel_x = round(float(accel["x"]) * 9.81, 3)
                        accel_y = round(float(accel["y"]) * 9.81, 3)
                        accel_z = round(float(accel["z"]) * 9.81, 3)
                    except Exception:
                        pass
            else:
                t = time.time()
                pitch   = round(math.cos(t * 0.2) * 3.0, 2)
                accel_x = round(math.sin(t * 0.5) * 0.3, 3)
                accel_y = round(math.cos(t * 0.7) * 0.2, 3)
                accel_z = round(9.81 + math.sin(t * 1.2) * 0.1, 3)
        except Exception as e:
            print(f"[SENSOR IMU] {e}")
            pitch = accel_x = accel_y = 0.0
            accel_z = 9.81

        # ── HX711 ────────────────────────────────────────────────────────────
        try:
            if _hx711 is not None:
                if not _hx711_tared:
                    _hx711.tare(samples=20)
                    _hx711_tared = True
                weight = round(float(_hx711.get_weight()), 1)
            else:
                weight = round(random.uniform(0, 50), 1)
        except Exception as e:
            print(f"[SENSOR HX711] {e}")
            weight = 0.0

        # ── HC-SR04 ──────────────────────────────────────────────────────────
        try:
            if _ultra is not None:
                sonic = _ultra.get_distance()
                sonic = float(sonic) if sonic and sonic > 0 else 400.0
            else:
                sonic = round(random.uniform(30, 300), 1)
        except Exception as e:
            print(f"[SENSOR Ultra] {e}")
            sonic = 400.0

        # ── pitch_delta 및 accel_z_delta 계산 ────────────────────────────────
        with _lock:
            delta   = round(pitch - _prev_pitch, 3)
            # [수정 2] 이전 샘플 대비 Z축 순간 변화량 (절대값 기준이 아님)
            z_delta = round(accel_z - _prev_accel_z, 3)
            _prev_pitch    = pitch
            _prev_accel_z  = accel_z
            _pitch_delta   = delta
            _accel_z_delta = z_delta
            _sensor["pitch"]   = round(pitch, 2)
            _sensor["weight"]  = weight
            _sensor["sonic"]   = round(sonic, 1)
            _sensor["accel_x"] = accel_x
            _sensor["accel_y"] = accel_y
            _sensor["accel_z"] = accel_z

        time.sleep(0.1)


# ─────────────────────────────────────────────────────────────────────────────
# 백그라운드 스레드: CSV 기록 (10 Hz, recording=True 일 때만)
# ─────────────────────────────────────────────────────────────────────────────

def _logging_loop():
    global _row_count, _current_result

    while True:
        with _lock:
            if not _recording:
                time.sleep(0.1)
                continue
            snap  = dict(_sensor)
            delta = _pitch_delta
            spd   = _speed_cmd
            lbl   = _current_label
            res   = _current_result

        row = [
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            lbl,
            spd,
            snap["pitch"],
            snap["weight"],
            snap["sonic"],
            snap["accel_x"],
            snap["accel_y"],
            snap["accel_z"],
            delta,
            res,
        ]

        try:
            with open(CSV_PATH, "a", newline="") as f:
                csv.writer(f).writerow(row)
            with _lock:
                _row_count += 1
        except Exception as e:
            print(f"[CSV] 기록 오류: {e}")

        time.sleep(0.1)


# ─────────────────────────────────────────────────────────────────────────────
# 백그라운드 스레드: SocketIO 브로드캐스트 (1 Hz)
# ─────────────────────────────────────────────────────────────────────────────

def _broadcast_loop():
    while True:
        with _lock:
            snap    = dict(_sensor)
            delta   = _pitch_delta
            z_delta = _accel_z_delta  # [수정 2] Z축 순간 변화량
            rec     = _recording
            rows    = _row_count
            lbl     = _current_label
            res     = _current_result
            spd     = _speed_cmd

        # 이상 감지 경보 로직
        alerts = []
        if abs(delta) > 3.0:
            alerts.append("⚠ pitch 급변 감지 (등판 실패 주의)")
        # [수정 2] Z축 스파이크: 절대값(9.81 기준) 대신 순간 변화량으로 판단
        #          정지 상태에서도 오경보가 발생하던 문제 수정
        if abs(z_delta) > 4.0:
            alerts.append("⚠ Z축 충격 스파이크 감지")
        # 슬립 의심: 속도 명령 높은데 가속도 변화 없음
        if spd > 40 and abs(snap["accel_x"]) < 0.05:
            alerts.append("⚠ 슬립 의심 (모터 명령 대비 가속도 불일치)")

        socketio.emit("sensor_update", {
            "pitch":         snap["pitch"],
            "weight":        snap["weight"],
            "sonic":         snap["sonic"],
            "accel_x":       snap["accel_x"],
            "accel_y":       snap["accel_y"],
            "accel_z":       snap["accel_z"],
            "pitch_delta":   delta,
            "accel_z_delta": z_delta,   # [수정 2] Z축 순간 변화량 추가
            "recording":     rec,
            "row_count":     rows,
            "label":         lbl,
            "result":        res,
            "speed_cmd":     spd,
            "alerts":        alerts,
        })
        time.sleep(1.0)


# ─────────────────────────────────────────────────────────────────────────────
# [수정 3] 교정 루틴: HX711 tare → MPU-6050 영점 → 초음파 기준 거리
# 별도 데몬 스레드에서 실행되므로 socketio.emit() (전역) 사용
# ─────────────────────────────────────────────────────────────────────────────

def _run_calibration():
    """
    3단계 순차 교정 수행:
      1단계 - HX711 영점(tare), 20회 샘플링
      2단계 - MPU-6050 pitch/roll 영점 (init_chassis_pitch_calibration 또는 수동 offset)
      3단계 - 초음파 5회 샘플링 평균으로 기준 거리 확인
    완료 후 calibration_done 이벤트를 브로드캐스트한다.
    """
    global _calibrating, _calib_result, _hx711_tared
    result = {}

    # ── 1단계: HX711 영점 ────────────────────────────────────────────────────
    socketio.emit("calibration_status", {"msg": "1/3 HX711 영점(tare) 중... 바구니를 비워주세요."})
    try:
        if _hx711 is not None:
            _hx711.tare(samples=20)  # 20회 평균으로 영점 기준 설정
            _hx711_tared = True
            result["hx711_tare"] = "완료 (20샘플 평균)"
        else:
            result["hx711_tare"] = "mock (하드웨어 없음)"
    except Exception as e:
        result["hx711_tare"] = f"실패: {e}"
    time.sleep(0.5)

    # ── 2단계: MPU-6050 pitch/roll 영점 ─────────────────────────────────────
    socketio.emit("calibration_status", {"msg": "2/3 IMU pitch/roll 영점 교정 중... 로봇을 수평에 놓아주세요."})
    try:
        if _imu is not None:
            if hasattr(_imu, "init_chassis_pitch_calibration"):
                # 기존 함수 사용 (내부에서 20회 샘플링 후 zero_bias 설정)
                _imu.init_chassis_pitch_calibration(times=20)
            else:
                # 함수 없을 경우: 현재 필터 각도를 offset으로 직접 저장
                angles = _imu.get_filtered_angles()
                _imu.pitch_zero_bias = angles.get("pitch", 0.0)
                _imu.roll_zero_bias  = angles.get("roll",  0.0)
            result["imu_pitch_zero"] = round(_imu.pitch_zero_bias, 3)
            result["imu_roll_zero"]  = round(_imu.roll_zero_bias,  3)
        else:
            result["imu_pitch_zero"] = "mock"
            result["imu_roll_zero"]  = "mock"
    except Exception as e:
        result["imu_pitch_zero"] = f"실패: {e}"
        result["imu_roll_zero"]  = f"실패: {e}"
    time.sleep(0.5)

    # ── 3단계: 초음파 5회 샘플링 기준 거리 ──────────────────────────────────
    socketio.emit("calibration_status", {"msg": "3/3 초음파 기준 거리 측정 중..."})
    try:
        if _ultra is not None:
            samples = []
            for _ in range(5):
                d = _ultra.get_distance()
                if d and d > 0:
                    samples.append(float(d))
                time.sleep(0.15)
            # 유효 샘플 평균 계산
            avg = round(sum(samples) / len(samples), 1) if samples else -1.0
            result["sonic_baseline_cm"] = avg
        else:
            import random
            result["sonic_baseline_cm"] = round(random.uniform(30, 200), 1)
    except Exception as e:
        result["sonic_baseline_cm"] = f"실패: {e}"
    time.sleep(0.3)

    # ── 교정 완료: 결과를 전역에 저장하고 브라우저에 브로드캐스트 ────────────
    with _lock:
        _calibrating  = False
        _calib_result = result
    socketio.emit("calibration_done", result)
    print(f"[CALIB] 교정 완료: {result}")


# ─────────────────────────────────────────────────────────────────────────────
# [수정 4] 웹캠 백그라운드 루프: 최신 프레임을 버퍼에 지속 갱신
# 여러 브라우저 클라이언트가 공유 버퍼에서 읽어 카메라 충돌 방지
# ─────────────────────────────────────────────────────────────────────────────

def _camera_loop():
    """카메라에서 프레임을 지속 읽어 JPEG 버퍼(_latest_frame)에 저장"""
    global _latest_frame
    while True:
        if cv2 is None or _camera is None or not _camera.isOpened():
            time.sleep(0.1)
            continue
        ret, frame = _camera.read()
        if ret:
            # JPEG 품질 70으로 인코딩 (대역폭/화질 균형)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with _frame_lock:
                    _latest_frame = buf.tobytes()
        time.sleep(0.05)  # ~20fps 상한


def _gen_frames():
    """MJPEG 멀티파트 스트림 제너레이터: 공유 버퍼에서 최신 프레임 반환"""
    while True:
        with _frame_lock:
            frame_data = _latest_frame
        if frame_data is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame_data + b"\r\n")
        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Flask 라우트
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("drive.html")


@app.route("/status")
def status():
    with _lock:
        return jsonify({
            "recording": _recording,
            "row_count": _row_count,
            "label":     _current_label,
            "speed_cmd": _speed_cmd,
        })


@app.route("/video_feed")
def video_feed():
    """[수정 4] 웹캠 MJPEG 스트리밍 엔드포인트. 카메라 없으면 503 반환."""
    if cv2 is None or _camera is None:
        return Response("카메라 없음", status=503, mimetype="text/plain")
    return Response(
        _gen_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ─────────────────────────────────────────────────────────────────────────────
# SocketIO 이벤트 핸들러
# ─────────────────────────────────────────────────────────────────────────────

@socketio.on("connect")
def on_connect():
    # 최초 접속 시 현재 센서 스냅샷 전송 (accel_z_delta 포함)
    with _lock:
        emit("sensor_update", {
            "pitch":         _sensor["pitch"],
            "weight":        _sensor["weight"],
            "sonic":         _sensor["sonic"],
            "accel_x":       _sensor["accel_x"],
            "accel_y":       _sensor["accel_y"],
            "accel_z":       _sensor["accel_z"],
            "pitch_delta":   _pitch_delta,
            "accel_z_delta": _accel_z_delta,  # [수정 2]
            "recording":     _recording,
            "row_count":     _row_count,
            "label":         _current_label,
            "result":        _current_result,
            "speed_cmd":     _speed_cmd,
            "alerts":        [],
        })


@socketio.on("motor_cmd")
def on_motor_cmd(data):
    """[수정 1] 방향 + 속도 명령 처리: Move.py API 사용
    move.move(speed, direction, turn)
      direction: 1(전진) / -1(후진)
      turn: "mid"(직진) / "left" / "right" / "rotate-left" / "rotate-right"
    """
    global _speed_cmd
    direction = str(data.get("direction", "stop"))
    speed     = int(data.get("speed", 50))
    speed     = max(0, min(100, speed))

    with _lock:
        _speed_cmd = speed

    if _move_ok:
        try:
            if direction == "forward":
                move.move(speed, 1, "mid")      # 전진 직진
            elif direction == "backward":
                move.move(speed, -1, "mid")     # 후진 직진
            elif direction == "left":
                move.move(speed, 1, "left")     # 좌회전
            elif direction == "right":
                move.move(speed, 1, "right")    # 우회전
            else:
                move.motorStop()                # 정지 (stop / space)
        except Exception as e:
            print(f"[MOTOR CMD] {e}")
    else:
        # Move.py 없을 때 mock 로그
        if direction != "stop":
            print(f"[MOTOR MOCK] {direction} @ {speed}%")


@socketio.on("start_calibration")
def on_start_calibration():
    """[수정 3] 교정 시작 이벤트: 중복 실행 방지 후 별도 스레드에서 교정 루틴 실행"""
    global _calibrating
    with _lock:
        if _calibrating:
            emit("calibration_status", {"msg": "이미 교정 진행 중입니다."})
            return
        _calibrating = True
    print("[CALIB] 교정 시작")
    threading.Thread(target=_run_calibration, daemon=True, name="calibration").start()
    emit("calibration_status", {"msg": "교정 시작..."})


@socketio.on("set_label")
def on_set_label(data):
    global _current_label
    label = str(data.get("label", "정상 평지"))
    with _lock:
        _current_label = label
    emit("label_ack", {"label": label})


@socketio.on("set_result")
def on_set_result(data):
    global _current_result
    result = str(data.get("result", "normal"))
    with _lock:
        _current_result = result
    emit("result_ack", {"result": result})


@socketio.on("start_recording")
def on_start_recording():
    global _recording
    with _lock:
        _recording = True
    print("[LOG] 기록 시작")
    emit("recording_state", {"recording": True})


@socketio.on("stop_recording")
def on_stop_recording():
    global _recording
    with _lock:
        _recording = False
    # [수정 1] 기록 정지 시 모터도 정지 (Move.motorStop 사용)
    if _move_ok:
        try:
            move.motorStop()
        except Exception:
            pass
    print("[LOG] 기록 정지")
    emit("recording_state", {"recording": False})


# ─────────────────────────────────────────────────────────────────────────────
# 종료 핸들러
# ─────────────────────────────────────────────────────────────────────────────

def _cleanup(signum, frame):
    print("\n[SHUTDOWN] 하드웨어 자원 반환 중...")
    # [수정 1] MotorController.stop() → move.motorStop()
    if _move_ok:
        try:
            move.motorStop()
        except Exception:
            pass
    if _imu is not None:
        try:
            _imu.close()
        except Exception:
            pass
    if _ultra is not None:
        try:
            _ultra.close()
        except Exception:
            pass
    # [수정 4] 웹캠 자원 반환
    if _camera is not None:
        try:
            _camera.release()
        except Exception:
            pass
    print("[SHUTDOWN] 완료.")
    os._exit(0)


signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

# ─────────────────────────────────────────────────────────────────────────────
# 진입점
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  INSITE 수동 조종 데이터 수집 서버")
    print("  URL : http://192.168.0.50:5001")
    print(f"  CSV : {CSV_PATH}")
    print("=" * 60)

    threading.Thread(target=_sensor_loop,    daemon=True, name="sensor").start()
    threading.Thread(target=_logging_loop,   daemon=True, name="logger").start()
    threading.Thread(target=_broadcast_loop, daemon=True, name="broadcast").start()
    # [수정 4] 웹캠 프레임 갱신 스레드 (카메라 없으면 idle 상태로 대기)
    threading.Thread(target=_camera_loop,    daemon=True, name="camera").start()

    socketio.run(
        app,
        host="0.0.0.0",
        port=5001,
        debug=False,
        allow_unsafe_werkzeug=True,
    )
