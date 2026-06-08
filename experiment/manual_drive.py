#!/usr/bin/env python3
# experiment/manual_drive.py
# 고도화된 수동 조종 + 실시간 데이터 수집 마스터 서버 (포트 5001)

import os
import sys
import csv
import time
import math
import signal
import subprocess
import threading
from pathlib import Path
from datetime import datetime
from typing import Any

# insite 루트를 주입하여 하드웨어 및 설정 파일 임포트 보장
INSITE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(INSITE_ROOT))

from flask import Flask, render_template, jsonify, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS

from sensors.motor import MotorController
from ai_core import config  # 전역 임계값 레지스터 연동

_motor = MotorController()
print("[OK] 하드웨어 모터 드라이버 인스턴스 초기화 완료")

# ── [안전망] 하드웨어 센서 동적 타입 명시 (Pylance 경고 완전 해결) ──────────
_imu: Any   = None
_hx711: Any = None
_ultra: Any = None

try:
    from sensors.mpu6050 import MPU6050
    _imu = MPU6050(bus_id=5, address=0x68)
    print("[OK] MPU6050 IMU 센서")
except Exception as e:
    print(f"[SKIP] MPU6050 IMU 가상 에뮬레이터 구동: {e}")

try:
    from sensors.hx711 import HX711
    _hx711 = HX711(dout=5, pd_sck=6)
    print("[OK] HX711 로드셀 센서")
except Exception as e:
    print(f"[SKIP] HX711 로드셀 가상 에뮬레이터 구동: {e}")

try:
    from sensors.ultra import UltrasonicSensor
    _ultra = UltrasonicSensor()
    print("[OK] HC-SR04 초음파 센서")
except Exception as e:
    print(f"[SKIP] 초음파 센서 가상 에뮬레이터 구동: {e}")

# ── 웹캠 멀티 스레드 버퍼 파이프라인 ─────────────────────────────────────────
cv2: Any = None
_camera: Any = None
_frame_lock = threading.Lock()
_latest_frame: Any = None

def _find_usb_webcam_path():
    try:
        proc = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, timeout=3)
        output = proc.stdout
    except Exception:
        return None, None

    device_name = None
    current_block_is_usb = False
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            current_block_is_usb = False
            device_name = None
            continue
        if not line.startswith(("\t", " ")):
            upper = stripped.upper()
            if "USB" in upper or "PC CAMERA" in upper:
                current_block_is_usb = True
                device_name = stripped.split("(")[0].strip().rstrip(":")
            else:
                current_block_is_usb = False
                device_name = None
        elif current_block_is_usb and stripped.startswith("/dev/video"):
            return stripped, device_name
    return None, None

try:
    import cv2 as _cv2_module
    cv2 = _cv2_module
    _webcam_opened = False

    _usb_path, _usb_name = _find_usb_webcam_path()
    if _usb_path is not None:
        _cap = cv2.VideoCapture(_usb_path)
        if _cap.isOpened():
            _cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            _cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            _camera = _cap
            print(f"[OK] USB 웹캠 탐지 성공: {_usb_path} ({_usb_name})")
            _webcam_opened = True
        else:
            _cap.release()

    if not _webcam_opened:
        for _fallback in ["/dev/video1", "/dev/video2", "/dev/video3"]:
            _cap = cv2.VideoCapture(_fallback)
            if _cap.isOpened():
                _cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                _cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                _camera = _cap
                print(f"[OK] USB 웹캠 폴백 연결 성공: {_fallback}")
                _webcam_opened = True
                break
            _cap.release()
except ImportError:
    print("[SKIP] OpenCV 라이브러리 부재로 비디오 기능 비활성화")

# ── Flask 및 웹 서비스 레이어 ────────────────────────────────────────────────
TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
app = Flask(__name__, template_folder=str(TEMPLATE_DIR))
app.config['SECRET_KEY'] = 'insite_core_data_2026'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

DATA_DIR = Path(__file__).resolve().parent / "data"
DATA_DIR.mkdir(exist_ok=True)
CSV_PATH = DATA_DIR / "raw_experiment.csv"
CSV_HEADER = ["timestamp", "label", "speed_cmd", "pitch", "weight",
              "sonic", "accel_x", "accel_y", "accel_z", "pitch_delta", "result"]

if not CSV_PATH.exists():
    with open(CSV_PATH, "w", newline="") as f:
        csv.writer(f).writerow(CSV_HEADER)

# ── 고도화된 전역 동기화 레지스터 ──────────────────────────────────────────────
_lock = threading.Lock()

_sensor = {
    "pitch":    0.0,
    "weight":   0.0,
    "sonic":    200.0,
    "accel_x":  0.0,
    "accel_y":  0.0,
    "accel_z":  0.0,
}
_prev_pitch = 0.0
_pitch_delta = 0.0
_prev_accel_z = 0.0
_accel_z_delta = 0.0

# [재정의] 머신러닝 학습 타겟팅 코어 파라미터
_recording = False
_current_label = "정상 평지"    # trainer.py의 OBS_ENC와 일치하는 키셋으로 강제 제어
_current_result = "normal"      # 주행 세션 결과 마킹 (normal, hill_fail, cautious_pass, impact, slip)
_speed_cmd = 50
_user_mode = "SAFE"             # SAFE / FAST 명시적 추적 추가
_row_count = 0
_hx711_tared = False
_calibrating = False
_calib_result: dict = {}

# ── 센서 동기화 스레드 (10 Hz) ───────────────────────────────────────────────
def _sensor_loop():
    global _prev_pitch, _pitch_delta, _hx711_tared, _prev_accel_z, _accel_z_delta
    import random

    while True:
        # 1. IMU 연산
        try:
            if _imu is not None:
                data = _imu.get_all()
                pitch = float(data.get("pitch", 0.0))
                accel_x = float(data.get("accel_x", 0.0))
                accel_y = float(data.get("accel_y", 0.0))
                accel_z = float(data.get("accel_z", 0.0))
                
                if accel_x == 0.0 and accel_y == 0.0 and accel_z == 0.0:
                    accel = _imu.get_accel()
                    accel_x = round(float(accel["x"]) * 9.81, 3)
                    accel_y = round(float(accel["y"]) * 9.81, 3)
                    accel_z = round(float(accel["z"]) * 9.81, 3)
            else:
                # 하드웨어 없을 시 시뮬레이션 데이터 생성
                t = time.time()
                pitch = round(math.cos(t * 0.2) * 2.0, 2)
                accel_x = round(math.sin(t * 0.5) * 0.2, 3)
                accel_y = round(math.cos(t * 0.7) * 0.1, 3)
                accel_z = round(9.81 + math.sin(t * 1.2) * 0.1, 3)
        except Exception:
            pitch = accel_x = accel_y = 0.0
            accel_z = 9.81

        # 2. 로드셀 연산
        try:
            if _hx711 is not None:
                if not _hx711_tared:
                    _hx711.tare(samples=15)
                    _hx711_tared = True
                weight = round(float(_hx711.get_weight()), 1)
            else:
                weight = round(random.uniform(10, 30), 1)
        except Exception:
            weight = 0.0

        # 3. 초음파 거리 연산
        try:
            if _ultra is not None:
                d = _ultra.get_distance()
                sonic = float(d) if d and d > 0 else 400.0
            else:
                sonic = round(random.uniform(40, 250), 1)
        except Exception:
            sonic = 400.0

        # 4. 차분 벡터 연산 및 락 스왑
        with _lock:
            p_delta = round(pitch - _prev_pitch, 3)
            z_delta = round(accel_z - _prev_accel_z, 3)
            
            _prev_pitch = pitch
            _prev_accel_z = accel_z
            _pitch_delta = p_delta
            _accel_z_delta = z_delta
            
            _sensor["pitch"] = round(pitch, 2)
            _sensor["weight"] = max(0.0, weight)
            _sensor["sonic"] = round(sonic, 1)
            _sensor["accel_x"] = accel_x
            _sensor["accel_y"] = accel_y
            _sensor["accel_z"] = accel_z

        time.sleep(0.1)

# ── 고품질 CSV 인젝션 로거 스레드 (10 Hz) ─────────────────────────────────────
def _logging_loop():
    global _row_count
    while True:
        with _lock:
            if not _recording:
                time.sleep(0.1)
                continue
            snap = dict(_sensor)
            p_delta = _pitch_delta
            spd = _speed_cmd
            lbl = _current_label
            res = _current_result

        # 정형화된 시나리오 레코드 빌드
        row = [
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            lbl, spd, snap["pitch"], snap["weight"], snap["sonic"],
            snap["accel_x"], snap["accel_y"], snap["accel_z"], p_delta, res
        ]

        try:
            with open(CSV_PATH, "a", newline="") as f:
                csv.writer(f).writerow(row)
            with _lock:
                _row_count += 1
        except Exception as e:
            print(f"[CSV ERROR] 세션 인젝션 실패: {e}")

        time.sleep(0.1)

# ── 지능형 텔레메트리 및 오작동 진단 스레드 (1 Hz) ──────────────────────────────
def _broadcast_loop():
    while True:
        with _lock:
            snap = dict(_sensor)
            p_delta = _pitch_delta
            z_delta = _accel_z_delta
            rec = _recording
            rows = _row_count
            lbl = _current_label
            res = _current_result
            spd = _speed_cmd
            mode = _user_mode

        # 임계값 분석 기반의 실시간 품질 경보 필터
        alerts = []
        if abs(p_delta) > config.TH_PITCH_DELTA_FAIL:
            alerts.append("⚠ [위험] 등판 각도 한계 초과 ➡️ 토크 부족으로 인한 Slip/Fail 위험성 농후")
        if abs(z_delta) > config.TH_IMPACT_Z:
            alerts.append("⚠ [감지] Z축 가속도 서지 발생 ➡️ 방지턱 충격 이벤트가 동기화 세션에 수집 중")
        if spd > 40 and abs(snap["accel_x"]) < config.TH_SLIP_ACCEL:
            alerts.append("⚠ [주의] 모터 고출력 대비 전진 가속도 미동 탐지 ➡️ 구동 바퀴 공회전(Slip) 의심")

        socketio.emit("sensor_update", {
            "pitch": snap["pitch"], "weight": snap["weight"], "sonic": snap["sonic"],
            "accel_x": snap["accel_x"], "accel_y": snap["accel_y"], "accel_z": snap["accel_z"],
            "pitch_delta": p_delta, "accel_z_delta": z_delta, "recording": rec,
            "row_count": rows, "label": lbl, "result": res, "speed_cmd": spd,
            "user_mode": mode, "alerts": alerts
        })
        time.sleep(1.0)

# ── [3단계 통합 교정] 하드웨어 캘리브레이션 ─────────────────────────────────────
def _run_calibration():
    global _calibrating, _calib_result, _hx711_tared
    res = {}

    # 1단계 : 로드셀 교정
    socketio.emit("calibration_status", {"msg": "1/3 [캘리브레이션] 로드셀 센서 영점 정렬 중... 화물을 내려주세요."})
    try:
        if _hx711 is not None:
            _hx711.tare(samples=20)
            _hx711_tared = True
            res["hx711_tare"] = "SUCCESS (20샘플 고해상도 평균 필터 완료)"
        else:
            res["hx711_tare"] = "MOCK_OK"
    except Exception as e:
        res["hx711_tare"] = f"FAIL: {e}"
    time.sleep(0.6)

    # 2단계 : IMU 자세 수평 정렬
    socketio.emit("calibration_status", {"msg": "2/3 [캘리브레이션] MPU-6050 수평 가이드 피팅 중... AGV를 평지에 거치해 주세요."})
    try:
        if _imu is not None:
            if hasattr(_imu, "init_chassis_pitch_calibration"):
                _imu.init_chassis_pitch_calibration(times=20)
            else:
                _imu.pitch_zero_bias = 0.0
            res["imu_pitch_zero"] = "SUCCESS (섀시 수평 오프셋 레지스터 고정 완료)"
        else:
            res["imu_pitch_zero"] = "MOCK_OK"
    except Exception as e:
        res["imu_pitch_zero"] = f"FAIL: {e}"
    time.sleep(0.6)

    # 3단계 : 초음파 센서 정렬
    socketio.emit("calibration_status", {"msg": "3/3 [캘리브레이션] 초음파 센서 반사파 세기 스캔 중..."})
    try:
        if _ultra is not None:
            buf = []
            for _ in range(5):
                d = _ultra.get_distance()
                if d and d > 0: buf.append(float(d))
                time.sleep(0.1)
            res["sonic_baseline_cm"] = round(sum(buf)/len(buf), 1) if buf else 400.0
        else:
            res["sonic_baseline_cm"] = 200.0
    except Exception as e:
        res["sonic_baseline_cm"] = f"FAIL: {e}"

    with _lock:
        _calibrating = False
        _calib_result = res
    socketio.emit("calibration_done", res)

# ── 카메라 공유 프레임 파이프라인 스레드 ────────────────────────────────────────
def _camera_loop():
    global _latest_frame
    while True:
        if cv2 is None or _camera is None or not _camera.isOpened():
            time.sleep(0.1)
            continue
        _camera.read()
        ret, frame = _camera.read()
        if ret:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            if ok:
                with _frame_lock:
                    _latest_frame = buf.tobytes()
        time.sleep(0.033)

def _gen_frames():
    while True:
        with _frame_lock:
            f_data = _latest_frame
        if f_data is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f_data + b"\r\n")
        time.sleep(0.05)

# ── 웹앱 엔드포인트 라우팅 ───────────────────────────────────────────────────
@app.route("/")
def index(): return render_template("drive.html")

@app.route("/status")
def status():
    with _lock:
        return jsonify({"recording": _recording, "row_count": _row_count, "label": _current_label, "speed_cmd": _speed_cmd, "user_mode": _user_mode})

@app.route("/video_feed")
def video_feed():
    if cv2 is None or _camera is None:
        return Response("카메라 드라이버를 로드할 수 없습니다.", status=503, mimetype="text/plain")
    return Response(_gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ── 소켓 엔드포인트 이벤트 인터페이스 ──────────────────────────────────────────
@socketio.on("connect")
def on_connect():
    with _lock:
        emit("sensor_update", {
            "pitch": _sensor["pitch"], "weight": _sensor["weight"], "sonic": _sensor["sonic"],
            "accel_x": _sensor["accel_x"], "accel_y": _sensor["accel_y"], "accel_z": _sensor["accel_z"],
            "pitch_delta": _pitch_delta, "accel_z_delta": _accel_z_delta, "recording": _recording,
            "row_count": _row_count, "label": _current_label, "result": _current_result,
            "speed_cmd": _speed_cmd, "user_mode": _user_mode, "alerts": []
        })

@socketio.on("motor_cmd")
def on_motor_cmd(data):
    global _speed_cmd
    direction = str(data.get("direction", "stop"))
    speed = max(0, min(100, int(data.get("speed", 50))))

    with _lock: _speed_cmd = speed
    try:
        if direction == "forward": _motor.forward(speed)
        elif direction == "backward": _motor.backward(speed)
        elif direction == "left": _motor.rotate_left(speed)
        elif direction == "right": _motor.rotate_right(speed)
        else: _motor.stop()
    except Exception as e:
        print(f"[MOTOR ERR] 제어 명령 거부: {e}")

@socketio.on("start_calibration")
def on_start_calibration():
    global _calibrating
    with _lock:
        if _calibrating: return
        _calibrating = True
    threading.Thread(target=_run_calibration, daemon=True, name="calibration").start()

@socketio.on("set_label")
def on_set_label(data):
    global _current_label
    lbl = str(data.get("label", "정상 평지"))
    with _lock: _current_label = lbl
    emit("label_ack", {"label": lbl})

@socketio.on("set_result")
def on_set_result(data):
    global _current_result
    res = str(data.get("result", "normal"))
    with _lock: _current_result = res
    emit("result_ack", {"result": res})

# 사용자가 조종 모드(SAFE/FAST)를 바꿨을 때 명시적으로 가로채서 학습 인코딩 정확도 상승 보조
@socketio.on("set_mode")
def on_set_mode(data):
    global _user_mode
    mode = str(data.get("mode", "SAFE")).upper()
    with _lock: _user_mode = mode
    print(f"[CONFIG] 데이터 수집 모드 변경 ➡️ {mode}")

@socketio.on("start_recording")
def on_start_recording():
    global _recording
    with _lock: _recording = True
    emit("recording_state", {"recording": True})

@socketio.on("stop_recording_with_result")
def on_stop_recording_with_result(data):
    global _recording, _row_count
    res = str(data.get("result", "normal"))

    with _lock:
        _recording = False
        row_count = _row_count

    time.sleep(0.15)
    try: _motor.stop()
    except Exception: pass

    # 수집 완료된 파일 세션의 마지막 데이터 블록 전체 소급 타겟 갱신 적용
    if row_count > 0:
        try:
            with open(CSV_PATH, "r", newline="") as f:
                all_rows = list(csv.reader(f))
            result_col = CSV_HEADER.index("result")
            start = max(1, len(all_rows) - row_count)
            for i in range(start, len(all_rows)):
                if len(all_rows[i]) > result_col:
                    all_rows[i][result_col] = res
            with open(CSV_PATH, "w", newline="") as f:
                csv.writer(f).writerows(all_rows)
            print(f"[DATA LOG] 데이터 동기화 완료: [Result ➡️ {res}] 수집 볼륨: {row_count}행")
        except Exception as e:
            print(f"[CSV BACKFILL ERROR] 소급 덮어쓰기 에러: {e}")

    with _lock: _row_count = 0
    emit("recording_state", {"recording": False})

# ── 안전한 프로세스 리소스 셧다운 구조 ─────────────────────────────────────────
def _cleanup(signum, frame):
    print("\n[SHUTDOWN] 하드웨어 자원 분배 해제 프로세스 개시...")
    try: _motor.stop()
    except Exception: pass
    if _camera is not None:
        try: _camera.release()
        except Exception: pass
    print("[SHUTDOWN] 시스템 안전 종료 완료.")
    os._exit(0)

signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

if __name__ == "__main__":
    print("=" * 60)
    print("  INSITE AI 코어 최적화 고품질 데이터 수집 허브 가동")
    print("  접속 엔드포인트: http://192.168.0.50:5001")
    print(f"  타겟 스토리지: {CSV_PATH}")
    print("=" * 60)

    threading.Thread(target=_sensor_loop,    daemon=True, name="sensor").start()
    threading.Thread(target=_logging_loop,   daemon=True, name="logger").start()
    threading.Thread(target=_broadcast_loop, daemon=True, name="broadcast").start()
    threading.Thread(target=_camera_loop,    daemon=True, name="camera").start()

    socketio.run(app, host="0.0.0.0", port=5001, debug=False, allow_unsafe_werkzeug=True)