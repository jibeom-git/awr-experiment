# app/dashboard.py
# Flask 통합 대시보드
# - 실시간 RGB/깊이 카메라 스트리밍
# - 센서 수치 실시간 표시
# - 수동 조종 (키보드/버튼)
#
# 실행: python app/dashboard.py
# 접속: http://192.168.0.50:5000

import sys, os, time, threading, copy
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template, jsonify, request
import cv2
import numpy as np

app = Flask(__name__, template_folder='templates', static_folder='static')

# ══════════════════════════════════════════════════════
# 로드셀 설정값
# ══════════════════════════════════════════════════════
# 상단 설정값
HX711_REF_UNIT   = -188.72
HX711_DEADZONE   = 10.0
HX711_SMOOTH     = 20
HX711_TARE_EVERY = 300
# ══════════════════════════════════════════════════════
# 센서 초기화 (각각 독립적으로 try-except)
# ══════════════════════════════════════════════════════

# ── RGB 카메라 ────────────────────────────────────────
try:
    from sensors.camera import Camera
    cam = Camera(width=640, height=480)
    CAM_AVAILABLE = True
    print("[OK]   RGB 카메라")
except Exception as e:
    cam = None
    CAM_AVAILABLE = False
    print(f"[SKIP] RGB 카메라: {e}")

# ── Astra 깊이 카메라 ─────────────────────────────────
try:
    from sensors.astra import AstraCamera
    astra = AstraCamera()
    ASTRA_AVAILABLE = True
    print("[OK]   Astra 깊이 카메라")
except Exception as e:
    astra = None
    ASTRA_AVAILABLE = False
    print(f"[SKIP] Astra: {e}")

# ── MPU-6050 IMU ──────────────────────────────────────
try:
    from sensors.mpu6050 import MPU6050
    from tests.calibration_gyro import calibrate_gyro
    imu = MPU6050()
    # IMU_AVAILABLE = True
    # print("[OK]   MPU-6050")
    # 자동 gyro calibration
    imu.gyro_offset = calibrate_gyro(imu)

    print("Loaded gyro offset:")
    print(imu.gyro_offset)

    IMU_AVAILABLE = True
    print("[OK]   MPU-6050")
except Exception as e:
    imu = None
    IMU_AVAILABLE = False
    print(f"[SKIP] MPU-6050: {e}")

# ── HX711 로드셀 ──────────────────────────────────────
try:
    from sensors.hx711 import HX711
    hx711 = HX711()
    time.sleep(1)             # 초기 안정화
    hx711.REF_UNIT_A = HX711_REF_UNIT
    print("  바구니를 로드셀 위에 올려주세요 (영점 설정 중...)")
    hx711.tare(samples=50)   # 샘플 50회로 정밀도 향상
    HX711_AVAILABLE = True
    print("[OK]   HX711 로드셀")
except Exception as e:
    hx711 = None
    HX711_AVAILABLE = False
    print(f"[SKIP] HX711: {e}")
# ── 초음파 ────────────────────────────────────────────
try:
    import warnings
    from gpiozero import DistanceSensor
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
    ULTRA_AVAILABLE = True
    print("[OK]   초음파 HC-SR04")
except Exception as e:
    ultra = None
    ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파: {e}")

# ── LED ───────────────────────────────────────────────
try:
    from sensors.led import LEDController
    led = LEDController()
    led.set_running()
    LED_AVAILABLE = True
    print("[OK]   LED")
except Exception as e:
    led = None
    LED_AVAILABLE = False
    print(f"[SKIP] LED: {e}")

# ── 모터 ──────────────────────────────────────────────
try:
    from sensors.motor import MotorController
    motor = MotorController()
    MOTOR_AVAILABLE = True
    print("[OK]   모터")
except Exception as e:
    motor = None
    MOTOR_AVAILABLE = False
    print(f"[SKIP] 모터: {e}")

print()
print("┌─────────────────────────────┐")
print("│       센서 상태 요약         │")
print("├─────────────────────────────┤")
print(f"│ RGB 카메라  : {'✓ OK  ' if CAM_AVAILABLE   else '✗ SKIP'} │")
print(f"│ Astra 깊이  : {'✓ OK  ' if ASTRA_AVAILABLE else '✗ SKIP'} │")
print(f"│ MPU-6050    : {'✓ OK  ' if IMU_AVAILABLE   else '✗ SKIP'} │")
print(f"│ HX711       : {'✓ OK  ' if HX711_AVAILABLE else '✗ SKIP'} │")
print(f"│ 초음파      : {'✓ OK  ' if ULTRA_AVAILABLE else '✗ SKIP'} │")
print(f"│ LED         : {'✓ OK  ' if LED_AVAILABLE   else '✗ SKIP'} │")
print(f"│ 모터        : {'✓ OK  ' if MOTOR_AVAILABLE else '✗ SKIP'} │")
print("└─────────────────────────────┘")
print()

# ══════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════
lock         = threading.Lock()
latest_rgb   = None
latest_depth = None
latest_telem = {
    'accel':    {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'gyro':     {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'weight':   0.0,
    'distance': 0.0,
    'motor':    {'cmd': 'stop', 'speed': 0},
}

# ══════════════════════════════════════════════════════
# 로드셀 이동평균 버퍼 (값 안정화용)
# ══════════════════════════════════════════════════════
_weight_buf   = []   # 이동평균 버퍼
_tare_counter = 0    # 자동 재영점 카운터

def _read_weight_stable():
    global _weight_buf, _tare_counter

    raw = hx711.get_grams()

    # ── 이상값 필터 (버퍼 평균에서 50g 이상 벗어나면 무시) ──
    if len(_weight_buf) >= 3:
        current_avg = sum(_weight_buf) / len(_weight_buf)
        if abs(raw - current_avg) > 50:
            raw = current_avg  # 튀는 값 무시

    # ── 이동평균 (HX711_SMOOTH 샘플) ──────────────────
    _weight_buf.append(raw)
    if len(_weight_buf) > HX711_SMOOTH:
        _weight_buf.pop(0)
    avg = sum(_weight_buf) / len(_weight_buf)

    # ── 자동 재영점 (크리프 방지) ──────────────────────
    # 무게가 없는 상태(데드존 이내)일 때만 재영점 시도
    _tare_counter += 1
    if _tare_counter >= HX711_TARE_EVERY:
        _tare_counter = 0
        if abs(avg) < HX711_DEADZONE * 3:
            # 버퍼 초기화 후 재영점
            _weight_buf.clear()
            hx711.tare(samples=10)
            print("[HX711] 자동 재영점 완료 (크리프 보정)")

    # ── 데드존 적용 ────────────────────────────────────
    if abs(avg) <= HX711_DEADZONE:
        return 0.0
    return round(avg, 1)

# ══════════════════════════════════════════════════════
# 백그라운드 스레드
# ══════════════════════════════════════════════════════

def rgb_loop():
    global latest_rgb
    if not CAM_AVAILABLE:
        return
    while True:
        try:
            frame = cam.capture()
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_rgb = jpeg.tobytes()
        except Exception as e:
            print(f"[RGB] {e}")
            time.sleep(1)


def depth_loop():
    global latest_depth
    if not ASTRA_AVAILABLE:
        return
    while True:
        try:
            depth = astra.get_depth_frame()
            if depth is None:
                time.sleep(0.05)
                continue
            depth_vis  = np.clip(depth, 100, 1000).astype(np.float32)
            depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colormap   = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cy, cx     = depth.shape[0] // 2, depth.shape[1] // 2
            dist_c     = int(depth[cy, cx])
            cv2.putText(colormap, f"{dist_c}mm", (cx - 40, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.circle(colormap, (cx, cy), 5, (255, 255, 255), -1)
            _, jpeg = cv2.imencode('.jpg', colormap, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_depth = jpeg.tobytes()
        except Exception as e:
            print(f"[Depth] {e}")
            time.sleep(1)


def telem_loop():
    print("[Telem] 스레드 시작")
    while True:
        update = {}

        # ── IMU ───────────────────────────────────────
        if IMU_AVAILABLE:
            try:
                update['accel'] = imu.get_accel()
                update['gyro']  = imu.get_gyro()
            except Exception as e:
                print(f"[Telem/IMU] {e}")

        # ── 로드셀 ────────────────────────────────────
        if HX711_AVAILABLE:
            try:
                update['weight'] = _read_weight_stable()
            except Exception as e:
                print(f"[Telem/HX711] {e}")

        # ── 초음파 ────────────────────────────────────
        if ULTRA_AVAILABLE:
            try:
                d = ultra.distance
                update['distance'] = round(d * 100, 2) if d is not None else -1
            except Exception as e:
                print(f"[Telem/Ultra] {e}")

        if update:
            with lock:
                latest_telem.update(update)

        time.sleep(0.05)   # 20Hz — HX711_SMOOTH=8이면 약 0.4초 평균


# 스레드 시작
threading.Thread(target=rgb_loop,   daemon=True, name="rgb").start()
threading.Thread(target=depth_loop, daemon=True, name="depth").start()
threading.Thread(target=telem_loop, daemon=True, name="telem").start()

# ══════════════════════════════════════════════════════
# Flask 라우트
# ══════════════════════════════════════════════════════

def gen_stream(get_frame_fn):
    while True:
        with lock:
            frame = get_frame_fn()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/rgb_feed')
def rgb_feed():
    return Response(
        gen_stream(lambda: latest_rgb),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/depth_feed')
def depth_feed():
    return Response(
        gen_stream(lambda: latest_depth),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/telemetry')
def telemetry():
    with lock:
        data = copy.deepcopy(latest_telem)
    return jsonify(data)


@app.route('/status')
def status():
    return jsonify({
        'cam':   CAM_AVAILABLE,
        'astra': ASTRA_AVAILABLE,
        'imu':   IMU_AVAILABLE,
        'hx711': HX711_AVAILABLE,
        'ultra': ULTRA_AVAILABLE,
        'motor': MOTOR_AVAILABLE,
        'led':   LED_AVAILABLE,
    })


@app.route('/tare', methods=['POST'])
def tare():
    """수동 재영점 엔드포인트 — 대시보드 버튼에서 호출 가능"""
    if not HX711_AVAILABLE:
        return jsonify({'status': 'error', 'msg': 'hx711 not available'})
    try:
        global _weight_buf
        _weight_buf.clear()
        hx711.tare(samples=20)
        return jsonify({'status': 'ok', 'msg': '영점 완료'})
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)})


@app.route('/control', methods=['POST'])
def control():
    if not MOTOR_AVAILABLE:
        return jsonify({'status': 'error', 'msg': 'motor not available'})

    data  = request.get_json(force=True)
    cmd   = data.get('cmd', 'stop')
    speed = int(data.get('speed', 50))

    print(f"[CONTROL] {cmd} speed={speed}")

    try:
        if cmd == 'forward':
            motor.forward(speed)
        elif cmd == 'backward':
            motor.backward(speed)
        elif cmd == 'left':
            motor.set_motor(1, -1, speed)
            motor.set_motor(2,  1, speed)
            motor.set_motor(3, -1, speed)
            motor.set_motor(4,  1, speed)
        elif cmd == 'right':
            motor.set_motor(1,  1, speed)
            motor.set_motor(2, -1, speed)
            motor.set_motor(3,  1, speed)
            motor.set_motor(4, -1, speed)
        elif cmd == 'stop':
            motor.stop()
            if LED_AVAILABLE:
                led.set_running()
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)})

    with lock:
        latest_telem['motor'] = {'cmd': cmd, 'speed': speed}

    return jsonify({'status': 'ok', 'cmd': cmd})


if __name__ == '__main__':
    print("대시보드 시작: http://192.168.0.50:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)
