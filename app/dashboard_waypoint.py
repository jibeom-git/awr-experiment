# app/dashboard.py
# Flask 통합 대시보드 서버

import sys, os, time, threading, copy
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template, jsonify, request
import cv2
import numpy as np

app = Flask(__name__, template_folder='templates', static_folder='static')

# ══════════════════════════════════════════════════════
# 로드셀 설정값
# ══════════════════════════════════════════════════════
HX711_REF_UNIT = -262.5   # 200g 기준 캘리브레이션
HX711_DEADZONE = 20.0     # ±20g 이하는 0으로 처리
HX711_SMOOTH   = 20       # 이동평균 샘플 수

# ══════════════════════════════════════════════════════
# 센서 초기화
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
    imu = MPU6050()
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
    time.sleep(1)
    hx711.REF_UNIT_A = HX711_REF_UNIT
    print("  로드셀에 아무것도 올리지 마세요 (5초 후 영점 설정)")
    time.sleep(5)
    hx711.tare(samples=50)
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

# #경로학습
# from core.waypoint_recorder import WaypointRecorder
# from core.heading_tracker   import HeadingTracker
# from core.waypoint_runner   import WaypointRunner

# recorder = WaypointRecorder()
# if IMU_AVAILABLE:
#     heading = HeadingTracker(imu)
#     heading.start()
# else:
#     heading = None

# ══════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════
lock             = threading.Lock()
latest_rgb       = None
latest_depth     = None
latest_astra_rgb = None
latest_telem = {
    'accel':    {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'gyro':     {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'weight':   0.0,
    'distance': 0.0,
    'motor':    {'cmd': 'stop', 'speed': 0},
}

# ══════════════════════════════════════════════════════
# 로드셀 이동평균 버퍼
# ══════════════════════════════════════════════════════
_weight_buf = []

def _read_weight_stable():
    global _weight_buf

    if hx711 is None:
        return 0.0

    raw = hx711.get_grams()

    # 이동평균
    _weight_buf.append(raw)
    if len(_weight_buf) > HX711_SMOOTH:
        _weight_buf.pop(0)
    avg = sum(_weight_buf) / len(_weight_buf)

    # 데드존
    if abs(avg) <= HX711_DEADZONE:
        return 0.0
    return round(avg, 1)

# ══════════════════════════════════════════════════════
# 백그라운드 스레드
# ══════════════════════════════════════════════════════

def rgb_loop():
    global latest_rgb
    if not CAM_AVAILABLE or cam is None:
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
def depth_loop():
    global latest_depth, latest_astra_rgb
    if not ASTRA_AVAILABLE or astra is None:
        return
    while True:
        try:
            depth = astra.get_depth_frame()

            if depth is not None:
                depth_vis  = np.clip(depth, 0, 1000).astype(np.float32)
                depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)  # type: ignore
                depth_norm = depth_norm.astype(np.uint8)
                colormap   = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

                cy, cx = depth.shape[0] // 2, depth.shape[1] // 2
                roi    = depth[cy-5:cy+5, cx-5:cx+5]
                valid  = roi[roi > 0]
                dist_c = int(np.percentile(valid, 20)) if len(valid) > 0 else -1

                floor_roi   = depth[380:470, 250:390]
                floor_valid = floor_roi[floor_roi > 0]
                floor_depth = int(np.mean(floor_valid)) if len(floor_valid) > 0 else 0
                obstacle_height = dist_c - floor_depth

                cv2.putText(colormap, f"{dist_c}mm", (cx-40, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.putText(colormap, f"Obstacle: {obstacle_height}mm", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.circle(colormap, (cx, cy), 5, (255,255,255), -1)
                cv2.rectangle(colormap, (250,380), (390,470), (0,255,0), 2)
                cv2.rectangle(
                    colormap,
                    (cx-5, cy-5),
                    (cx+5, cy+5),
                    (255,255,255),
                    2
                )

                _, jpeg_depth = cv2.imencode('.jpg', colormap, [cv2.IMWRITE_JPEG_QUALITY, 70])
                with lock:
                    latest_depth = jpeg_depth.tobytes()

        except Exception as e:
            print(f"[Depth] {e}")
            time.sleep(1)


def telem_loop():
    print("[Telem] 스레드 시작")
    while True:
        update = {}

        if IMU_AVAILABLE and imu is not None:
            try:
                update['accel'] = imu.get_accel()
                update['gyro']  = imu.get_gyro()
            except Exception as e:
                print(f"[Telem/IMU] {e}")

        if HX711_AVAILABLE and hx711 is not None:
            try:
                update['weight'] = _read_weight_stable()
            except Exception as e:
                print(f"[Telem/HX711] {e}")

        if ULTRA_AVAILABLE and ultra is not None:
            try:
                d = ultra.distance
                update['distance'] = round(d * 100, 2) if d is not None else -1
            except Exception as e:
                print(f"[Telem/Ultra] {e}")

        if update:
            with lock:
                latest_telem.update(update)

        time.sleep(0.05)


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
    return Response(gen_stream(lambda: latest_rgb),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_feed')
def depth_feed():
    return Response(gen_stream(lambda: latest_depth),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/astra_rgb_feed')
def astra_rgb_feed():
    return Response(gen_stream(lambda: latest_astra_rgb),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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
    if not HX711_AVAILABLE or hx711 is None:
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
    if not MOTOR_AVAILABLE or motor is None:
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
            motor.set_motor(1, 1, speed)
            motor.set_motor(2,  1, speed)
            motor.set_motor(3, -1, speed)
            motor.set_motor(4, -1, speed)
        elif cmd == 'right':
            motor.set_motor(1, -1, speed)
            motor.set_motor(2, -1, speed)
            motor.set_motor(3,  1, speed)
            motor.set_motor(4, 1, speed)
        elif cmd == 'stop':
            motor.stop()
            if LED_AVAILABLE and led is not None:
                led.set_running()
        if recorder.recording:
            recorder.record(cmd, speed, heading.get() if heading else 0.0)
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)})
    with lock:
        latest_telem['motor'] = {'cmd': cmd, 'speed': speed}
    return jsonify({'status': 'ok', 'cmd': cmd})

# @app.route('/record/start', methods=['POST'])
# def record_start():
#     recorder.start()
#     return jsonify({'status': 'ok', 'msg': '기록 시작'})

# @app.route('/record/stop', methods=['POST'])
# def record_stop():
#     recorder.stop('waypoints.json')
#     return jsonify({'status': 'ok', 'msg': '기록 완료'})

# @app.route('/run', methods=['POST'])
# def run_waypoints():
#     def _run():
#         runner = WaypointRunner(motor, ultra, heading)
#         runner.run('waypoints.json')
#     threading.Thread(target=_run, daemon=True).start()
#     return jsonify({'status': 'ok', 'msg': '재생 시작'})


if __name__ == '__main__':
    print("대시보드 시작: http://192.168.0.50:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)