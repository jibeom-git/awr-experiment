# app/dashboard.py
# Flask 통합 대시보드 — AI 판단 엔진 연동 버전
#
# 변경사항 (engine 연동):
#   - ObstacleEngine 초기화 (서버 시작 시 1회)
#   - /control  : 장애물 감지 시 engine.decide() 호출
#   - /decision : 최근 판단 결과 조회 엔드포인트 추가
#   - /mode     : user_mode 변경 엔드포인트 추가 (fast/safe)
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
HX711_REF_UNIT   = -188.72
HX711_DEADZONE   = 10.0
HX711_SMOOTH     = 20
HX711_TARE_EVERY = 300

# ══════════════════════════════════════════════════════
# AI 엔진 초기화
# ══════════════════════════════════════════════════════
try:
    from ai_core.Engine_by import ObstacleEngine
    from ai_core.Logger_by import DecisionLogger
    from ai_core.Config_by import ULTRA_TRIGGER_CM, USER_MODE_DEFAULT

    engine    = ObstacleEngine()
    ai_logger = DecisionLogger()
    AI_AVAILABLE = True
    print("[OK]   AI 엔진")
except Exception as e:
    engine       = None
    ai_logger    = None
    AI_AVAILABLE = False
    print(f"[SKIP] AI 엔진: {e}")

# ── AI 상태 (런타임 변경 가능) ────────────────────────
current_route = "A"                                        # 현재 루트
user_mode     = USER_MODE_DEFAULT if AI_AVAILABLE else "safe"  # fast | safe

# ══════════════════════════════════════════════════════
# 센서 초기화 (각각 독립적으로 try-except)
# ══════════════════════════════════════════════════════
try:
    from sensors.camera import Camera
    cam = Camera(width=640, height=480)
    CAM_AVAILABLE = True
    print("[OK]   RGB 카메라")
except Exception as e:
    cam = None; CAM_AVAILABLE = False
    print(f"[SKIP] RGB 카메라: {e}")

try:
    from sensors.astra import AstraCamera
    astra = AstraCamera()
    ASTRA_AVAILABLE = True
    print("[OK]   Astra 깊이 카메라")
except Exception as e:
    astra = None; ASTRA_AVAILABLE = False
    print(f"[SKIP] Astra: {e}")

try:
    from sensors.mpu6050 import MPU6050
    from tests.calibration_gyro import calibrate_gyro
    imu = MPU6050()
    imu.gyro_offset = calibrate_gyro(imu)
    IMU_AVAILABLE = True
    print("[OK]   MPU-6050")
except Exception as e:
    imu = None; IMU_AVAILABLE = False
    print(f"[SKIP] MPU-6050: {e}")

try:
    from sensors.hx711 import HX711
    hx711 = HX711()
    time.sleep(1)
    hx711.REF_UNIT_A = HX711_REF_UNIT
    hx711.tare(samples=50)
    HX711_AVAILABLE = True
    print("[OK]   HX711 로드셀")
except Exception as e:
    hx711 = None; HX711_AVAILABLE = False
    print(f"[SKIP] HX711: {e}")

try:
    import warnings
    from gpiozero import DistanceSensor
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
    ULTRA_AVAILABLE = True
    print("[OK]   초음파 HC-SR04")
except Exception as e:
    ultra = None; ULTRA_AVAILABLE = False
    print(f"[SKIP] 초음파: {e}")

try:
    from sensors.led import LEDController
    led = LEDController()
    led.set_running()
    LED_AVAILABLE = True
    print("[OK]   LED")
except Exception as e:
    led = None; LED_AVAILABLE = False
    print(f"[SKIP] LED: {e}")

try:
    from sensors.motor import MotorController
    motor = MotorController()
    MOTOR_AVAILABLE = True
    print("[OK]   모터")
except Exception as e:
    motor = None; MOTOR_AVAILABLE = False
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
print(f"│ AI 엔진     : {'✓ OK  ' if AI_AVAILABLE    else '✗ SKIP'} │")
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
# 로드셀 이동평균
# ══════════════════════════════════════════════════════
_weight_buf   = []
_tare_counter = 0

def _read_weight_stable():
    global _weight_buf, _tare_counter
    raw = hx711.get_grams()
    if len(_weight_buf) >= 3:
        current_avg = sum(_weight_buf) / len(_weight_buf)
        if abs(raw - current_avg) > 50:
            raw = current_avg
    _weight_buf.append(raw)
    if len(_weight_buf) > HX711_SMOOTH:
        _weight_buf.pop(0)
    avg = sum(_weight_buf) / len(_weight_buf)
    _tare_counter += 1
    if _tare_counter >= HX711_TARE_EVERY:
        _tare_counter = 0
        if abs(avg) < HX711_DEADZONE * 3:
            _weight_buf.clear()
            hx711.tare(samples=10)
    if abs(avg) <= HX711_DEADZONE:
        return 0.0
    return round(avg, 1)

# ══════════════════════════════════════════════════════
# 백그라운드 스레드
# ══════════════════════════════════════════════════════
def rgb_loop():
    global latest_rgb
    if not CAM_AVAILABLE: return
    while True:
        try:
            frame = cam.capture()
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_rgb = jpeg.tobytes()
        except Exception as e:
            print(f"[RGB] {e}"); time.sleep(1)

def depth_loop():
    global latest_depth
    if not ASTRA_AVAILABLE: return
    while True:
        try:
            depth = astra.get_depth_frame()
            if depth is None: time.sleep(0.05); continue
            depth_vis  = np.clip(depth, 100, 1000).astype(np.float32)
            depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colormap   = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cy, cx     = depth.shape[0] // 2, depth.shape[1] // 2
            cv2.putText(colormap, f"{int(depth[cy,cx])}mm", (cx-40, cy-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.circle(colormap, (cx, cy), 5, (255,255,255), -1)
            _, jpeg = cv2.imencode('.jpg', colormap, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_depth = jpeg.tobytes()
        except Exception as e:
            print(f"[Depth] {e}"); time.sleep(1)

def telem_loop():
    while True:
        update = {}
        if IMU_AVAILABLE:
            try:
                update['accel'] = imu.get_accel()
                update['gyro']  = imu.get_gyro()
            except Exception as e:
                print(f"[Telem/IMU] {e}")
        if HX711_AVAILABLE:
            try:
                update['weight'] = _read_weight_stable()
            except Exception as e:
                print(f"[Telem/HX711] {e}")
        if ULTRA_AVAILABLE:
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
# AI 판단 헬퍼
# ══════════════════════════════════════════════════════
def _run_ai_decision(speed: int) -> dict | None:
    """
    현재 텔레메트리 기반으로 AI 판단 실행
    초음파 거리가 ULTRA_TRIGGER_CM 이하일 때만 호출할 것
    반환: engine.decide() 결과 or None (AI 미사용 시)
    """
    if not AI_AVAILABLE:
        return None

    with lock:
        telem = copy.deepcopy(latest_telem)

    sensor = {
        "speed":   speed,
        "gyro_x":  telem["gyro"]["x"],
        "gyro_y":  telem["gyro"]["y"],
        "gyro_z":  telem["gyro"]["z"],
        "accel_x": telem["accel"]["x"],
        "accel_y": telem["accel"]["y"],
        "accel_z": telem["accel"]["z"],
        "weight":  telem["weight"],
    }

    result = engine.decide(sensor, current_route=current_route, user_mode=user_mode)

    # 판단 기록
    ai_logger.log(sensor, result, current_route=current_route, user_mode=user_mode)

    print(f"[AI] {result['reason']} | action={result['action']} route={result['route']} speed={result['speed']}")
    return result


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

@app.route('/telemetry')
def telemetry():
    with lock:
        data = copy.deepcopy(latest_telem)
    return jsonify(data)

@app.route('/status')
def status():
    return jsonify({
        'cam': CAM_AVAILABLE, 'astra': ASTRA_AVAILABLE,
        'imu': IMU_AVAILABLE, 'hx711': HX711_AVAILABLE,
        'ultra': ULTRA_AVAILABLE, 'motor': MOTOR_AVAILABLE,
        'led': LED_AVAILABLE, 'ai': AI_AVAILABLE,
        'route': current_route, 'mode': user_mode,
    })

@app.route('/tare', methods=['POST'])
def tare():
    if not HX711_AVAILABLE:
        return jsonify({'status': 'error', 'msg': 'hx711 not available'})
    try:
        global _weight_buf
        _weight_buf.clear()
        hx711.tare(samples=20)
        return jsonify({'status': 'ok', 'msg': '영점 완료'})
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)})


@app.route('/mode', methods=['POST'])
def set_mode():
    """user_mode 변경 — {"mode": "fast"} 또는 {"mode": "safe"}"""
    global user_mode
    data = request.get_json(force=True)
    mode = data.get('mode', '').lower()
    if mode not in ('fast', 'safe'):
        return jsonify({'status': 'error', 'msg': 'mode는 fast 또는 safe'})
    user_mode = mode
    print(f"[Mode] user_mode → {user_mode}")
    return jsonify({'status': 'ok', 'mode': user_mode})


@app.route('/decision', methods=['GET'])
def get_decision():
    """최근 AI 판단 결과 조회 (대시보드 표시용)"""
    if not AI_AVAILABLE:
        return jsonify({'status': 'error', 'msg': 'AI 엔진 없음'})
    n = int(request.args.get('n', 10))
    return jsonify({'status': 'ok', 'decisions': ai_logger.get_latest(n)})


@app.route('/control', methods=['POST'])
def control():
    global current_route

    if not MOTOR_AVAILABLE:
        return jsonify({'status': 'error', 'msg': 'motor not available'})

    data  = request.get_json(force=True)
    cmd   = data.get('cmd', 'stop')
    speed = int(data.get('speed', 50))

    # ── AI 판단: forward 명령 + 초음파 근접 시에만 실행 ──────────
    ai_result = None
    if AI_AVAILABLE and cmd == 'forward':
        with lock:
            dist = latest_telem.get('distance', 999)
        if dist != -1 and dist <= ULTRA_TRIGGER_CM:
            ai_result = _run_ai_decision(speed)
            if ai_result:
                # 엔진 결과 반영
                speed         = ai_result['speed']
                current_route = ai_result['route']

                if ai_result['action'] == 'detour':
                    # 우회 → 일단 정지 후 대시보드에 알림
                    if MOTOR_AVAILABLE:
                        motor.stop()
                    if LED_AVAILABLE:
                        led.set_thinking()
                    return jsonify({
                        'status':  'detour',
                        'route':   current_route,
                        'speed':   speed,
                        'reason':  ai_result['reason'],
                    })

    print(f"[CONTROL] {cmd} speed={speed}")

    try:
        if cmd == 'forward':
            motor.forward(speed)
        elif cmd == 'backward':
            motor.backward(speed)
        elif cmd == 'left':
            motor.set_motor(1, -1, speed); motor.set_motor(2,  1, speed)
            motor.set_motor(3, -1, speed); motor.set_motor(4,  1, speed)
        elif cmd == 'right':
            motor.set_motor(1,  1, speed); motor.set_motor(2, -1, speed)
            motor.set_motor(3,  1, speed); motor.set_motor(4, -1, speed)
        elif cmd == 'stop':
            motor.stop()
            if LED_AVAILABLE:
                led.set_running()
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)})

    with lock:
        latest_telem['motor'] = {'cmd': cmd, 'speed': speed}

    resp = {'status': 'ok', 'cmd': cmd}
    if ai_result:
        resp['ai'] = {'action': ai_result['action'], 'reason': ai_result['reason']}
    return jsonify(resp)


if __name__ == '__main__':
    print("대시보드 시작: http://192.168.0.50:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)