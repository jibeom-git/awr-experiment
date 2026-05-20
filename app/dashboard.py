# app/dashboard.py
# Flask 통합 대시보드
# - 실시간 RGB/깊이 카메라 스트리밍
# - 센서 수치 실시간 표시
# - 수동 조종 (키보드/버튼)
# - 데이터 수집 시작/종료
#
# 실행: python app/dashboard.py
# 접속: http://192.168.0.50:5000

import sys, os, time, threading, json
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, Response, render_template, jsonify, request
from sensors.camera import Camera
from sensors.astra import AstraCamera
from sensors.mpu6050 import MPU6050
from sensors.hx711 import HX711
from sensors.led import LEDController
import cv2
import numpy as np

app = Flask(__name__, template_folder='templates', static_folder='static')

# ── 센서 초기화 ──────────────────────────────────────────
cam   = Camera(width=640, height=480)
astra = AstraCamera()
imu   = MPU6050()
hx711 = HX711()
hx711.tare(samples=5)
led   = LEDController()
led.set_running()

# ── 공유 상태 ────────────────────────────────────────────
lock         = threading.Lock()
latest_rgb   = None
latest_depth = None
latest_telem = {
    'accel': {'x': 0, 'y': 0, 'z': 0},
    'gyro':  {'x': 0, 'y': 0, 'z': 0},
    'weight': 0,
    'distance': 0,
    'motor': {'left': 0, 'right': 0},
}

# ── 모터 (PCA9685) ───────────────────────────────────────
try:
    from sensors.motor import MotorController
    motor = MotorController()
    MOTOR_AVAILABLE = True
except Exception as e:
    print(f"모터 초기화 실패: {e}")
    MOTOR_AVAILABLE = False

# ── 센서 수집 스레드 ──────────────────────────────────────
def rgb_loop():
    global latest_rgb
    while True:
        try:
            frame = cam.capture()
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_rgb = jpeg.tobytes()
        except Exception as e:
            print(f"[RGB] {e}")

def depth_loop():
    global latest_depth
    while True:
        try:
            depth = astra.get_depth_frame()
            if depth is None:
                continue
            depth_vis  = np.clip(depth, 100, 1000).astype(np.float32)
            depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colormap   = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cy, cx     = depth.shape[0]//2, depth.shape[1]//2
            dist_c     = int(depth[cy, cx])
            cv2.putText(colormap, f"{dist_c}mm", (cx-40, cy-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.circle(colormap, (cx, cy), 5, (255,255,255), -1)
            _, jpeg = cv2.imencode('.jpg', colormap, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_depth = jpeg.tobytes()
        except Exception as e:
            print(f"[Depth] {e}")

def telem_loop():
    from gpiozero import DistanceSensor
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)

    while True:
        try:
            accel = imu.get_accel()
            gyro  = imu.get_gyro()
            w     = hx711.read()['diff']
            dist  = round(ultra.distance * 100, 2)
            with lock:
                latest_telem['accel']    = accel
                latest_telem['gyro']     = gyro
                latest_telem['weight']   = round(w, 1)
                latest_telem['distance'] = dist
        except Exception as e:
            print(f"[Telem] {e}")
        time.sleep(0.1)

# 스레드 시작
for fn in [rgb_loop, depth_loop, telem_loop]:
    threading.Thread(target=fn, daemon=True).start()

# ── Flask 라우트 ──────────────────────────────────────────
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
        data = dict(latest_telem)
    return jsonify(data)

@app.route('/control', methods=['POST'])
def control():
    if not MOTOR_AVAILABLE:
        return jsonify({'status': 'motor not available'})
    cmd   = request.json.get('cmd', 'stop')
    speed = int(request.json.get('speed', 50))

    if   cmd == 'forward':  motor.forward(speed)
    elif cmd == 'backward': motor.backward(speed)
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
        led.set_running()

    with lock:
        latest_telem['motor'] = {'cmd': cmd, 'speed': speed}
    return jsonify({'status': 'ok', 'cmd': cmd})

if __name__ == '__main__':
    print("대시보드 시작: http://192.168.0.50:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)