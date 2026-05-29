# app/dashboard_simple.py
# 센서 테스트용 간단한 대시보드

import sys, os, time
sys.path.insert(0, '/home/pi/insite')

from flask import Flask, jsonify, render_template
import threading

app = Flask(__name__, template_folder='templates', static_folder='static')

# ══════════════════════════════════════════════════════
# 센서 상태
# ══════════════════════════════════════════════════════
latest_telem = {
    'accel':    {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'gyro':     {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'weight':   0.0,
    'distance': 0.0,
}

lock = threading.Lock()

# ══════════════════════════════════════════════════════
# 센서 폴링
# ══════════════════════════════════════════════════════
def telem_loop():
    print("[Telem] 백그라운드 폴링 시작")
    while True:
        try:
            update = {}

            # IMU
            try:
                from sensors.mpu6050 import MPU6050
                imu = MPU6050()
                update['accel'] = imu.get_accel()
                update['gyro']  = imu.get_gyro()
                print(f"[IMU] ✓ {update['accel']}")
            except Exception as e:
                print(f"[IMU] ✗ {e}")

            # 초음파
            try:
                import warnings
                from gpiozero import DistanceSensor
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    ultra = DistanceSensor(echo=24, trigger=23, max_distance=2)
                d = ultra.distance
                if d is not None:
                    update['distance'] = round(d * 100, 2)
                    print(f"[ULTRA] ✓ {update['distance']} cm")
            except Exception as e:
                print(f"[ULTRA] ✗ {e}")

            if update:
                with lock:
                    latest_telem.update(update)

        except Exception as e:
            print(f"[TLoop] {e}")

        time.sleep(0.5)

# ── Flask 라우트 ────────────────────────────────────

@app.route('/')
def index():
    return render_template('index_simple.html')

@app.route('/telemetry')
def telemetry():
    with lock:
        import copy
        data = copy.deepcopy(latest_telem)
    return jsonify(data)

@app.route('/status')
def status():
    return jsonify({'status': 'simple_dashboard_running'})

# ══════════════════════════════════════════════════════

threading.Thread(target=telem_loop, daemon=True, name="telem").start()

if __name__ == '__main__':
    print("\n=== 간단 대시보드 시작 ===")
    print("http://192.168.0.50:5000\n")
    app.run(host='0.0.0.0', port=5000, threaded=True)
