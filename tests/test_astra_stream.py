# tests/test_astra_stream.py
# Astra 깊이 카메라 Flask 실시간 스트리밍 (30fps 고정)

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from flask import Flask, Response
from sensors.astra import AstraCamera
import cv2
import numpy as np
import threading

app          = Flask(__name__)
cam          = None
lock         = threading.Lock()
latest_frame = None

TARGET_FPS    = 30
FRAME_INTERVAL = 1.0 / TARGET_FPS  # 0.0333초

def capture_loop():
    global latest_frame
    while True:
        t0 = time.time()
        try:
            depth = cam.get_depth_frame()
            if depth is None:
                continue

            depth_norm = cv2.normalize(
                depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            cy, cx = depth.shape[0] // 2, depth.shape[1] // 2
            dist = depth[cy, cx]
            cv2.putText(colormap, f"{dist}mm",
                        (cx - 40, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.circle(colormap, (cx, cy), 5, (255, 255, 255), -1)

            _, jpeg = cv2.imencode('.jpg', colormap, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with lock:
                latest_frame = jpeg.tobytes()

        except Exception as e:
            print(f"프레임 오류: {e}")

        # 30fps 유지를 위한 남은 시간 대기
        elapsed = time.time() - t0
        sleep_time = FRAME_INTERVAL - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

def generate():
    while True:
        with lock:
            frame = latest_frame
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return '''
    <html>
    <head>
        <title>Astra Depth Stream</title>
        <meta http-equiv="cache-control" content="no-cache">
        <style>
            body { background:#111; display:flex; flex-direction:column;
                   align-items:center; justify-content:center;
                   height:100vh; margin:0; }
            h2   { color:#0ff; margin-bottom:12px; }
            img  { border:2px solid #0ff; }
        </style>
    </head>
    <body>
        <h2>Orbbec Astra Depth Stream</h2>
        <img src="/video_feed">
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={
                        'Cache-Control': 'no-cache, no-store, must-revalidate',
                        'Pragma': 'no-cache',
                        'Expires': '0'
                    })

if __name__ == "__main__":
    cam = AstraCamera()
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    print(f"스트리밍 시작 ({TARGET_FPS}fps) — http://pi.local:5000")
    app.run(host='0.0.0.0', port=5000, threaded=False)