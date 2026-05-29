# ~/insite/app/dashboard_preview.py
import sys
import os
import cv2
import numpy as np
import time
from flask import Flask, render_template_string, Response

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from sensors.astra import AstraDepthSensor

app = Flask(__name__)
driver = AstraDepthSensor()

def generate_mjpeg_stream():
    while True:
        depth_img, color_bgr = driver.get_frames()
        dc, dg, dmin, dstd, dhole, is_comp = driver.filter.process_filter(depth_img, ultrasonic_cm=15.0)
        
        depth_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        
        # 320x240 스케일에 맞게 ROI 디스플레이 가이드라인 좌표 수정 (110, 120) -> (210, 220)
        cv2.rectangle(depth_colormap, (110, 120), (210, 220), (255, 255, 255), 1)
        
        text_color = (0, 0, 255) if is_comp else (0, 0, 0)
        base_y = 25
        cv2.putText(depth_colormap, f"Min: {dmin:.1f}mm", (5, base_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv2.LINE_AA)
        cv2.putText(depth_colormap, f"Hole: {dhole:.1f}%", (5, base_y+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv2.LINE_AA)
        
        # 수평 가로 병합 ($320 + 320 = 640\text{px}$)
        display_combined = np.hstack((color_bgr, depth_colormap))
        
        ret, jpeg = cv2.imencode('.jpg', display_combined, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if not ret:
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
        time.sleep(0.06) # 프레임 드롭 가드를 위한 15fps 전송 주기 고정

@app.route('/')
def index():
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Astra Pi Optimized Dashboard</title>
        <style>
            body { background-color: #1e1e1e; color: #ffffff; font-family: sans-serif; text-align: center; padding-top: 30px; }
            .container { margin: 0 auto; width: 680px; background-color: #2d2d2d; padding: 15px; border-radius: 8px; }
            img { border: 2px solid #444; border-radius: 4px; background-color: #000; }
            h2 { color: #00adb5; margin-bottom: 5px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h2>Insite 경량화 원격 관제 대시보드 (320x240)</h2>
            <img src="/video_feed" width="640" height="240">
        </div>
    </body>
    </html>
    """
    return render_template_string(html_template)

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    finally:
        driver.close()