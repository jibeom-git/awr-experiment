# tests/test_astra_stream_udp.py
import sys, os, time, socket
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from sensors.astra import AstraCamera
import cv2
import numpy as np
import threading

MAC_IP   = "192.168.0.20"
UDP_PORT = 5000

Gst.init(None)

PIPELINE_STR = (
    f"appsrc name=src is-live=true block=false format=GST_FORMAT_TIME "
    f"caps=image/jpeg,width=640,height=480,framerate=30/1 ! "
    f"jpegparse ! rtpjpegpay pt=26 ! "
    f"udpsink host={MAC_IP} port={UDP_PORT} sync=false async=false"
)

class UDPStreamer:
    def __init__(self):
        self.pipeline = Gst.parse_launch(PIPELINE_STR)
        self.appsrc   = self.pipeline.get_by_name("src")
        self.pts      = 0
        self.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
        self.pipeline.set_state(Gst.State.PLAYING)
        time.sleep(0.5)

    def push_frame(self, jpeg_bytes: bytes):
        buf = Gst.Buffer.new_wrapped(jpeg_bytes)
        buf.pts      = self.pts
        buf.dts      = self.pts
        buf.duration = self.duration
        self.pts    += self.duration
        ret = self.appsrc.emit("push-buffer", buf)
        return ret

    def stop(self):
        self.appsrc.emit("end-of-stream")
        self.pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    print(f"Pi → Mac({MAC_IP}:{UDP_PORT}) UDP 스트리밍")
    print("Mac 수신 명령어:")
    print(f"  gst-launch-1.0 udpsrc port={UDP_PORT} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! rtpjpegdepay ! jpegdec ! videoconvert ! osxvideosink")

    cam      = AstraCamera()
    streamer = UDPStreamer()

    interval = 1.0 / 30
    frame_count = 0

    try:
        while True:
            t0    = time.time()
            depth = cam.get_depth_frame()
            if depth is None:
                continue

            # 깊이 → 컬러맵
            depth_vis = np.clip(depth, 0, 4000).astype(np.float32)
            depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colormap   = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            cy, cx = 240, 320
            dist   = depth[cy, cx]
            cv2.putText(colormap, f"{dist}mm", (cx-40, cy-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.circle(colormap, (cx, cy), 5, (255,255,255), -1)

            ret, jpeg = cv2.imencode('.jpg', colormap, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ret:
                continue

            streamer.push_frame(jpeg.tobytes())
            frame_count += 1

            if frame_count % 30 == 0:
                print(f"송출 중... {frame_count}프레임 | 중앙거리: {dist}mm")

            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n종료")
    finally:
        streamer.stop()
        cam.close()