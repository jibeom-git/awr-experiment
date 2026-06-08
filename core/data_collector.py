# core/data_collector.py
# 센서 데이터 비동기 수집 및 저장

import os
import time
import threading
import queue
import json
import csv
from datetime import datetime
import cv2
import numpy as np

DATA_ROOT = os.path.join(os.path.dirname(__file__), '..', 'data')


class DataCollector:
    """
    세션별 폴더에 RGB 프레임, Depth NPY, 텔레메트리 CSV를 비동기로 저장.
    session/
      rgb/   NNNNNN.jpg
      depth/ NNNNNN.npy
      telem.csv
      meta.json
    """

    def __init__(self):
        self._session_dir: str | None = None
        self._q: queue.Queue = queue.Queue(maxsize=500)
        self._worker = threading.Thread(target=self._save_loop, daemon=True, name="dc-save")
        self._worker.start()

        self.recording  = False
        self.frame_count = 0
        self._csv_writer = None
        self._csv_file   = None

    # ── 공개 API ─────────────────────────────────────────────

    def start(self) -> str:
        """세션 시작. 세션 폴더 경로 반환."""
        if self.recording:
            return self._session_dir

        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._session_dir = os.path.join(DATA_ROOT, ts)
        os.makedirs(os.path.join(self._session_dir, 'rgb'),   exist_ok=True)
        os.makedirs(os.path.join(self._session_dir, 'depth'), exist_ok=True)

        self._csv_file   = open(os.path.join(self._session_dir, 'telem.csv'), 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp', 'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'weight_g', 'distance_cm', 'motor_cmd', 'motor_speed',
        ])

        self.frame_count = 0
        self.recording   = True
        print(f"[DataCollector] 세션 시작: {self._session_dir}")
        return self._session_dir

    def stop(self) -> dict:
        """세션 종료. 메타 저장 후 요약 반환."""
        if not self.recording:
            return {}

        self.recording = False
        self._q.join()   # 저장 큐 소진 대기

        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None

        meta = {
            'session':     os.path.basename(self._session_dir),
            'frame_count': self.frame_count,
            'ended_at':    datetime.now().isoformat(),
        }
        with open(os.path.join(self._session_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)

        print(f"[DataCollector] 세션 종료 — {self.frame_count}프레임 저장")
        return meta

    def add_frame(
        self,
        rgb:   np.ndarray | None,
        depth: np.ndarray | None,
        telem: dict,
    ):
        """
        프레임 큐에 추가 (논블로킹).
        recording=False 이면 무시.
        """
        if not self.recording:
            return
        idx = self.frame_count
        self.frame_count += 1
        try:
            self._q.put_nowait(('frame', idx, rgb, depth, telem))
        except queue.Full:
            pass  # 큐 가득 찼으면 프레임 드롭

    # ── 내부 저장 워커 ────────────────────────────────────────

    def _save_loop(self):
        while True:
            item = self._q.get()
            try:
                kind = item[0]
                if kind == 'frame':
                    _, idx, rgb, depth, telem = item
                    self._save_frame(idx, rgb, depth, telem)
            except Exception as e:
                print(f"[DataCollector/save] {e}")
            finally:
                self._q.task_done()

    def _save_frame(self, idx: int, rgb, depth, telem: dict):
        if self._session_dir is None:
            return

        name = f'{idx:06d}'

        if rgb is not None:
            path = os.path.join(self._session_dir, 'rgb', name + '.jpg')
            cv2.imwrite(path, rgb, [cv2.IMWRITE_JPEG_QUALITY, 85])

        if depth is not None:
            path = os.path.join(self._session_dir, 'depth', name + '.npy')
            np.save(path, depth)

        if self._csv_writer and telem:
            a = telem.get('accel', {})
            g = telem.get('gyro',  {})
            m = telem.get('motor', {})
            self._csv_writer.writerow([
                time.time(),
                a.get('x', 0), a.get('y', 0), a.get('z', 0),
                g.get('x', 0), g.get('y', 0), g.get('z', 0),
                telem.get('weight',   0),
                telem.get('distance', 0),
                m.get('cmd',   'stop'),
                m.get('speed', 0),
            ])

    # ── 상태 조회 ─────────────────────────────────────────────

    def status(self) -> dict:
        return {
            'recording':   self.recording,
            'frame_count': self.frame_count,
            'session':     os.path.basename(self._session_dir) if self._session_dir else None,
            'queue_size':  self._q.qsize(),
        }
