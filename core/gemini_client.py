# core/gemini_client.py
# Gemini Flash API - 장애물 분석 및 자연어 명령 해석

import os
import time
import json
import io
import cv2
import numpy as np

COOLDOWN_SEC = 3.0
MODEL_NAME   = 'gemini-1.5-flash'

class GeminiClient:
    def __init__(self):
        import google.generativeai as genai
        api_key = os.environ.get('GEMINI_API_KEY', '')
        if not api_key:
            raise ValueError("GEMINI_API_KEY 환경변수 없음")
        genai.configure(api_key=api_key)
        self._genai = genai
        self.model  = genai.GenerativeModel(MODEL_NAME)
        self._last_call = 0.0
        print(f"[Gemini] {MODEL_NAME} 초기화 완료")

    # ── 장애물 분석 ───────────────────────────────────────────
    def analyze_obstacle(
        self,
        rgb_frame:     np.ndarray,
        depth_summary: dict,
        sensor_data:   dict,
    ) -> dict | None:
        """
        rgb_frame    : BGR numpy array (원본 해상도)
        depth_summary: {center_mm, min_mm, std, gradient}
        sensor_data  : {distance_cm, weight_g}
        returns      : {obstacle:0-4, passable:bool, path:'A/B/C', reason:str}
                       쿨다운 중이면 None 반환
        """
        if time.time() - self._last_call < COOLDOWN_SEC:
            return None

        pil_img = self._to_pil(rgb_frame, size=(320, 240))
        depth   = depth_summary or {}
        sensor  = sensor_data   or {}

        prompt = (
            "You are a robot obstacle analyzer. Examine the image and sensor data.\n\n"
            f"Depth sensor: center={depth.get('center_mm', 0)}mm  "
            f"min={depth.get('min_mm', 0)}mm  "
            f"std={depth.get('std', 0):.0f}  "
            f"gradient={depth.get('gradient', 0):.2f}\n"
            f"Ultrasonic: {sensor.get('distance_cm', 0)} cm\n"
            f"Load cell: {sensor.get('weight_g', 0)} g\n\n"
            "Reply ONLY with valid JSON, no markdown:\n"
            '{"obstacle":<0-4>,"passable":<true/false>,"path":"<A/B/C>","reason":"<max 15 words>"}\n\n'
            "obstacle: 0=clear 1=minor 2=moderate 3=severe 4=blocked\n"
            "path: A=steep(20deg) B=medium(10deg) C=flat"
        )

        try:
            response = self.model.generate_content([prompt, pil_img])
            result   = self._parse_json(response.text)
            self._last_call = time.time()
            return result
        except Exception as e:
            print(f"[Gemini/obstacle] {e}")
            return {'obstacle': 0, 'passable': True, 'path': 'B', 'reason': str(e)[:60]}

    # ── 자연어 명령 해석 ─────────────────────────────────────
    def parse_command(self, text: str) -> dict:
        """
        자연어 → 주행 명령
        returns: {mode:'fast/safe/stop', reason:str}
        """
        prompt = (
            "You control an autonomous robot. Parse the user command.\n\n"
            f'Command: "{text}"\n\n'
            "Reply ONLY with valid JSON:\n"
            '{"mode":"<fast/safe/stop>","reason":"<max 10 words>"}\n\n'
            "fast = take shortest steep path, safe = take longest flat path, stop = halt"
        )
        try:
            response = self.model.generate_content(prompt)
            return self._parse_json(response.text)
        except Exception as e:
            print(f"[Gemini/cmd] {e}")
            return {'mode': 'stop', 'reason': str(e)[:60]}

    # ── 유틸 ─────────────────────────────────────────────────
    def _to_pil(self, bgr: np.ndarray, size: tuple = (320, 240)):
        from PIL import Image
        small = cv2.resize(bgr, size)
        rgb   = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)

    @staticmethod
    def _parse_json(text: str) -> dict:
        t = text.strip()
        if '```' in t:
            parts = t.split('```')
            t = parts[1] if len(parts) > 1 else parts[0]
            if t.startswith('json'):
                t = t[4:]
        start = t.find('{')
        end   = t.rfind('}')
        if start != -1 and end != -1:
            t = t[start:end+1]
        return json.loads(t)
