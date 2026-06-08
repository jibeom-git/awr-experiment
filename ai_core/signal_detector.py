# ai_core/signal_detector.py
# OpenCV 고속 픽셀 필터링 + Gemini VLM 하이브리드 (토큰 절약 버전)

import os
import json
import urllib.request
from typing import Any

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")
GEMINI_MODEL   = "gemini-2.5-flash"

SIGNAL_PROMPT = """You are a traffic light recognition system for an industrial AGV robot.
Analyze the traffic light in the image and detect its current active color.
Respond with ONLY a JSON object: {"signal_color":"red","confidence":0.95}"""

class SignalDetectorVLM:
    def __init__(self):
        self.url = f"https://generativelanguage.googleapis.com/v1beta/models/{GEMINI_MODEL}:generateContent?key={GEMINI_API_KEY}"

    def detect_via_cv(self, image_bytes: bytes) -> str:
        """
        [지범님 긴급 튜닝판] 소형/원거리 신호등 인식을 위한 초고감도 픽셀 스캔 엔진
        """
        try:
            import cv2
            import numpy as np
            
            nparr = np.frombuffer(image_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None: return "none"
            
            # 조명 노이즈 및 빛 번짐에 강한 HSV 색공간 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 💡 [튜닝 1] 빛이 번져서 흐려진 LED 색상을 잡기 위해 채도(S)와 명도(V) 하한선을 50 -> 30으로 대폭 하향
            # 💡 [튜닝 2] 초록색(Green)의 범위를 40~90에서 35~100까지 넓혀 청록색/연초록 감지 보장
            lower_green = np.array([35, 30, 40]);   upper_green = np.array([95, 255, 255])
            lower_yellow = np.array([12, 60, 60]);  upper_yellow = np.array([32, 255, 255])

            lower_red1 = np.array([0, 30, 30]);     upper_red1 = np.array([12, 255, 255])
            lower_red2 = np.array([165, 30, 30]);   upper_red2 = np.array([180, 255, 255])
            
            mask_g = cv2.inRange(hsv, lower_green, upper_green)
            mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_r = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
            
            # 각 색상별 매칭 픽셀 개수 계산
            cnt_g = int(np.sum(mask_g > 0))
            cnt_y = int(np.sum(mask_y > 0))
            cnt_r = int(np.sum(mask_r > 0))
            
            # 💡 [튜닝 3] 임계값을 150에서 '8'로 획기적으로 낮춥니다!
            # 화면이 상단 50%로 이미 크롭되어 노이즈가 적으므로, 불빛이 손톱만 해도 무조건 잡아냅니다.
            TH_PIXEL = 8
            
            print(f"[CV-Signal-Debug] 실측 픽셀수 ➡️ R: {cnt_r} | Y: {cnt_y} | G: {cnt_g} (기준 임계값: {TH_PIXEL})")
            
            max_cnt = max(cnt_g, cnt_y, cnt_r)
            if max_cnt < TH_PIXEL:
                return "none" # 픽셀이 아예 없으면 신호 없음 반환
                
            if max_cnt == cnt_g: return "green_suspect"
            if max_cnt == cnt_y: return "yellow_suspect"
            return "red_suspect"
            
        except Exception as e:
            print(f"[CV 1차 필터 오류] {e}")
            return "error"