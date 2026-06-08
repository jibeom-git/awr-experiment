# ai_core/client.py
# 구글 Gemini 2.5 Flash 멀티모달 API 전송 및 JSON 제어 토큰 파싱 클라이언트 모듈

import os
import json
import cv2
import numpy as np
import google.generativeai as genai  # type: ignore
from . import config

class GeminiClient:
    def __init__(self):
        """[디자인 사상] 전역 config에 록킹된 API Key와 최신 모델명을 바인딩하여 백본 수립"""
        # 환경변수에서 API 키 파싱 수용
        self.api_key = os.getenv("GEMINI_API_KEY", "YOUR_ACTUAL_API_KEY")
        self.model_name = config.GEMINI_MODEL_NAME
        
        genai.configure(api_key=self.api_key) # type: ignore
        self.model = genai.GenerativeModel(self.model_name) # type: ignore

    def analyze_obstacle(self, frame: np.ndarray, sensors: dict, drive_mode: str) -> dict:
        """FSM 엔진(engine.py)에서 호출하는 원격 시각 상황 인지 코어 함수"""
        fallback_response = {
            "obstacle_type": "unknown_failed",
            "passable": False,
            "recommended_speed_limit": config.SPEED_STOP,
            "reason": "VLM_COMMUNICATION_CRASH"
        }

        if frame is None or frame.size == 0:
            return fallback_response

        try:
            success, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not success:
                return fallback_response
                
            image_bytes = encoded_img.tobytes()

            prompt = f"""
            너는 자율주행 AGV의 위험을 인지하고 속도 및 경로 변경 여부를 결정하는 스마트 팩토리 인공지능 제어 장치이다.
            현재 차량 제어 주행 모드: [{drive_mode}]
            하부 결합 센서 팩 실측 수치:
            - 초음파 장애물 잔여 거리: {sensors.get('distance_cm', 400.0)} cm
            - 로드셀 적재 화물 중량: {sensors.get('weight_g', 0.0)} g (Heavy 임계치: {config.TH_LOAD_HEAVY}g)
            - IMU 가속도계 전후 경사각: {sensors.get('pitch', 0.0)} deg

            전방 카메라 프레임 이미지를 정밀 분석하여, 아래 팩토리 주행 규칙 조건문 매트릭스에 맞춰 최종 의사결정을 내려라.
            [주행 규칙 매트릭스]
            1. 높이 5cm 방지턱은 차체 지상고 초과이므로 통과 불가(passable: false) 처리한다.
            2. 높이 3cm 방지턱은 통과 가능(passable: true)하나, 주행 모드가 'SAFE'이고 화물 중량이 {config.TH_LOAD_HEAVY}g을 초과하는 고중량 상태이면 차체 충격 완화를 위해 recommended_speed_limit를 {config.SPEED_SAFE_LOW}로 극도로 낮추어야 한다.
            3. 평지 비닐 재질은 통과 가능(passable: true)하나 미끄러짐 방지를 위해 속도를 {config.SPEED_SAFE_LOW}로 제어한다. 단, IMU 경사각이 {config.TH_PITCH_MEDIUM_HILL}도 이상인 언덕 구역에서 비닐을 조우하면 등판 불능 상태이므로 즉시 통과 불가(passable: false) 처리한다.

            반드시 하단의 정형화된 JSON 형식을 완벽하게 엄수하여 단 한 줄의 사설 없이 덤프 출력하라. json 외에 문장이나 마크다운 백틱(```) 기호조차 일절 붙이지 마라.
            {{
                "obstacle_type": "식별된_장애물_이름(bump_3cm/bump_5cm/vinyl/unknown)",
                "passable": true_또는_false,
                "recommended_speed_limit": 추천_PWM_듀티비_정수_값,
                "reason": "판단에_대한_공학적_근거_요약_영어문장"
            }}
            """

            contents = [
                {"mime_type": "image/jpeg", "data": image_bytes},
                prompt
            ]
            
            response = self.model.generate_content(contents) # type: ignore
            clean_text = response.text.strip()
            
            if clean_text.startswith("```"):
                clean_text = clean_text.split("```")[1]
                if clean_text.startswith("json"):
                    clean_text = clean_text[4:]
            clean_text = clean_text.strip()

            parsed_json = json.loads(clean_text)
            return parsed_json

        except Exception as e:
            print(f"[Gemini Client API Fault] 원격 네트워크 타임아웃 예외 처리: {e}")
            return fallback_response