# ai_core/vlm_client.py
# Gemini 2.5 Flash VLM 클라이언트: Few-shot 프롬프트 관리 + 장애물 판단

import os
import json

import google.generativeai as genai  # type: ignore

from . import config


class VLMClient:
    """
    Gemini 2.5 Flash API를 연동하여 장애물을 판단하는 클라이언트.
    obstacle_db에서 최근 사례를 Few-shot 예시로 포함하여 정확도를 향상한다.
    """

    # AGV 하드웨어 스펙 시스템 프롬프트
    _SYSTEM_PROMPT = (
        "너는 스마트 팩토리 자율주행 AGV(무게 약 500g, 지상고 3cm, 4륜구동)의 "
        "전방 위험을 인지하고 통과 가능 여부를 판단하는 AI 제어 시스템이다.\n"
        "판단 규칙:\n"
        "1. 5cm 방지턱: 지상고(3cm) 초과 → 반드시 통과 불가(passable: false)\n"
        "2. 3cm 방지턱: 통과 가능하나 SAFE 모드 + 고중량(>150g)이면 속도를 15로 제한\n"
        "3. 평지 비닐: 통과 가능, 속도 15 제한. 단 경사 7도 이상이면 통과 불가\n"
    )

    def __init__(self):
        api_key = os.getenv("GEMINI_API_KEY", "")
        if api_key:
            genai.configure(api_key=api_key)  # type: ignore
        self.model = genai.GenerativeModel(config.GEMINI_MODEL_NAME)  # type: ignore

    def analyze_obstacle(
        self,
        image_path: str,
        sensor_context: dict,
        obstacle_db: list,
    ) -> dict:
        """
        전방 카메라 이미지와 센서값, 과거 장애물 DB를 받아 장애물을 판단한다.

        Args:
            image_path: 전방 카메라 이미지 파일 경로
            sensor_context: {"pitch": float, "weight_g": float, "sonic": float}
            obstacle_db: 과거 장애물 결과 리스트 (few-shot 사례 생성에 사용)

        Returns:
            {
                "obstacle_type": str,       # bump_3cm|bump_5cm|vinyl|unknown
                "passable": bool,
                "recommended_speed_limit": int,
                "confidence": float,        # 0.0~1.0
                "reason": str,
            }
        """
        fallback = {
            "obstacle_type": "unknown",
            "passable": False,
            "recommended_speed_limit": config.SPEED_STOP,
            "confidence": 0.0,
            "reason": "VLM_COMMUNICATION_CRASH",
        }

        if not os.path.exists(image_path):
            return fallback

        try:
            # ── 이미지 로드 ────────────────────────────────────────────────
            with open(image_path, 'rb') as f:
                image_bytes = f.read()

            # ── Few-shot 사례 구성 ─────────────────────────────────────────
            few_shot_text = self._build_fewshot(obstacle_db)

            # ── 현재 센서 컨텍스트 ─────────────────────────────────────────
            sensor_text = (
                f"현재 센서값:\n"
                f"  - pitch(경사각): {sensor_context.get('pitch', 0.0):.1f}°\n"
                f"  - weight(적재중량): {sensor_context.get('weight_g', 0.0):.0f}g "
                f"(고중량 기준: {config.TH_LOAD_HEAVY}g)\n"
                f"  - sonic(전방거리): {sensor_context.get('sonic', 400.0):.0f}cm\n"
            )

            # ── 최종 프롬프트 조합 ─────────────────────────────────────────
            prompt = (
                self._SYSTEM_PROMPT + "\n"
                + few_shot_text
                + sensor_text
                + "\n전방 이미지를 분석하여 아래 JSON 형식으로만 응답하라. "
                "마크다운 코드블록이나 설명 없이 JSON만 출력:\n"
                "{\n"
                '  "obstacle_type": "bump_3cm|bump_5cm|vinyl|unknown",\n'
                '  "passable": true_or_false,\n'
                '  "recommended_speed_limit": 정수,\n'
                '  "confidence": 0.0~1.0,\n'
                '  "reason": "영어_근거_한_문장"\n'
                "}"
            )

            contents = [
                {"mime_type": "image/jpeg", "data": image_bytes},
                prompt,
            ]

            response = self.model.generate_content(contents)  # type: ignore
            return self._parse_response(response.text)

        except Exception as e:
            print(f"[VLMClient] Gemini API 호출 실패: {e}")
            return fallback

    def _build_fewshot(self, obstacle_db: list) -> str:
        """
        obstacle_db에서 최근 MAX_FEWSHOT_EXAMPLES 개의 결과를 Few-shot 텍스트로 변환.
        실제 결과가 있는 항목만 사용한다.
        """
        # actual_result 가 있는 항목만 필터링
        valid = [e for e in obstacle_db if e.get("actual_result") and e.get("obstacle_type")]
        recent = valid[-config.MAX_FEWSHOT_EXAMPLES:]  # 최신 N개

        if not recent:
            return ""

        lines = ["[과거 조우 사례 - 참고용]\n"]
        for i, entry in enumerate(recent, 1):
            lines.append(
                f"사례{i}: 장애물={entry['obstacle_type']} "
                f"판단={entry.get('gemini_judgment', '?')} "
                f"실제결과={entry['actual_result']} "
                f"충격량={entry.get('impact_z', 0):.2f} "
                f"pitch={entry.get('pitch', 0):.1f}° "
                f"weight={entry.get('weight', 0):.0f}g\n"
            )
        return "".join(lines) + "\n"

    def _parse_response(self, raw_text: str) -> dict:
        """Gemini 응답 텍스트를 파싱하여 결과 딕셔너리 반환"""
        fallback = {
            "obstacle_type": "unknown",
            "passable": False,
            "recommended_speed_limit": config.SPEED_STOP,
            "confidence": 0.0,
            "reason": "PARSE_FAILED",
        }
        try:
            text = raw_text.strip()
            # 마크다운 코드블록 제거
            if "```" in text:
                parts = text.split("```")
                for part in parts:
                    p = part.strip()
                    if p.startswith("json"):
                        p = p[4:].strip()
                    if p.startswith("{"):
                        text = p
                        break
            parsed = json.loads(text)
            # confidence 필드가 없으면 0.8 기본값
            if "confidence" not in parsed:
                parsed["confidence"] = 0.8
            return parsed
        except Exception as e:
            print(f"[VLMClient] 응답 파싱 실패: {e}\n원문: {raw_text[:200]}")
            return fallback
