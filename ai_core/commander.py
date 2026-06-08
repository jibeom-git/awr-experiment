# ai_core/commander.py
# 사용자 텍스트 명령 → Gemini → safe/fast 모드 판단
#
# 사용:
#   from ai_core.commander import CommanderVLM
#   cmd = CommanderVLM()
#   result = cmd.parse("물건 떨어뜨리면 안돼")
#   # {"mode": "safe", "route": "B", "reason": "..."}

import os
import json
import re
import urllib.request
from typing import Any

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")
GEMINI_MODEL   = "gemini-2.5-flash"

# 파이썬이 영문 Instructions를 문자열로 정확히 인식하도록 triple quotes로 안전하게 감쌉니다.
PROMPT = """You are a logistics AGV driving mode selector.
The user gives a natural language command in Korean or English.
Analyze the intent, decide the driving mode, and select the most appropriate route based on the factory topology.

Route Information (Crucial):
- ROUTE_A: Steep hill (20 degrees), shortest distance. (Best for speed/FAST mode, but dangerous for heavy/fragile cargo)
- ROUTE_B: Medium hill (10 degrees), medium distance. (Balanced route)
- ROUTE_C: Flat ground (0 degrees), longest distance. (Safest route, best for SAFE mode and heavy/fragile cargo)

Rules:
- If the user wants MAXIMUM SAFETY (fragile cargo, don't drop, careful, slow, safely):
  → mode: "safe", route: "C"
- If the user wants moderate caution or general driving:
  → mode: "safe", route: "B"
- If the user wants SPEED (fast, hurry, quick delivery, rush):
  → mode: "fast", route: "A"

Few-shot Examples (Strictly follow this mapping):
Input: "물건 떨어뜨리면 안돼"
Output: {"mode":"safe","route":"C","reason":"화물 낙하 방지가 최우선이므로 언덕이 없는 평지인 C경로를 선택합니다."}

Input: "빨리 가줘"
Output: {"mode":"fast","route":"A","reason":"신속한 배송이 필요하므로 최단 거리이자 고경사인 A경로를 선택합니다."}

Input: "안전하게 가줘"
Output: {"mode":"safe","route":"C","reason":"안전 주행 및 화물 보호를 원하므로 경사가 전혀 없는 평지인 C경로를 선택합니다."}

Input: "조심히 배달해라"
Output: {"mode":"safe","route":"C","reason":"조심스러운 운반이 필요하므로 가장 안전한 C경로로 주행합니다."}

Respond ONLY with a JSON object, no other text.

User command: """


class CommanderVLM:
    def __init__(self):
        if not GEMINI_API_KEY:
            raise EnvironmentError("GEMINI_API_KEY 환경변수 없음")
        self.url = (
            f"https://generativelanguage.googleapis.com/v1beta/models/"
            f"{GEMINI_MODEL}:generateContent?key={GEMINI_API_KEY}"
        )
        print("[Commander] Gemini 준비 완료")

    def parse(self, user_text: str) -> dict:
        """
        사용자 텍스트 → 모드 판단

        Returns:
            {"mode": "safe"|"fast", "route": "A"|"B"|"C", "reason": str}
        """
        try:
            payload = {
                "contents": [{
                    "parts": [{"text": PROMPT + user_text}]
                }],
                "generationConfig": {
                    "temperature":     0.1,
                    "maxOutputTokens": 500,
                }
            }
            body = json.dumps(payload).encode("utf-8")
            req  = urllib.request.Request(
                self.url, data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                raw = json.loads(resp.read().decode("utf-8"))

            text = raw["candidates"][0]["content"]["parts"][0]["text"]
            print(f"[Commander] raw: {repr(text[:200])}")

            text = text.replace("```json", "").replace("```", "").strip()
            start = text.find("{")
            end   = text.rfind("}") + 1
            if start == -1 or end == 0:
                raise ValueError(f"JSON 없음: {text[:100]}")
            result = json.loads(text[start:end])

            result = json.loads(text[start:end])
            result.setdefault("mode",   "safe")
            result.setdefault("route",  "B")  # 기본 폴백 경로를 C로 안전하게 변경
            result.setdefault("reason", "")
            return result

        except Exception as e:
            print(f"[Commander] 오류: {e}")
            # 오류 발생 시 시스템 최하위 안전망으로 SAFE 및 C경로 기본 적용
            return {
                "mode":   "safe",
                "route":  "C",
                "reason": f"판단 오류, 안전 모드 기본 적용: {e}"
            }


if __name__ == "__main__":
    cmd = CommanderVLM()
    tests = [
        "물건 떨어뜨리면 안돼",
        "빨리 가줘",
        "유리 제품이야 조심해",
        "최대한 빠르게 배달해줘",
        "안전하게 가줘",
    ]
    for t in tests:
        result = cmd.parse(t)
        print(f"\n입력: {t}")
        print(f"  모드: {result['mode']} | 경로: {result['route']}")
        print(f"  이유: {result['reason']}")
