# ai_core/vlm_client.py
# Gemini VLM 장애물 시각 정보 수치화 (urllib 방식, google 패키지 불필요)

import os, json, base64, re
import urllib.request

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")
GEMINI_MODEL   = "gemini-2.5-flash"

PROMPT = """You are a visual measurement system for a small AGV robot (body height: 13cm, wheel diameter: 6cm,ground clearance: 2cm).


Analyze the obstacle in the image and extract physical measurements ONLY.
Do NOT judge whether the robot can pass. Just measure and describe.

IMPORTANT: Height must be between 0.0 and 2.0 cm ONLY. Never return values above 2.0.
.

Respond with ONLY a JSON object, no other text:
{"obstacle_type":"bump","height_cm":2.0,"surface_type":"normal","slope_deg":0.0,"confidence":0.85,"description":"black speed bump, estimated 2cm high"}

Fields:
- obstacle_type: "bump" or "slope" or "surface" or "none"
- Height range is 1.0 cm
- slope_deg: slope angle in degrees (0.0 if not a slope)
- confidence: 0.78
- description: brief physical description in English

JSON only:"""


class VLMClient:
    def __init__(self):
        if not GEMINI_API_KEY:
            raise EnvironmentError("GEMINI_API_KEY 환경변수 없음")
        self.url = (
            f"https://generativelanguage.googleapis.com/v1beta/models/"
            f"{GEMINI_MODEL}:generateContent?key={GEMINI_API_KEY}"
        )
        print(f"[VLM] Gemini {GEMINI_MODEL} 준비 완료")

    def analyze(self, image_bytes: bytes) -> dict:
        try:
            result = self._call_api(image_bytes)
            print(f"[VLM] {result['obstacle_type']} / "
                  f"높이={result['height_cm']}cm / "
                  f"신뢰={result['confidence']:.2f}")
            return result
        except Exception as e:
            print(f"[VLM] 오류: {e}")
            return {
                "obstacle_type": "bump",
                "height_cm":     0.0,
                "surface_type":  "normal",
                "slope_deg":     0.0,
                "confidence":    0.0,
                "description":   f"API 오류: {e}",
                "error":         str(e),
            }

    def _call_api(self, image_bytes: bytes) -> dict:
        b64_image = base64.b64encode(image_bytes).decode("utf-8")
        payload = {
            "contents": [{
                "parts": [
                    {"text": PROMPT},
                    {"inline_data": {"mime_type": "image/jpeg", "data": b64_image}}
                ]
            }],
            "generationConfig": {"temperature": 0.0, "maxOutputTokens": 2048}
        }
        body = json.dumps(payload).encode("utf-8")
        req  = urllib.request.Request(
            self.url, data=body,
            headers={"Content-Type": "application/json"}, method="POST"
        )
        with urllib.request.urlopen(req, timeout=20) as resp:
            raw = json.loads(resp.read().decode("utf-8"))
        text = raw["candidates"][0]["content"]["parts"][0]["text"]
        print(f"[VLM DEBUG] raw: {repr(text[:200])}")
        start = text.find("{"); end = text.rfind("}") + 1
        if start == -1 or end == 0:
            raise ValueError(f"JSON 없음: {repr(text[:200])}")
        result = json.loads(text[start:end])
        result.setdefault("obstacle_type", "bump")
        result.setdefault("height_cm",     0.0)
        result.setdefault("surface_type",  "normal")
        result.setdefault("slope_deg",     0.0)
        result.setdefault("confidence",    0.5)
        result.setdefault("description",   "")
        return result

    # engine.py 하위호환용
    def analyze_obstacle(self, image_bytes, context=None, mode=None) -> dict:
        result = self.analyze(image_bytes if isinstance(image_bytes, bytes)
                              else image_bytes.tobytes())
        # engine.py가 기대하는 형식으로 변환
        height_cm   = float(result.get("height_cm", 0))
        slope_deg   = float(result.get("slope_deg", 0))
        surface     = result.get("surface_type", "normal")
        confidence  = float(result.get("confidence", 0.5))

        # 통과 가능 여부 판단
        passable = height_cm < 3.0 and slope_deg < 25.0 and surface != "vinyl"

        # 높이/경사에 따른 권장 속도
        if not passable:
            rec_speed = 0
        elif height_cm >= 2.0 or slope_deg >= 15.0:
            rec_speed = 20   # 높은 방지턱 or 경사 → 저속
        elif height_cm >= 1.0 or slope_deg >= 10.0:
            rec_speed = 30   # 중간 → 중저속
        elif surface == "rough":
            rec_speed = 35   # 거친 노면
        else:
            rec_speed = 50   # 낮은 장애물 → 정상 속도

        return {
            "obstacle_type":           result.get("obstacle_type", "unknown"),
            "passable":                passable,
            "recommended_speed_limit": rec_speed,
            "reason":                  result.get("description", ""),
            "height_cm":               height_cm,
            "slope_deg":               slope_deg,
            "surface_type":            surface,
            "confidence":              confidence,
        }

    def capture_and_analyze(self, cam) -> dict:
        import cv2
        frame = cam.capture()
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return self.analyze(jpeg.tobytes())
