# ai_core/signal_detector.py
# 신호등 색상 감지 모듈: OpenCV HSV 1차 판단 + Gemini fallback

import os
import time
import numpy as np

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

# Gemini API (fallback용 — 초기화는 지연 로드로 처리)
_gemini_model = None


def _get_gemini_model():
    """Gemini 모델 인스턴스를 처음 필요할 때만 초기화 (지연 로드)"""
    global _gemini_model
    if _gemini_model is not None:
        return _gemini_model
    try:
        import google.generativeai as genai  # type: ignore
        from . import config
        api_key = os.getenv("GEMINI_API_KEY", "")
        if api_key:
            genai.configure(api_key=api_key)  # type: ignore
        _gemini_model = genai.GenerativeModel(config.GEMINI_MODEL_NAME)  # type: ignore
        return _gemini_model
    except Exception as e:
        print(f"[SignalDetector] Gemini fallback 초기화 실패: {e}")
        return None


def _calc_color_ratio(frame, lower1, upper1, lower2=None, upper2=None) -> float:
    """
    HSV 마스크 비율 계산.
    lower2/upper2 가 주어지면 두 범위를 OR 합산 (빨강 등 두 범위 필요한 색상용).
    """
    if not _CV2_OK or frame is None:
        return 0.0
    try:
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1  = cv2.inRange(hsv, lower1, upper1)
        if lower2 is not None and upper2 is not None:
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask  = cv2.bitwise_or(mask1, mask2)
        else:
            mask = mask1
        total = mask.shape[0] * mask.shape[1]
        return float(np.sum(mask > 0)) / total if total > 0 else 0.0
    except Exception:
        return 0.0


def _gemini_fallback(frame) -> str:
    """
    OpenCV 판단이 불확실할 때 Gemini에 신호등 색상 판단을 위임.
    반환: "GO" | "STOP" | "UNKNOWN"
    """
    model = _get_gemini_model()
    if model is None or frame is None or not _CV2_OK:
        return "UNKNOWN"
    try:
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return "UNKNOWN"
        image_bytes = buf.tobytes()
        prompt = (
            "이미지에 신호등이 있다면 현재 켜진 색상을 판단하라. "
            "초록(진행 가능)이면 GO, 빨강(정지)이면 STOP, 판단 불가면 UNKNOWN "
            "딱 한 단어만 응답하라."
        )
        contents = [{"mime_type": "image/jpeg", "data": image_bytes}, prompt]
        response = model.generate_content(contents)  # type: ignore
        text = response.text.strip().upper()
        if "GO" in text:
            return "GO"
        if "STOP" in text:
            return "STOP"
        return "UNKNOWN"
    except Exception as e:
        print(f"[SignalDetector] Gemini fallback 오류: {e}")
        return "UNKNOWN"


def detect_traffic_light(frame) -> str:
    """
    신호등 색상을 감지하여 "GO" | "STOP" | "UNKNOWN" 을 반환한다.

    1차: OpenCV HSV 색공간에서 초록/빨강 픽셀 비율 계산
      - 초록 HSV 범위: H(40~80), S(50~255), V(50~255)
      - 빨강 HSV 범위: H(0~10) + H(160~180), S(50~255), V(50~255)
    2차: 두 색상 모두 불확실하면 Gemini VLM fallback 호출

    Args:
        frame: OpenCV BGR numpy 프레임 (None이면 "UNKNOWN" 반환)

    Returns:
        "GO" | "STOP" | "UNKNOWN"
    """
    if frame is None or not _CV2_OK:
        return "UNKNOWN"

    import numpy as np

    # ── 초록 신호 감지 ──────────────────────────────────────────────────────
    green_lower = np.array([40,  50, 50])
    green_upper = np.array([80, 255, 255])
    green_ratio = _calc_color_ratio(frame, green_lower, green_upper)

    # ── 빨강 신호 감지 (두 HSV 범위 합산) ──────────────────────────────────
    red_lower1 = np.array([0,   50,  50])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160,  50,  50])
    red_upper2 = np.array([180, 255, 255])
    red_ratio  = _calc_color_ratio(frame, red_lower1, red_upper1, red_lower2, red_upper2)

    _THRESHOLD = 0.03  # 3% 이상이어야 유효 신호로 판단

    if green_ratio > _THRESHOLD and green_ratio >= red_ratio:
        return "GO"
    if red_ratio > _THRESHOLD and red_ratio > green_ratio:
        return "STOP"

    # 두 색 모두 불확실 → Gemini fallback
    return _gemini_fallback(frame)
