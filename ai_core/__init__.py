# ai_core/__init__.py
# 패키지 공개 인터페이스 — 기존 하위호환 export 유지

from .engine     import AGVAIEngine
from .client     import GeminiClient
from .vlm_client import VLMClient
from .logger     import BlackboxLogger
from .trainer    import ModelTrainer
from .signal_detector import detect_traffic_light

__all__ = [
    'AGVAIEngine',
    'GeminiClient',
    'VLMClient',
    'BlackboxLogger',
    'ModelTrainer',
    'detect_traffic_light',
]
