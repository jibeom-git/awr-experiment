# sensors/ultra.py
# HC-SR04 초음파 센서 드라이버
# GPIO23: Trig, GPIO24: Echo
# gpiozero 라이브러리 사용 (공식 문서 기준)

from gpiozero import DistanceSensor

TRIG = 23
ECHO = 24

# max_distance=2 → 최대 측정 거리 2m
sensor = DistanceSensor(echo=ECHO, trigger=TRIG, max_distance=2)

def get_distance() -> float:
    """거리 측정 후 cm 단위로 반환"""
    return round(sensor.distance * 100, 2)

def close():
    """GPIO 자원 해제"""
    sensor.close()