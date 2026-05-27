# Insite 프로젝트

## 개요
Adeept AWR 4WD 로봇 자율주행 시스템.
Raspberry Pi 4 (4GB), Debian Trixie 64-bit.

## 가상환경
source ~/insite/.venv/bin/activate

## 하드웨어
- RGB 카메라: OV5647 (CSI, picamera2, 180도 회전)
- 깊이 카메라: Orbbec Astra (USB, OpenNI2 ctypes, ARM64)
  - RGB 스트림 색상 채널 불량 문제 디버깅 중
- IMU: MPU-6050 (I2C5, GPIO12/13, smbus2)
- 로드셀: HX711 (GPIO5/6, RPi.GPIO, VDD/VCC 모두 5V)
- 초음파: HC-SR04 (GPIO23/24, gpiozero)
- 모터: PCA9685 (I2C1, 주소 0x5f, Adafruit)
- LED: WS2812 x2 (SPI, GPIO10)

## 주요 특이사항
- PCA9685 주소: 0x5f (기본값 0x40 아님)
- pigpio: apt 제거됨, 소스 빌드 필요
- GitHub master 브랜치 사용
- 대시보드: http://192.168.0.50:5000

## 현재 미완성 항목
- dashboard.py telem_loop 센서값 0 문제
- Astra RGB 색상 채널 불량
- 데이터 수집 파이프라인 (core/data_collector.py)
- XGBoost 모델 학습
- Gemini API 연동

## 코드 수정 규칙
- 수정 전 반드시 변경 내용 설명 후 승인 받을 것
- 무단 파일 수정 금지
