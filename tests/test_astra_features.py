# ~/insite/tests/test_astra_features.py
import os
import sys
import time
import cv2
import numpy as np

# 우리가 방금 성공시킨 드라이버 모듈 로드
from sensors.astra import AstraDepthSensor

def main():
    # 드라이버 인스턴스 할당
    try:
        sensor = AstraDepthSensor()
    except Exception as e:
        print(f"[Error] 센서 초기화 실패: {e}")
        return

    print("\n" + "="*60)
    print(" 라즈베리파이 4 하드웨어 기하학적 피처 추출 실측 실험")
    print(" 종료하려면 터미널 창에서 Ctrl + C를 누르십시오.")
    print("="*60 + "\n")

    try:
        while True:
            # 원시 480x640 16비트 깊이 행렬 수신
            depth_img = sensor.get_depth_frame()
            
            # [설계 핵심 로직]: 전체 화면 연산 병목 방지를 위한 주행 궤적 ROI 슬라이싱
            # 세로 240~440, 가로 220~420 영역 가공 ($200 \times 200$ 매트릭스)
            roi = depth_img[240:440, 220:420]
            total_pixels = roi.size
            
            # 피처 1: 사각지대 및 테이프 흡수로 인한 0픽셀 유실율 계산
            zero_pixels = np.sum(roi == 0)
            depth_hole_ratio = (zero_pixels / total_pixels) * 100.0
            
            # 유효 원소 필터 가드
            valid_pixels = roi[roi > 0]
            
            if len(valid_pixels) == 0:
                # ROI 전체가 사각지대(Dead Zone)일 경우의 예외 제어 가드
                sys.stdout.write(
                    f"\r[BLIND ZONE TRIGGERED] 유실율: {depth_hole_ratio:.1f}% | "
                    f"center: 0.0mm | min: 0.0mm | std: 0.0mm          "
                )
                sys.stdout.flush()
                time.sleep(0.05)
                continue
                
            # 피처 2: ROI 중심부 20x20 영역의 중앙값 산출 (산술 평균의 아웃라이어 취약성 방어)
            roi_center = roi[90:110, 90:110]
            valid_center = roi_center[roi_center > 0]
            depth_center = np.median(valid_center) if len(valid_center) > 0 else np.median(valid_pixels)
            
            # 피처 3: 상하단 행 평균 편차 기반 경사 변화율 산출 (Finite Difference Approximation)
            upper_rows = roi[0:5, :]
            lower_rows = roi[-5:, :]
            valid_upper = upper_rows[upper_rows > 0]
            valid_lower = lower_rows[lower_rows > 0]
            mean_upper = np.mean(valid_upper) if len(valid_upper) > 0 else depth_center
            mean_lower = np.mean(valid_lower) if len(valid_lower) > 0 else depth_center
            depth_gradient = mean_upper - mean_lower
            
            # 피처 4, 5: 최소 거리 및 공간 표준편차 도출
            depth_min = np.min(valid_pixels)
            depth_std = np.std(valid_pixels)
            
            # [출력 최적화]: \r 지시자를 이용해 터미널 한 줄에서 수치가 실시간 갱신되도록 빌드
            print_msg = (
                f"\r[Astra Live] Center: {depth_center:4.1f}mm | "
                f"Grad: {depth_gradient:+6.1f}mm/px | "
                f"Min: {depth_min:4.1f}mm | "
                f"Std: {depth_std:5.1f}mm | "
                f"Hole: {depth_hole_ratio:5.1f}%"
            )
            sys.stdout.write(print_msg)
            sys.stdout.flush()
            
            # 20fps 구동 주기를 맞추기 위한 지연 시간 제어
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print(" 실험 주기가 안전하게 종료되었습니다. 하드웨어 포트를 닫습니다.")
        print("="*60)
    finally:
        sensor.close()

if __name__ == "__main__":
    main()