# run_real_agv.py
# FSM 제어 엔진 모듈 결합 및 수동 실험 데이터 덤프 최종 통합 런처

import os
import sys
import time
import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 전역 레지스터 동기 인입
from ai_core import config, AGVAIEngine
from sensors.hx711 import HX711
from sensors.ultra import UltrasonicSensor
from sensors.mpu6050 import MPU6050
from sensors.camera import Camera, USBCamera

class AGVInsiteMainLauncher:
    def __init__(self):
        # 1. 하드웨어 드라이버 채널 빌드
        self.loadcell = HX711(dout=5, pd_sck=6)
        self.sonar = UltrasonicSensor(trigger_pin=23, echo_pin=24)
        self.imu = MPU6050(bus_id=5, address=0x68)
        
        # 원본 이원화 카메라 할당
        self.line_cam = Camera(width=640, height=480)
        self.ai_cam = USBCamera(device_index=0, width=640, height=480)
        
        # 2. 상위 AI FSM 제어 엔진 생성 (기본 주행 사상 'SAFE' 모드 고정)
        self.ai_engine = AGVAIEngine(mode="SAFE")
        self.boot_diagnostics()

    def boot_diagnostics(self):
        print("="*85)
        print(f"     AGV INSITE COMPONENT ENGINE CORE RUNNER (VLM Target: {config.GEMINI_MODEL_NAME})")
        print("="*85)
        self.loadcell.init_chassis_zero_calibration(times=20)
        self.imu.init_chassis_pitch_calibration(times=20)
        self.imu.reset_yaw()
        print(f"[SYSTEM] 하드웨어 자동 영점 조절 완료. 실험 데이터 적산 경로: {self.ai_engine.log_path}")
        print("="*85 + "\n")

    def spin_simulation(self):
        cv2.namedWindow("AGV INSITE PANEL")
        mock_obs = "None"
        
        print(" -> [실험 조작 방법] 생성된 이미지 창을 선택한 상태에서 가상 장애물을 인입하십시오.")
        print("    - 'f' : FAST 모드 전환   |  's' : SAFE 모드 전환")
        print("    - '1' : 3cm 방지턱 조우  |  '2' : 5cm 방지턱 조우 (통과 불가 우회)")
        print("    - '3' : 비닐 노면 진입   |  '0' : 장애물 조건 없음 초기화")
        print("    - 'q' : 수동 실험 세션 안전 마감 및 자원 반환\n")
        
        print(f"{'Time Stamp':<10} | {'Mode':<5} | {'Weight':<7} | {'Dist':<7} | {'Pitch':<6} | {'Out Speed':<9} | {'FSM State':<22} | {'Active Route'}")
        print("-"*110)

        while True:
            try:
                # 1. 물리 하드웨어 레이어 실시간 스캔
                weight_g = self.loadcell.get_weight(times=3)
                distance_cm = self.sonar.read_average_distance(times=3)
                imu_data = self.imu.get_all()
                pitch_deg = imu_data.get("pitch", 0.0)
                
                # 2. AI FSM 입력 구조체 패킹
                sensor_snapshot = {
                    'pitch': pitch_deg,
                    'weight_g': weight_g,
                    'distance_cm': distance_cm,
                    'node_trigger': False  # 모의 테스트 중 노드 주입은 기본값 고정
                }
                
                # 3. 오르벡 아스트라 이미지 프레임 취득
                frame = self.ai_cam.capture()
                if frame is None or frame.size == 0:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)

                # 4. [핵심 동기화] 통합 AI 상태 평가 엔진 연산 호출 후 가중치 명령 수렴
                control_output = self.ai_engine.evaluate_state_and_calculate_output(
                    sensor_snapshot=sensor_snapshot,
                    ai_camera_frame=frame,
                    mock_obs=mock_obs
                )
                
                speed_cmd = control_output.get("speed_limit_pct", config.SPEED_STOP)
                fsm_state = control_output.get("state", "UNKNOWN")
                active_rt = control_output.get("route", "UNKNOWN")

                # 콘솔 디스플레이 출력 마그네이션
                time_str = time.strftime("%H:%M:%S")
                print(f"{time_str:<10} | {self.ai_engine.mode:<5} | {weight_g:<5.1f}g | {distance_cm:<5.1f}cm | {pitch_deg:<5.1f}° | {speed_cmd:<7}% | {fsm_state:<22} | {active_rt}")

                # 영상 모니터 창 그래픽 렌더링 오버레이
                cv2.putText(frame, f"MODE: {self.ai_engine.mode} | ROUTE: {active_rt}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(frame, f"FSM: {fsm_state} | MOCK_OBS: {mock_obs}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.imshow("AGV INSITE PANEL", frame)

                # 키보드 입력 인터럽트 분기
                key = cv2.waitKey(150) & 0xFF
                if key == ord('q'):
                    print("\n[System] 사용자 종료 신호 감지.")
                    break
                elif key == ord('f'):
                    self.ai_engine.mode = "FAST"
                    print("\n[Command] 주행 알고리즘 가치 평가 기조가 'FAST' 모드로 승격되었습니다.")
                elif key == ord('s'):
                    self.ai_engine.mode = "SAFE"
                    print("\n[Command] 주행 알고리즘 가치 평가 기조가 'SAFE' 모드로 원복되었습니다.")
                elif key == ord('1'):
                    mock_obs = "Bump_3cm"
                elif key == ord('2'):
                    mock_obs = "Bump_5cm"
                elif key == ord('3'):
                    mock_obs = "Vinyl"
                elif key == ord('0'):
                    mock_obs = "None"

            except Exception as e:
                print(f"\n[Launcher Crash Gate] 예외 대피 루프 가동: {e}")
                time.sleep(0.5)

        self.terminate_session()

    def terminate_session(self):
        self.line_cam.close()
        self.ai_cam.close()
        cv2.destroyAllWindows()
        print("[System] 하드웨어 버스 폐쇄 및 단독 실험 시퀀스 완전 마감.")

if __name__ == "__main__":
    launcher = AGVInsiteMainLauncher()
    launcher.spin_simulation()