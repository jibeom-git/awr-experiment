# standalone_test.py
# 타임아웃 가드가 추가되어 절대로 무한 프리징에 빠지지 않는 단독 검증 스크립트

import ctypes
import numpy as np
import cv2
import time

OPENNI2_LIB = "/usr/local/lib/libOpenNI2.so"
ONI_STATUS_OK = 0

class StandaloneAstraChecker:
    def __init__(self):
        """
        OpenNI2 라이브러리를 직접 로드하고 카메라 장치를 초기화합니다.
        """
        self.lib = ctypes.CDLL(OPENNI2_LIB)
        
        self.lib.oniInitialize.restype          = ctypes.c_int
        self.lib.oniDeviceOpen.restype          = ctypes.c_int
        self.lib.oniDeviceCreateStream.restype  = ctypes.c_int
        self.lib.oniStreamStart.restype         = ctypes.c_int
        self.lib.oniStreamReadFrame.restype     = ctypes.c_int
        
        # 비동기 멀티스트림 대기 함수 정적 선언
        self.lib.oniWaitForAnyStream.restype    = ctypes.c_int
        self.lib.oniWaitForAnyStream.argtypes   = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]
        
        self.lib.oniFrameRelease.argtypes       = [ctypes.c_void_p]
        self.lib.oniStreamStop.argtypes         = [ctypes.c_void_p]
        self.lib.oniDeviceClose.argtypes        = [ctypes.c_void_p]

        self.lib.oniInitialize(0)

        self.device = ctypes.c_void_p()
        self.lib.oniDeviceOpen(None, ctypes.byref(self.device))

        # 깊이 스트림 (1) 및 컬러 스트림 (2) 핸들 생성
        self.depth_stream = ctypes.c_void_p()
        self.lib.oniDeviceCreateStream(self.device, ctypes.c_int(1), ctypes.byref(self.depth_stream))

        self.color_stream = ctypes.c_void_p()
        self.lib.oniDeviceCreateStream(self.device, ctypes.c_int(2), ctypes.byref(self.color_stream))

        self.width  = 640
        self.height = 480
        self.is_running = False

    def start_camera(self):
        """
        카메라 스트림 가동
        """
        self.lib.oniStreamStart(self.depth_stream)
        self.lib.oniStreamStart(self.color_stream)
        self.is_running = True
        print("[정보] 아스트라 듀얼 스트림 가동 시작 완료.")

    def capture_with_timeout(self):
        """
        [핵심 보호 조치] 2000ms(2초) 동안 데이터가 안 오면 프리징을 깨고 에러를 출력합니다.
        """
        # 두 스트림을 배열로 묶음
        streams_arr = (ctypes.c_void_p * 2)(self.depth_stream, self.color_stream)
        ready_idx = ctypes.c_int()
        
        depth_received = False
        color_received = False

        # 최대 2000ms(2초) 동안 장치 신호 대기
        print("[대기] 카메라 센서 신호 수신 대기 중 (최대 2초)...")
        
        # 각 스트림별로 데이터를 채우기 위해 최대 2번 시도
        for attempt in range(2):
            status = self.lib.oniWaitForAnyStream(streams_arr, 2, ctypes.byref(ready_idx), 2000)
            
            if status != ONI_STATUS_OK:
                print(f"[타임아웃 알림] {attempt+1}차 신호 획득 실패. 카메라가 데이터를 전송하지 않습니다.")
                continue

            frame = ctypes.c_void_p()
            if ready_idx.value == 0 and not depth_received:
                # 깊이 프레임 읽기
                self.lib.oniStreamReadFrame(self.depth_stream, ctypes.byref(frame))
                if frame.value is not None:
                    d_addr = int.from_bytes(ctypes.string_at(frame.value + 8, 8), 'little')
                    d_ptr = ctypes.cast(d_addr, ctypes.POINTER(ctypes.c_uint16))
                    depth_array = np.ctypeslib.as_array(d_ptr, shape=(self.height, self.width)).copy()
                    
                    center_dist = depth_array[self.height // 2, self.width // 2]
                    print(f" -> [성공] 깊이 데이터 수신 완료! (정중앙 거리: {center_dist}mm)")
                    
                    # 시각화 후 저장
                    depth_vis = np.clip(depth_array, 200, 3000).astype(np.float32)
                    depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8) # type: ignore
                    colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                    cv2.imwrite("test_depth.jpg", colormap)
                    
                    self.lib.oniFrameRelease(frame)
                    depth_received = True

            elif ready_idx.value == 1 and not color_received:
                # 컬러 프레임 읽기
                self.lib.oniStreamReadFrame(self.color_stream, ctypes.byref(frame))
                if frame.value is not None:
                    c_addr = int.from_bytes(ctypes.string_at(frame.value + 8, 8), 'little')
                    c_ptr = ctypes.cast(c_addr, ctypes.POINTER(ctypes.c_uint8))
                    rgb_array = np.ctypeslib.as_array(c_ptr, shape=(self.height, self.width, 3)).copy()
                    
                    print(" -> [성공] 내장 RGB 컬러 데이터 수신 완료!")
                    
                    bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
                    cv2.imwrite("test_rgb.jpg", bgr_array)
                    
                    self.lib.oniFrameRelease(frame)
                    color_received = True

        if not depth_received and not color_received:
            print("[에러 결과] 두 센서 모두 신호를 받지 못했습니다. USB 연결 상태나 프로세스 경합을 확인하세요.")

    def close_camera(self):
        """
        자원 해제
        """
        if self.is_running:
            self.lib.oniStreamStop(self.depth_stream)
            self.lib.oniStreamStop(self.color_stream)
            self.lib.oniDeviceClose(self.device)
            self.lib.oniShutdown()
            print("[정보] 하드웨어 자원 닫기 완료.")

if __name__ == "__main__":
    print("=== 안전 모드 아스트라 하드웨어 단독 테스트 ===")
    checker = None
    try:
        checker = StandaloneAstraChecker()
        checker.start_camera()
        
        # 1번만 확실하게 찔러봅니다.
        checker.capture_with_timeout()
        
    except Exception as error:
        print(f"[초기화 실패] 장치 개방 중 오류: {error}")
    finally:
        if checker is not None:
            checker.close_camera()
    print("=== 테스트 종료 ===")