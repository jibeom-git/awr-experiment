# sensors/astra.py
# Orbbec Astra 깊이 카메라 드라이버
# OpenNI2 ctypes 직접 바인딩 (Astra 0x0401 전용)

import ctypes
import numpy as np

OPENNI2_LIB = "/usr/local/lib/libOpenNI2.so"
ONI_STATUS_OK = 0

class AstraCamera:
    def __init__(self):
        self.lib = ctypes.CDLL(OPENNI2_LIB)
        self._setup_api()

        self.lib.oniInitialize(0)

        self.device = ctypes.c_void_p()
        self.lib.oniDeviceOpen(None, ctypes.byref(self.device))

        self.stream = ctypes.c_void_p()
        self.lib.oniDeviceCreateStream(
            self.device, ctypes.c_int(1), ctypes.byref(self.stream)
        )
        self.lib.oniStreamStart(self.stream)

        # 초기 5프레임 버려서 안정화
        for _ in range(5):
            f = ctypes.c_void_p()
            self.lib.oniStreamReadFrame(self.stream, ctypes.byref(f))
            self.lib.oniFrameRelease(f)

        self.width  = 640
        self.height = 480
        print("Astra 초기화 완료")

    def _setup_api(self):
        self.lib.oniInitialize.restype          = ctypes.c_int
        self.lib.oniDeviceOpen.restype          = ctypes.c_int
        self.lib.oniDeviceCreateStream.restype  = ctypes.c_int
        self.lib.oniStreamStart.restype         = ctypes.c_int
        self.lib.oniStreamReadFrame.restype     = ctypes.c_int
        self.lib.oniFrameRelease.argtypes       = [ctypes.c_void_p]
        self.lib.oniStreamStop.argtypes         = [ctypes.c_void_p]
        self.lib.oniDeviceClose.argtypes        = [ctypes.c_void_p]

    def get_depth_frame(self) -> np.ndarray:
        frame = ctypes.c_void_p()
        status = self.lib.oniStreamReadFrame(self.stream, ctypes.byref(frame))
        if status != ONI_STATUS_OK:
            return None

        data_addr = int.from_bytes(
            ctypes.string_at(frame.value + 8, 8), 'little'
        )
        data_ptr = ctypes.cast(data_addr, ctypes.POINTER(ctypes.c_uint16))
        depth = np.ctypeslib.as_array(
            data_ptr, shape=(self.height, self.width)
        ).copy()
        self.lib.oniFrameRelease(frame)
        return depth

    def get_center_distance(self) -> float:
        depth = self.get_depth_frame()
        if depth is None:
            return 0.0
        return float(depth[self.height // 2, self.width // 2])

    def close(self):
        self.lib.oniStreamStop(self.stream)
        self.lib.oniDeviceClose(self.device)
        self.lib.oniShutdown()