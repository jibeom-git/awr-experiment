# sensors/mpu6050.py
# MPU-6050 IMU 드라이버 (호환성 가드 및 자가 영점 교정 통합본)
# I2C5 소프트웨어 버스 (GPIO12=SDA, GPIO13=SCL), smbus2

import os
import time
import math

try:
    from smbus2 import SMBus
    _SMBUS_AVAILABLE = True
except ImportError:
    _SMBUS_AVAILABLE = False
    print("[IMU] smbus2 없음 — mock 모드로 동작")


class MPU6050:
    """
    I2C5 소프트웨어 버스를 제어하고 오프셋 교정 및 상보 필터를 수행하는 IMU 제어 클래스.
    부팅 시 전압 안정화 및 정적 영점 교정(init_chassis_pitch_calibration) 기능을 기본 내장합니다.
    """
    def __init__(self, bus_id: int = 5, address: int = 0x68):
        self.bus_id  = bus_id
        self.address = address
        self.bus     = None
        self._mock   = False

        if not _SMBUS_AVAILABLE:
            self._mock = True
            print("[IMU] mock 모드 (smbus2 없음)")
        else:
            try:
                self.bus = SMBus(self.bus_id)
                # 1. Sleep 모드 해제 + PLL 클록 안정화
                self.bus.write_byte_data(self.address, 0x6B, 0x01)
                # 2. DLPF 하드웨어 저역 필터 설정 (44Hz / 42Hz)
                self.bus.write_byte_data(self.address, 0x1A, 0x03)
                # 3. 자이로 풀스케일 범위: ±250°/s
                self.bus.write_byte_data(self.address, 0x1B, 0x00)
                # 4. 가속도계 풀스케일 범위: ±2g
                self.bus.write_byte_data(self.address, 0x1C, 0x00)
                # 5. 샘플레이트 분주기 설정 (100Hz)
                self.bus.write_byte_data(self.address, 0x19, 0x09)
            except Exception as e:
                print(f"[IMU] I2C 버스 개방 실패 — mock 모드 전환: {e}")
                self._mock = True

        # 교정 오프셋 레지스터 배열
        self.offsets = {
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
        }
        self.offset_file = os.path.expanduser("~/insite/gyro_offset.txt")
        if not self._mock:
            self.load_offsets()

        # 소프트웨어 상보 필터 하이퍼파라미터
        self.alpha     = 0.95
        self.last_time = time.time()
        self.roll      = 0.0
        self.pitch     = 0.0
        self.yaw       = 0.0
        
        # [추가 반영] 초기 구동 시 평지 수평 오차 바이어스를 격리하기 위한 추가 변수
        self.pitch_zero_bias = 0.0
        self.roll_zero_bias  = 0.0

        # 데드레코닝용 보조 레지스터
        self._pos_x         = 0.0
        self._pos_y         = 0.0
        self._last_accel_x  = 0.0
        self._last_accel_y  = 0.0

    def load_offsets(self):
        """저장된 영점 오프셋 파일 로드"""
        if not os.path.exists(self.offset_file):
            return
        try:
            with open(self.offset_file, "r") as f:
                lines = f.readlines()
            if len(lines) >= 6:
                keys = ["ax", "ay", "az", "gx", "gy", "gz"]
                for i, k in enumerate(keys):
                    self.offsets[k] = float(lines[i].strip())
        except Exception as e:
            print(f"[IMU] 오프셋 로드 예외 — 기본값 사용: {e}")

    def read_raw_word(self, reg: int) -> float:
        """16비트 2의 보수 직렬 변환 (소프트웨어 I2C 재시도 가드 포함)"""
        if self._mock or self.bus is None:
            return 0.0
        for attempt in range(3):
            try:
                data = self.bus.read_i2c_block_data(self.address, reg, 2)
                val  = (data[0] << 8) | data[1]
                if val >= 0x8000:
                    val = -((65535 - val) + 1)
                return float(val)
            except Exception as e:
                if attempt == 2:
                    print(f"[IMU] 레지스터 블록 읽기 최종 실패 (0x{reg:02X}): {e}")
                time.sleep(0.002)
        return 0.0

    def calibrate_sensors(self, samples: int = 1000):
        """정지 상태에서 하드웨어 원시 오프셋 바이어스 연산 후 파일 기록"""
        if self._mock:
            print("[IMU] mock 모드 — 교정 불가")
            return
        print(f"[IMU] 교정 시작 ({samples}개 샘플 — 로봇을 움직이지 마십시오)")
        sums = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
        regs = {"ax": 0x3B, "ay": 0x3D, "az": 0x3F, "gx": 0x43, "gy": 0x45, "gz": 0x47}
        for _ in range(samples):
            for k, r in regs.items():
                sums[k] += self.read_raw_word(r)
            time.sleep(0.002)
        for k in sums:
            self.offsets[k] = sums[k] / samples

        accel_offsets = {k: self.offsets[k] for k in ("ax", "ay", "az")}
        gravity_axis = max(accel_offsets, key=lambda k: abs(accel_offsets[k]))
        sign = 1.0 if accel_offsets[gravity_axis] > 0 else -1.0
        self.offsets[gravity_axis] -= sign * 16384.0
        
        print(f"[IMU] 중력축 감지 보정: {gravity_axis}")
        with open(self.offset_file, "w") as f:
            for k in ["ax", "ay", "az", "gx", "gy", "gz"]:
                f.write(f"{self.offsets[k]}\n")
        print("[IMU] 오프셋 파일 저장 완료")

    def init_chassis_pitch_calibration(self, times: int = 20):
        """
        [자가 진단 핵심 요구사항]
        로봇이 주행을 시작하기 직전, 상보 필터가 연산해 내는 현재 정차 상태의 각도를 
        정적 바이어스로 상쇄하여 수평 기준점(완벽한 0.0도)을 강제 정렬합니다.
        """
        print("\n" + "="*50)
        print("[HARDWARE INIT] AGV I2C5 IMU 자가 정적 수평 교정 개시")
        print(" -> 차량을 평평한 평지 트랙에 정차시킨 상태를 유지하십시오.")
        
        total_roll = 0.0
        total_pitch = 0.0
        # 상보 필터의 초기 과도 응답을 수렴시키며 평균 덤프 수집
        for _ in range(times):
            angles = self.get_filtered_angles()
            total_roll += angles["roll"]
            total_pitch += angles["pitch"]
            time.sleep(0.01)
            
        self.roll_zero_bias  = total_roll / float(times)
        self.pitch_zero_bias = total_pitch / float(times)
        print(f" -> [CALIBRATION SUCCESS] IMU 소프트웨어 오프셋 세팅 완료 (Zero Pitch: {self.pitch_zero_bias:.2f} deg)")
        print("="*50 + "\n")

    def get_accel(self) -> dict:
        return {
            "x": (self.read_raw_word(0x3B) - self.offsets["ax"]) / 16384.0,
            "y": (self.read_raw_word(0x3D) - self.offsets["ay"]) / 16384.0,
            "z": (self.read_raw_word(0x3F) - self.offsets["az"]) / 16384.0,
        }

    def get_gyro(self) -> dict:
        return {
            "x": (self.read_raw_word(0x43) - self.offsets["gx"]) / 131.0,
            "y": (self.read_raw_word(0x45) - self.offsets["gy"]) / 131.0,
            "z": (self.read_raw_word(0x47) - self.offsets["gz"]) / 131.0,
        }
    
    def get_gyro_raw(self) -> dict:
        return {
            "x": self.read_raw_word(0x43) / 131.0,
            "y": self.read_raw_word(0x45) / 131.0,
            "z": self.read_raw_word(0x47) / 131.0,
        }

    def get_filtered_angles(self) -> dict:
        """상보 필터로 roll/pitch 계산, 자이로 Z축 적분으로 yaw 누적"""
        if self._mock:
            t = time.time()
            self.roll  = math.sin(t * 0.3) * 5.0
            self.pitch = math.cos(t * 0.2) * 3.0
            self.yaw   = (t * 5.0) % 360.0
            return {"roll": self.roll, "pitch": self.pitch, "yaw_rate": 0.0}

        accel = self.get_accel()
        gyro  = self.get_gyro()

        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        if dt <= 0.001 or dt > 0.2:
            dt = 0.01

        denom       = math.sqrt(accel["y"] ** 2 + accel["z"] ** 2) or 1e-6
        accel_roll  = math.atan2(accel["y"], accel["z"]) * 180.0 / math.pi
        accel_pitch = math.atan2(-accel["x"], denom)     * 180.0 / math.pi

        MAX_ACCEL_DIFF = 45.0
        if abs(accel_roll  - self.roll)  > MAX_ACCEL_DIFF:
            accel_roll  = self.roll
        if abs(accel_pitch - self.pitch) > MAX_ACCEL_DIFF:
            accel_pitch = self.pitch

        self.roll  = self.alpha * (self.roll  + gyro["x"] * dt) + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gyro["y"] * dt) + (1.0 - self.alpha) * accel_pitch

        self.yaw = (self.yaw + gyro["z"] * dt) % 360.0
        if self.yaw < 0:
            self.yaw += 360.0

        # 오프셋 보정 수치를 가감하여 최종 기하 각도 도출
        corrected_roll  = self.roll - self.roll_zero_bias
        corrected_pitch = self.pitch - self.pitch_zero_bias

        return {
            "roll":     round(corrected_roll,  2),
            "pitch":    round(corrected_pitch, 2),
            "yaw_rate": round(gyro["z"],  2),
        }

    def get_all(self) -> dict:
        """대시보드 서버 인터페이스 호환 — pitch/roll/yaw/accel_x/accel_y/accel_z 모두 반환"""
        angles = self.get_filtered_angles()
        corrected_roll  = self.roll - self.roll_zero_bias
        corrected_pitch = self.pitch - self.pitch_zero_bias

        # 가속도 데이터 추가 (단위: m/s²)
        if not self._mock:
            accel = self.get_accel()
            ax = round(float(accel["x"]) * 9.81, 3)
            ay = round(float(accel["y"]) * 9.81, 3)
            az = round(float(accel["z"]) * 9.81, 3)
        else:
            # mock 모드: 중력 방향 기본값
            ax, ay, az = 0.0, 0.0, 9.81

        return {
            "roll":     round(corrected_roll,  1),
            "pitch":    round(corrected_pitch, 1),
            "yaw":      round(self.yaw,   1),
            "yaw_rate": round(angles.get("yaw_rate", 0.0), 1),
            "accel_x":  ax,
            "accel_y":  ay,
            "accel_z":  az,
            "pos_x":    round(self._pos_x, 1),
            "pos_y":    round(self._pos_y, 1),
        }

    def get_heading(self) -> float:
        """waypoint_recorder.py 연동 인터페이스 동기화"""
        self.get_filtered_angles()
        return round(self.yaw, 1)

    def reset_yaw(self):
        self.yaw          = 0.0
        self._pos_x       = 0.0
        self._pos_y       = 0.0
        self._last_accel_x = 0.0
        self._last_accel_y = 0.0

    def reset_position(self):
        self._pos_x       = 0.0
        self._pos_y       = 0.0
        self._last_accel_x = 0.0
        self._last_accel_y = 0.0

    def close(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None