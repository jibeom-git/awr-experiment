# sensors/mpu6050.py
# MPU-6050 IMU 드라이버
# I2C5 소프트웨어 버스 (GPIO12=SDA, GPIO13=SCL), smbus2
#
# [수정 내역]
# 1. yaw 누적 계산 추가
#    - 기존: get_filtered_angles()가 yaw_rate(각속도)만 반환, 누적 yaw 없음
#    - 수정: self.yaw에 gyro_z * dt 적분하여 360° 범위로 정규화
# 2. get_all() 메서드 추가
#    - 서버(insite_dashboard_server.py)에서 imu.get_all() 호출 → AttributeError 방지
#    - 반환: {"roll": float, "pitch": float, "yaw": float}
# 3. get_heading() 메서드 추가
#    - waypoint_recorder.py에서 heading 값 단독 취득 용도
# 4. SMBus 초기화 실패 시 예외 전파 대신 경고 출력 후 mock 모드 진입
#    (Pi 외 환경에서 import 만으로 크래시 나던 문제 방지)

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
    I2C5 소프트웨어 버스를 직접 제어하고 오프셋 교정 및 상보 필터를 수행하는 IMU 제어 클래스.
    상보 필터(Complementary Filter)로 roll/pitch를 계산하고,
    자이로 Z축 적분으로 yaw를 누적하여 제공한다.
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

                # 1. Sleep 모드 해제 + PLL with X axis gyroscope reference (클록 안정화)
                self.bus.write_byte_data(self.address, 0x6B, 0x01)

                # 2. DLPF (Digital Low Pass Filter) 설정 — 레지스터 0x1A
                #    CONFIG[2:0] = 3 → 가속도계 44Hz / 자이로 42Hz 저역 필터
                #    진동·충격 고주파 노이즈를 하드웨어 레벨에서 차단
                #    옵션: 0=260Hz(끄기), 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
                self.bus.write_byte_data(self.address, 0x1A, 0x03)

                # 3. 자이로 풀스케일 범위: ±250°/s (레지스터 0x1B, FS_SEL=0)
                #    분해능 131 LSB/°/s — 민감도 최대
                self.bus.write_byte_data(self.address, 0x1B, 0x00)

                # 4. 가속도계 풀스케일 범위: ±2g (레지스터 0x1C, AFS_SEL=0)
                #    분해능 16384 LSB/g — 민감도 최대
                self.bus.write_byte_data(self.address, 0x1C, 0x00)

                # 5. 샘플레이트 분주기 설정 — 레지스터 0x19
                #    SMPLRT_DIV = 9 → 샘플레이트 = 1000Hz / (1+9) = 100Hz
                #    DLPF 활성 시 자이로 기준 클록 1kHz
                self.bus.write_byte_data(self.address, 0x19, 0x09)
            except Exception as e:
                print(f"[IMU] I2C 버스 개방 실패 — mock 모드: {e}")
                self._mock = True

        # 교정 오프셋
        self.offsets = {
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
        }
        self.offset_file = os.path.expanduser("~/insite/gyro_offset.txt")
        if not self._mock:
            self.load_offsets()

        # 상보 필터 가중치
        # α=0.98: 자이로 바이어스 드리프트 누적 심함 (정지 10초에 0.7° 밀림)
        # α=0.95: 가속도계 보정 비중 5% → 드리프트 억제, 순간 노이즈 소폭 증가
        # 실측 노이즈 std=0.02~0.05° 이므로 0.95에서도 충분히 낮은 노이즈 유지
        self.alpha     = 0.95
        self.last_time = time.time()
        self.roll      = 0.0
        self.pitch     = 0.0
        # yaw 누적 변수 (자이로 Z축 적분)
        self.yaw            = 0.0
        # 2D 위치 dead-reckoning 변수 (get_filtered_angles 내부에서 사용)
        self._pos_x         = 0.0
        self._pos_y         = 0.0
        self._last_accel_x  = 0.0
        self._last_accel_y  = 0.0

    # ── 오프셋 로드 ──────────────────────────────────────────────────────────
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
            print(f"[IMU] 오프셋 로드 예외 — 기본값 0 사용: {e}")

    # ── 레지스터 읽기 ─────────────────────────────────────────────────────────
    def read_raw_word(self, reg: int) -> float:
        """
        16비트 부호 있는 정수(2의 보수) 변환.
        소프트웨어 I2C 버스 특성상 간헐 오류가 발생하므로 최대 3회 재시도.
        """
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
                    # 3회 모두 실패 시에만 출력 (1~2회 오류는 조용히 재시도)
                    print(f"[IMU] 레지스터 읽기 실패 (0x{reg:02X}): {e}")
                time.sleep(0.002)
        return 0.0

    # ── 교정 ─────────────────────────────────────────────────────────────────
    def calibrate_sensors(self, samples: int = 1000):
        """정지 상태에서 오프셋 바이어스 연산 후 파일 저장"""
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

        # 중력 가속도 1g (16384 LSB) 동적 제거
        # 하드코딩(az만)이 아닌, 실제 중력이 걸린 축을 동적으로 찾아 제거
        # 이 로봇의 경우 실측상 ay ≈ 1g (Y축에 중력)이지만
        # 장착 방향이 바뀔 수 있으므로 일반화
        accel_offsets = {k: self.offsets[k] for k in ("ax", "ay", "az")}
        gravity_axis = max(accel_offsets, key=lambda k: abs(accel_offsets[k]))
        sign = 1.0 if accel_offsets[gravity_axis] > 0 else -1.0
        self.offsets[gravity_axis] -= sign * 16384.0
        print(f"[IMU] 중력축 감지: {gravity_axis} (보정 전 {accel_offsets[gravity_axis]:.1f} → 후 {self.offsets[gravity_axis]:.1f})")
        with open(self.offset_file, "w") as f:
            for k in ["ax", "ay", "az", "gx", "gy", "gz"]:
                f.write(f"{self.offsets[k]}\n")
        print("[IMU] 오프셋 파일 저장 완료")

    # ── 센서 데이터 취득 ──────────────────────────────────────────────────────
    def get_accel(self) -> dict:
        """오프셋 제거된 가속도 (g 단위)"""
        return {
            "x": (self.read_raw_word(0x3B) - self.offsets["ax"]) / 16384.0,
            "y": (self.read_raw_word(0x3D) - self.offsets["ay"]) / 16384.0,
            "z": (self.read_raw_word(0x3F) - self.offsets["az"]) / 16384.0,
        }

    def get_gyro(self) -> dict:
        """오프셋 제거된 각속도 (°/s 단위)"""
        return {
            "x": (self.read_raw_word(0x43) - self.offsets["gx"]) / 131.0,
            "y": (self.read_raw_word(0x45) - self.offsets["gy"]) / 131.0,
            "z": (self.read_raw_word(0x47) - self.offsets["gz"]) / 131.0,
        }
    
    def get_gyro_raw(self) -> dict:
        """오프셋 미적용 원본 자이로값 (캘리브레이션용)"""
        return {
            "x": self.read_raw_word(0x43) / 131.0,
            "y": self.read_raw_word(0x45) / 131.0,
            "z": self.read_raw_word(0x47) / 131.0,
        }

    # ── 상보 필터 ─────────────────────────────────────────────────────────────
    def get_filtered_angles(self) -> dict:
        """
        상보 필터로 roll/pitch 계산, 자이로 Z축 적분으로 yaw 누적.
        반환: {"roll": float, "pitch": float, "yaw_rate": float}
        (기존 인터페이스 유지 + yaw_rate 포함)
        """
        if self._mock:
            # mock 모드: 시간 기반 더미 값 반환
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

        # 초기 진입 또는 루프 지연 시 타임스탬프 왜곡 방지
        # MPU-6050 100Hz 샘플링 기준 최소 dt=0.005s, 루프 지연 허용 최대 0.2s
        if dt <= 0.001 or dt > 0.2:
            dt = 0.01

        # 가속도 기반 절대 기하각 추정
        denom       = math.sqrt(accel["y"] ** 2 + accel["z"] ** 2) or 1e-6
        accel_roll  = math.atan2(accel["y"], accel["z"]) * 180.0 / math.pi
        accel_pitch = math.atan2(-accel["x"], denom)     * 180.0 / math.pi

        # 가속도계 스파이크 클램프: 현재 필터 값과 45° 이상 차이나면 무시
        # 충격·진동 시 순간적으로 튀는 가속도계 값이 필터에 반영되는 것을 방지
        MAX_ACCEL_DIFF = 45.0
        if abs(accel_roll  - self.roll)  > MAX_ACCEL_DIFF:
            accel_roll  = self.roll
        if abs(accel_pitch - self.pitch) > MAX_ACCEL_DIFF:
            accel_pitch = self.pitch

        # 상보 필터 수식 (α=0.95)
        self.roll  = self.alpha * (self.roll  + gyro["x"] * dt) + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gyro["y"] * dt) + (1.0 - self.alpha) * accel_pitch

        # yaw 누적 (자이로 Z축 적분, 0~360° 범위 정규화)
        self.yaw = (self.yaw + gyro["z"] * dt) % 360.0
        if self.yaw < 0:
            self.yaw += 360.0

        # pos_x/y: 가속도 이중 적분은 드리프트가 너무 커서 사용 불가
        # 향후 모터 encoder 기반으로 교체 예정. 현재는 0 유지.
        # self._pos_x, self._pos_y 는 reset_yaw() 로만 초기화 가능

        return {
            "roll":     round(self.roll,  2),
            "pitch":    round(self.pitch, 2),
            "yaw_rate": round(gyro["z"],  2),
        }

    # ── 공개 인터페이스 (서버용) ──────────────────────────────────────────────
    def get_all(self) -> dict:
        """
        서버(insite_dashboard_server.py)에서 imu.get_all() 로 호출.
        반환:
          roll      : 좌우 기울기 (°)
          pitch     : 전후 기울기 (°)
          yaw       : 방위각 누적 0~360° (heading)
          yaw_rate  : 현재 회전 각속도 (°/s) — 회전 방향 판단에 사용
          pos_x     : 전방 상대 변위 추정 (cm, dead-reckoning)
          pos_y     : 측방 상대 변위 추정 (cm, dead-reckoning)
        """
        angles = self.get_filtered_angles()  # 내부 상태 갱신
        return {
            "roll":     round(self.roll,  1),
            "pitch":    round(self.pitch, 1),
            "yaw":      round(self.yaw,   1),
            "yaw_rate": round(angles.get("yaw_rate", 0.0), 1),
            "pos_x":    round(self._pos_x, 1),
            "pos_y":    round(self._pos_y, 1),
        }

    def get_heading(self) -> float:
        """
        waypoint_recorder.py에서 heading 단독 취득 용도.
        yaw 값 (0~360°) 반환.
        """
        self.get_filtered_angles()
        return round(self.yaw, 1)

    def reset_yaw(self):
        """yaw + 위치 누적값 초기화 (출발점 기준 재설정 시 호출)"""
        self.yaw          = 0.0
        self._pos_x       = 0.0
        self._pos_y       = 0.0
        self._last_accel_x = 0.0
        self._last_accel_y = 0.0

    def reset_position(self):
        """위치만 초기화 (yaw 유지)"""
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