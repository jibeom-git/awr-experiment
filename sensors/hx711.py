# sensors/hx711.py
# HX711 로드셀 드라이버
# tatobari/hx711py (hx711v0_5_1.py) 구조 기반으로 재작성
# DAT: GPIO5, CLK: GPIO6
#
# [노이즈/음수값 원인과 해결]
#
# 원인 1 — 온도 드리프트 & 크리프(creep)
#   HX711은 온도 변화와 장시간 하중으로 인해 offset이 서서히 이동함.
#   해결: 이동평균(window=20) + 중앙값 기반 이상값(spike) 필터 적용.
#
# 원인 2 — 물체 제거 후 음수
#   tare offset이 실제 제로보다 살짝 위에 잡혀 있으면 제거 후 음수 가능.
#   해결: tare 시 samples 수를 늘리고 안정화 대기(0.1s 간격)를 적용.
#         데드밴드(±DEADZONE_G) 이하는 0으로 강제 처리.
#
# 원인 3 — Pi 4 타이밍 문제
#   기존 구조에서 이미 interrupt 방식으로 해결됨. 유지.
#
# [캘리브레이션 워크플로우]
#   1. 바구니(용기)만 올린 상태에서 tare(samples=50) → offset 저장
#   2. 알고 있는 무게(g)의 추를 올리고 calibrate(known_grams) 호출
#      → REF_UNIT 계산 후 ~/insite/loadcell_cal.txt 에 저장
#   3. 이후 get_weight() 호출 시 자동으로 캘리브레이션 값 적용

import threading
import time
import os
from collections import deque
from statistics import median

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    _GPIO_AVAILABLE = False
    print("[HX711] RPi.GPIO 없음 — mock 모드로 동작")

DAT             = 5
CLK             = 6
READY_TIMEOUT_S = 1.0    # isReady() 최대 대기 시간 (초)
DEADZONE_G      = 5.0    # ±5g 이하는 0으로 처리 (크리프 노이즈 억제)
SPIKE_FACTOR    = 4.0    # 이동평균 대비 SPIKE_FACTOR배 초과 시 이상값으로 제거
SMOOTH_WINDOW   = 20     # 이동평균 윈도우 크기
CAL_FILE        = os.path.expanduser('~/insite/loadcell_cal.txt')


class HX711:
    def __init__(self, dout: int = DAT, pd_sck: int = CLK, gain: int = 128):
        self.DOUT   = dout
        self.PD_SCK = pd_sck
        self._mock  = not _GPIO_AVAILABLE

        self.readLock   = threading.Lock()
        self.GAIN       = None
        self.OFFSET_A   = 0.0
        self.REF_UNIT_A = 1.0
        self.byteFormat = 'MSB'
        self.bitFormat  = 'MSB'
        self.lastVal    = 0

        # 노이즈 필터 버퍼
        self._buf: deque[float] = deque(maxlen=SMOOTH_WINDOW)

        if self._mock:
            print("[HX711] mock 모드")
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.PD_SCK, GPIO.OUT)
            GPIO.setup(self.DOUT,   GPIO.IN)
            self.setGain(gain)
            time.sleep(0.5)
            # 저장된 캘리브레이션 로드
            self._load_cal()
        except Exception as e:
            print(f"[HX711] 초기화 실패 — mock 모드: {e}")
            self._mock = True

    # ── 캘리브레이션 저장/로드 ──────────────────────────────────────────────
    def _load_cal(self):
        """~/insite/loadcell_cal.txt 에서 offset, ref_unit 로드"""
        if not os.path.exists(CAL_FILE):
            return
        try:
            with open(CAL_FILE) as f:
                lines = f.read().splitlines()
            if len(lines) >= 2:
                self.OFFSET_A   = float(lines[0])
                self.REF_UNIT_A = float(lines[1])
                print(f"[HX711] 캘리브레이션 로드 | offset={self.OFFSET_A:.0f}  ref={self.REF_UNIT_A:.4f}")
        except Exception as e:
            print(f"[HX711] 캘리브레이션 로드 실패: {e}")

    def _save_cal(self):
        """offset, ref_unit 를 파일에 저장"""
        try:
            os.makedirs(os.path.dirname(CAL_FILE) or '.', exist_ok=True)
            with open(CAL_FILE, 'w') as f:
                f.write(f"{self.OFFSET_A}\n{self.REF_UNIT_A}\n")
            print(f"[HX711] 캘리브레이션 저장 | offset={self.OFFSET_A:.0f}  ref={self.REF_UNIT_A:.4f}")
        except Exception as e:
            print(f"[HX711] 캘리브레이션 저장 실패: {e}")

    # ── 전원 제어 ──────────────────────────────────────────────────────────
    def powerDown(self):
        self.readLock.acquire()
        GPIO.output(self.PD_SCK, False)
        GPIO.output(self.PD_SCK, True)
        time.sleep(0.0001)
        self.readLock.release()

    def powerUp(self):
        self.readLock.acquire()
        GPIO.output(self.PD_SCK, False)
        time.sleep(0.0001)
        self.readLock.release()

    def reset(self):
        self.powerDown()
        self.powerUp()

    # ── Gain 설정 ──────────────────────────────────────────────────────────
    def setGain(self, gain: int) -> bool:
        if gain == 128:
            self.GAIN = 1
        elif gain == 64:
            self.GAIN = 3
        elif gain == 32:
            self.GAIN = 2
        else:
            return False
        if not self._mock:
            self.reset()
            GPIO.output(self.PD_SCK, False)
            self.readRawBytes()   # 첫 샘플 버림 (gain 설정 반영)
        return True

    # ── 비트/바이트 읽기 ───────────────────────────────────────────────────
    def isReady(self) -> bool:
        return GPIO.input(self.DOUT) == GPIO.LOW

    def readNextBit(self) -> int:
        GPIO.output(self.PD_SCK, True)
        GPIO.output(self.PD_SCK, False)
        return int(GPIO.input(self.DOUT))

    def readNextByte(self) -> int:
        val = 0
        for _ in range(8):
            if self.bitFormat == 'MSB':
                val <<= 1
                val |= self.readNextBit()
            else:
                val >>= 1
                val |= self.readNextBit() * 0x80
        return val

    def readRawBytes(self) -> list:
        if self._mock:
            return [0x80, 0x00, 0x00]

        self.readLock.acquire()
        deadline = time.time() + READY_TIMEOUT_S
        while not self.isReady():
            if time.time() > deadline:
                self.readLock.release()
                print("[HX711] 센서 응답 타임아웃")
                return [0x80, 0x00, 0x00]
            time.sleep(0.001)

        b1 = self.readNextByte()
        b2 = self.readNextByte()
        b3 = self.readNextByte()

        for _ in range(self.GAIN or 1):
            self.readNextBit()

        self.readLock.release()

        return [b1, b2, b3] if self.byteFormat == 'MSB' else [b3, b2, b1]

    # ── 변환 ───────────────────────────────────────────────────────────────
    def convertFromTwosComplement24bit(self, val: int) -> int:
        return -(val & 0x800000) + (val & 0x7FFFFF)

    def rawBytesToLong(self, rawBytes: list) -> int:
        twos   = (rawBytes[0] << 16) | (rawBytes[1] << 8) | rawBytes[2]
        signed = self.convertFromTwosComplement24bit(twos)
        self.lastVal = signed
        return int(signed)

    def getLong(self) -> int:
        return self.rawBytesToLong(self.readRawBytes())

    # ── 필터 적용 읽기 ─────────────────────────────────────────────────────
    def _read_filtered(self) -> float:
        """
        이동평균 + 스파이크 필터 적용 raw 값 반환.

        스파이크 필터:
          버퍼에 값이 SMOOTH_WINDOW/2 이상 쌓이면
          현재 버퍼 중앙값 기준으로 SPIKE_FACTOR 배 초과 편차는 이전 값으로 대체.
          물체를 올리거나 뗄 때 순간적으로 튀는 값(충격)을 억제.
        """
        if self._mock:
            return 0.0

        raw = float(self.getLong())

        # 버퍼가 절반 이상 찼으면 스파이크 판정
        if len(self._buf) >= SMOOTH_WINDOW // 2:
            med = median(self._buf)
            spread = max(abs(v - med) for v in self._buf) or 1.0
            if abs(raw - med) > SPIKE_FACTOR * spread:
                raw = med   # 이상값 → 현재 중앙값으로 대체

        self._buf.append(raw)
        return sum(self._buf) / len(self._buf)   # 이동평균 반환

    # ── 공개 API ───────────────────────────────────────────────────────────
    def get_raw(self) -> int:
        return self.getLong()

    def tare(self, samples: int = 50):
        """
        영점 설정.
        samples=50, 읽기 간격 100ms → 약 5초 소요.
        물체가 완전히 정지한 뒤 호출할 것.
        캘리브레이션 파일에도 저장.
        """
        if self._mock:
            print("[HX711] mock 모드 — tare 불가")
            return
        print(f"[HX711] tare 시작 ({samples}샘플, ~{samples*0.1:.0f}초 소요)...")
        vals = []
        for _ in range(samples):
            vals.append(float(self.getLong()))
            time.sleep(0.1)   # 100ms 간격 — 전기적 노이즈 평균화
        # 이상값 제거: 중앙값 ± 3σ 이내만 사용
        med = median(vals)
        std = (sum((v - med)**2 for v in vals) / len(vals)) ** 0.5 or 1.0
        clean = [v for v in vals if abs(v - med) <= 3 * std]
        self.OFFSET_A = sum(clean) / len(clean)
        self._buf.clear()
        self._save_cal()
        print(f"[HX711] tare 완료 | offset={self.OFFSET_A:.0f} (유효샘플 {len(clean)}/{samples})")

    def calibrate(self, known_grams: float):
        """
        캘리브레이션.
        알고 있는 무게(g)의 추를 로드셀에 올린 뒤 호출.
        REF_UNIT = (현재 raw 평균 - offset) / known_grams 로 계산.

        사용 예:
            hx711.tare(samples=50)          # 1단계: 빈 상태 영점
            # 100g 추를 올린다
            hx711.calibrate(100.0)          # 2단계: 캘리브레이션
        """
        if self._mock:
            print("[HX711] mock 모드 — 캘리브레이션 불가")
            return
        print(f"[HX711] 캘리브레이션 시작 (기준 무게: {known_grams}g, 20샘플)...")
        vals = [float(self.getLong()) for _ in range(20) if not time.sleep(0.1)]
        avg  = sum(vals) / len(vals)
        diff = avg - self.OFFSET_A
        if abs(diff) < 100:
            print(f"[HX711] 캘리브레이션 실패: raw 변화량({diff:.0f})이 너무 작음. 추가 올렸는지 확인.")
            return
        self.REF_UNIT_A = diff / known_grams
        self._buf.clear()
        self._save_cal()
        print(f"[HX711] 캘리브레이션 완료 | REF_UNIT={self.REF_UNIT_A:.4f}")

    def get_grams(self) -> float:
        """필터 적용 그램 값 반환. 데드밴드 ±DEADZONE_G 이하는 0."""
        if self._mock:
            return 0.0
        try:
            raw_avg = self._read_filtered()
            grams   = (raw_avg - self.OFFSET_A) / self.REF_UNIT_A
            if abs(grams) <= DEADZONE_G:
                return 0.0
            return round(grams, 1)
        except Exception as e:
            print(f"[HX711] get_grams 오류: {e}")
            return 0.0

    def get_weight(self) -> float:
        """서버 인터페이스 — get_grams() 별칭."""
        return self.get_grams()

    def read(self) -> dict:
        raw  = self.getLong()
        diff = raw - self.OFFSET_A
        return {'raw': raw, 'diff': diff}

    def setReferenceUnit(self, ref: float):
        """수동 REF_UNIT 설정 (calibrate() 대신 직접 지정할 때)"""
        self.REF_UNIT_A = ref
        self._save_cal()

    def close(self):
        if not self._mock and _GPIO_AVAILABLE:
            try:
                GPIO.cleanup([self.DOUT, self.PD_SCK])
            except Exception as e:
                print(f"[HX711] GPIO 해제 오류: {e}")