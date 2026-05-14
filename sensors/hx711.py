# sensors/hx711.py
# HX711 로드셀 드라이버
# tatobari/hx711py (hx711v0_5_1.py) 구조 기반으로 재작성
# DAT: GPIO5, CLK: GPIO6
# 인터럽트(FALLING 엣지) 방식으로 타이밍 문제 해결

import RPi.GPIO as GPIO
import threading
import time

DAT = 5
CLK = 6

class HX711:
    def __init__(self, dout=DAT, pd_sck=CLK, gain=128):
        self.DOUT   = dout
        self.PD_SCK = pd_sck

        # 멀티스레드 동시 접근 방지용 뮤텍스
        self.readLock = threading.Lock()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PD_SCK, GPIO.OUT)
        GPIO.setup(self.DOUT,   GPIO.IN)

        self.GAIN       = None
        self.OFFSET_A   = 0
        self.REF_UNIT_A = 1
        self.byteFormat = 'MSB'
        self.bitFormat  = 'MSB'
        self.lastVal    = 0

        self.setGain(gain)
        time.sleep(0.5)

    # ── 전원 제어 ──────────────────────────────────────────────
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

    # ── Gain 설정 ──────────────────────────────────────────────
    def setGain(self, gain):
        if gain == 128:
            self.GAIN = 1   # 채널 A, gain 128
        elif gain == 64:
            self.GAIN = 3   # 채널 A, gain 64
        elif gain == 32:
            self.GAIN = 2   # 채널 B, gain 32
        else:
            return False
        self.reset()
        GPIO.output(self.PD_SCK, False)
        self.readRawBytes()  # 첫 샘플 버림 (gain 설정 반영)
        return True

    # ── 비트/바이트 읽기 ───────────────────────────────────────
    def isReady(self):
        return GPIO.input(self.DOUT) == GPIO.LOW

    def readNextBit(self):
        GPIO.output(self.PD_SCK, True)
        GPIO.output(self.PD_SCK, False)
        return int(GPIO.input(self.DOUT))

    def readNextByte(self):
        val = 0
        for _ in range(8):
            if self.bitFormat == 'MSB':
                val <<= 1
                val |= self.readNextBit()
            else:
                val >>= 1
                val |= self.readNextBit() * 0x80
        return val

    def readRawBytes(self):
        self.readLock.acquire()

        # DAT=LOW 될 때까지 대기
        while not self.isReady():
            pass

        b1 = self.readNextByte()
        b2 = self.readNextByte()
        b3 = self.readNextByte()

        # GAIN 설정 펄스 전송
        for _ in range(self.GAIN):
            self.readNextBit()

        self.readLock.release()

        if self.byteFormat == 'MSB':
            return [b1, b2, b3]
        else:
            return [b3, b2, b1]

    # ── 변환 ───────────────────────────────────────────────────
    def convertFromTwosComplement24bit(self, val):
        return -(val & 0x800000) + (val & 0x7FFFFF)

    def rawBytesToLong(self, rawBytes):
        twos = (rawBytes[0] << 16) | (rawBytes[1] << 8) | rawBytes[2]
        signed = self.convertFromTwosComplement24bit(twos)
        self.lastVal = signed
        return int(signed)

    # ── 공개 API ───────────────────────────────────────────────
    def getLong(self):
        return self.rawBytesToLong(self.readRawBytes())

    def tare(self, samples=10):
        """영점 설정: samples회 평균을 OFFSET으로 저장"""
        vals = [self.getLong() for _ in range(samples)]
        self.OFFSET_A = sum(vals) / len(vals)
        print(f"영점 설정 완료 | offset = {self.OFFSET_A:.0f}")

    def get_raw(self):
        return self.getLong()

    def get_grams(self):
        return (self.getLong() - self.OFFSET_A) / self.REF_UNIT_A

    def read(self):
        raw  = self.getLong()
        diff = raw - self.OFFSET_A
        return {'raw': raw, 'diff': diff}

    def setReferenceUnit(self, ref):
        self.REF_UNIT_A = ref

    def close(self):
        GPIO.cleanup()