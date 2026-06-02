# sensors/led.py
# LED 통합 드라이버
# WS2812 RGB LED (GPIO10, SPI0) — 상태 색상 표시
# 전방 단색 LED 3개 (공식 Robot HAT 회로도 기준 핀 사용)
#
# 상태 정의:
#   running  — WS2812 초록 고정,  전방 LED OFF
#   thinking — WS2812 노랑 점멸,  전방 LED OFF
#   error    — WS2812 빨강 점멸,  전방 LED ON (경고 강조)
#
# [수정 내역]
# 1. GPIO 충돌 해소
#    - 기존: 전방 LED에 GPIO9(SPI0_MISO), GPIO11(SPI0_CLK) 사용
#      → WS2812가 SPI0(GPIO10)을 점유하고 있어 GPIO9/11과 버스 충돌 발생
#    - 수정: Robot HAT v3.3 회로도 기준 전방 단색 LED 실제 핀
#      GPIO25(L4), GPIO27(L5), GPIO22(L6) 사용
#      (공식 Robot HAT V3.1 회로도 X4/X5 RGB LED Module Port 참조)
#      ※ 실제 조립 구성이 다를 경우 LED_FRONT_PINS를 직접 수정할 것
# 2. SPI 초기화 및 spidev import 실패 시 mock 모드 진입
# 3. gpiozero LED import 실패 처리
# 4. _stop_blink() 스레드 join 타임아웃 추가 (무한 대기 방지)
# 5. close() 중복 호출 안전화

import threading
import time

try:
    import spidev
    import numpy
    _SPI_AVAILABLE = True
except ImportError:
    _SPI_AVAILABLE = False
    print("[LED] spidev/numpy 없음 — WS2812 mock 모드")

try:
    from gpiozero import LED as GPLED
    _GPIOZERO_AVAILABLE = True
except (ImportError, Exception):
    _GPIOZERO_AVAILABLE = False
    print("[LED] gpiozero 없음 — 전방 LED mock 모드")

# WS2812 설정
LED_COUNT  = 2
SPI_BUS    = 0
SPI_DEVICE = 0
SPI_SPEED  = 6400000

# 전방 단색 LED — 비활성화
# set_error() 에서만 켜는 용도였으나 항상 켜진 채로 유지되는 문제가 있어 끔.
# GPIO 핀을 아예 초기화하지 않으면 HAT 기본 상태(꺼짐)가 유지됨.
# 향후 필요 시 아래 리스트에 핀 번호를 넣으면 재활성화 가능.
LED_FRONT_PINS = []   # 비활성화: 원래 [25, 27, 22]


class LEDController:
    def __init__(self):
        self._mock_spi   = not _SPI_AVAILABLE
        self._mock_front = not _GPIOZERO_AVAILABLE
        self._closed     = False

        # WS2812 SPI 초기화
        self.spi = None
        if not self._mock_spi:
            try:
                self.spi = spidev.SpiDev()
                self.spi.open(SPI_BUS, SPI_DEVICE)
                self.spi.max_speed_hz = SPI_SPEED
                self.spi.mode = 0
            except Exception as e:
                print(f"[LED] SPI 초기화 실패 — WS2812 mock 모드: {e}")
                self._mock_spi = True
                self.spi = None

        # 전방 단색 LED 초기화
        self.front_leds = []
        if not self._mock_front:
            for pin in LED_FRONT_PINS:
                try:
                    self.front_leds.append(GPLED(pin))
                except Exception as e:
                    print(f"[LED] 전방 LED GPIO{pin} 초기화 실패: {e}")

        # WS2812 상태
        self.led_count  = LED_COUNT
        self.led_color  = [0] * LED_COUNT * 3
        self.brightness = 255
        # GRB 순서 (WS2812 표준)
        self.r_off, self.g_off, self.b_off = 1, 0, 2

        # 점멸 스레드
        self.blink_thread = None
        self.blink_stop   = threading.Event()

        self._all_off()
        self._front_off()

    # ── WS2812 내부 메서드 ───────────────────────────────────────────────
    def _set_pixel(self, index: int, r: int, g: int, b: int):
        if self._mock_spi:
            return
        p = [0, 0, 0]
        p[self.r_off] = round(r * self.brightness / 255)
        p[self.g_off] = round(g * self.brightness / 255)
        p[self.b_off] = round(b * self.brightness / 255)
        for i in range(3):
            self.led_color[index * 3 + i] = p[i]

    def _show(self):
        if self._mock_spi or self.spi is None:
            return
        try:
            d  = numpy.array(self.led_color).ravel()
            tx = numpy.zeros(len(d) * 8, dtype=numpy.uint8)
            for i, byte in enumerate(d):
                for j in range(8):
                    tx[i * 8 + j] = 0xF8 if (byte >> (7 - j)) & 1 else 0xC0
            self.spi.xfer(tx.tolist(), SPI_SPEED)
        except Exception as e:
            print(f"[LED] SPI 전송 오류: {e}")

    def _all_off(self):
        for i in range(self.led_count):
            self._set_pixel(i, 0, 0, 0)
        self._show()

    # ── 전방 단색 LED 메서드 ─────────────────────────────────────────────
    def _front_on(self):
        for led in self.front_leds:
            try:
                led.on()
            except Exception:
                pass

    def _front_off(self):
        for led in self.front_leds:
            try:
                led.off()
            except Exception:
                pass

    # ── 점멸 제어 ─────────────────────────────────────────────────────────
    def _stop_blink(self):
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_stop.set()
            # [수정] join 타임아웃 1초 — 무한 대기 방지
            self.blink_thread.join(timeout=1.0)
            self.blink_stop.clear()

    def _blink_loop(self, r: int, g: int, b: int, interval: float):
        while not self.blink_stop.is_set():
            for i in range(self.led_count):
                self._set_pixel(i, r, g, b)
            self._show()
            time.sleep(interval)
            self._all_off()
            time.sleep(interval)

    def _start_blink(self, r: int, g: int, b: int, interval: float):
        self._stop_blink()
        self.blink_thread = threading.Thread(
            target=self._blink_loop,
            args=(r, g, b, interval),
            daemon=True
        )
        self.blink_thread.start()

    # ── 공개 상태 인터페이스 ──────────────────────────────────────────────
    def set_running(self):
        """완전 소등 (WS2812 + 전방 LED)"""
        self._stop_blink()
        self._all_off()
        self._front_off()

    def set_thinking(self):
        """완전 소등 유지 (WS2812 점멸 비활성화)"""
        self._stop_blink()
        self._all_off()
        self._front_off()

    def set_error(self):
        """오류 상태 — WS2812 빨강 빠른 점멸 (디버깅 시각화 복원)"""
        self._front_off()
        self._start_blink(255, 0, 0, interval=0.15)

    def set_color(self, r: int, g: int, b: int):
        """WS2812 색 설정 인터페이스 유지 (현재 비활성화 — 항상 소등)"""
        self._stop_blink()
        self._all_off()
        self._front_off()

    def off(self):
        """전체 소등"""
        self._stop_blink()
        self._all_off()
        self._front_off()

    def close(self):
        """
        [수정] _closed 플래그로 중복 close 방지
        """
        if self._closed:
            return
        self._closed = True
        self.off()
        if self.spi is not None:
            try:
                self.spi.close()
            except Exception:
                pass
        for led in self.front_leds:
            try:
                led.close()
            except Exception:
                pass