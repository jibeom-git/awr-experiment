# sensors/led.py
# LED 통합 드라이버
# WS2812 RGB LED (GPIO10, SPI) — 상태 색상 표시
# 전방 단색 LED 3개 (GPIO9, GPIO25, GPIO11) — 오류 시 보조 경고
#
# 상태 정의:
#   running  — WS2812 초록 고정,  전방 LED OFF
#   thinking — WS2812 노랑 점멸,  전방 LED OFF
#   error    — WS2812 빨강 점멸,  전방 LED ON (경고 강조)

import spidev
import numpy
import threading
import time
from gpiozero import LED

# WS2812 설정
LED_COUNT   = 2
SPI_BUS     = 0
SPI_DEVICE  = 0
SPI_SPEED   = 6400000

# 전방 단색 LED GPIO 핀 (공식 문서 기준)
LED_FRONT_PINS = [9, 25, 11]

class LEDController:
    def __init__(self):
        # WS2812 SPI 초기화
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        self.spi.mode = 0

        # 전방 단색 LED 초기화
        self.front_leds = [LED(pin) for pin in LED_FRONT_PINS]

        # WS2812 상태
        self.led_count      = LED_COUNT
        self.led_color      = [0] * LED_COUNT * 3
        self.brightness     = 255
        self.r_off, self.g_off, self.b_off = 1, 0, 2

        # 점멸 스레드
        self.blink_thread = None
        self.blink_stop   = threading.Event()

        self._all_off()
        self._front_off()

    # ── WS2812 내부 메서드 ──────────────────────────────────
    def _set_pixel(self, index, r, g, b):
        p = [0, 0, 0]
        p[self.r_off] = round(r * self.brightness / 255)
        p[self.g_off] = round(g * self.brightness / 255)
        p[self.b_off] = round(b * self.brightness / 255)
        for i in range(3):
            self.led_color[index * 3 + i] = p[i]

    def _show(self):
        d = numpy.array(self.led_color).ravel()
        tx = numpy.zeros(len(d) * 8, dtype=numpy.uint8)
        for i, byte in enumerate(d):
            for j in range(8):
                tx[i * 8 + j] = 0xF8 if (byte >> (7 - j)) & 1 else 0xC0
        self.spi.xfer(tx.tolist(), SPI_SPEED)

    def _all_off(self):
        for i in range(self.led_count):
            self._set_pixel(i, 0, 0, 0)
        self._show()

    # ── 전방 단색 LED 메서드 ───────────────────────────────
    def _front_on(self):
        for led in self.front_leds:
            led.on()

    def _front_off(self):
        for led in self.front_leds:
            led.off()

    # ── 점멸 제어 ──────────────────────────────────────────
    def _stop_blink(self):
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_stop.set()
            self.blink_thread.join()
            self.blink_stop.clear()

    def _blink_loop(self, r, g, b, interval):
        while not self.blink_stop.is_set():
            for i in range(self.led_count):
                self._set_pixel(i, r, g, b)
            self._show()
            time.sleep(interval)
            self._all_off()
            time.sleep(interval)

    def _start_blink(self, r, g, b, interval):
        self._stop_blink()
        self.blink_thread = threading.Thread(
            target=self._blink_loop,
            args=(r, g, b, interval),
            daemon=True
        )
        self.blink_thread.start()

    # ── 공개 상태 인터페이스 ───────────────────────────────
    def set_running(self):
        """정상 가동 — WS2812 초록 고정, 전방 LED OFF"""
        self._stop_blink()
        self._front_off()
        for i in range(self.led_count):
            self._set_pixel(i, 0, 255, 0)
        self._show()

    def set_thinking(self):
        """모델 추론 중 — WS2812 노랑 점멸, 전방 LED OFF"""
        self._front_off()
        self._start_blink(255, 200, 0, interval=0.4)

    def set_error(self):
        """오류/관리자 판단 필요 — WS2812 빨강 점멸 + 전방 LED ON"""
        self._front_on()
        self._start_blink(255, 0, 0, interval=0.15)

    def set_color(self, r, g, b):
        """WS2812 단일 색상 직접 설정"""
        self._stop_blink()
        self._front_off()
        for i in range(self.led_count):
            self._set_pixel(i, r, g, b)
        self._show()

    def off(self):
        """전체 소등"""
        self._stop_blink()
        self._all_off()
        self._front_off()

    def close(self):
        self.off()
        self.spi.close()
        for led in self.front_leds:
            led.close()