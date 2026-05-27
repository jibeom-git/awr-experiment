import sys, time
sys.path.insert(0, '/home/pi/insite')
from sensors.hx711 import HX711

HX711_REF_UNIT = -163.03
HX711_DEADZONE = 3.0
HX711_SMOOTH   = 10

print("HX711 초기화 중...")
hx = HX711()
time.sleep(1)
hx.REF_UNIT_A = HX711_REF_UNIT

print("바구니를 올려주세요.")
input("준비되면 Enter ▶ ")
print("영점 설정 중... (50회 평균)")
hx.tare(samples=50)
print("영점 완료\n")

_weight_buf  = []
_prev_weight = 0.0

def _read_weight_stable():
    global _weight_buf, _prev_weight

    raw = hx.get_grams()

    _weight_buf.append(raw)
    if len(_weight_buf) > HX711_SMOOTH:
        _weight_buf.pop(0)

    if len(_weight_buf) < 5:
        return 0.0

    avg = sum(_weight_buf) / len(_weight_buf)

    if abs(avg) <= HX711_DEADZONE:
        _weight_buf.clear()
        _prev_weight = 0.0
        return 0.0

    result = round(avg, 1)
    _prev_weight = result
    return result

print("측정 시작 (Ctrl+C 로 종료)")
print("─" * 50)
print(f"{'시간':>8} | {'raw(g)':>10} | {'출력(g)':>10} | 버퍼")
print("─" * 50)

start = time.time()
try:
    while True:
        stable_val = _read_weight_stable()
        raw_val    = (hx.lastVal - hx.OFFSET_A) / hx.REF_UNIT_A
        elapsed    = time.time() - start
        buf_len    = len(_weight_buf)
        print(f"{elapsed:>7.1f}s | {raw_val:>10.1f} | {stable_val:>10.1f} | [{buf_len:>2}/{HX711_SMOOTH}]")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n종료")
    hx.close()
