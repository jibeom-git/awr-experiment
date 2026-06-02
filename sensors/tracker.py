# sensors/tracker.py
# Adeept 3채널 IR 라인트래킹 센서 및 교차로 에지 디텍션 코어

import time
from gpiozero import DigitalInputDevice

class LineTracker:
    _threshold = 0.5

    def __init__(self, pin_s1: int = 17, pin_s2: int = 27, pin_s3: int = 22):
        # S1=GPIO17(Pin11), S2=GPIO27(Pin13), S3=GPIO22(Pin15) — 하드웨어 배선 기준
        self.s1 = DigitalInputDevice(pin_s1)   
        self.s2 = DigitalInputDevice(pin_s2)   
        self.s3 = DigitalInputDevice(pin_s3)   

        self._white_vals  = [0, 0, 0]   
        self._black_vals  = [0, 0, 0]   
        self._calibrated  = False
        
        # [AI 파트 추가] 노드 통과 시 다중 카운팅 노이즈를 제어하기 위한 직전 상태 메모리 레지스터
        self._was_junction = False

        print("[Tracker] 초기화 완료 | S1=GPIO17 S2=GPIO27 S3=GPIO22")

    def read_raw(self) -> tuple[int, int, int]:
        return (
            int(self.s1.value),
            int(self.s2.value),
            int(self.s3.value),
        )

    def read(self) -> dict:
        l, c, r = self.read_raw()
        pattern = (
            ('L' if l else '_') +
            ('C' if c else '_') +
            ('R' if r else '_')
        )
        return {
            'left':    l,
            'center':  c,
            'right':   r,
            'on_line': bool(l or c or r),
            'pattern': pattern,
        }

    def is_junction(self) -> bool:
        l, c, r = self.read_raw()
        return bool(l and c and r)

    def detect_node_trigger(self) -> bool:
        """
        [AI 파트 추가] AGV가 검은색 정사각형 교차로 노드에 진입하는 물리적 순간(Rising Edge)을 포착합니다.
        차체가 노드 위에 머무는 동안(High State) 중복 카운트가 발생하여 경로 맵이 이탈하는 현상을 완벽히 차단합니다.
        """
        current_junction = self.is_junction()
        trigger = False

        # 상태 변화 패턴 식별 논리: (이전 루프 = False) 이고 (현재 루프 = True) 인 조건 판별
        if current_junction and not self._was_junction:
            trigger = True
            print("[Tracker AI Internal] 교차로 노드 인입 신호 발생 포착")

        self._was_junction = current_junction
        return trigger

    def is_on_line(self) -> bool:
        l, c, r = self.read_raw()
        return bool(l or c or r)

    def is_lost(self) -> bool:
        l, c, r = self.read_raw()
        return not bool(l or c or r)

    def get_error(self) -> int:
        l, c, r = self.read_raw()

        if   l and     c and     r: return  0   
        elif not l and     c and not r: return  0   
        elif l and     c and not r: return -1   
        elif not l and     c and     r: return  1   
        elif l and not c and not r: return -1   
        elif not l and not c and     r: return  1   
        elif l and not c and     r: return  0   
        else:                          return  9   

    def calibrate(self, samples: int = 30) -> dict:
        print("[Tracker] ── 캘리브레이션 시작 ──")
        print("[Tracker] 흰 바닥에 올려놓고 Enter...")
        input()
        white_counts = [0, 0, 0]
        for _ in range(samples):
            l, c, r = self.read_raw()
            white_counts[0] += l
            white_counts[1] += c
            white_counts[2] += r
            time.sleep(0.05)
        white_avg = [round(v / samples, 2) for v in white_counts]

        print("[Tracker] 검정 선 위에 올려놓고 Enter...")
        input()
        black_counts = [0, 0, 0]
        for _ in range(samples):
            l, c, r = self.read_raw()
            black_counts[0] += l
            black_counts[1] += c
            black_counts[2] += r
            time.sleep(0.05)
        black_avg = [round(v / samples, 2) for v in black_counts]

        self._white_vals = white_avg
        self._black_vals = black_avg
        self._calibrated = True

        ok = all(b > w for b, w in zip(black_avg, white_avg))
        print(f"[Tracker] 캘리브레이션 {'정상 ✓' if ok else '이상 ✗'}")
        return {'white': white_avg, 'black': black_avg, 'ok': ok}

    def close(self):
        self.s1.close()
        self.s2.close()
        self.s3.close()
        print("[Tracker] 종료")