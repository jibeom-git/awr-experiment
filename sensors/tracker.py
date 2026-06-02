# sensors/tracker.py
# Adeept 3채널 IR 라인트래킹 센서
# S1: GPIO17 (좌), S2: GPIO27 (중앙), S3: GPIO22 (우)

import time
from gpiozero import DigitalInputDevice

class LineTracker:
    """
    Adeept 3채널 IR 라인트래킹 센서 드라이버
    
    센서 출력:
        검정 선 위  → 1 (HIGH)
        흰 바닥 위  → 0 (LOW)
    
    핀 배치 (로봇 전면 기준):
        S1 (GPIO17) — 왼쪽
        S2 (GPIO27) — 중앙
        S3 (GPIO22) — 오른쪽
    """

    # 캘리브레이션 기본값
    # calibrate() 실행 후 자동으로 갱신됨
    _threshold = 0.5   # 디지털 센서라 임계값 불필요, 아날로그 확장 대비

    def __init__(self,
                 pin_s1: int = 22,
                 pin_s2: int = 27,
                 pin_s3: int = 17):
        """
        Args:
            pin_s1: 왼쪽 센서 GPIO 번호
            pin_s2: 중앙 센서 GPIO 번호
            pin_s3: 오른쪽 센서 GPIO 번호
        """
        self.s1 = DigitalInputDevice(pin_s1)   # 왼쪽
        self.s2 = DigitalInputDevice(pin_s2)   # 중앙
        self.s3 = DigitalInputDevice(pin_s3)   # 오른쪽

        # 캘리브레이션 데이터
        self._white_vals  = [0, 0, 0]   # 흰 바닥 기준값
        self._black_vals  = [0, 0, 0]   # 검정 선 기준값
        self._calibrated  = False

        print("[Tracker] 초기화 완료 | S1=GPIO17 S2=GPIO27 S3=GPIO22")

    # ── 원시 읽기 ────────────────────────────────────────────────────────────
    def read_raw(self) -> tuple[int, int, int]:
        """
        3채널 디지털 값 반환.
        Returns:
            (s1, s2, s3) — 각각 0 또는 1
            1 = 검정 선, 0 = 흰 바닥
        """
        return (
            int(self.s1.value),
            int(self.s2.value),
            int(self.s3.value),
        )
    # def read_raw(self) -> tuple[int, int, int]:
    #         return (
    #             1 - int(self.s1.value),
    #             1 - int(self.s2.value),
    #             1 - int(self.s3.value),
    #         )
    def read(self) -> dict:
        """
        센서 상태 딕셔너리 반환.
        Returns:
            {
                'left':   int,  # 0 or 1
                'center': int,  # 0 or 1
                'right':  int,  # 0 or 1
                'on_line': bool,  # 하나라도 선 위이면 True
                'pattern': str,   # 'LCR' 형태 문자열
            }
        """
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

    

    # ── 교차점 감지 ──────────────────────────────────────────────────────────
    def is_junction(self) -> bool:
        """
        3채널이 모두 1이면 교차점 (T자/십자) 으로 판단.
        Returns:
            True  → 교차점
            False → 일반 직선/곡선
        """
        l, c, r = self.read_raw()
        return bool(l and c and r)

    def is_on_line(self) -> bool:
        """하나라도 선 위이면 True"""
        l, c, r = self.read_raw()
        return bool(l or c or r)

    def is_lost(self) -> bool:
        """3채널 모두 0이면 선 이탈"""
        l, c, r = self.read_raw()
        return not bool(l or c or r)

    # ── 조향 오차 계산 ───────────────────────────────────────────────────────
    def get_error(self) -> int:
        """
        PID 제어용 조향 오차 반환.
        중앙 기준으로 얼마나 치우쳤는지.

        Returns:
            -1 : 선이 왼쪽 (우회전 필요)
             0 : 선이 중앙 (직진)
            +1 : 선이 오른쪽 (좌회전 필요)
             9 : 선 이탈 (lost)
        
        패턴별 오차:
            _C_ →  0  (정중앙)
            LC_ → -1  (왼쪽 치우침)
            _CR → +1  (오른쪽 치우침)
            L__ → -1  (많이 왼쪽)
            __R → +1  (많이 오른쪽)
            LCR →  0  (교차점, 직진)
            ___ →  9  (선 이탈)
        """
        l, c, r = self.read_raw()

        if   l and     c and     r: return  0   # 교차점
        elif not l and     c and not r: return  0   # 정중앙
        elif l and     c and not r: return -1   # 약간 왼쪽
        elif not l and     c and     r: return  1   # 약간 오른쪽
        elif l and not c and not r: return -1   # 많이 왼쪽
        elif not l and not c and     r: return  1   # 많이 오른쪽
        elif l and not c and     r: return  0   # 양끝만 (곡선)
        else:                          return  9   # 선 이탈

    # ── 캘리브레이션 ─────────────────────────────────────────────────────────
    def calibrate(self, samples: int = 30) -> dict:
        """
        흰 바닥 / 검정 선 기준값 측정.
        디지털 센서라 실제 임계값 조정은 불필요하지만
        노이즈 비율 확인 및 정상 동작 검증용으로 사용.

        Args:
            samples: 샘플 수 (기본 30회)
        Returns:
            {'white': [...], 'black': [...], 'ok': bool}
        """
        print("[Tracker] ── 캘리브레이션 시작 ──")

        # 1단계: 흰 바닥
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

        # 2단계: 검정 선
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

        # 정상 여부: 흰 바닥은 0에 가깝고, 검정 선은 1에 가까워야 함
        ok = all(b > w for b, w in zip(black_avg, white_avg))

        print(f"[Tracker] 흰 바닥 평균: {white_avg}")
        print(f"[Tracker] 검정 선 평균: {black_avg}")
        print(f"[Tracker] 캘리브레이션 {'정상 ✓' if ok else '이상 ✗ — 센서 확인 필요'}")

        return {
            'white': white_avg,
            'black': black_avg,
            'ok':    ok,
        }

    # ── 정리 ─────────────────────────────────────────────────────────────────
    def close(self):
        self.s1.close()
        self.s2.close()
        self.s3.close()
        print("[Tracker] 종료")