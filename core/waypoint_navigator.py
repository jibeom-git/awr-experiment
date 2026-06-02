# ~/insite/core/waypoint_navigator.py
# =============================================================================
# 웨이포인트 그래프 기반 자율주행 네비게이터
#
# 역할:
#   - waypoint_graph.json 로드
#   - 저장된 경로를 웨이포인트 순서대로 주행
#   - yaw 피드백 회전 제어
#   - 직진/후진 중 pitch 기반 언덕 감지
#   - 초음파 기반 장애물 감지 (평지 구간에서만 활성화)
#   - 도달 판정: yaw 일치 + (pitch 평지일 때만 ultra 확인)
#
# 대시보드(dashboard.py)에서 사용법:
#   from core.waypoint_navigator import WaypointNavigator
#   nav = WaypointNavigator(motor, imu, ultra)
#   nav.run_path("Path_A")   # 비블로킹 (별도 스레드)
#   nav.stop()
#
# 이 파일에서만 수정:
#   - 판단 파라미터 (PARAMS)
#   - 도달 판정 로직 (_is_arrived)
#   - 장애물/언덕 대응 로직
# =============================================================================

import json
import math
import time
import threading
from pathlib import Path
from typing import Callable, Optional

# ── 경로 데이터 파일 위치 ──────────────────────────────────────────────────────
GRAPH_PATH = Path(__file__).parent / 'waypoint_graph.json'

# =============================================================================
# 판단 파라미터 — 이 블록만 수정해서 동작 조정
# =============================================================================
PARAMS = {
    # 회전 제어
    'yaw_tol_deg':      5.0,   # 회전 수렴 허용 오차 (±°)
    'rotate_timeout_s': 10.0,  # 회전 최대 허용 시간 (초)

    # 직진 도달 판정
    'arrive_yaw_tol':   8.0,   # 도달 판정 yaw 허용 오차 (±°)
    'arrive_ultra_tol': 12.0,  # 도달 판정 초음파 허용 오차 (±cm)
    'drive_timeout_k':  3.0,   # 직진 타임아웃 = 기록 시간 × K

    # 언덕 판정 — 두 조건 중 하나라도 만족하면 언덕으로 판단
    'hill_pitch_deg':       5.0,  # pitch 절대값 임계값 (°)
                                   # 실제 트랙 경사가 20°/10° → 5° 이상이면 언덕 진입으로 판단
    'hill_pitch_rate_dps':  3.0,  # pitch 변화율 임계값 (°/s)
                                   # 언덕 진입 직전 pitch가 급격히 커지는 순간을 미리 감지
                                   # → 초음파가 바닥을 향하기 전에 선제적으로 장애물 감지 OFF

    # 장애물 감지 (평지 전진 구간에서만 활성)
    'obstacle_cm':      20.0,  # 이 거리 이하면 장애물로 판단
    'obstacle_wait_s':  3.0,   # 장애물 해제 대기 시간 (초)
}


# =============================================================================
# WaypointNavigator
# =============================================================================
class WaypointNavigator:
    """
    센서 인스턴스를 받아 웨이포인트 경로를 자율주행하는 클래스.

    Parameters
    ----------
    motor : MotorController
        sensors/motor.py의 MotorController 인스턴스
    imu : MPU6050
        sensors/mpu6050.py의 MPU6050 인스턴스
        get_all() → {"roll", "pitch", "yaw", "yaw_rate"} 사용
    ultra : UltrasonicSensor
        sensors/ultra.py의 UltrasonicSensor 인스턴스
        get_distance() → float (cm, 실패 -1.0) 사용
    speed : int
        기본 이동 속도 (0~100)
    """

    def __init__(self, motor, imu, ultra, speed: int = 40):
        self._motor  = motor
        self._imu    = imu
        self._ultra  = ultra
        self._speed  = speed

        self._stop_event  = threading.Event()
        self._run_thread: Optional[threading.Thread] = None
        # pitch 변화율 계산용 이전값 추적
        self._prev_pitch     = 0.0
        self._prev_pitch_t   = time.time()
        self._status: dict = {
            'running':    False,
            'path':       '',
            'step':       '',
            'current_wp': None,
            'state':      'IDLE',   # IDLE / ROTATING / DRIVING / OBSTACLE / HILL
            'message':    '',
        }
        self._status_lock = threading.Lock()
        # 상태 변경 시 호출되는 콜백 (dashboard에서 socketio emit 등으로 연결)
        self._on_status: Optional[Callable[[dict], None]] = None

    # ── 공개 API ─────────────────────────────────────────────────────────────

    def set_status_callback(self, cb: Callable[[dict], None]):
        """
        상태 변경 시 호출될 콜백 등록.
        cb(status: dict) 형태.
        dashboard.py 에서 socketio.emit 연결용.
        """
        self._on_status = cb

    def run_path(self, path_name: str) -> bool:
        """
        경로 비블로킹 실행. 이미 실행 중이면 False 반환.
        """
        if self._run_thread and self._run_thread.is_alive():
            return False

        graph = self._load_graph()
        if path_name not in graph.get('paths', {}):
            self._emit(f'경로 "{path_name}" 없음', 'error')
            return False

        node_ids = graph['paths'][path_name]
        if not node_ids:
            self._emit('경로에 노드가 없습니다.', 'error')
            return False

        self._stop_event.clear()
        self._run_thread = threading.Thread(
            target=self._run_loop,
            args=(path_name, node_ids, graph),
            daemon=True,
            name='navigator',
        )
        self._run_thread.start()
        return True

    def stop(self):
        """실행 중인 경로 즉시 중단."""
        self._stop_event.set()
        self._motor_stop()

    def is_running(self) -> bool:
        return self._run_thread is not None and self._run_thread.is_alive()

    def get_status(self) -> dict:
        with self._status_lock:
            return dict(self._status)

    def set_speed(self, speed: int):
        self._speed = max(10, min(100, speed))

    # ── 내부 센서 읽기 헬퍼 ──────────────────────────────────────────────────

    def _read_imu(self) -> dict:
        """IMU 값 읽기. 실패 시 기본값 반환."""
        try:
            return self._imu.get_all()
        except Exception as e:
            print(f"[Nav] IMU 읽기 오류: {e}")
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'yaw_rate': 0.0}

    def _read_ultra(self) -> float:
        """초음파 거리 읽기 (cm). 실패 시 999.0 반환."""
        try:
            d = self._ultra.get_distance()
            return float(d) if d is not None and d >= 0 else 999.0
        except Exception as e:
            print(f"[Nav] 초음파 읽기 오류: {e}")
            return 999.0

    # ── 모터 제어 헬퍼 ───────────────────────────────────────────────────────

    def _motor_stop(self):
        try: self._motor.stop()
        except Exception: pass

    def _motor_go(self, direction: str):
        s = self._speed
        try:
            if   direction == 'fwd':   self._motor.move(s,  1, 'mid')
            elif direction == 'bwd':   self._motor.move(s, -1, 'mid')
            elif direction == 'left':  self._motor.rotate_left(s)
            elif direction == 'right': self._motor.rotate_right(s)
        except Exception as e:
            print(f"[Nav] 모터 오류: {e}")

    # ── 유틸 ────────────────────────────────────────────────────────────────

    @staticmethod
    def _yaw_diff(target: float, current: float) -> float:
        """두 yaw 각도의 최단 차이 (-180 ~ +180°)"""
        return (target - current + 180.0) % 360.0 - 180.0

    def _update_status(self, **kwargs):
        with self._status_lock:
            self._status.update(kwargs)
        if self._on_status:
            try:
                self._on_status(dict(self._status))
            except Exception:
                pass

    def _emit(self, msg: str, typ: str = 'info'):
        print(f"[Nav] [{typ.upper()}] {msg}")
        self._update_status(message=msg)

    def _is_hill(self, pitch: float) -> bool:
        """
        언덕 판단: 다음 두 조건 중 하나라도 만족하면 True.

        조건 1: pitch 절대값 >= hill_pitch_deg
          → 이미 언덕에 올라가 있는 상태

        조건 2: pitch 변화율 >= hill_pitch_rate_dps
          → 언덕 진입 직전 pitch가 급격히 증가하는 순간 선제 감지
          → 초음파가 바닥을 향해 거리가 줄어들기 전에 장애물 감지를 미리 OFF
        """
        now = time.time()
        dt  = now - self._prev_pitch_t
        if dt > 0.01:
            pitch_rate = abs(pitch - self._prev_pitch) / dt
        else:
            pitch_rate = 0.0
        self._prev_pitch   = pitch
        self._prev_pitch_t = now

        return (abs(pitch) >= PARAMS['hill_pitch_deg'] or
                pitch_rate  >= PARAMS['hill_pitch_rate_dps'])

    # ── 핵심 동작 로직 ───────────────────────────────────────────────────────

    def _rotate_by_rel(self, rel_yaw: float) -> bool:
        """
        상대 회전각만큼 회전.

        로직:
          - 현재 yaw를 기준으로 rel_yaw 만큼 회전한 목표 yaw 계산
          - rel_yaw > 0: 우회전 / rel_yaw < 0: 좌회전
          - 출발 yaw가 무엇이든 상관없이 상대적으로 정확한 회전 가능

        반환: True=수렴, False=타임아웃/중단
        """
        if abs(rel_yaw) < PARAMS['yaw_tol_deg']:
            return True  # 회전량이 너무 작으면 스킵

        self._update_status(state='ROTATING')
        start_yaw  = self._read_imu()['yaw']
        target_yaw = (start_yaw + rel_yaw) % 360.0
        deadline   = time.time() + PARAMS['rotate_timeout_s']

        self._emit(f'회전 {rel_yaw:+.1f}° (현재={start_yaw:.0f}° → 목표={target_yaw:.0f}°)', 'info')

        while not self._stop_event.is_set():
            imu  = self._read_imu()
            diff = self._yaw_diff(target_yaw, imu['yaw'])

            if abs(diff) <= PARAMS['yaw_tol_deg']:
                self._motor_stop()
                return True

            if time.time() > deadline:
                self._motor_stop()
                self._emit(f'회전 타임아웃 (잔여 {diff:.1f}°)', 'error')
                return False

            direction = 'right' if diff > 0 else 'left'
            self._motor_go(direction)
            time.sleep(0.05)

        self._motor_stop()
        return False

    def _drive_to_wp(self, direction: str,
                     target_ultra: float, target_yaw: float,
                     recorded_time: float) -> bool:
        """
        direction 방향으로 주행하면서 웨이포인트 도달 판정.

        도달 판정 기준:
          [평지 + 전진 + ultra 저장된 경우]
            - 초음파 ≈ target_ultra ± arrive_ultra_tol
            - AND yaw ≈ target_yaw ± arrive_yaw_tol

          [언덕 구간 or 후진 or ultra 미저장]
            - yaw ≈ target_yaw ± arrive_yaw_tol
            - AND 타임아웃 (recorded_time × drive_timeout_k) 경과

        장애물 처리 (평지 + 전진 구간에서만):
          - 초음파 < obstacle_cm → 정지 → obstacle_wait_s 대기
          - 해제: 재출발 / 지속: 경로 중단

        언덕 구간:
          - 초음파 도달 판정 OFF
          - 초음파 장애물 감지 OFF (기울어진 초음파는 신뢰 불가)
          - yaw + 타임아웃으로만 판단

        반환: True=도달, False=중단/장애물 지속
        """
        self._update_status(state='DRIVING')
        timeout    = recorded_time * PARAMS['drive_timeout_k']
        deadline   = time.time() + max(timeout, 1.0)
        use_ultra  = (direction == 'fwd') and (target_ultra > 0)

        self._motor_go(direction)

        while not self._stop_event.is_set():
            imu       = self._read_imu()
            ultra     = self._read_ultra()
            pitch     = imu['pitch']
            now_yaw   = imu['yaw']
            on_hill   = self._is_hill(pitch)
            yaw_ok    = abs(self._yaw_diff(target_yaw, now_yaw)) <= PARAMS['arrive_yaw_tol']

            # ── 언덕 상태 알림 ────────────────────────────────────────────────
            if on_hill:
                self._update_status(state='HILL')
            else:
                self._update_status(state='DRIVING')

            # ── 장애물 감지 (평지 전진 구간에서만) ───────────────────────────
            if direction == 'fwd' and not on_hill and 0 < ultra < PARAMS['obstacle_cm']:
                self._motor_stop()
                self._update_status(state='OBSTACLE')
                self._emit(f'장애물 감지 {ultra:.1f}cm — 대기', 'error')

                clear = self._wait_obstacle_clear()
                if not clear:
                    return False

                # 해제 후 재출발 + 타임아웃 리셋
                self._motor_go(direction)
                self._update_status(state='DRIVING')
                deadline = time.time() + max(timeout, 1.0)
                continue

            # ── 도달 판정 ─────────────────────────────────────────────────────
            if use_ultra and not on_hill:
                # 평지 + 전진 + ultra 저장 → ultra + yaw 모두 만족 시 도달
                ultra_ok = abs(ultra - target_ultra) <= PARAMS['arrive_ultra_tol']
                if ultra_ok and yaw_ok:
                    self._motor_stop()
                    return True
            else:
                # 언덕 or 후진 or ultra 미저장
                # OR 조건: yaw가 맞거나 타임아웃이 됐으면 도달로 판단
                # (AND면 yaw가 맞아도 시간이 안 되면 계속 달리는 갇힘 발생)
                if yaw_ok or time.time() > deadline:
                    self._motor_stop()
                    return True

            # ── 타임아웃 ─────────────────────────────────────────────────────
            if time.time() > deadline:
                self._motor_stop()
                if not use_ultra:
                    # ultra 미사용 구간은 타임아웃 = 도달 간주
                    return True
                self._emit(
                    f'도달 타임아웃 (ultra={ultra:.1f}cm 목표={target_ultra:.1f}cm '
                    f'yaw_diff={self._yaw_diff(target_yaw, now_yaw):.1f}°)', 'error'
                )
                return False

            time.sleep(0.05)

        self._motor_stop()
        return False

    def _wait_obstacle_clear(self) -> bool:
        """
        장애물 해제 대기.
        obstacle_wait_s 초 이내 해제: True / 지속: False
        """
        deadline = time.time() + PARAMS['obstacle_wait_s']
        while time.time() < deadline:
            if self._stop_event.is_set():
                return False
            if self._read_ultra() > PARAMS['obstacle_cm']:
                self._emit('장애물 해제 — 재개', 'info')
                return True
            time.sleep(0.1)
        self._emit('장애물 지속 — 경로 중단', 'error')
        return False

    # ── 메인 루프 ────────────────────────────────────────────────────────────

    def _run_loop(self, path_name: str, node_ids: list, graph: dict):
        """경로 실행 스레드 메인 루프."""
        self._update_status(running=True, path=path_name, state='DRIVING')
        self._emit(f'경로 "{path_name}" 실행 시작', 'info')
        self._emit('로봇이 수집 시 출발 방향과 동일하게 정렬됐는지 확인하세요.', 'info')

        for i, node_id in enumerate(node_ids):
            if self._stop_event.is_set():
                break

            wp = graph['waypoints'].get(str(node_id))
            if wp is None:
                self._emit(f'웨이포인트 #{node_id} 없음 — 건너뜀', 'error')
                continue

            target_yaw   = float(wp.get('heading',     0.0))
            rel_yaw      = float(wp.get('rel_yaw',    0.0))  # 상대 회전각
            target_ultra = float(wp.get('ultra_front', -1.0))
            move         = wp.get('move', {})

            # 구버전 encoder_offset 호환
            if not move:
                x_time = wp.get('encoder_offset', {}).get('x_time', 0.0)
                move = {'fwd': x_time, 'bwd': 0.0, 'left': 0.0, 'right': 0.0}

            fwd_t = float(move.get('fwd', 0.0))
            bwd_t = float(move.get('bwd', 0.0))

            step_str = f'{i+1}/{len(node_ids)}'
            self._update_status(current_wp=node_id, step=step_str)
            self._emit(
                f'[{step_str}] #{node_id} | '
                f'rel_yaw={rel_yaw:+.1f}° ultra={target_ultra:.0f}cm '
                f'fwd={fwd_t:.1f}s bwd={bwd_t:.1f}s',
                'info'
            )

            # ── Step 1: 상대 회전각만큼 회전 ─────────────────────────────────
            # rel_yaw=0이면 회전 없이 스킵
            # 첫 번째 웨이포인트(rel_yaw=0.0)는 이미 올바른 방향이라 가정
            if abs(rel_yaw) >= PARAMS['yaw_tol_deg']:
                ok = self._rotate_by_rel(rel_yaw)
                if not ok and self._stop_event.is_set():
                    break
                time.sleep(0.2)   # 회전 후 안정화

            # ── Step 2: 전진 또는 후진 ────────────────────────────────────────
            # 직진 중 도달 판정의 yaw 기준은 현재 실제 yaw 사용
            current_yaw_now = self._read_imu()['yaw']
            if fwd_t > 0:
                ok = self._drive_to_wp('fwd', target_ultra, current_yaw_now, fwd_t)
                if not ok and self._stop_event.is_set():
                    break
            elif bwd_t > 0:
                ok = self._drive_to_wp('bwd', -1.0, current_yaw_now, bwd_t)
                if not ok and self._stop_event.is_set():
                    break

            if self._stop_event.is_set():
                break
            time.sleep(0.3)   # 웨이포인트 간 정지 여유

        # ── 종료 ─────────────────────────────────────────────────────────────
        self._motor_stop()
        stopped = self._stop_event.is_set()
        msg = f'경로 "{path_name}" {"중단됨" if stopped else "완료"}'
        self._emit(msg, 'info' if stopped else 'ok')
        self._update_status(running=False, state='IDLE', message=msg)
        self._stop_event.clear()

    # ── 그래프 로드 ──────────────────────────────────────────────────────────

    @staticmethod
    def _load_graph() -> dict:
        if GRAPH_PATH.exists():
            try:
                return json.loads(GRAPH_PATH.read_text(encoding='utf-8'))
            except Exception as e:
                print(f"[Nav] 그래프 로드 오류: {e}")
        return {'waypoints': {}, 'paths': {}}