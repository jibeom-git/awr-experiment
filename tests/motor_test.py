# python 인터랙티브 셸에서
import sys; sys.path.insert(0, '/home/pi/insite')
from sensors.motor import MotorController
mc = MotorController()

mc.set_motor(1, 1, 30)  # M1만 전진
# 어느 바퀴가 돌아가는지 확인
mc.stop()

mc.set_motor(2, 1, 30)  # M2만 전진
mc.stop()

mc.set_motor(3, 1, 30)  # M3만 전진
mc.stop()

mc.set_motor(4, 1, 30)  # M4만 전진
mc.stop()