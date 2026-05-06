#!/usr/bin/env/python3
#!/usr/bin/env/python
# File name   : Servo100Degree.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/08/11
'''
# SPDX-License-Identifier: MIT
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# sudo pip3 install adafruit-circuitpython-motor
# sudo pip3 install adafruit-circuitpython-pca9685
'''
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c, address=0x5f)  # default 0x40

pca.frequency = 50


def set_angle(ID, angle):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = angle


def test(channel):
    for i in range(100):  # The servo turns from 0 to 100 degrees.
        set_angle(channel, i)
        time.sleep(0.01)
    time.sleep(0.5)
    for i in range(100):  # The servo turns from 100 to 0 degrees.
        set_angle(channel, 100 - i)
        time.sleep(0.01)
    time.sleep(0.5)


if __name__ == "__main__":
    channel = 0
    try:
        print(f"Servo on channel {channel} starts to rotate 100 degrees.")
        while True:
            test(channel)

    except KeyboardInterrupt:
        print("Ctrl + C detected. Setting servo to 90 degrees.")
        set_angle(channel, 90)
        pca.deinit()  # Release PCA9685 resources
