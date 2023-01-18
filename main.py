import time
import math

import machine

from machine import SPI, Pin, I2C
from time import sleep

# from imu import MPU6050

from mympu import MyMPU

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
# imu = MPU6050(i2c)
mympu = MyMPU(i2c, leveled=True, filtered=True)

throttle = 0
PWM_MAX = 2000
PWM_MIN = 1000

pwm_L_F = 0
idle_L_F = 1488
pwm_L_B = 0
idle_L_B = 1488
pwm_R_F = 0
idle_R_F = 1488
pwm_R_B = 0
idle_R_B = 1488

roll_angle = 0
roll_desired_angle = 0
roll_error = 0
roll_PID = 0
roll_PID_last = 0
roll_p = 0
roll_const_p = 0.7
roll_i = 0
roll_const_i = 0.006
roll_d = 0
roll_const_d = 1.2

pitch_angle = 0
pitch_desired_angle = 0
pitch_error = 0
pitch_PID = 0
pitch_PID_last = 0
pitch_p = 0
pitch_const_p = 0.7
pitch_i = 0
pitch_const_i = 0.006
pitch_d = 0
pitch_const_d = 1.2


last_time = time.time()
current_time = time.time()
while True:
    last_time = current_time
    current_time = time.time()
    delta_time = current_time - last_time

    roll, pitch = mympu.get_roll_pitch()
    roll_angle = round(roll)
    pitch_angle = round(pitch)
    print("roll: ", roll_angle, "\u00B0\tpitch: ", pitch_angle, '\u00B0', sep='')
