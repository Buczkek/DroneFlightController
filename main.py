import time
import math

import machine

from machine import SPI, Pin, I2C
from time import sleep

# from imu import MPU6050

from mympu import MyMPU

current_time = time.ticks_ms()  # get current time in milliseconds

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000) # TODO ZMIEŃ PORTY PO PRZELUTOWANIU
# TODO  !!!
# TODO  !!!
# TODO  !!!
# TODO  !!!
# TODO  !!!


# imu = MPU6050(i2c)
mympu = MyMPU(i2c, leveled=True, filtered=True)

throttle = 1700
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
roll_error = roll_desired_angle - roll_angle
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
pitch_error = pitch_desired_angle - pitch_angle
pitch_PID = 0
pitch_PID_last = 0
pitch_p = 0
pitch_const_p = 0.7
pitch_i = 0
pitch_const_i = 0.006
pitch_d = 0
pitch_const_d = 1.2


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def convert(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


# NRF24L01
from nrf24l01 import NRF24L01
import struct

csn = Pin(14, mode=Pin.OUT, value=1)  # Chip Select Not
ce = Pin(17, mode=Pin.OUT, value=0)  # Chip Enable
led = Pin(25, Pin.OUT)  # Onboard LED
payload_size = 32

receive_pipe = b"\xd2\xf0\xf0\xf0\xf0"
send_pipe = b"\xe1\xf0\xf0\xf0\xf0"

def setup_nrf():
    print("Initialising the nRF24L0+ Module")
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=payload_size)
    nrf.open_tx_pipe(send_pipe)
    nrf.open_rx_pipe(1, receive_pipe)
    nrf.start_listening()
    return nrf


def flash_led(times: int = None):
    ''' Flashed the built in LED the number of times defined in the times parameter '''
    for _ in range(times):
        led.value(1)
        sleep(0.01)
        led.value(0)
        sleep(0.01)


# def send(nrf, msg):
#     msg += ';'
#
#     nrf.stop_listening()
#     for n in range(len(msg)):
#         try:
#             encoded_string = msg[n].encode()
#             byte_array = bytearray(encoded_string)
#             buf = struct.pack("s", byte_array)
#             nrf.send_start(buf)
#             # print("message",msg[n],"sent")
#             # print("sent")
#             flash_led(1)
#         except OSError:
#             print("Sorry message not sent")
#     print("message sent")
#     nrf.start_listening()


def send(nrf, msg):
    msg += ';'

    nrf.stop_listening()
    try:
        encoded_string = msg.encode()
        byte_array = bytearray(encoded_string)
        buf = struct.pack("s", byte_array)
        nrf.send_start(byte_array)
        print(f"msg: {msg}")
        print(f"encoded_string: {encoded_string}")
        print(f"byte_array: {byte_array}")
        print(f"buf: {buf}")
        # print("message",msg[n],"sent")
        # print("sent")
        flash_led(1)
    except OSError:
        print("Sorry message not sent")
    print("message sent")
    nrf.start_listening()


# init nrf
nrf = setup_nrf()
nrf.start_listening()
msg_string = ""

while True:
    # time update
    last_time = current_time
    current_time = time.ticks_ms()  # get current time in milliseconds
    delta_time = time.ticks_diff(current_time, last_time) / 1000  # get delta time in seconds
    # print(f"delta_time: {delta_time} \t current_time: {current_time} \t last_time: {last_time}")

    # refresh data from IMU
    roll, pitch = mympu.get_roll_pitch()
    roll_angle = round(roll)
    pitch_angle = round(pitch)
    print(f"roll: {roll_angle}\u00B0 \t pitch: {pitch_angle}\u00B0 \t throttle: {throttle}")
    print(f"roll_error: {roll_error}\u00B0 \t pitch_error: {pitch_error}\u00B0 \t " +
          f"desired_roll: {roll_desired_angle}\u00B0 \t desired_pitch: {pitch_desired_angle}\u00B0")

    message = f"roll: {roll_angle} pitch: {pitch_angle}"  # \n throttle: {throttle}"
    send(nrf, message)

    # Calculate angle error
    pitch_error = pitch_desired_angle - pitch_angle
    roll_error = roll_desired_angle - roll_angle

    # Calculate PID
    # pitch
    pitch_p = pitch_const_p * pitch_error

    if -3 < pitch_error < 3:
        pitch_i += pitch_const_i * pitch_error

    pitch_d = pitch_const_d * (pitch_error - pitch_PID_last) / delta_time

    pitch_PID = pitch_p + pitch_i + pitch_d

    pitch_PID_last = pitch_error

    # roll
    roll_p = roll_const_p * roll_error

    if -3 < roll_error < 3:
        roll_i += roll_const_i * roll_error

    roll_d = roll_const_d * (roll_error - roll_PID_last) / delta_time

    roll_PID = roll_p + roll_i + roll_d

    roll_PID_last = roll_error

    # Clamp PID values (depends on the drone motors)
    clamp(pitch_PID, -200, 200)
    clamp(roll_PID, -200, 200)

    # Calculate PWM
    pwm_L_F = throttle + pitch_PID + roll_PID
    pwm_L_B = throttle - pitch_PID + roll_PID
    pwm_R_F = throttle + pitch_PID - roll_PID
    pwm_R_B = throttle - pitch_PID - roll_PID

    # Clamp PWM values
    pwm_L_F = clamp(pwm_L_F, PWM_MIN, PWM_MAX)
    pwm_L_B = clamp(pwm_L_B, PWM_MIN, PWM_MAX)
    pwm_R_F = clamp(pwm_R_F, PWM_MIN, PWM_MAX)
    pwm_R_B = clamp(pwm_R_B, PWM_MIN, PWM_MAX)

    # Print PWM values
    print(f"pwm_L_F: {pwm_L_F}\tpwm_L_B: {pwm_L_B}\tpwm_R_F: {pwm_R_F}\tpwm_R_B: {pwm_R_B}")
