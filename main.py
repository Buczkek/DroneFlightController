import time

from machine import SPI, Pin, I2C, PWM
from time import sleep

from mympu import MyMPU

current_time = time.ticks_ms()  # get current time in milliseconds

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)

MOTOR_L_F = PWM(Pin(0))
MOTOR_R_B = PWM(Pin(1))
MOTOR_L_B = PWM(Pin(2))
MOTOR_R_F = PWM(Pin(3))

MOTOR_L_F.freq(50)
MOTOR_R_B.freq(50)
MOTOR_L_B.freq(50)
MOTOR_R_F.freq(50)

# imu = MPU6050(i2c)
mympu = MyMPU(i2c, leveled=True, filtered=True)

throttle = 0
PWM_MAX = 5300  # 2000
PWM_MIN = 4960  # 1480? + 100

pwm_L_F = 0
idle_L_F = 4860  # 1478
pwm_L_B = 0
idle_L_B = 4860  # 1478
pwm_R_F = 0
idle_R_F = 4860  # 1478
pwm_R_B = 0
idle_R_B = 4860  # 1478

roll_angle = 0
roll_desired_angle = 0
roll_error = roll_desired_angle - roll_angle
roll_PID = 0
roll_PID_last = 0
roll_p = 0
roll_const_p = 1
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
pitch_const_p = 1
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
        sleep(0.01)
        led.value(0)
        sleep(0.01)


def decode_angles(package: bytearray):
    # command = int.from_bytes(package[:1], "big")
    command = package[:1]
    uid = int.from_bytes(package[2:12], "big")
    if command != b'\x00\xf0':
        throttle_target = int.from_bytes(package[-2:], "big")
        pitch_target = int.from_bytes(package[-4:-2], "big") - 50
        roll_target = int.from_bytes(package[-6:-4], "big") - 50
        return throttle_target, pitch_target, roll_target, uid
    return None


def send_str(nrf, msg):
    msg += ';'

    nrf.stop_listening()
    try:
        encoded_string = msg.encode()
        byte_array = bytearray(encoded_string)
        nrf.send_start(byte_array)
    except OSError:
        print("Sorry message not sent")
    nrf.start_listening()


calibration = True
if calibration:
    input("Press enter to start calibration")
    MOTOR_L_F.duty_u16(idle_L_F)
    MOTOR_L_B.duty_u16(idle_L_B)
    MOTOR_R_F.duty_u16(idle_R_F)
    MOTOR_R_B.duty_u16(idle_R_B)
    print("Now power on the ESCs")
    input("Press enter to end calibration")


# init nrf
nrf = setup_nrf()
nrf.start_listening()
msg_string = ""

start_time = time.ticks_ms()
last_sec = time.ticks_ms()
times = 0
ids = []


pwm_L_F = 4860
pwm_L_B = 4860
pwm_R_F = 4860
pwm_R_B = 4860

first = True
led.value(1)
while True:
    # time update
    last_time = current_time
    current_time = time.ticks_ms()  # get current time in milliseconds
    if time.ticks_diff(current_time, last_sec) > 1000:
        last_sec = current_time
        print(f"times: {times}")
        times = 0
    times += 1
    delta_time = time.ticks_diff(current_time, last_time) / 1000  # get delta time in seconds

    if time.ticks_diff(current_time, start_time) > 3000 and first:
        first = False

    # refresh data from IMU
    roll, pitch = mympu.get_roll_pitch_filtered_acc_based()
    roll_angle = round(roll)
    pitch_angle = round(pitch)

    message = f"roll: {roll_angle} pitch: {pitch_angle}"  # \n throttle: {throttle}"
    send_str(nrf, message)

    # Calculate angle error
    pitch_error = pitch_desired_angle - pitch_angle
    roll_error = roll_desired_angle - roll_angle

    # Calculate PID
    # pitch
    pitch_p = pitch_const_p * pitch_error

    if -2 < pitch_error < 2:
        pitch_i += pitch_const_i * pitch_error

    pitch_d = pitch_const_d * (pitch_error - pitch_PID_last) / delta_time

    pitch_PID = pitch_p + pitch_i + pitch_d

    pitch_PID_last = pitch_error

    # roll
    roll_p = roll_const_p * roll_error

    if -2 < roll_error < 2:
        roll_i += roll_const_i * roll_error

    roll_d = roll_const_d * (roll_error - roll_PID_last) / delta_time

    roll_PID = roll_p + roll_i + roll_d

    roll_PID_last = roll_error

    # Clamp PID values (depends on the drone motors)
    clamp(pitch_PID, -200, 200)
    clamp(roll_PID, -200, 200)

    # Calculate PWM
    if not first:
        pwm_L_F = PWM_MIN + pitch_PID + roll_PID + throttle
        pwm_L_B = PWM_MIN - pitch_PID + roll_PID + throttle
        pwm_R_F = PWM_MIN + pitch_PID - roll_PID + throttle
        pwm_R_B = PWM_MIN - pitch_PID - roll_PID + throttle

    # Clamp PWM values
    pwm_L_F = int(clamp(pwm_L_F, PWM_MIN, PWM_MAX))
    pwm_L_B = int(clamp(pwm_L_B, PWM_MIN, PWM_MAX))
    pwm_R_F = int(clamp(pwm_R_F, PWM_MIN, PWM_MAX))
    pwm_R_B = int(clamp(pwm_R_B, PWM_MIN, PWM_MAX))

    # Set PWM
    MOTOR_L_F.duty_u16(pwm_L_F)
    MOTOR_L_B.duty_u16(pwm_L_B)
    MOTOR_R_F.duty_u16(pwm_R_F)
    MOTOR_R_B.duty_u16(pwm_R_B)

    # Print PWM values
    print(f"pwm_L_F: {pwm_L_F}\tpwm_L_B: {pwm_L_B}\tpwm_R_F: {pwm_R_F}\tpwm_R_B: {pwm_R_B} throttle: {throttle}")

    if nrf.any():
        package = nrf.recv()

        data = decode_angles(package)
        if data is not None:
            throttle_val, pitch_desired_angle, roll_desired_angle, uid = data
            # throttle = convert(throttle_val, 0, 400, 4860, 5300)
            throttle = throttle_val
            if uid in ids:
                print("duplicate")
                continue
            else:
                ids.append(uid)
                if len(ids) > 50:
                    ids.pop(0)
