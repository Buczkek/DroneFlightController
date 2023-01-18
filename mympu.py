from imu import MPU6050
from time import sleep
import math


class MyMPU:
    def __init__(self, i2c, *, leveled=False, filtered=True):
        self.imu = MPU6050(i2c)
        self.leveled = leveled
        self.filtered = filtered
        self.roll_offset = 0
        self.pitch_offset = 0
        self.roll_raw = 0
        self.pitch_raw = 0
        self.roll_out = 0
        self.pitch_out = 0
        if self.leveled:
            self.calc_offset()

    def calc_offset(self):
        for i in range(100):
            roll, pitch = self.get_raw_roll_pitch()
            self.roll_offset += roll
            self.pitch_offset += pitch
            sleep(0.01)
        self.roll_offset /= 100
        self.pitch_offset /= 100

    def _update(self):
        ax = round(self.imu.accel.x, 2)
        ay = round(self.imu.accel.y, 2)
        az = round(self.imu.accel.z, 2)
        gx = round(self.imu.gyro.x, 2)
        gy = round(self.imu.gyro.y, 2)
        gz = round(self.imu.gyro.z, 2)

        roll = self.roll_raw
        pitch = self.pitch_raw

        try:
            roll = (math.atan(ay / math.sqrt(ax ** 2 + az ** 2)) * 180 / math.pi)

            pitch = (math.atan(-1 * ax / math.sqrt(ay ** 2 + az ** 2)) * 180 / math.pi)
        except ZeroDivisionError:
            pass
        self.roll_raw = roll
        self.pitch_raw = pitch

    def get_raw_roll_pitch(self):
        self._update()
        return self.roll_raw, self.pitch_raw

    def get_roll_pitch(self):
        self._update()
        if self.filtered:
            self.roll_out = self.roll_out * .9 + (self.roll_raw - self.roll_offset) * .1
            self.pitch_out = self.pitch_out * .9 + (self.pitch_raw - self.pitch_offset) * .1
        else:
            self.roll_out = self.roll_raw - self.roll_offset
            self.pitch_out = self.pitch_raw - self.pitch_offset
        return self.roll_out, self.pitch_out

    def get_offset(self):
        return self.roll_offset, self.pitch_offset
