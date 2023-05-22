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

        self.raw_ax = 0
        self.raw_ay = 0
        self.raw_az = 0
        self.raw_gx = 0
        self.raw_gy = 0
        self.raw_gz = 0

        if self.leveled:
            self.calc_offset()

    def calc_offset(self):
        for i in range(100):
            roll, pitch = self.get_raw_roll_pitch_acc_based()
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

        self.raw_ax = ax
        self.raw_ay = ay
        self.raw_az = az
        self.raw_gx = gx
        self.raw_gy = gy
        self.raw_gz = gz

        self._calc_roll_pitch_acc_based()
        # self._calc_roll_pitch_gyro_based()  # TODO ELO
        self._calc_roll_pitch_filtered_acc_based()

    def _calc_roll_pitch_acc_based(self) -> bool:
        try:
            roll = (math.atan(self.raw_ay / math.sqrt(self.raw_ax ** 2 + self.raw_az ** 2)) * 180 / math.pi)
            pitch = (math.atan(-1 * self.raw_ax / math.sqrt(self.raw_ay ** 2 + self.raw_az ** 2)) * 180 / math.pi)

            self.roll_raw = roll
            self.pitch_raw = pitch
            return True
        except ZeroDivisionError:
            return False

    def get_raw_roll_pitch_acc_based(self):
        self._update()
        return self.roll_raw, self.pitch_raw

    def _calc_roll_pitch_filtered_acc_based(self):
        self.roll_out = self.roll_out * .9 + (self.roll_raw - self.roll_offset) * .1
        self.pitch_out = self.pitch_out * .9 + (self.pitch_raw - self.pitch_offset) * .1

    def get_roll_pitch_filtered_acc_based(self):
        self._update()
        return self.roll_out, self.pitch_out

    # def

    def get_offset(self):
        return self.roll_offset, self.pitch_offset
