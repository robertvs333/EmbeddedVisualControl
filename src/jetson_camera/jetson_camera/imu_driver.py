#!/usr/bin/env python3

import smbus2


class Mpu6050:
    GRAVITY_MS2 = 9.80665

    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_BW_256 = 0x00
    FILTER_BW_188 = 0x01
    FILTER_BW_98 = 0x02
    FILTER_BW_42 = 0x03
    FILTER_BW_20 = 0x04
    FILTER_BW_10 = 0x05
    FILTER_BW_5 = 0x06

    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F
    TEMP_OUT0 = 0x41
    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = smbus2.SMBus(bus)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def read_i2c_word(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low

        if value >= 0x8000:
            return -((65535 - value) + 1)
        return value

    def set_accel_range(self, accel_range):
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def set_gyro_range(self, gyro_range):
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def set_filter_range(self, filter_range=FILTER_BW_42):
        ext_sync_set = self.bus.read_byte_data(self.address, self.MPU_CONFIG) & 0b00111000
        self.bus.write_byte_data(self.address, self.MPU_CONFIG, ext_sync_set | filter_range)

    def read_accel_range(self):
        return self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

    def read_gyro_range(self):
        return self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

    def get_temp(self):
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
        return (raw_temp / 340.0) + 36.53

    def get_accel_data(self):
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_range = self.read_accel_range()
        scale_modifier = {
            self.ACCEL_RANGE_2G: self.ACCEL_SCALE_MODIFIER_2G,
            self.ACCEL_RANGE_4G: self.ACCEL_SCALE_MODIFIER_4G,
            self.ACCEL_RANGE_8G: self.ACCEL_SCALE_MODIFIER_8G,
            self.ACCEL_RANGE_16G: self.ACCEL_SCALE_MODIFIER_16G,
        }.get(accel_range, self.ACCEL_SCALE_MODIFIER_2G)

        return {
            'x': (x / scale_modifier) * self.GRAVITY_MS2,
            'y': (y / scale_modifier) * self.GRAVITY_MS2,
            'z': (z / scale_modifier) * self.GRAVITY_MS2,
        }

    def get_gyro_data(self):
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_range = self.read_gyro_range()
        scale_modifier = {
            self.GYRO_RANGE_250DEG: self.GYRO_SCALE_MODIFIER_250DEG,
            self.GYRO_RANGE_500DEG: self.GYRO_SCALE_MODIFIER_500DEG,
            self.GYRO_RANGE_1000DEG: self.GYRO_SCALE_MODIFIER_1000DEG,
            self.GYRO_RANGE_2000DEG: self.GYRO_SCALE_MODIFIER_2000DEG,
        }.get(gyro_range, self.GYRO_SCALE_MODIFIER_250DEG)

        return {
            'x': x / scale_modifier,
            'y': y / scale_modifier,
            'z': z / scale_modifier,
        }

    def close(self):
        """Safely closes the I2C bus instance."""
        if hasattr(self, 'bus') and self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass