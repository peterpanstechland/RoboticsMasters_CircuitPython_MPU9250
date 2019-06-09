__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/robotics-masters/RoboticsMasters_CircuitPython_MPU9250.git"

from time import sleep
try:
    import struct
except ImportError:
    import ustruct as struct

import adafruit_bus_device.i2c_device as i2c_device
from micropython import const

_MPU9250_ADDRESS_ACCELGYRO       = const(0x69) #corrected
_MPU9250_ADDRESS_MAG             = const(0x0C) #corrected - bypass
_MPU9250_XG_ID                   = const(0b01110001) #corrected (0x71)
_MPU9250_MAG_ID                  = const(0b01001000) #corrected (0x48) 0 1 0 0   1 0 0 0

_MPU9250_ACCEL_MG_LSB_2G         = 0.061
_MPU9250_ACCEL_MG_LSB_4G         = 0.122
_MPU9250_ACCEL_MG_LSB_8G         = 0.244
_MPU9250_ACCEL_MG_LSB_16G        = 0.732
_MPU9250_GYRO_DPS_DIGIT_245DPS   = 0.00875
_MPU9250_GYRO_DPS_DIGIT_500DPS   = 0.01750
_MPU9250_GYRO_DPS_DIGIT_2000DPS  = 0.07000
_MPU9250_TEMP_LSB_DEGREE_CELSIUS = 8 # 1°C = 8, 25° = 200, etc.
_MPU9250_TEMP_SENS = 333.87
_MPU9250_TEMP_OFFSET = 21

_MPU9250_REGISTER_WHO_AM_I_XG    = const(0x75) #reports 0x71

_MPU9250_SMPLRT_DIV             = const(0x19)
_MPU9250_CONFIG                 = const(0x1A)
_MPU9250_GYRO_CONFIG            = const(0x1B)
_MPU9250_ACCEL_CONFIG           = const(0x1C)
_MPU9250_ACCEL_CONFIG2          = const(0x1D)

_MPU9250_INT_PIN_CFG            = const(0x37) # for bypass
_MPU9250_INT_ENABLE             = const(0x38) # for bypass
_I2C_BYPASS_MASK = 0b00000010
_I2C_BYPASS_EN = 0b00000010

_MPU9250_REGISTER_ACCEL_XOUT_H     = const(0x3B)
_MPU9250_REGISTER_ACCEL_XOUT_L     = const(0x3C)
_MPU9250_REGISTER_ACCEL_YOUT_H     = const(0x3D)
_MPU9250_REGISTER_ACCEL_YOUT_L     = const(0x3E)
_MPU9250_REGISTER_ACCEL_ZOUT_H     = const(0x3F)
_MPU9250_REGISTER_ACCEL_ZOUT_L     = const(0x40)
_MPU9250_REGISTER_TEMP_OUT_H     = const(0x41)
_MPU9250_REGISTER_TEMP_OUT_L     = const(0x42)
_MPU9250_REGISTER_GYRO_XOUT_H      = const(0x43)
_MPU9250_REGISTER_GYRO_XOUT_L      = const(0x44)
_MPU9250_REGISTER_GYRO_YOUT_H      = const(0x45)
_MPU9250_REGISTER_GYRO_YOUT_L      = const(0x46)
_MPU9250_REGISTER_GYRO_ZOUT_H      = const(0x47)
_MPU9250_REGISTER_GYRO_ZOUT_L      = const(0x48)

_MPU9250_PWR_MGMT_1     = const(0x6B)

_MPU9250_REGISTER_WHO_AM_I_M     = const(0x00) #reports 0x48
_MPU9250_REGISTER_STATUS_REG1_M   = const(0x02) # ST1
_MPU9250_REGISTER_MAG_XOUT_L      = const(0x03)
_MPU9250_REGISTER_MAG_XOUT_H      = const(0x04)
_MPU9250_REGISTER_MAG_YOUT_L      = const(0x05)
_MPU9250_REGISTER_MAG_YOUT_H      = const(0x06)
_MPU9250_REGISTER_MAG_ZOUT_L      = const(0x07)
_MPU9250_REGISTER_MAG_ZOUT_H      = const(0x08)
_MPU9250_REGISTER_STATUS_REG2_M   = const(0x09) # ST2
_MPU9250_REGISTER_ASTC_M          = const(0x0C) # self-test
_MPU9250_REGISTER_MAG_ASAX        = const(0x10)
_MPU9250_REGISTER_MAG_ASAY        = const(0x11)
_MPU9250_REGISTER_MAG_ASAZ        = const(0x12)

_MPU9250_REGISTER_CNTL_M    = const(0x0A) #corrected - RW (CNTL)


_MAGTYPE                         = True
_XGTYPE                          = False
_SENSORS_GRAVITY_STANDARD        = 9.80665

ACCELRANGE_2G                = (0b00 << 3)
ACCELRANGE_16G               = (0b01 << 3)
ACCELRANGE_4G                = (0b10 << 3)
ACCELRANGE_8G                = (0b11 << 3)
GYROSCALE_245DPS             = (0b00 << 3)  # +/- 245 degrees/s rotation
GYROSCALE_500DPS             = (0b01 << 3)  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS            = (0b11 << 3)  # +/- 2000 degrees/s rotation

def _twos_comp(val, bits):
    if val & (1 << (bits - 1)) != 0:
        return val - (1 << bits)
    return val


class MPU9250:
    _BUFFER = bytearray(6)

    def __init__(self, address=_MPU9250_ADDRESS_ACCELGYRO):
        self.address = address
        if self._read_u8(_XGTYPE, _MPU9250_REGISTER_WHO_AM_I_XG) != _MPU9250_XG_ID:
            raise RuntimeError('Could not find MPU9250, check wiring!')

        self._write_u8(_XGTYPE, _MPU9250_PWR_MGMT_1, 0x00)
        sleep(0.1)
        self._write_u8(_XGTYPE, _MPU9250_PWR_MGMT_1, 0x01)
        sleep(0.2)

        self._write_u8(_XGTYPE, _MPU9250_CONFIG, 0x03)

        self._write_u8(_XGTYPE, _MPU9250_SMPLRT_DIV, 0x04)

        self._write_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2, 0x03)

        if self._read_u8(_MAGTYPE, _MPU9250_REGISTER_WHO_AM_I_M) != _MPU9250_MAG_ID:
            raise RuntimeError('Could not find MPU9250 Magnetometer, enable I2C bypass!')

        self._write_u8(_MAGTYPE, _MPU9250_REGISTER_CNTL_M, 0x0F)

        asax = self._read_u8(_MAGTYPE, _MPU9250_REGISTER_MAG_ASAX)
        asay = self._read_u8(_MAGTYPE, _MPU9250_REGISTER_MAG_ASAY)
        asaz = self._read_u8(_MAGTYPE, _MPU9250_REGISTER_MAG_ASAZ)
        self._write_u8(_MAGTYPE, _MPU9250_REGISTER_CNTL_M, 0x00)
        sleep(100e-6) #RaspberryPi much faster than original platform

        self._adjustment_mag = (
            (0.5 * (asax -128)) / 128 + 1,
            (0.5 * (asay -128)) / 128 + 1,
            (0.5 * (asaz -128)) / 128 + 1
        )

        del asax, asay, asaz

        self._write_u8(_MAGTYPE, _MPU9250_REGISTER_CNTL_M, (0b00000010 | 0b00010000)) # 8hz 16bit
        self._read_u8(_MAGTYPE, _MPU9250_REGISTER_STATUS_REG2_M)
        sleep(0.1)

        self._accel_mg_lsb = None
        self._gyro_dps_digit = None
        self.accel_range = ACCELRANGE_2G
        self.gyro_scale = GYROSCALE_245DPS
        self._scale_mag = (1,1,1)
        self._offset_mag = (0,0,0)
        self._mag_calibrated = False


    @property
    def accel_range(self):
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG)
        return (reg & 0b00011000) & 0xFF

    @accel_range.setter
    def accel_range(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG, reg)
        if val == ACCELRANGE_2G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_2G
        elif val == ACCELRANGE_4G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_4G
        elif val == ACCELRANGE_8G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_8G
        elif val == ACCELRANGE_16G:
            self._accel_mg_lsb = _MPU9250_ACCEL_MG_LSB_16G

    @property
    def accel_rate(self):
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2)
        return (reg & 0b00011000) & 0xFF

    @accel_rate.setter
    def accel_rate(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2)
        reg = reg & ~0x0F
        reg |= 0x03
        self._write_u8(_XGTYPE, _MPU9250_ACCEL_CONFIG2, reg)

    @property
    def gyro_scale(self):
        reg = self._read_u8(_XGTYPE, _MPU9250_GYRO_CONFIG)
        return (reg & 0b00011000) & 0xFF

    @gyro_scale.setter
    def gyro_scale(self, val):
        assert val in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)

        reg = self._read_u8(_XGTYPE, _MPU9250_GYRO_CONFIG)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _MPU9250_GYRO_CONFIG, reg)
        if val == GYROSCALE_245DPS:
            self._gyro_dps_digit = _MPU9250_GYRO_DPS_DIGIT_245DPS
        elif val == GYROSCALE_500DPS:
            self._gyro_dps_digit = _MPU9250_GYRO_DPS_DIGIT_500DPS
        elif val == GYROSCALE_2000DPS:
            self._gyro_dps_digit = _MPU9250_GYRO_DPS_DIGIT_2000DPS

    def read_accel_raw(self):
        self._read_bytes(_XGTYPE, 0x80 | _MPU9250_REGISTER_ACCEL_XOUT_L, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def acceleration(self):
        raw = self.read_accel_raw()
        return map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD,
                   raw)

    def read_mag_raw(self):
        self._read_bytes(_MAGTYPE, _MPU9250_REGISTER_MAG_XOUT_L, 6,
                         self._BUFFER)
        sleep(0.02)

        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def magnetic(self):
        raw = list(self.read_mag_raw())
        self._read_u8(_MAGTYPE, _MPU9250_REGISTER_STATUS_REG2_M)

        raw[0] *= self._adjustment_mag[0]
        raw[1] *= self._adjustment_mag[1]
        raw[2] *= self._adjustment_mag[2]

        if self._mag_calibrated:
            raw[0] -= self._offset_mag[0]
            raw[1] -= self._offset_mag[1]
            raw[2] -= self._offset_mag[2]

            raw[0] *= self._scale_mag[0]
            raw[1] *= self._scale_mag[1]
            raw[2] *= self._scale_mag[2]

        return (raw[0], raw[1], raw[2])

    def calibrate_mag(self, count=256, delay=200):
        raw = self.readmag_raw()
        self._offset_mag = (0,0,0)
        self._scale_mag = (1,1,1)

        reading = self.magnetic

        minx = maxx = reading[0]
        miny = maxy = reading[1]
        minz = maxz = reading[2]

        while count:
            sleep(delay / 1000)
            reading = self.magnetic
            minx = min(minx, reading[0])
            maxx = max(maxx, reading[0])
            miny = min(miny, reading[1])
            maxy = max(maxy, reading[1])
            minz = min(minz, reading[2])
            maxz = max(maxz, reading[2])
            count -= 1

        offset_x = (maxx + minx) / 2
        offset_y = (maxy + miny) / 2
        offset_z = (maxz + minz) / 2

        self._offset_mag = (offset_x, offset_y, offset_z)

        del offset_x, offset_y, offset_z

        avg_delta_x = (maxx - minx) / 2
        avg_delta_y = (maxy - miny) / 2
        avg_delta_z = (maxz - minz) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

        scale_x = avg_delta / avg_delta_x
        scale_y = avg_delta / avg_delta_y
        scale_z = avg_delta / avg_delta_z

        del avg_delta_x, avg_delta_y, avg_delta_z, avg_delta

        self._scale_mag = (scale_x, scale_y, scale_z)

        del scale_x, scale_y, scale_z

        self._mag_calibrated = True

        return self._offset_mag, self._scale_mag

    def read_gyro_raw(self):
        self._read_bytes(_XGTYPE, 0x80 | _MPU9250_REGISTER_GYRO_XOUT_L, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def gyro(self):
        raw = self.read_gyro_raw()
        return map(lambda x: x * self._gyro_dps_digit, raw)

    def read_temp_raw(self):
        self._read_bytes(_XGTYPE, 0x80 | _MPU9250_REGISTER_TEMP_OUT_H, 2,
                         self._BUFFER)
        temp = ((self._BUFFER[1] << 8) | self._BUFFER[0]) >> 4
        return _twos_comp(temp, 12)

    @property
    def temperature(self):
        temp = self.read_temp_raw()
        temp = ((temp - _MPU9250_TEMP_OFFSET) / _MPU9250_TEMP_SENS) + _MPU9250_TEMP_OFFSET
        return temp

    def _read_u8(self, sensor_type, address):
        raise NotImplementedError()

    def _read_bytes(self, sensor_type, address, count, buf):
        raise NotImplementedError()

    def _write_u8(self, sensor_type, address, val):
        raise NotImplementedError()


class MPU9250_I2C(MPU9250):

    def __init__(self, i2c, address=_MPU9250_ADDRESS_ACCELGYRO):
        self._xg_device = i2c_device.I2CDevice(i2c, address)
        self._bypass()
        self._mag_device = i2c_device.I2CDevice(i2c, _MPU9250_ADDRESS_MAG)
        super().__init__()

    def _bypass(self):
        if self._read_u8(_XGTYPE, _MPU9250_REGISTER_WHO_AM_I_XG) != _MPU9250_XG_ID:
            raise RuntimeError('Could not find MPU9250, check wiring!')
        self._write_u8(_XGTYPE, _MPU9250_INT_PIN_CFG, 0x02) # could also be 0x02, 0x22, 0x12
        self._write_u8(_XGTYPE, _MPU9250_INT_ENABLE, 0x01)

    def _read_u8(self, sensor_type, address):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write(self._BUFFER, end=1, stop=False)
            i2c.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _read_bytes(self, sensor_type, address, count, buf):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            buf[0] = address & 0xFF
            i2c.write(buf, end=1, stop=False)
            i2c.readinto(buf, end=count)

    def _write_u8(self, sensor_type, address, val):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)
