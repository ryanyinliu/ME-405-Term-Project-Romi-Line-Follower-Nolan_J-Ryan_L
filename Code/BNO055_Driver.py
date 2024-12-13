# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 16:58:07 2024

@author: NJ
"""

import struct
import time
from pyb import I2C, Pin


class IMU:
    def __init__ (self,i2c):
        self.i2c = I2C(i2c, I2C.CONTROLLER)
        self.BNO055_ADDR = 0x28
        self.CHIP_ID_ADDR = 0x00
        self.OPR_MODE = 0x3D
        self.CONFIG_MODE = 0x00
        self.NDOF_MODE = 0x0C
        self.CALIB_STAT_ADDR = 0x35
        self.CCEL_DATA_ADDR = 0x08
        self.EULA_DATA_ADDR = 0x1A
        self.GYRO_DATA_ADDR = 0x14
        # register addresses
        self.ACC_OFFSET_X_LSB = 0x55
        self.ACC_OFFSET_X_MSB = 0x56
        self.ACC_OFFSET_Y_LSB = 0x57
        self.ACC_OFFSET_Y_MSB = 0x58
        self.ACC_OFFSET_Z_LSB = 0x59
        self.ACC_OFFSET_Z_MSB = 0x5A
        self.MAG_OFFSET_X_LSB = 0x5B
        self.MAG_OFFSET_X_MSB = 0x5C
        self.MAG_OFFSET_Y_LSB = 0x5D
        self.MAG_OFFSET_Y_MSB = 0x5E
        self.MAG_OFFSET_Z_LSB = 0x5F
        self.MAG_OFFSET_Z_MSB = 0x60
        self.GYR_OFFSET_X_LSB = 0x61
        self.GYR_OFFSET_X_MSB = 0x62
        self.GYR_OFFSET_Y_LSB = 0x63
        self.GYR_OFFSET_Y_MSB = 0x64
        self.GYR_OFFSET_Z_LSB = 0x65
        self.GYR_OFFSET_Z_MSB = 0x66
        self.ACC_RADIUS_LSB = 0x67
        self.ACC_RADIUS_MSB = 0x68
        self.MAG_RADIUS_LSB = 0x69
        self.MAG_RADIUS_MSB = 0x6A
        self.reset_pin = Pin(Pin.cpu.C9, mode=Pin.OUT_PP)
    def set_mode(self,mode):
        self.chip_id = self.i2c.mem_read(1, self.BNO055_ADDR, self.CHIP_ID_ADDR)[0]
        #print("Chip ID:", hex(self.chip_id))
        self.i2c.mem_write(self.CONFIG_MODE, self.BNO055_ADDR, self.OPR_MODE)
        time.sleep(.1)
        #print("Config Mode")
        self.i2c.mem_write(mode, self.BNO055_ADDR, self.OPR_MODE)
        time.sleep(.1)
        #print("Mode changed:",mode)
    def read_cal_byte(self):
        while True:
            try:
                self.status = self.i2c.mem_read(1, self.BNO055_ADDR, self.CALIB_STAT_ADDR)[0]
                self.sys_status = (self.status >> 6) & 0x03
                self.gyr_status = (self.status >> 4) & 0x03
                self.acc_status = (self.status >> 2) & 0x03
                self.mag_status = self.status & 0x03
        
                #print("Calibration Status - Sys:", self.sys_status, "Gyro:", self.gyr_status, "Accel:", self.acc_status, "Mag:", self.mag_status)
                time.sleep(1)
                # If the system is fully calibrated, break the loop
                if self.sys_status == 3 and self.gyr_status == 3 and self.acc_status == 3 and self.mag_status == 3:
                    #print("Calibration Done")
                    break
            except KeyboardInterrupt:
                break
    #def read_cal_co(self):
    def read_cal_co(self):
        def read_16bit(lsb_addr):
            lsb = self.i2c.mem_read(1, self.BNO055_ADDR, lsb_addr)[0]
            msb = self.i2c.mem_read(1, self.BNO055_ADDR, lsb_addr + 1)[0]
            return (msb << 8) | lsb
        # Read and display calibration coefficients
        self.calibration_data = {}
        self.calibration_data['acc_offset_x'] = read_16bit(self.ACC_OFFSET_X_LSB)
        self.calibration_data['acc_offset_y'] = read_16bit(self.ACC_OFFSET_Y_LSB)
        self.calibration_data['acc_offset_z'] = read_16bit(self.ACC_OFFSET_Z_LSB)
        self.calibration_data['mag_offset_x'] = read_16bit(self.MAG_OFFSET_X_LSB)
        self.calibration_data['mag_offset_y'] = read_16bit(self.MAG_OFFSET_Y_LSB)
        self.calibration_data['mag_offset_z'] = read_16bit(self.MAG_OFFSET_Z_LSB)
        self.calibration_data['gyr_offset_x'] = read_16bit(self.GYR_OFFSET_X_LSB)
        self.calibration_data['gyr_offset_y'] = read_16bit(self.GYR_OFFSET_Y_LSB)
        self.calibration_data['gyr_offset_z'] = read_16bit(self.GYR_OFFSET_Z_LSB)
        self.calibration_data['acc_radius'] = read_16bit(self.ACC_RADIUS_LSB)
        self.calibration_data['mag_radius'] = read_16bit(self.MAG_RADIUS_LSB)
        #print("Calibration Coefficients:", self.calibration_data)
        
    #def write_cal_co(self):
    def write_cal_co(self):
        self.calibration_data = {
            'acc_offset_x': 65529,
            'acc_offset_y': 65400,
            'acc_offset_z': 65526,
            'mag_offset_x': 1764,
            'mag_offset_y': 63741,
            'mag_offset_z': 1703,
            'gyr_offset_x': 65534,
            'gyr_offset_y': 65535,
            'gyr_offset_z': 65535,
            'acc_radius': 1000,
            'mag_radius': 611,}
        def write_16bit_register(i2c, device_address, lsb_reg, msb_reg, value):
            lsb = value & 0xFF
            msb = (value >> 8) & 0xFF
            i2c.mem_write(lsb, device_address, lsb_reg)
            i2c.mem_write(msb, device_address, msb_reg)
            time.sleep(0.01)  # Small delay for stability
        write_16bit_register(self.i2c, 0x28, self.ACC_OFFSET_X_LSB, self.ACC_OFFSET_X_MSB, self.calibration_data['acc_offset_x'])
        write_16bit_register(self.i2c, 0x28, self.ACC_OFFSET_Y_LSB, self.ACC_OFFSET_Y_MSB, self.calibration_data['acc_offset_y'])
        write_16bit_register(self.i2c, 0x28, self.ACC_OFFSET_Z_LSB, self.ACC_OFFSET_Z_MSB, self.calibration_data['acc_offset_z'])
        write_16bit_register(self.i2c, 0x28, self.MAG_OFFSET_X_LSB, self.MAG_OFFSET_X_MSB, self.calibration_data['mag_offset_x'])
        write_16bit_register(self.i2c, 0x28, self.MAG_OFFSET_Y_LSB, self.MAG_OFFSET_Y_MSB, self.calibration_data['mag_offset_y'])
        write_16bit_register(self.i2c, 0x28, self.MAG_OFFSET_Z_LSB, self.MAG_OFFSET_Z_MSB, self.calibration_data['mag_offset_z'])
        write_16bit_register(self.i2c, 0x28, self.GYR_OFFSET_X_LSB, self.GYR_OFFSET_X_MSB, self.calibration_data['gyr_offset_x'])
        write_16bit_register(self.i2c, 0x28, self.GYR_OFFSET_Y_LSB, self.GYR_OFFSET_Y_MSB, self.calibration_data['gyr_offset_y'])
        write_16bit_register(self.i2c, 0x28, self.GYR_OFFSET_Z_LSB, self.GYR_OFFSET_Z_MSB, self.calibration_data['gyr_offset_z'])
        write_16bit_register(self.i2c, 0x28, self.ACC_RADIUS_LSB, self.ACC_RADIUS_MSB, self.calibration_data['acc_radius'])
        write_16bit_register(self.i2c, 0x28, self.MAG_RADIUS_LSB, self.MAG_RADIUS_MSB, self.calibration_data['mag_radius'])

        
    def read_eula(self):
        self.eula_buf = bytearray([0 for n in range(6)])
        self.i2c.mem_read(self.eula_buf, self.BNO055_ADDR, self.EULA_DATA_ADDR)
        self.eula_x, self.eula_y, self.eula_z = struct.unpack('<hhh', self.eula_buf)
        self.eula_x /= 16.0
        self.eula_y /= 16.0
        self.eula_z /= 16.0
        #print("Euler Angle X:", self.eula_x, "Y:", self.eula_y, "Z:", self.eula_z)
        return self.eula_x, self.eula_y, self.eula_z
    def read_angv(self):
        self.gyro_buf = bytearray([0 for n in range(6)])
        self.i2c.mem_read(self.gyro_buf, self.BNO055_ADDR, self.GYRO_DATA_ADDR)
        self.gyro_x, self.gyro_y, self.gyro_z = struct.unpack('<hhh', self.gyro_buf)
        self.gyro_x /= 16.0
        self.gyro_y /= 16.0
        self.gyro_z /= 16.0
        #print("Angular Velocity X:", self.gyro_x, "Y:", self.gyro_y, "Z:", self.gyro_z)
        return self.gyro_x, self.gyro_y, self.gyro_z
    def reset_IMU(self):
        self.reset_pin.low()
        #print("Reset pin low")
        time.sleep(.5)
        self.reset_pin.high()
        #print("Reset pin high")
        time.sleep(.5)