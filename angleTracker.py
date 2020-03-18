import time
from math import *

import board
import busio
import adafruit_lsm9ds1

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

angle = 0.0
mag_angle_offset = 0.0
t_prev = time.time()
tau_gyro = 0.8 #for gyro data filter
tau_mag = 1.0 #tau_mag = 0.05 #0.0 to disable magnetic angle correction, 1.0 to only use the magnetic angle
min_gauss = 0.0 #min_gauss = 0.15 #0.0 to always be active, 100.0 to never be active

accel_x, accel_y, accel_z = sensor.acceleration
mag_x, mag_y, mag_z = sensor.magnetic
gyro_x, gyro_y, gyro_z = sensor.gyro
temp = sensor.temperature

gyro_z_filtered = 0.0

def zeroAngle():
    angle = 0.0
    mag_angle_offset = 0.0
    mag_angle_offset = getMagAngle()
    gyro_z_filtered = 0.0
    resetTimer()

def updateSensorData():
    global gyro_z_filtered
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    gyro_z_filtered = tau_gyro*(gyro_z - gyro_z_filtered)

def resetTimer():
    t_prev = time.time()

def updateAngle():
    global angle
    updateSensorData()
    t_delta = time.time() - t_prev
    angle += gyro_z_filtered*t_delta
    if (getMagMag() >= min_gauss):
        angle += tau_mag*(getMagAngle() - angle)
    resetTimer()

def getMagAngle():
    angle_m = degrees(atan(abs(mag_y/mag_x))) - mag_angle_offset
    if mag_x < 0.0 and mag_y < 0.0:
        angle_m += 180.0
    elif mag_x < 0.0:
        angle_m = 180.0 - angle
    elif mag_y < 0.0:
        ange_m = 360.0 - angle
    return angle_m

def getMagMag():
    return sqrt(mag_y**2.0 + mag_x**2.0)

zeroAngle()

while 1:
    updateAngle()
    #updateSensorData()
    print(angle)
    time.sleep(0.01)
