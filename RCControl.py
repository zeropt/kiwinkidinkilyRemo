import time
from math import *
 
import board
import busio
import adafruit_bme280
from adafruit_motorkit import MotorKit
 
# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit()

address = 0x08

roll = 0.0
pitch = 0.0
throttle = 0.0
yaw = 0.0

kr = 0.8
kp = 0.8
ky = 0.8

data = [0, 0, 0, 0]

def readData():
	i2c.readfrom_into(address, data, *, start = 0, end=len(data))

def constrain(x, Min, Max):
	if x > Max:
		x = Max
	if x < Min:
		x = Min
	return i

def combine(Roll, Pitch, Yaw, Theta):
	rVal = ky*Yaw + kp*cos(Theta)*Pitch + kr*cos(Theta+pi/2.0)*Roll
	return constrain(rVal, -1.0, 1.0)

while 1:
	readData
	roll = data[0]/127.0
	pitch = data[1]/127.0
	throttle = data[2]/127.0
	yaw = data[3]/127.0
	print roll
	time.sleep(0.01)