import time
from math import *
 
import board
import busio
from adafruit_motorkit import MotorKit
 
# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit()

address = 0x08

roll = 0.0
pitch = 0.0
throttle = 0.0
yaw = 0.0

kr = 0.9
kp = 2.0
ky = 0.6

data = bytearray(4)

def readData():
	i2c.readfrom_into(address, data, start = 0, end=len(data))

def constrain(x, Min, Max):
	if x > Max:
		x = Max
	if x < Min:
		x = Min
	return x

def combine(Roll, Pitch, Yaw, Theta):
	rVal = ky*Yaw + kp*cos(Theta)*Pitch + kr*cos(Theta+pi/2.0)*Roll
	return constrain(rVal, -1.0, 1.0)

while 1:
	readData()
	roll = (data[0]-127.0)/127.0
	pitch = (data[1]-127.0)/127.0
	throttle = (data[2]-127.0)/127.0
	yaw = (data[3]-127.0)/127.0
	kit.motor1.throttle = constrain(throttle, -1.0, 1.0)
	kit.motor2.throttle = combine(roll, pitch, yaw, pi/3.0)
	kit.motor3.throttle = combine(roll, pitch, yaw, 5.0*pi/6.0)
	kit.motor4.throttle = combine(roll, pitch, yaw, 3.0*pi/2.0)
	time.sleep(0.1)
