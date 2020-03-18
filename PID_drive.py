import time
from math import *
 
import board
import busio
import adafruit_bno055
from adafruit_motorkit import MotorKit
 
# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)
kit = MotorKit()

address = 0x08

roll = 0.0
pitch = 0.0
throttle = 0.0
yaw = 0.0

setpoint = 0.0

kr = 0.8
kp = 0.8
ky = 0.8

#PID Coefficients
kP = 1.5
kI = 0.1
kD = 0.05

class pid_controller:
    def __init__(self, kP = 1.0, kI = 1.0, kD = 1.0, setpoint = 0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.setpoint = setpoint
        self.out = 0.0
        self.I = 0.0
        self.t_prev = time.time()
        self.e_prev = 0.0

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def getOutput(self):
        return self.out

    def resetTime(self):
        self.t_prev = time.time()

    def update(self, sensorIn):
        error = self.setpoint - sensorIn
        t_delta = time.time() - self.t_prev #use time
        #t_delta = 1.0; #don't use time
        P = self.kP * error
        self.I += self.kI * error * t_delta
        D = self.kD * (error - self.e_prev)/t_delta
        self.out = P+self.I+D
        self.t_prev = time.time()
        self.e_prev = error

pid = pid_controller(kP, kI, kD, setpoint)

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

def getGyroData():
    angle_z, angle_x, angle_y = sensor.euler
    return angle_z

while 1:
    #readData()
    in_angle = getGyroData()
    pid.setSetpoint(setpoint)
    roll = 0.0 #(data[0]-127.0)/127.0
    pitch = 0.0 #(data[1]-127.0)/127.0
    throttle = 0.0 #(data[2]-127.0)/127.0
    yaw = 0.0 #(data[3]-127.0)/127.0
    if in_angle == None:
        pid.resetTime()
    else:
        angle = in_angle*2*pi/370.0
        if abs(setpoint-(angle+2*pi)) < abs(setpoint-angle):
            angle += 2*pi
        elif abs(setpoint-(angle-2*pi)) < abs(setpoint-angle):
            angle -= 2*pi
        pid.update(angle)
        yaw = -pid.getOutput()
    print("setpoint: {}".format(setpoint))
    print("angle: {}".format(in_angle))
    print("output: {}".format(yaw))
    print()
    kit.motor1.throttle = throttle
    kit.motor2.throttle = combine(roll, pitch, yaw, pi/3.0)
    kit.motor3.throttle = combine(roll, pitch, yaw, 5.0*pi/6.0)
    kit.motor4.throttle = combine(roll, pitch, yaw, 5.0*pi/2.0)
    time.sleep(0.1)
