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

prev_active = 0
prev_throttle = 0.0

roll = 0.0
pitch = 0.0
throttle = 0.0
yaw = 0.0

setpoint = 0.0
heading = 0.0
field_zero = 0.0

kr = 0.8
kp = 0.8
ky = 0.8

yaw_limiter = 0.25;

#PID Coefficients
kP = 1.0
kI = 0.0
kD = 0.01

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

data = bytearray(5)

def readData():
    i2c.readfrom_into(address, data, start = 0, end=len(data))

def constrain(x, Min, Max):
    if x > Max:
        x = Max
    if x < Min:
        x = Min
    return x

def combine(Roll, Pitch, Yaw, Theta):
    rVal = Yaw + cos(Theta)*Pitch + cos(Theta+pi/2.0)*Roll
    return rVal, -1.0, 1.0

def getGyroData():
    angle_z, angle_x, angle_y = sensor.euler
    return angle_z

while 1:
    readData()
    in_angle = getGyroData()
    if data[0] > 0:
        roll = (data[1]-127.0)/127.0
        pitch = (data[2]-127.0)/127.0
        throttle = (data[3]-127.0)/127.0
        yaw = 0.0 #(data[4]-127.0)/127.0
        setpoint -= yaw_limiter*(data[4]-127.0)/127.0
        if setpoint >= 2*pi:
            setpoint -= 2*pi
        if setpoint < 0.0:
            setpoint += 2*pi
        pid.setSetpoint(setpoint)
        if in_angle == None:
            pid.resetTime()
        else:
            angle = in_angle*2*pi/370.0
            if abs(setpoint-(angle+2*pi)) < abs(setpoint-angle):
                angle += 2*pi
            elif abs(setpoint-(angle-2*pi)) < abs(setpoint-angle):
                angle -= 2*pi
            pid.update(angle)
            heading = angle
            yaw -= pid.getOutput()
        #print("setpoint: {}".format(setpoint))
        #print("angle: {}".format(in_angle))
        #print("output: {}".format(yaw))
        #print()
        kit.motor1.throttle = 0.0
        throttle_max = max([roll, pitch, yaw])
        print("throttle_max: {}".format(throttle_max))
        motor2_speed = 0.0
        motor3_speed = 0.0
        motor4_speed = 0.0
        if throttle > 0.0:
            motor2_speed = combine(roll, pitch, yaw, pi/3.0)
            motor3_speed = combine(roll, pitch, yaw, 5.0*pi/6.0)
            motor4_speed = combine(roll, pitch, yaw, 3.0*pi/2.0)
        else:
            if prev_throttle > 0.0:
                field_zero = heading
            motor2_speed = combine(roll, pitch, yaw, pi/3.0 - heading + field_zero)
            motor3_speed = combine(roll, pitch, yaw, 5.0*pi/6.0 - heading + field_zero)
            motor4_speed = combine(roll, pitch, yaw, 3.0*pi/2.0 - heading + field_zero)
        motor_max = max([motor2_speed, motor3_speed, motor4_speed])
        throttle_mult = throttle_max/motor_max
        kit.motor2.throttle = constrain(throttle_mult*motor2_speed, -1.0, 1.0)
        kit.motor3.throttle = constrain(throttle_mult*motor3_speed, -1.0, 1.0)
        kit.motor4.throttle = constrain(throttle_mult*motor4_speed, -1.0, 1.0)
        prev_throttle = throttle
        prev_active = data[0]
    else:
        if prev_active > 0:
            kit.motor1.throttle = 0.0
            kit.motor2.throttle = 0.0
            kit.motor3.throttle = 0.0
            kit.motor4.throttle = 0.0
            prev_throttle = 0.0
            prev_active = 0
    time.sleep(0.1)
